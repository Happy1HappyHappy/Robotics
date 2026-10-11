"""
Driving node for the day3pkg package
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from day3_interfaces.action import Drive


def yaw_from_quaternion(q):
    """
    Extract yaw (rotation about z) from a geometry_msgs/Quaternion
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    """
    Wrap an angle to [-pi, pi)
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class DrivingNode(Node):
    """
    Driving node class for the day3pkg package
    Handles driving goals via the /drive action server
    Logs the motion reported by odometry after each goal
    """
    def __init__(self):
        super().__init__('driving_node')

        # tune speeds without rebuilding
        self.declare_parameter('linear_speed', 0.15)   # m/s
        self.declare_parameter('angular_speed', 0.5)   # rad/s
        self.declare_parameter('max_angular_speed', 1.82)  # rad/s, Waffle limit (Burger 2.84)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('settle_time', 1.0)     # s, wait after stop before reading odom

        self.cmd_pub_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # latest odometry message, overwritten on every callback (single writer)
        # relative name 'odom' so it follows the same namespace as 'cmd_vel'
        self.latest_odom_ = None
        self.odom_sub_ = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)

        self.action_server_ = ActionServer(
            self, Drive, 'drive',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('Driving Node ready (action: drive)')

    # ---------- odometry ----------
    def odom_callback(self, msg: Odometry):
        """
        Store the latest odometry message
        Runs in the node's default (MutuallyExclusive) callback group, which is
        separate from the action's reentrant group, so with the MultiThreadedExecutor
        it keeps running while execute_callback is busy in its loop
        """
        self.latest_odom_ = msg

    def snapshot_pose(self):
        """
        Return the current odometry pose as (x, y, yaw), or None if no odom yet
        """
        msg = self.latest_odom_
        if msg is None:
            return None
        p = msg.pose.pose.position
        return p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)

    def report_motion(self, start_pose, end_pose, target):
        """
        Log the motion between two odometry poses, and its error from the target
        forward/lateral are the displacement expressed in the robot's starting frame,
        i.e. the translation part of inv(T_start) * T_end
        target: commanded (forward m, lateral m, yaw rad) in the same frame
        """
        x0, y0, th0 = start_pose
        x1, y1, th1 = end_pose
        dx, dy = x1 - x0, y1 - y0
        forward = math.cos(th0) * dx + math.sin(th0) * dy
        lateral = -math.sin(th0) * dx + math.cos(th0) * dy
        dyaw = wrap_angle(th1 - th0)
        self.get_logger().info(
            f'Odom: dist={math.hypot(dx, dy):.4f} m, forward={forward:.4f} m, '
            f'lateral={lateral:.4f} m, dyaw={math.degrees(dyaw):.2f} deg')
        tf, tl, tyaw = target
        ef, el = forward - tf, lateral - tl
        self.get_logger().info(
            f'Error (odom - target): forward={ef:.4f} m, lateral={el:.4f} m, '
            f'position={math.hypot(ef, el):.4f} m, '
            f'yaw={math.degrees(wrap_angle(dyaw - tyaw)):.2f} deg')

    # ---------- accept / reject ----------
    def goal_callback(self, goal: Drive.Goal):
        """
        Accept or reject a driving goal based on distance, angle and radius
        Reject if distance or radius is negative, if both distance and angle are non-zero,
        or if an arc has a distance or an angle outside [0, 360]
        A negative angle is allowed only for a turn in place (turn right)
        """
        d, a, r = goal.distance, goal.angle, goal.radius
        if d < 0.0 or r < 0.0 or (r > 0.0 and a < 0.0):
            self.get_logger().warn(
                f'Reject: negative value (distance={d}, angle={a}, radius={r})')
            return GoalResponse.REJECT
        if r > 0.0:
            if d != 0.0 or a > 360.0:
                self.get_logger().warn(
                    f'Reject: bad arc (distance={d}, angle={a}, radius={r})')
                return GoalResponse.REJECT
            side = 'right' if goal.turn_right else 'left'
            self.get_logger().info(f'Accept: arc {side}, radius={r} m, angle={a} deg')
            return GoalResponse.ACCEPT
        if d != 0.0 and a != 0.0:
            self.get_logger().warn(f'Reject: both non-zero (distance={d}, angle={a})')
            return GoalResponse.REJECT
        if a != 0.0:
            side = 'right' if a < 0.0 else 'left'
            self.get_logger().info(f'Accept: turn {side} {abs(a)} deg')
        else:
            self.get_logger().info(f'Accept: move {d} m')
        return GoalResponse.ACCEPT

    # ---------- cancel ----------
    def cancel_callback(self, goal_handle):
        """
        Handle a cancel request for a driving goal
        Always accept cancel requests
        """
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    # ---------- publish velocity command ----------
    def publish_cmd(self, linear: float, angular: float):
        """
        Publish a velocity command to the robot
        linear: linear velocity in m/s
        angular: angular velocity in rad/s
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear     # only linear.x and angular.z are ever set
        msg.twist.angular.z = angular
        self.cmd_pub_.publish(msg)

    # ---------- the long-running part ----------
    def execute_callback(self, goal_handle):
        """
        Execute a driving goal by publishing velocity commands over time
        Handles straight moves, turns in place, and circular arcs
        Stops the robot if the goal is canceled
        Logs the odometry-reported motion once the robot has come to rest
        """
        d = goal_handle.request.distance
        a = goal_handle.request.angle
        c = goal_handle.request.radius
        v = self.get_parameter('linear_speed').value
        w = self.get_parameter('angular_speed').value
        period = 1.0 / self.get_parameter('rate_hz').value

        if c > 0.0:
            # arc (Day 4 inverse kinematics): w = v / c, t = c * theta / v
            # slow down on tight arcs so w stays under the robot's limit
            v = min(v, self.get_parameter('max_angular_speed').value * c)
            sign = -1.0 if goal_handle.request.turn_right else 1.0   # +angular.z = left
            linear, angular, duration = v, sign * v / c, c * math.radians(a) / v
            # arc end point in the start frame: (c sin(theta), +-c (1 - cos(theta)))
            th = math.radians(a)
            target = (c * math.sin(th), sign * c * (1.0 - math.cos(th)), sign * th)
        elif d > 0.0:
            linear, angular, duration = v, 0.0, d / v
            target = (d, 0.0, 0.0)
        else:
            # turn in place: sign of angle picks the direction (+ = left/CCW, - = right/CW)
            linear, angular, duration = 0.0, math.copysign(w, a), math.radians(abs(a)) / w
            target = (0.0, 0.0, math.radians(a))

        start_pose = self.snapshot_pose()
        if start_pose is None:
            self.get_logger().warn('No odometry received yet; motion will not be reported')

        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < duration:
            if goal_handle.is_cancel_requested:
                self.publish_cmd(0.0, 0.0)  # stop the robot immediately upon cancel request
                goal_handle.canceled()
                self.get_logger().info('Goal canceled, robot stopped')
                return Drive.Result()
            self.publish_cmd(linear, angular)
            time.sleep(period)

        self.publish_cmd(0.0, 0.0)          # explicit stop
        elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
        self.get_logger().info(f'Planned {duration:.3f} s, stop sent at {elapsed:.3f} s')

        # let the robot come to rest before reading the end pose
        time.sleep(self.get_parameter('settle_time').value)
        end_pose = self.snapshot_pose()
        if start_pose is not None and end_pose is not None:
            self.report_motion(start_pose, end_pose, target)

        goal_handle.succeed()
        self.get_logger().info('Goal succeeded')
        return Drive.Result()               # empty for now


def main(args=None):
    """
    Entry point for the driving node
    Initializes the ROS 2 system, creates the node, and spins the executor
    Ensures the robot is stopped and resources are cleaned up on shutdown
    """
    rclpy.init(args=args)
    node = DrivingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
