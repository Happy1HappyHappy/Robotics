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
from geometry_msgs.msg import TwistStamped

from day3_interfaces.action import Drive


class DrivingNode(Node):
    """
    Driving node class for the day3pkg package
    Handles driving goals via the /drive action server
    """
    def __init__(self):
        super().__init__('driving_node')

        # tune speeds without rebuilding
        self.declare_parameter('linear_speed', 0.15)   # m/s
        self.declare_parameter('angular_speed', 0.5)   # rad/s
        self.declare_parameter('max_angular_speed', 2.84)  # rad/s, adjust for your robot
        #   so tight arcs can cap w (Burger ~2.84 rad/s, Waffle ~1.82 rad/s)
        self.declare_parameter('rate_hz', 10.0)

        self.cmd_pub_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.action_server_ = ActionServer(
            self, Drive, 'drive',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('Driving Node ready (action: drive)')

    # ---------- accept / reject ----------
    def goal_callback(self, goal: Drive.Goal):
        """
        Accept or reject a driving goal based on distance, angle and radius
        Reject if any is negative, if both distance and angle are non-zero,
        or if an arc has a distance or an angle outside [0, 360]
        """
        d, a, r = goal.distance, goal.angle, goal.radius
        if d < 0.0 or a < 0.0 or r < 0.0:
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
        self.get_logger().info(f'Accept: distance={d} m, angle={a} deg')
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
        elif d > 0.0:
            linear, angular, duration = v, 0.0, d / v
        else:
            linear, angular, duration = 0.0, w, math.radians(a) / w

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
