"""
Driving node for the day3pkg package
Drives with the Create 3's built-in actions (drive_distance, rotate_angle, drive_arc),
which the robot runs closed loop on its odometry. The old version timed cmd_vel
commands; that code is kept in execute_callback as comments marked 'old:'
"""

import csv
import math
import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.action import DriveArc, DriveDistance, RotateAngle
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
    Handles driving goals via the /drive action server, passing each one on to the
    robot's built-in drive_distance, rotate_angle or drive_arc action
    Logs the motion reported by odometry after each goal
    """
    def __init__(self):
        super().__init__('driving_node')

        # tune speeds without rebuilding
        self.declare_parameter('linear_speed', 0.15)   # m/s
        self.declare_parameter('angular_speed', 0.5)   # rad/s
        self.declare_parameter('max_angular_speed', 1.82)  # rad/s, Waffle limit (Burger 2.84)
        self.declare_parameter('rate_hz', 10.0)        # Hz, how often to check for a cancel
        self.declare_parameter('settle_time', 1.0)     # s, wait after stop before reading odom
        self.declare_parameter('csv_path', 'odom_results.csv')  # '' = do not save

        # old: drove with this; now only used for a zero command on shutdown
        self.cmd_pub_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # the robot's built-in actions (run on the robot, closed loop on its odometry)
        # their own reentrant group, so their replies are handled while
        # execute_callback waits for them
        robot_group = ReentrantCallbackGroup()
        self.robot_clients_ = {
            'drive_distance': ActionClient(self, DriveDistance, 'drive_distance',
                                           callback_group=robot_group),
            'rotate_angle': ActionClient(self, RotateAngle, 'rotate_angle',
                                         callback_group=robot_group),
            'drive_arc': ActionClient(self, DriveArc, 'drive_arc', callback_group=robot_group),
        }
        self.robot_goal_ = None     # the robot's goal running now, so shutdown can cancel it

        # latest odometry message, overwritten on every callback (single writer)
        # relative name 'odom' so it follows the same namespace as 'cmd_vel'
        self.latest_odom_ = None
        self.odom_sub_ = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)

        # odometry results of the most recent goals, oldest dropped first
        self.history_ = deque(maxlen=20)
        self.run_count_ = 0

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

    def report_motion(self, start_pose, end_pose, target, label):
        """
        Log the motion between two odometry poses, and its error from the target
        forward/lateral are the displacement expressed in the robot's starting frame,
        i.e. the translation part of inv(T_start) * T_end
        target: commanded (forward m, lateral m, yaw rad) in the same frame
        label: short text for the command, shown in the results table
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

        self.run_count_ += 1
        self.history_.append((self.run_count_, label, forward, lateral, math.degrees(dyaw),
                              ef, math.hypot(ef, el),
                              math.degrees(wrap_angle(dyaw - tyaw))))
        self.log_history()

    def log_history(self):
        """
        Log a table of the last 20 odometry results, with the mean and sample
        standard deviation of each error column
        """
        rows = [f'{"run":>4} {"command":<14} {"forward":>8} {"lateral":>8} {"dyaw":>8} '
                f'{"err_fwd":>8} {"err_pos":>8} {"err_yaw":>8}']
        for n, label, fwd, lat, dyaw, ef, ep, ey in self.history_:
            rows.append(f'{n:>4} {label:<14} {fwd:>8.4f} {lat:>8.4f} {dyaw:>8.2f} '
                        f'{ef:>8.4f} {ep:>8.4f} {ey:>8.2f}')
        errors = list(zip(*[h[5:] for h in self.history_]))
        means = [statistics.mean(col) for col in errors]
        stds = [statistics.stdev(col) if len(col) > 1 else 0.0 for col in errors]
        pad = ' ' * 43    # skip run..dyaw so the stats line up under the error columns
        rows.append(f'{"mean":>4}{pad}{means[0]:>8.4f} {means[1]:>8.4f} {means[2]:>8.2f}')
        rows.append(f'{"std":>4}{pad}{stds[0]:>8.4f} {stds[1]:>8.4f} {stds[2]:>8.2f}')
        self.get_logger().info('Odometry results (last 20):\n' + '\n'.join(rows))
        self.save_csv(means, stds)

    def save_csv(self, means, stds):
        """
        Overwrite the CSV file with the same table as log_history (last 20 results,
        then mean and std rows), so the file always matches the latest table
        """
        path = self.get_parameter('csv_path').value
        if not path:
            return
        try:
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['run', 'command', 'forward_m', 'lateral_m', 'dyaw_deg',
                                 'err_forward_m', 'err_position_m', 'err_yaw_deg'])
                for n, label, fwd, lat, dyaw, ef, ep, ey in self.history_:
                    writer.writerow([n, label, f'{fwd:.4f}', f'{lat:.4f}', f'{dyaw:.2f}',
                                     f'{ef:.4f}', f'{ep:.4f}', f'{ey:.2f}'])
                for name, stats in (('mean', means), ('std', stds)):
                    writer.writerow([name, '', '', '', '', f'{stats[0]:.4f}',
                                     f'{stats[1]:.4f}', f'{stats[2]:.2f}'])
        except OSError as e:
            self.get_logger().warn(f'Could not save {path}: {e}')

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

    # ---------- the robot's built-in actions ----------
    def run_robot_goal(self, goal_handle, action, goal, duration):
        """
        Send goal to the robot's built-in action (drive_distance, rotate_angle or
        drive_arc) and wait until the robot has finished it
        A cancel of our drive goal is passed on to the robot. A watchdog also cancels the
        robot's goal if it runs longer than 2 * duration + 5 s (duration: planned time, s)
        Returns 'succeeded', 'canceled', or what went wrong
        """
        client = self.robot_clients_[action]
        period = 1.0 / self.get_parameter('rate_hz').value
        if not client.wait_for_server(timeout_sec=2.0):
            return 'not available (is the robot connected?)'
        deadline = time.monotonic() + 2.0 * duration + 5.0

        send_future = client.send_goal_async(goal)
        while not send_future.done():
            if time.monotonic() > deadline:
                return 'got no answer from the robot'
            time.sleep(period)
        robot_goal = send_future.result()
        if not robot_goal.accepted:
            return 'rejected by the robot (is it docked?)'

        self.robot_goal_ = robot_goal
        try:
            result_future = robot_goal.get_result_async()
            stop_reason = None      # set once we cancel the robot's goal
            while not result_future.done():
                if stop_reason is None and (goal_handle.is_cancel_requested or
                                            time.monotonic() > deadline):
                    # the robot ignores cmd_vel while it runs a goal: cancel the goal itself
                    stop_reason = 'canceled' if goal_handle.is_cancel_requested else 'timed out'
                    robot_goal.cancel_goal_async()
                    deadline = time.monotonic() + 5.0     # time for the robot to confirm
                elif stop_reason is not None and time.monotonic() > deadline:
                    return f'{stop_reason}, but the robot did not confirm the cancel'
                time.sleep(period)
        finally:
            self.robot_goal_ = None

        if stop_reason is not None:
            return stop_reason
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            # the robot aborts its goal on a cliff or a wheel stall (but not on a bump)
            return f'ended by the robot with status {status}'
        return 'succeeded'

    # ---------- the long-running part ----------
    def execute_callback(self, goal_handle):
        """
        Execute a driving goal with one of the robot's built-in actions:
        drive_distance for straight moves, rotate_angle for turns in place, and
        drive_arc for circular arcs; the robot runs them closed loop on its odometry
        (old: published velocity commands over time; kept below as 'old:' comments)
        Stops the robot if the goal is canceled
        Logs the odometry-reported motion once the robot has come to rest
        """
        d = goal_handle.request.distance
        a = goal_handle.request.angle
        c = goal_handle.request.radius
        v = self.get_parameter('linear_speed').value
        w = self.get_parameter('angular_speed').value
        # old: period = 1.0 / self.get_parameter('rate_hz').value

        if c > 0.0:
            # arc (Day 4 inverse kinematics): w = v / c, t = c * theta / v
            # slow down on tight arcs so w stays under the robot's limit
            v = min(v, self.get_parameter('max_angular_speed').value * c)
            sign = -1.0 if goal_handle.request.turn_right else 1.0   # + = left (CCW)
            # old: linear, angular, duration = v, sign * v / c, c * math.radians(a) / v
            th = math.radians(a)
            duration = c * th / v                       # s at full speed (for the watchdog)
            action, goal = 'drive_arc', DriveArc.Goal()
            goal.translate_direction = DriveArc.Goal.TRANSLATE_FORWARD
            goal.angle = sign * th                      # rad along the arc, + = left
            goal.radius = float(c)                      # m
            goal.max_translation_speed = float(v)       # m/s
            # arc end point in the start frame: (c sin(theta), +-c (1 - cos(theta)))
            target = (c * math.sin(th), sign * c * (1.0 - math.cos(th)), sign * th)
            label = f'a {"r" if goal_handle.request.turn_right else "l"} {c:g} {a:g}'
        elif d > 0.0:
            # old: linear, angular, duration = v, 0.0, d / v
            duration = d / v
            action, goal = 'drive_distance', DriveDistance.Goal()
            goal.distance = float(d)                    # m
            goal.max_translation_speed = float(v)       # m/s
            target = (d, 0.0, 0.0)
            label = f'm {d:g}'
        else:
            # turn in place: sign of angle picks the direction (+ = left/CCW, - = right/CW)
            # old: linear, angular, duration = 0.0, math.copysign(w, a), math.radians(abs(a)) / w
            duration = math.radians(abs(a)) / w
            action, goal = 'rotate_angle', RotateAngle.Goal()
            goal.angle = math.radians(a)                # rad, the sign gives the direction
            goal.max_rotation_speed = float(w)          # rad/s
            target = (0.0, 0.0, math.radians(a))
            label = f't {a:g}'

        start_pose = self.snapshot_pose()
        if start_pose is None:
            self.get_logger().warn('No odometry received yet; motion will not be reported')

        start = self.get_clock().now()
        result = self.run_robot_goal(goal_handle, action, goal, duration)
        elapsed = (self.get_clock().now() - start).nanoseconds / 1e9

        # old: publish the velocity for the planned time, then stop
        # while (self.get_clock().now() - start).nanoseconds / 1e9 < duration:
        #     if goal_handle.is_cancel_requested:
        #         self.publish_cmd(0.0, 0.0)  # stop the robot immediately upon cancel request
        #         goal_handle.canceled()
        #         self.get_logger().info('Goal canceled, robot stopped')
        #         return Drive.Result()
        #     self.publish_cmd(linear, angular)
        #     time.sleep(period)
        #
        # self.publish_cmd(0.0, 0.0)          # explicit stop
        # elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
        # self.get_logger().info(f'Planned {duration:.3f} s, stop sent at {elapsed:.3f} s')

        if result == 'canceled':
            goal_handle.canceled()
            self.get_logger().info('Goal canceled, robot stopped')
            return Drive.Result()
        if result != 'succeeded':
            goal_handle.abort()
            self.get_logger().warn(f'Goal aborted: {action} {result}')
            return Drive.Result()
        self.get_logger().info(f'{action} done in {elapsed:.3f} s '
                               f'({duration:.3f} s at full speed)')

        # let the robot come to rest before reading the end pose
        time.sleep(self.get_parameter('settle_time').value)
        end_pose = self.snapshot_pose()
        if start_pose is not None and end_pose is not None:
            self.report_motion(start_pose, end_pose, target, label)

        goal_handle.succeed()
        self.get_logger().info('Goal succeeded')
        return Drive.Result()               # empty for now

    # ---------- shutdown ----------
    def stop_robot(self, executor):
        """
        Stop the robot on shutdown: cancel the robot's goal if one is running (the robot
        ignores cmd_vel while it runs a goal), then send a zero velocity as before
        Best effort: once ROS itself is shutting down, the cancel may not get through
        """
        robot_goal = self.robot_goal_
        if robot_goal is not None:
            try:
                future = robot_goal.cancel_goal_async()
                executor.spin_until_future_complete(future, timeout_sec=2.0)
                if future.done():
                    self.get_logger().info('Canceled the robot goal')
                else:
                    self.get_logger().warn('No answer to canceling the robot goal')
            except Exception as e:      # whatever fails here, still send the zero command
                self.get_logger().warn(f'Could not cancel the robot goal: {e}')
        self.publish_cmd(0.0, 0.0)


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
        # old: node.publish_cmd(0.0, 0.0)    (now done inside stop_robot)
        node.stop_robot(executor)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
