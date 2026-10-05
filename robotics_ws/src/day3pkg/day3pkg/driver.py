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
    def __init__(self):
        super().__init__('driving_node')

        # Parameters (slides 56-58): tune speeds without rebuilding
        self.declare_parameter('linear_speed', 0.15)   # m/s
        self.declare_parameter('angular_speed', 0.5)   # rad/s
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

    # ---------- accept / reject (slide 23) ----------
    def goal_callback(self, goal: Drive.Goal):
        d, a = goal.distance, goal.angle
        if d < 0.0 or a < 0.0:
            self.get_logger().warn(f'Reject: negative value (distance={d}, angle={a})')
            return GoalResponse.REJECT
        if d != 0.0 and a != 0.0:
            self.get_logger().warn(f'Reject: both non-zero (distance={d}, angle={a})')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accept: distance={d} m, angle={a} deg')
        return GoalResponse.ACCEPT

    # ---------- cancel (slide 54) ----------
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def publish_cmd(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear     # only linear.x and angular.z are ever set
        msg.twist.angular.z = angular
        self.cmd_pub_.publish(msg)

    # ---------- the long-running part (slide 24) ----------
    def execute_callback(self, goal_handle):
        d = goal_handle.request.distance
        a = goal_handle.request.angle
        v = self.get_parameter('linear_speed').value
        w = self.get_parameter('angular_speed').value
        period = 1.0 / self.get_parameter('rate_hz').value

        if d > 0.0:
            linear, angular, duration = v, 0.0, d / v
        else:
            linear, angular, duration = 0.0, w, math.radians(a) / w

        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds / 1e9 < duration:
            if goal_handle.is_cancel_requested:
                self.publish_cmd(0.0, 0.0)
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

