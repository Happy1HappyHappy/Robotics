import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        self.get_logger().info('node1 started')
        # use _ to indicate that the variable is private for this class
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        # register a timer callback, which will be called every 1 second
        self.timer_ = self.create_timer(1.0, self.callback)
        self.counter = 0    # count the number of messages sent
        self.done = False   # indicate whether the node has finished sending msgs

    def send(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'   # robot base frame
        self.publisher_.publish(msg)

    def callback(self):
        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().info('Waiting for robot..')
            return

        if self.counter < 3:
            msg = TwistStamped()
            msg.twist.linear.x = 0.1
            self.send(msg)
            self.counter += 1
            self.get_logger().info(f'Callback {self.counter}')
        else:
            # in case robot not get the last velocity command, send three times
            for _ in range(3):
                # send zero velocity message to stop the robot
                self.send(TwistStamped())
            self.get_logger().info('Robot stopped')
            self.timer_.cancel()    # stop the timer to prevent further callbacks
            self.done = True        # indicate that the node has finished sending messages


def main(args=None):
    rclpy.init(args=args)
    node = Node1()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.2)      # wait for the last message to be sent
    node.destroy_node()  # destroy the node to clean up resources
    rclpy.shutdown()     # shutdown the ROS client library


if __name__ == '__main__':
    main()
