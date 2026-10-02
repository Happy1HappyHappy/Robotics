import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        self.get_logger().info('node1 started')
        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.callback)
        self.counter = 0
        self.done = False


    def callback(self):
        if self.publisher.get_subscription_count() == 0:
            self.get_logger().info('Waiting for robot..')
            return
        
        msg = TwistStamped()
        msg.header.stamp =self.get_clock().now().to_msg()

        if self.counter < 3:
            msg.twist.linear.x = 0.1
            self.publisher.publish(msg)
            self.counter += 1
            self.get_logger().info(f'Callback {self.counter}')
        else:
            self.publisher.publish(msg)
            self.get_logger().info('stop')
            self.timer.cancel()
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = Node1()
    while rclpy.ok() and not node.done:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
