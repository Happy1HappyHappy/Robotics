import rclpy
from rclpy.node import Node

class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        self.get_logger().info('node1 started')

def main(args=None):
    rclpy.init(args=args)
    node = Node1()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
