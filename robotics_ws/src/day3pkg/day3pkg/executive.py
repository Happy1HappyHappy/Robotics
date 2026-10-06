"""
Executive node for controlling the robot via high-level commands
Provides a command-line interface to send driving goals to the Driving Node
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from day3_interfaces.action import Drive

HELP = 'Commands:  m <meters>   t <degrees>   p (letter P)   8 (figure-8)   q (quit)'


class ExecutiveNode(Node):
    """
    Node responsible for sending high-level driving commands to Driving Node
    """
    def __init__(self):
        super().__init__('executive_node')
        self.client_ = ActionClient(self, Drive, 'drive')

    def send(self, distance: float, angle: float) -> bool:
        """
        Send a driving goal to the Driving Node
        distance: linear distance in meters
        angle: rotation angle in degrees
        Returns True if the goal was accepted and succeeded, False otherwise
        """
        if not self.client_.wait_for_server(timeout_sec=5.0):
            print('Driving Node is not available.')
            return False

        goal = Drive.Goal()
        goal.distance = float(distance)   # float64 fields reject Python ints
        goal.angle = float(angle)

        # send goal, wait for accept/reject
        send_future = self.client_.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print('Request REJECTED by Driving Node.')
            return False
        print('Request accepted, moving...')

        # wait for the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print('Done.')
            return True
        if status == GoalStatus.STATUS_CANCELED:
            print('Action was CANCELED.')
        else:
            print(f'Action ended with status {status}.')
        return False


# Extra Credit A: each shape is a list of single-step commands
SHAPES = {
    # squared "P": stem up, then the bowl on the right
    'p': [('m', 1.0), ('t', 270), ('m', 0.4), ('t', 270),
          ('m', 0.5), ('t', 270), ('m', 0.4)],
    # squared figure-8: CCW square, straight through the center, CW square
    '8': [('m', 0.5), ('t', 90), ('m', 0.5), ('t', 90), ('m', 0.5), ('t', 90),
          ('m', 1.0), ('t', 270), ('m', 0.5), ('t', 270), ('m', 0.5), ('t', 270),
          ('m', 0.5)],
}


def run_shape(node, name):
    """
    Execute a predefined shape by sending a sequence of driving commands
    to the Executive Node
    node: instance of ExecutiveNode
    name: name of the shape to run (must be a key in SHAPES)
    """
    for kind, value in SHAPES[name]:
        ok = node.send(value, 0.0) if kind == 'm' else node.send(0.0, value)
        if not ok:
            print('Shape aborted.')
            return
    print(f'Shape {name} done.')


def main(args=None):
    """
    Entry point for the Executive Node command-line interface
    Initializes the ROS 2 system, creates the node, and processes user commands
    Cleans up resources on shutdown
    """
    rclpy.init(args=args)
    node = ExecutiveNode()
    print(HELP)
    try:
        while True:
            parts = input('> ').strip().split()
            if not parts:
                continue
            cmd = parts[0].lower()
            if cmd == 'q':
                break
            if cmd in SHAPES and len(parts) == 1:
                run_shape(node, cmd)
                continue
            if cmd not in ('m', 't') or len(parts) != 2:
                print(HELP)
                continue
            try:
                value = float(parts[1])
            except ValueError:
                print('Value must be a number.')
                continue
            if cmd == 'm':
                node.send(value, 0.0)
            else:
                node.send(0.0, value)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
