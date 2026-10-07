"""
Executive node for controlling the robot via high-level commands
Provides a command-line interface to send driving goals to the Driving Node
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from day3_interfaces.action import Drive

HELP = ('Commands:  m <meters>   t <degrees, - = right>   a <l|r> <radius m> <degrees> (arc)\n'
        '           p / rp (letter P)   8 / r8 (figure-8)   q (quit)')


class ExecutiveNode(Node):
    """
    Node responsible for sending high-level driving commands to Driving Node
    """
    def __init__(self):
        super().__init__('executive_node')
        self.client_ = ActionClient(self, Drive, 'drive')

    def send(self, distance: float, angle: float,
             radius: float = 0.0, turn_right: bool = False) -> bool:
        """
        Send a driving goal to the Driving Node
        distance: linear distance in meters
        angle: rotation angle in degrees (for an arc: degrees along the arc)
        radius: arc radius in meters (0 = not an arc)
        turn_right: arc direction, False = left, True = right
        Returns True if the goal was accepted and succeeded, False otherwise
        """
        if not self.client_.wait_for_server(timeout_sec=5.0):
            print('Driving Node is not available.')
            return False

        goal = Drive.Goal()
        goal.distance = float(distance)   # float64 fields reject Python ints
        goal.angle = float(angle)
        goal.radius = float(radius)
        goal.turn_right = bool(turn_right)

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
# ('m', meters), ('t', degrees), ('a', 'l' or 'r', radius, degrees)
SHAPES = {
    # squared "P": stem up, then the bowl on the right (-90 = right turn)
    'p': [('m', 1.0), ('t', -90), ('m', 0.4), ('t', -90),
          ('m', 0.5), ('t', -90), ('m', 0.4)],
    # squared figure-8: CCW square, straight through the center, CW square
    '8': [('m', 0.5), ('t', 90), ('m', 0.5), ('t', 90), ('m', 0.5), ('t', 90),
          ('m', 1.0), ('t', -90), ('m', 0.5), ('t', -90), ('m', 0.5), ('t', -90),
          ('m', 0.5)],
    # rounded "P": stem up, then a half-circle bowl on the right
    'rp': [('m', 1.0), ('a', 'r', 0.25, 180)],
    # rounded figure-8: full circle left, then full circle right
    'r8': [('a', 'l', 0.25, 360), ('a', 'r', 0.25, 360)],
}


def run_shape(node, name):
    """
    Execute a predefined shape by sending a sequence of driving commands
    to the Executive Node
    node: instance of ExecutiveNode
    name: name of the shape to run (must be a key in SHAPES)
    """
    for kind, *args in SHAPES[name]:
        if kind == 'm':
            ok = node.send(args[0], 0.0)
        elif kind == 't':
            ok = node.send(0.0, args[0])
        else:
            side, radius, degrees = args
            ok = node.send(0.0, degrees, radius, side == 'r')
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
            if cmd == 'a':
                if len(parts) != 4 or parts[1].lower() not in ('l', 'r'):
                    print(HELP)
                    continue
                try:
                    radius, degrees = float(parts[2]), float(parts[3])
                except ValueError:
                    print('Radius and degrees must be numbers.')
                    continue
                if radius <= 0.0 or not 0.0 <= degrees <= 360.0:
                    print('Radius must be > 0 and degrees must be 0 to 360.')
                    continue
                node.send(0.0, degrees, radius, parts[1].lower() == 'r')
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
