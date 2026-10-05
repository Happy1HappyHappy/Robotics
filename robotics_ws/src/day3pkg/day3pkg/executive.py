import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from day3_interfaces.action import Drive

HELP = 'Commands:  m <meters>   t <degrees>   q (quit)'


class ExecutiveNode(Node):
    def __init__(self):
        super().__init__('executive_node')
        self.client_ = ActionClient(self, Drive, 'drive')

    def send(self, distance: float, angle: float) -> bool:
        if not self.client_.wait_for_server(timeout_sec=5.0):
            print('Driving Node is not available.')
            return False

        goal = Drive.Goal()
        goal.distance = float(distance)   # float64 fields reject Python ints
        goal.angle = float(angle)

        # 1) send goal, wait for accept/reject
        send_future = self.client_.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print('Request REJECTED by Driving Node.')
            return False
        print('Request accepted, moving...')

        # 2) wait for the result
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
        for kind, value in SHAPES[name]:
            ok = node.send(value, 0.0) if kind == 'm' else node.send(0.0, value)
            if not ok:
                print('Shape aborted.')
                return


def main(args=None):
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
            if cmd not in ('m', 't') or len(parts) != 2:
                print(HELP)
                continue
            if cmd in SHAPES: run_shape(node, cmd); continue
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
