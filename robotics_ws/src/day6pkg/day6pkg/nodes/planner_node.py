"""
Planner node for the day6pkg package (Project 7, Task 3)
Plans a path by A* from the robot to a goal on the grid the Grid Node publishes (/grid),
with the obstacles grown by the robot's radius plus a safety margin, so that the path keeps
the robot's body clear of them (see helpers/astar.py)
The robot is at the last pose of /grid_path (the sensor at the latest grid update); the
goal is the parameters goal_x and goal_y in the grid's frame (world: x along the robot's
start heading, y to its left), and can be changed while it runs (ros2 param set)
Publishes the path on /planned_path (nav_msgs/Path) and the grown obstacles on
/inflated_grid (nav_msgs/OccupancyGrid), both in the grid's frame, for the nodes that use
them (the Viewer Node, later driving the path); it draws nothing itself
It saves the latest plan as <log_dir>/<run>_plan_latest.npz every save_period seconds while
it changes, and as <run>_plan_final.npz on shutdown (save)
This node never moves the robot
"""

import array
import math
import os
import time

from geometry_msgs.msg import PoseStamped               # one pose of the path
from nav_msgs.msg import OccupancyGrid as GridMsg       # the grid (/grid, /inflated_grid)
from nav_msgs.msg import Path                           # /grid_path, /planned_path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# /grid and /grid_path keep their latest message, so this node gets them even if it starts
# after the Grid Node; its outputs do the same for nodes that start after it
from rclpy.qos import DurabilityPolicy, QoSProfile

from day6pkg.helpers.astar import plan
from day6pkg.helpers.files import run_name, save_npz
from day6pkg.helpers.occupancy_grid import OccupancyGrid

# the parameters a plan depends on: a change to any of them plans again
PLAN_PARAMETERS = ('goal_x', 'goal_y', 'robot_radius', 'safety_margin', 'occupied',
                   'unknown_is_free')


class PlannerNode(Node):
    """
    Planner node class for the day6pkg package
    The subscriptions only store the latest grid and path; a timer plans when the grid or
    a parameter changed, so a slow plan skips to the newest grid instead of falling behind
    """
    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('goal_x', 2.5)               # m, goal in the grid's frame
        self.declare_parameter('goal_y', 0.0)               # m
        self.declare_parameter('robot_radius', 0.171)       # m, TurtleBot 4: 342 x 339 mm
        self.declare_parameter('safety_margin', 0.05)       # m, kept clear beyond the radius
        self.declare_parameter('occupied', 0.5)             # log-odds above which a cell blocks
        self.declare_parameter('unknown_is_free', False)    # cells never seen may be driven
        self.declare_parameter('min_goal_distance', 2.0)    # m, warn if the goal is nearer
        self.declare_parameter('period', 1.0)               # s, how often to check for changes
        self.declare_parameter('save', True)                # save <run>_plan_*.npz
        self.declare_parameter('save_period', 10.0)         # s between saves; 0: on shutdown only
        self.declare_parameter('log_dir', 'logs')           # where the .npz files are saved
        self.declare_parameter('run_id', '')                # names the files; '' = start time

        self.save_ = self.get_parameter('save').value
        log_dir = self.get_parameter('log_dir').value
        # <run>_plan_latest.npz while it runs, <run>_plan_final.npz on shutdown
        self.npz_path_ = os.path.join(
            log_dir, f'{run_name(self.get_parameter("run_id").value)}_plan_{{}}.npz')
        if self.save_:
            os.makedirs(log_dir, exist_ok=True)

        self.grid_msg_ = None           # latest /grid message
        self.path_msg_ = None           # latest /grid_path message
        self.new_grid_ = False          # the latest grid has not been planned on yet
        self.settings_ = None           # PLAN_PARAMETERS of the last plan
        self.planned_ = False           # a plan has been made at least once
        self.unsaved_ = False           # planned since the last _latest save
        self.frame_ = ''                # frame_id of the last grid planned on
        self.resolution_ = 0.0          # m, its cell side
        self.start_ = (0.0, 0.0)        # m, the robot's position of the last plan
        self.plan_ = None               # the last plan (helpers.astar.Plan)

        latest = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_sub_ = self.create_subscription(GridMsg, 'grid', self.grid_callback, latest)
        self.path_sub_ = self.create_subscription(Path, 'grid_path', self.path_callback, latest)
        self.plan_pub_ = self.create_publisher(Path, 'planned_path', latest)
        self.inflated_pub_ = self.create_publisher(GridMsg, 'inflated_grid', latest)
        self.plan_timer_ = self.create_timer(self.get_parameter('period').value,
                                             self.plan_callback)
        period = self.get_parameter('save_period').value
        if self.save_ and period > 0.0:
            self.save_timer_ = self.create_timer(period, self.save_plan)
        self.get_logger().info('Planner Node ready, waiting for /grid and /grid_path' +
                               (f'; saving to {self.npz_path_.format("latest")}'
                                if self.save_ else ''))

    def param(self, name):
        """
        Current value of parameter name
        """
        return self.get_parameter(name).value

    def grid_callback(self, msg: GridMsg):
        """
        Store the latest grid; the plan timer picks it up
        """
        self.grid_msg_ = msg
        self.new_grid_ = True

    def path_callback(self, msg: Path):
        """
        Store the latest path of the robot; its last pose is where the robot is
        """
        self.path_msg_ = msg

    def plan_callback(self):
        """
        Plan from the robot to the goal if the grid or a plan parameter changed, and publish
        """
        if self.grid_msg_ is None or self.path_msg_ is None or not self.path_msg_.poses:
            return
        settings = tuple(self.param(name) for name in PLAN_PARAMETERS)
        if not self.new_grid_ and settings == self.settings_:
            return
        self.new_grid_ = False
        self.settings_ = settings
        msg = self.grid_msg_
        if msg.info.width != msg.info.height:
            self.get_logger().warn('Only square grids are supported', throttle_duration_sec=5.0)
            return
        p = self.param
        robot = self.path_msg_.poses[-1].pose.position
        start, goal = (robot.x, robot.y), (p('goal_x'), p('goal_y'))
        distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
        if distance < p('min_goal_distance'):
            self.get_logger().warn(f'The goal is {distance:.2f} m from the robot, nearer than '
                                   f'min_goal_distance ({p("min_goal_distance"):.2f} m)')

        began = time.monotonic()
        grid = OccupancyGrid.from_ros_layout(msg.data, msg.info.resolution)
        result = plan(grid, start, goal, p('robot_radius'), p('safety_margin'), p('occupied'),
                      p('unknown_is_free'))
        self.planned_ = self.unsaved_ = True
        self.frame_, self.resolution_ = msg.header.frame_id, msg.info.resolution
        self.start_, self.plan_ = start, result
        self.publish(msg, grid)
        ms = (time.monotonic() - began) * 1000.0
        where = (f'from ({start[0]:.2f}, {start[1]:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f}) m '
                 f'({distance:.2f} m apart)')
        if result.problem:
            self.get_logger().warn(f'No path {where}: {result.problem} ({ms:.0f} ms)')
            return
        self.get_logger().info(
            f'Path {where}: {result.length:.2f} m, {len(result.cells)} cells, ' +
            ('straight (nothing in the way)' if result.straight
             else 'bends around obstacles (the straight line is blocked)') + f' ({ms:.0f} ms)')

    def publish(self, msg, grid):
        """
        Publish the last plan's path on /planned_path (no poses if there is no path) and
        its grown obstacles on /inflated_grid, stamped like the grid message msg it was
        planned on (grid: the same grid, as an OccupancyGrid)
        """
        path = Path()
        path.header = msg.header
        points = self.plan_.points
        for k, (x, y) in enumerate(points):
            # heading along the path: toward the next point; the last one keeps the one before
            ahead = min(k + 1, len(points) - 1)
            dx, dy = points[ahead] - points[max(ahead - 1, 0)]
            yaw = math.atan2(dy, dx)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)     # rotation about z only
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        self.plan_pub_.publish(path)

        # 100: the robot's center must not go there (grown obstacles), -1: never seen,
        # 0: free; through the grid's own layout, at its log-odds limits
        inflated = OccupancyGrid(grid.size, grid.resolution)
        inflated.log_odds = np.where(self.plan_.blocked, inflated.clamp,
                                     np.where(self.plan_.unknown, 0.0, -inflated.clamp))
        values, _ = inflated.ros_layout()
        out = GridMsg()
        out.header, out.info = msg.header, msg.info
        out.data = array.array('b', values.tobytes())
        self.inflated_pub_.publish(out)

    def save_plan(self, final=False):
        """
        Save the last plan (if save): as <log_dir>/<run>_plan_latest.npz every save_period
        seconds if it changed, or as <run>_plan_final.npz on shutdown (final)
        Each file is replaced whole, so a crash leaves the last complete save
        """
        if not self.save_ or not self.planned_:         # not wanted, or nothing planned yet
            return
        if not final and not self.unsaved_:             # nothing new since the last _latest
            return
        path = self.npz_path_.format('final' if final else 'latest')
        names = dict(zip(PLAN_PARAMETERS, self.settings_))
        try:
            save_npz(path, points=self.plan_.points, length=self.plan_.length,
                     straight=self.plan_.straight, problem=self.plan_.problem,
                     start=np.array(self.start_),
                     goal=np.array([names['goal_x'], names['goal_y']]),
                     robot_radius=names['robot_radius'], safety_margin=names['safety_margin'],
                     resolution=self.resolution_, frame=self.frame_)
            if not final:
                self.unsaved_ = False
            self.get_logger().info(f'Saved {path} ({len(self.plan_.points)} path points)')
        except OSError as e:
            self.get_logger().warn(f'Could not save {path}: {e}')


def main(args=None):
    """
    Entry point for the planner node
    Initializes the ROS 2 system, creates the node, and spins it
    Saves the final plan on shutdown
    """
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_plan(final=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
