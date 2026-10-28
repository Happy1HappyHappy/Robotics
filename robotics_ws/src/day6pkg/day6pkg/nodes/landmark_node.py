"""
Landmark node for the day6pkg package (Project 7, Task 2)
Finds corner landmarks in the grid the Grid Node publishes (/grid), every time it changes:
wall segments by RANSAC, then corners where two segments meet (see helpers/landmarks.py)
Publishes the corners on /landmarks (geometry_msgs/PoseArray) and the wall segments on
/wall_segments (visualization_msgs/Marker, a line list), both in the grid's frame, for the
nodes that use them (the Viewer Node, later localization); it draws nothing itself
It saves the latest ones as <log_dir>/<run>_landmarks.npz every save_period seconds while
they change, and on shutdown (save)
This node never moves the robot
"""

import os
import time

from geometry_msgs.msg import Point, Pose, PoseArray    # for the landmarks (/landmarks)
from nav_msgs.msg import OccupancyGrid as GridMsg       # for the grid (/grid)
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# /grid keeps its latest message, so this node gets it even if it starts after the Grid Node;
# its outputs do the same for nodes that start after it
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker               # for the wall segments (/wall_segments)

from day6pkg.helpers.files import run_name, save_npz
from day6pkg.helpers.landmarks import find_corners, find_segments, occupied_points
from day6pkg.helpers.occupancy_grid import OccupancyGrid


class LandmarkNode(Node):
    """
    Landmark node class for the day6pkg package
    The subscription only stores the latest grid; a timer searches it for landmarks, so a
    slow search skips to the newest grid instead of falling behind
    """
    def __init__(self):
        super().__init__('landmark_node')

        self.declare_parameter('occupied', 1.0)             # log-odds above which a cell counts
        self.declare_parameter('inlier_distance', 0.06)     # m, how close to a line is on it
        self.declare_parameter('sample_radius', 0.5)        # m, the two random points are closer
        self.declare_parameter('min_points', 8)             # points needed for a segment
        self.declare_parameter('min_length', 0.3)           # m, shortest segment kept
        self.declare_parameter('max_gap', 0.15)             # m, a wider gap splits a segment
        self.declare_parameter('trials', 2000)              # RANSAC tries per line
        self.declare_parameter('corner_angle', 60.0)        # deg, smallest angle between segments
        self.declare_parameter('corner_distance', 0.15)     # m, segment ends this near the corner
        self.declare_parameter('period', 1.0)               # s, how often to look for a new grid
        self.declare_parameter('save', True)                # save <run>_landmarks.npz
        self.declare_parameter('save_period', 10.0)         # s between saves; 0: on shutdown only
        self.declare_parameter('log_dir', 'logs')           # where the .npz files are saved
        self.declare_parameter('run_id', '')                # names the files; '' = start time

        self.save_ = self.get_parameter('save').value
        log_dir = self.get_parameter('log_dir').value
        self.npz_path_ = os.path.join(
            log_dir, f'{run_name(self.get_parameter("run_id").value)}_landmarks.npz')
        if self.save_:
            os.makedirs(log_dir, exist_ok=True)

        self.grid_msg_ = None           # latest /grid message
        self.new_grid_ = False          # it has not been searched yet
        self.unsaved_ = False           # landmarks found since the last save
        self.frame_ = ''                # frame_id of the last grid searched
        self.resolution_ = 0.0          # m, its cell side
        self.segments_ = []             # its wall segments [(start, end), ...], m
        self.corners_ = np.zeros((0, 2))    # its corner landmarks, K x 2, m

        latest = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_sub_ = self.create_subscription(GridMsg, 'grid', self.grid_callback, latest)
        self.landmark_pub_ = self.create_publisher(PoseArray, 'landmarks', latest)
        self.segment_pub_ = self.create_publisher(Marker, 'wall_segments', latest)
        self.search_timer_ = self.create_timer(self.get_parameter('period').value,
                                               self.search_callback)
        period = self.get_parameter('save_period').value
        if self.save_ and period > 0.0:
            self.save_timer_ = self.create_timer(period, self.save_landmarks)
        self.get_logger().info('Landmark Node ready, waiting for /grid' +
                               (f'; saving to {self.npz_path_}' if self.save_ else ''))

    def param(self, name):
        """
        Current value of parameter name
        """
        return self.get_parameter(name).value

    def grid_callback(self, msg: GridMsg):
        """
        Store the latest grid; the search timer picks it up
        """
        self.grid_msg_ = msg
        self.new_grid_ = True

    def search_callback(self):
        """
        Find the landmarks in the latest grid if it is new, and publish them
        """
        if not self.new_grid_:
            return
        self.new_grid_ = False
        msg = self.grid_msg_
        if msg.info.width != msg.info.height:
            self.get_logger().warn('Only square grids are supported', throttle_duration_sec=5.0)
            return
        p = self.param
        start = time.monotonic()
        grid = OccupancyGrid.from_ros_layout(msg.data, msg.info.resolution)
        points = occupied_points(grid, p('occupied'))
        segments = find_segments(points, p('inlier_distance'), p('sample_radius'),
                                 p('min_points'), p('min_length'), p('max_gap'), p('trials'))
        corners = find_corners(segments, p('corner_angle'), p('corner_distance'))
        self.unsaved_ = True
        self.frame_, self.resolution_ = msg.header.frame_id, msg.info.resolution
        self.segments_, self.corners_ = segments, corners
        self.publish(msg.header.stamp)
        self.get_logger().info(
            f'{len(points)} occupied cells -> {len(segments)} segments -> {len(corners)} '
            f'landmarks ({(time.monotonic() - start) * 1000.0:.0f} ms)')

    def publish(self, stamp):
        """
        Publish the corners on /landmarks and the segments on /wall_segments, stamped like
        the grid they were found in
        """
        landmarks = PoseArray()
        landmarks.header.stamp = stamp
        landmarks.header.frame_id = self.frame_
        for x, y in self.corners_:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose.orientation.w = 1.0
            landmarks.poses.append(pose)
        self.landmark_pub_.publish(landmarks)

        lines = Marker()
        lines.header = landmarks.header
        lines.ns, lines.id = 'wall_segments', 0
        lines.type, lines.action = Marker.LINE_LIST, Marker.ADD
        lines.pose.orientation.w = 1.0
        lines.scale.x = 0.03                # m, line width when shown in RViz
        lines.color.r, lines.color.g, lines.color.a = 1.0, 0.55, 1.0    # orange
        for start, end in self.segments_:   # a line list: two points per segment
            lines.points += [Point(x=float(start[0]), y=float(start[1])),
                             Point(x=float(end[0]), y=float(end[1]))]
        self.segment_pub_.publish(lines)

    def save_landmarks(self):
        """
        Save the latest landmarks and segments as <log_dir>/<run>_landmarks.npz (if save and
        they changed since the last save); runs every save_period seconds and on shutdown,
        and replaces the file whole, so a crash leaves the last complete save
        """
        if not self.save_ or not self.unsaved_:         # not wanted, or nothing new
            return
        try:
            save_npz(self.npz_path_, corners=self.corners_,
                     segments=np.array(self.segments_).reshape(-1, 2, 2),
                     resolution=self.resolution_, frame=self.frame_)
            self.unsaved_ = False
            self.get_logger().info(f'Saved {self.npz_path_} ({len(self.corners_)} landmarks)')
        except OSError as e:
            self.get_logger().warn(f'Could not save {self.npz_path_}: {e}')


def main(args=None):
    """
    Entry point for the landmark node
    Initializes the ROS 2 system, creates the node, and spins it
    Saves the last landmarks on shutdown
    """
    rclpy.init(args=args)
    node = LandmarkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_landmarks()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
