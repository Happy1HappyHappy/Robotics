"""
Viewer node for the day6pkg package (Projects 6 and 7)
Draws what the other nodes publish, and nothing else does any drawing: the grid (/grid) with
the sensor's path (/grid_path), the wall segments (/wall_segments), the corner landmarks
(/landmarks), the obstacles grown by the robot's size (/inflated_grid) and the planned path
(/planned_path) on top, shown in a window
Saves the image to <result_dir>/grid/ as <run>_grid_<distance>m.png at each snapshot
distance of travel (with <run>_grid_summary.png, all of them side by side), the latest one
as <run>_grid_latest.png every save_period seconds while it changes, the last one as
<run>_grid_final.png on shutdown, and the one with each new planned path as <run>_plan.png
(save_images)
Without this node the others run the same; without the Landmark Node or the Planner Node it
draws what it gets
"""

import math
import os

import cv2
from geometry_msgs.msg import PoseArray                 # the landmarks (/landmarks)
from nav_msgs.msg import OccupancyGrid as GridMsg       # the grid (/grid, /inflated_grid)
from nav_msgs.msg import Path                           # the paths (/grid_path, /planned_path)
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# the publishers keep their latest message (transient local), so this node gets each topic
# at once even if it starts later
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker               # the wall segments (/wall_segments)

from day6pkg.helpers.files import run_name
from day6pkg.helpers.occupancy_grid import OccupancyGrid
from day6pkg.helpers.render import map_image, save_png, summary_image

WINDOW = 'Occupancy grid'


class ViewerNode(Node):
    """
    Viewer node class for the day6pkg package
    Callbacks only store the latest messages; a timer redraws when something changed
    """
    def __init__(self):
        super().__init__('viewer_node')

        self.declare_parameter('snapshot_distances', [0.0, 0.5, 1.0])  # m, save an image at each
        self.declare_parameter('distance_tolerance', 0.03)  # m, this short of a distance counts
        self.declare_parameter('image_scale', 3)            # image pixels per cell side
        self.declare_parameter('display', True)             # show the image in a window
        self.declare_parameter('save_images', True)         # save snapshots and the final image
        self.declare_parameter('save_period', 10.0)         # s between saves; 0: on shutdown only
        self.declare_parameter('result_dir', 'result')      # the images go to its grid/ folder
        self.declare_parameter('run_id', '')                # names the files; '' = start time

        self.save_ = self.get_parameter('save_images').value
        self.image_dir_ = os.path.join(self.get_parameter('result_dir').value, 'grid')
        self.run_ = run_name(self.get_parameter('run_id').value)
        if self.save_:
            os.makedirs(self.image_dir_, exist_ok=True)

        self.grid_msg_ = None           # latest /grid
        self.path_msg_ = None           # latest /grid_path
        self.grid_ = None               # latest grid (day6pkg OccupancyGrid)
        self.poses_ = []                # (x, y, heading) of each update, from the path
        self.segments_ = []             # [(start, end), ...], m
        self.corners_ = np.zeros((0, 2))    # K x 2, m
        self.plan_ = np.zeros((0, 2))       # planned path, N x 2, m
        self.blocked_ = None            # mask of the grown obstacles, laid out like the grid
        self.new_plan_ = False          # a planned path not saved as <run>_plan.png yet
        self.last_update_ = None        # stamp of the last grid update handled
        self.changed_ = False           # something new to draw
        self.snapshots_ = {}            # snapshot distance (m) -> image
        self.image_ = None
        self.unsaved_ = False           # the image changed since _grid_latest.png was saved
        self.window_open_ = False

        latest = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GridMsg, 'grid', self.grid_callback, latest)
        self.create_subscription(Path, 'grid_path', self.path_callback, latest)
        self.create_subscription(PoseArray, 'landmarks', self.landmark_callback, latest)
        self.create_subscription(Marker, 'wall_segments', self.segment_callback, latest)
        self.create_subscription(Path, 'planned_path', self.plan_callback, latest)
        self.create_subscription(GridMsg, 'inflated_grid', self.inflated_callback, latest)
        self.draw_timer_ = self.create_timer(0.1, self.draw_callback)
        period = self.get_parameter('save_period').value
        if self.save_ and period > 0.0:
            self.save_timer_ = self.create_timer(period, self.save_latest)

        self.display_ = self.get_parameter('display').value
        if self.display_ and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            self.get_logger().warn('No display found; images are only saved to files')
            self.display_ = False
        self.get_logger().info(
            'Viewer Node ready, waiting for /grid' +
            (f'; saving to {os.path.join(self.image_dir_, self.run_)}_grid_*.png'
             if self.save_ else ''))

    # ---------- subscriptions: store only ----------
    def grid_callback(self, msg: GridMsg):
        """
        Store the latest grid
        """
        self.grid_msg_ = msg
        self.changed_ = True

    def path_callback(self, msg: Path):
        """
        Store the latest path
        """
        self.path_msg_ = msg
        self.changed_ = True

    def landmark_callback(self, msg: PoseArray):
        """
        Store the latest corner landmarks
        """
        self.corners_ = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
        self.changed_ = True

    def segment_callback(self, msg: Marker):
        """
        Store the latest wall segments (a line list: two points per segment)
        """
        ends = [np.array([p.x, p.y]) for p in msg.points]
        self.segments_ = list(zip(ends[0::2], ends[1::2]))
        self.changed_ = True

    def plan_callback(self, msg: Path):
        """
        Store the latest planned path (no poses: no path)
        """
        self.plan_ = np.array([[p.pose.position.x, p.pose.position.y]
                               for p in msg.poses]).reshape(-1, 2)
        self.new_plan_ = len(self.plan_) > 0
        self.changed_ = True

    def inflated_callback(self, msg: GridMsg):
        """
        Store the latest grown obstacles (occupancy 100) as a mask laid out like the grid
        """
        self.blocked_ = OccupancyGrid.from_ros_layout(msg.data, msg.info.resolution).log_odds > 0.0
        self.changed_ = True

    # ---------- drawing ----------
    def draw_callback(self):
        """
        Redraw if something changed; save the snapshots of each new grid update; show
        """
        if self.changed_ and self.grid_msg_ is not None:
            self.changed_ = False
            update = self.new_update()
            if update:
                self.grid_ = OccupancyGrid.from_ros_layout(self.grid_msg_.data,
                                                           self.grid_msg_.info.resolution)
                self.poses_ = [(p.pose.position.x, p.pose.position.y,
                                2.0 * math.atan2(p.pose.orientation.z, p.pose.orientation.w))
                               for p in self.path_msg_.poses]
            if self.grid_ is not None:
                self.image_ = map_image(self.grid_, self.get_parameter('image_scale').value,
                                        self.poses_, self.segments_, self.corners_,
                                        self.label(), self.blocked_, self.plan_)
                self.unsaved_ = True
                if update and self.save_:
                    self.save_snapshots()
                if self.new_plan_ and self.save_:
                    self.new_plan_ = False
                    self.save_image('plan.png', self.image_)
        if self.display_ and self.image_ is not None:
            try:
                cv2.imshow(WINDOW, self.image_)
                cv2.waitKey(1)
                self.window_open_ = True
            except cv2.error as e:
                self.get_logger().warn(f'Cannot show the image ({e}); it is still saved')
                self.display_ = False

    def new_update(self):
        """
        Check that the latest grid and path belong to the same grid update (the Grid Node
        stamps both alike) and that it has not been handled yet
        """
        if self.path_msg_ is None:
            return False
        stamp = self.grid_msg_.header.stamp
        if stamp != self.path_msg_.header.stamp or stamp == self.last_update_:
            return False
        self.last_update_ = stamp
        return True

    def traveled(self):
        """
        Straight-line distance (m) of the latest pose from the first one
        """
        if not self.poses_:
            return 0.0
        return math.hypot(self.poses_[-1][0] - self.poses_[0][0],
                          self.poses_[-1][1] - self.poses_[0][1])

    def label(self):
        """
        Text in the image's corner
        """
        text = (f'{self.traveled():.2f} m traveled, {len(self.poses_)} scans, '
                f'{len(self.corners_)} landmarks')
        if len(self.plan_):
            length = np.sum(np.linalg.norm(np.diff(self.plan_, axis=0), axis=1))
            text += f', plan {length:.2f} m'
        return text

    # ---------- saving ----------
    def save_snapshots(self):
        """
        Save the image for each snapshot distance just reached, then the summary image
        with all snapshots so far side by side
        """
        traveled = self.traveled()
        tolerance = self.get_parameter('distance_tolerance').value
        distances = self.get_parameter('snapshot_distances').value
        reached = [d for d in distances if d not in self.snapshots_ and traveled >= d - tolerance]
        if not reached:
            return
        for d in reached:
            self.snapshots_[d] = self.image_
            self.save_image(f'grid_{d:.2f}m.png', self.image_)
        self.save_image('grid_summary.png',
                        summary_image([self.snapshots_[d] for d in sorted(self.snapshots_)]))
        if len(self.snapshots_) == len(distances):
            self.get_logger().info('All snapshots saved')

    def save_image(self, name, img):
        """
        Write an image as <result_dir>/grid/<run>_<name> (replacing an older one whole),
        logging the path or the failure; returns whether it was written
        """
        path = os.path.join(self.image_dir_, f'{self.run_}_{name}')
        if save_png(path, img):
            self.get_logger().info(f'Saved {path}')
            return True
        self.get_logger().warn(f'Could not save {path}')
        return False

    def save_latest(self):
        """
        Save the latest image as <run>_grid_latest.png (if save_images and it changed since
        the last save); runs every save_period seconds
        """
        if self.save_ and self.unsaved_ and self.save_image('grid_latest.png', self.image_):
            self.unsaved_ = False

    def save_final(self):
        """
        Save the last image as <run>_grid_final.png (if save_images), on shutdown
        """
        if self.save_ and self.image_ is not None:
            self.save_image('grid_final.png', self.image_)


def main(args=None):
    """
    Entry point for the viewer node
    Initializes the ROS 2 system, creates the node, and spins it
    Saves the last image and closes the window on shutdown
    """
    rclpy.init(args=args)
    node = ViewerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_final()
        if node.window_open_:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
