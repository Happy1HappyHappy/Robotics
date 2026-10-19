"""
Grid node for the day6pkg package (Project 6)
Builds an occupancy grid from the LiDAR (/scan), placing each scan with odometry (/odom)
and the LiDAR's mounting on the robot (TF)
Task 1: the first scan is added with the sensor at the origin
Task 2: as the robot drives, a scan is added after every update_distance of travel, and the
grid is saved as an image at each snapshot distance (by default 0, 0.5 and 1.0 m)
This node never moves the robot; drive it with day3pkg or the robot's drive_distance action
"""

import math
import os

import cv2
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from day6pkg.grid import OccupancyGrid
from day6pkg.render import grid_image, summary_image

WINDOW = 'Occupancy grid'


def yaw_from_quaternion(q):
    """
    Extract yaw (rotation about z) from a geometry_msgs/Quaternion
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform(x, y, theta):
    """
    Homogeneous 3x3 matrix for a 2D pose (meters, radians)
    """
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def pose_of(T):
    """
    Return the 2D pose (x, y, yaw) held in homogeneous matrix T
    """
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


class GridNode(Node):
    """
    Grid node class for the day6pkg package
    Frames: odom (odometry), base (the robot, /odom's child frame), sensor (the LiDAR,
    /scan's frame), and world (the grid): origin at the sensor's start position, x forward
    and y left along the robot's start heading
    """
    def __init__(self):
        super().__init__('grid_node')

        self.declare_parameter('size', 200)                 # cells per side
        self.declare_parameter('resolution', 0.05)          # m, cell side
        self.declare_parameter('update_distance', 0.1)      # m traveled between grid updates
        self.declare_parameter('snapshot_distances', [0.0, 0.5, 1.0])  # m, save an image at each
        self.declare_parameter('distance_tolerance', 0.03)  # m, stopping this short counts
        self.declare_parameter('stopped_speed', 0.005)      # m/s, slower counts as stopped
        self.declare_parameter('max_odom_age', 0.5)         # s, skip scans if odom is older
        self.declare_parameter('use_tf', True)              # LiDAR mounting from TF
        self.declare_parameter('mount_yaw', 90.0)           # deg, LiDAR yaw if use_tf is false
        self.declare_parameter('output_dir', '.')           # where the images are saved
        self.declare_parameter('display', True)             # show the grid in a window
        self.declare_parameter('image_scale', 3)            # image pixels per cell side

        self.grid_ = OccupancyGrid(self.get_parameter('size').value,
                                   self.get_parameter('resolution').value)
        self.scale_ = self.get_parameter('image_scale').value

        # the LiDAR's mounting comes from the robot's static transforms (/tf_static)
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.odom_ = None               # latest odometry pose (x, y, yaw), odom frame
        self.odom_time_ = None          # when it arrived (node clock)
        self.odom_speed_ = 0.0          # m/s, latest forward speed from odometry
        self.base_frame_ = 'base_link'  # replaced by /odom's child_frame_id
        self.mount_ = None              # ^base T_sensor, looked up once
        self.world_from_odom_ = None    # ^world T_odom, fixed by the first update
        self.start_xy_ = None           # robot position (x, y) at the first update, odom frame
        self.next_update_ = 0.0         # m of travel at which the next scan is added
        self.poses_ = []                # (sensor x, y, robot heading) per update, world frame
        self.snapshots_ = {}            # snapshot distance (m) -> image
        self.image_ = None              # latest grid image
        self.window_open_ = False

        self.odom_sub_ = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.display_timer_ = None
        if self.get_parameter('display').value:
            if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
                self.display_timer_ = self.create_timer(0.1, self.display_callback)
            else:
                self.get_logger().warn('No display found; images are only saved to files')
        self.get_logger().info('Grid Node ready, waiting for /scan and /odom')

    # ---------- odometry ----------
    def odom_callback(self, msg: Odometry):
        """
        Store the latest odometry pose and forward speed
        """
        p = msg.pose.pose.position
        self.odom_ = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))
        self.odom_time_ = self.get_clock().now()
        self.odom_speed_ = msg.twist.twist.linear.x
        if msg.child_frame_id:
            self.base_frame_ = msg.child_frame_id

    # ---------- LiDAR mounting ----------
    def mount_transform(self, sensor_frame):
        """
        Return ^base T_sensor, where the LiDAR sits on the robot, or None until it is known
        From TF when use_tf is true; else a pure rotation by mount_yaw (the LiDAR's few cm
        offset from the robot's center is ignored; it is under one cell)
        """
        if self.mount_ is not None:
            return self.mount_
        if self.get_parameter('use_tf').value:
            try:
                t = self.tf_buffer_.lookup_transform(self.base_frame_, sensor_frame, Time())
            except TransformException as e:
                self.get_logger().warn(
                    f'Waiting for transform {self.base_frame_} -> {sensor_frame}: {e}',
                    throttle_duration_sec=5.0)
                return None
            tr = t.transform.translation
            self.mount_ = transform(tr.x, tr.y, yaw_from_quaternion(t.transform.rotation))
        else:
            self.mount_ = transform(0.0, 0.0, math.radians(self.get_parameter('mount_yaw').value))
        x, y, yaw = pose_of(self.mount_)
        self.get_logger().info(
            f'LiDAR on the robot: x={x:.3f} m, y={y:.3f} m, yaw={math.degrees(yaw):.1f} deg')
        return self.mount_

    # ---------- scans ----------
    def scan_callback(self, msg: LaserScan):
        """
        Add the scan to the grid if it is the first one, or once the robot has traveled
        to the next multiple of update_distance; skip it otherwise
        Travel is the straight-line distance from the start position, which is the distance
        driven for a straight drive, and does not add up odometry jitter while standing still
        If the robot stops up to distance_tolerance short of that distance (odometry rarely
        stops exactly on it), the distance counts as reached
        """
        max_age = self.get_parameter('max_odom_age').value
        if (self.odom_ is None or
                (self.get_clock().now() - self.odom_time_).nanoseconds / 1e9 > max_age):
            self.get_logger().warn('Waiting for odometry (/odom)', throttle_duration_sec=5.0)
            return
        tolerance = self.get_parameter('distance_tolerance').value
        traveled = 0.0
        if self.start_xy_ is not None:
            traveled = math.hypot(self.odom_[0] - self.start_xy_[0],
                                  self.odom_[1] - self.start_xy_[1])
        stopped = abs(self.odom_speed_) < self.get_parameter('stopped_speed').value
        if self.poses_ and not (traveled >= self.next_update_ or
                                (stopped and traveled >= self.next_update_ - tolerance)):
            return
        mount = self.mount_transform(msg.header.frame_id)
        if mount is None:
            return

        # ^odom T_sensor = ^odom T_base (odometry) . ^base T_sensor (mounting)
        odom_T_sensor = transform(*self.odom_) @ mount
        if not self.poses_:
            # world frame: origin at the sensor's start position, axes along the robot's
            # start heading, so the robot drives straight up the image
            self.world_from_odom_ = np.linalg.inv(
                transform(odom_T_sensor[0, 2], odom_T_sensor[1, 2], self.odom_[2]))
            self.start_xy_ = self.odom_[:2]
        pose = pose_of(self.world_from_odom_ @ odom_T_sensor)
        used = self.grid_.update(pose, msg.ranges, msg.angle_min, msg.angle_increment,
                                 msg.range_min, msg.range_max)
        heading = pose_of(self.world_from_odom_ @ transform(*self.odom_))[2]
        self.poses_.append((pose[0], pose[1], heading))

        # next update at the next multiple of update_distance past this one
        step = self.get_parameter('update_distance').value
        self.next_update_ = (math.floor((traveled + tolerance) / step) + 1) * step
        self.get_logger().info(
            f'Update {len(self.poses_)}: traveled {traveled:.3f} m, sensor at '
            f'x={pose[0]:.3f} m, y={pose[1]:.3f} m, robot heading '
            f'{math.degrees(heading):.1f} deg, {used} of {len(msg.ranges)} rays used')
        self.image_ = grid_image(self.grid_, self.scale_, self.poses_,
                                 f'{traveled:.2f} m traveled, {len(self.poses_)} scans')
        self.save_snapshots(traveled)

    # ---------- images ----------
    def save_snapshots(self, traveled):
        """
        Save the grid image for each snapshot distance just reached, then the summary
        image with all snapshots so far side by side
        """
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
        Write an image to output_dir, logging the path or the failure
        """
        path = os.path.join(self.get_parameter('output_dir').value, name)
        if cv2.imwrite(path, img):
            self.get_logger().info(f'Saved {path}')
        else:
            self.get_logger().warn(f'Could not save {path}')

    def save_final(self):
        """
        Save the grid as it is now, so the last state is kept even if a snapshot distance
        was never reached
        """
        if self.image_ is not None:
            self.save_image('grid_final.png', self.image_)

    def display_callback(self):
        """
        Show the latest grid image; cv2.waitKey lets the window draw itself
        """
        if self.image_ is None:
            return
        try:
            cv2.imshow(WINDOW, self.image_)
            cv2.waitKey(1)
            self.window_open_ = True
        except cv2.error as e:
            self.get_logger().warn(f'Cannot show the grid ({e}); images are still saved')
            self.display_timer_.cancel()


def main(args=None):
    """
    Entry point for the grid node
    Initializes the ROS 2 system, creates the node, and spins it
    Saves the final grid and closes the window on shutdown
    """
    rclpy.init(args=args)
    node = GridNode()
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
