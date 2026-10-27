"""
Grid node for the day6pkg package (Projects 6 and 7)
Builds an occupancy grid from the LiDAR (/scan), placing each scan with odometry (/odom)
and the LiDAR's mounting on the robot (TF)
Task 1: the first scan is added with the sensor at the origin
Task 2: as the robot drives, a scan is added after every update_distance of travel
After every update it publishes the grid (/grid) and the sensor's path so far (/grid_path)
for the nodes that use them (the Landmark Node, the Viewer Node, later path planning); it
draws nothing itself. On shutdown it saves the grid as grid.npz (save_grid)
This node never moves the robot; drive it with day3pkg or the robot's drive_distance action
"""

import array
import math
import os

from geometry_msgs.msg import PoseStamped       # one pose of the path
from nav_msgs.msg import OccupancyGrid as GridMsg   # for publishing the grid (/grid)
from nav_msgs.msg import Odometry           # for the robot's odometry(/odom)
from nav_msgs.msg import Path                   # for publishing the path (/grid_path)
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# /grid keeps its latest message for nodes that subscribe later (transient local)
from rclpy.qos import DurabilityPolicy, QoSProfile
# Quality of Service profile for sensor data, with a queue depth of 5 and best effort reliability
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan       # for the LiDAR scans(/scan)
from tf2_ros import TransformException      # for looking up transforms from TF (/tf_static)
from tf2_ros.buffer import Buffer
# for the LiDAR's mounting on the robot (TF)
from tf2_ros.transform_listener import TransformListener

from day6pkg.occupancy_grid import OccupancyGrid


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
        self.declare_parameter('distance_tolerance', 0.03)  # m, stopping this short counts
        self.declare_parameter('stopped_speed', 0.005)      # m/s, slower counts as stopped
        self.declare_parameter('max_odom_age', 0.5)         # s, skip scans if odom is older
        self.declare_parameter('use_tf', True)              # LiDAR mounting from TF
        self.declare_parameter('mount_yaw', 90.0)           # deg, LiDAR yaw if use_tf is false
        self.declare_parameter('grid_frame', 'world')       # frame_id of /grid and /grid_path
        self.declare_parameter('save_grid', True)           # save the grid as grid.npz on shutdown
        self.declare_parameter('output_dir', '.')           # where grid.npz is saved

        self.grid_ = OccupancyGrid(self.get_parameter('size').value,
                                   self.get_parameter('resolution').value)
        self.save_grid_ = self.get_parameter('save_grid').value
        if self.save_grid_:
            os.makedirs(self.get_parameter('output_dir').value, exist_ok=True)

        # the LiDAR's mounting comes from the robot's static transforms (/tf_static)
        self.tf_buffer_ = Buffer()  # store the transforms, so they can be looked up later
        # listen for the transforms and add them to the buffer
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

        self.odom_sub_ = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        # both keep their latest message, so a node started later still gets it at once
        latest = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub_ = self.create_publisher(GridMsg, 'grid', latest)
        self.path_pub_ = self.create_publisher(Path, 'grid_path', latest)
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
        stamp = self.get_clock().now().to_msg()
        self.publish_path(stamp)        # path first: a viewer drawing the grid has its path
        self.publish_grid(stamp)

    # ---------- publishing ----------
    def publish_grid(self, stamp):
        """
        Publish the grid on /grid as a nav_msgs/OccupancyGrid in the world frame
        (occupancy in percent, -1 unknown; see OccupancyGrid.ros_layout)
        """
        values, (x0, y0) = self.grid_.ros_layout()
        msg = GridMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter('grid_frame').value
        msg.info.resolution = float(self.grid_.resolution)
        msg.info.width = msg.info.height = self.grid_.size
        msg.info.origin.position.x, msg.info.origin.position.y = x0, y0
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', values.tobytes())
        self.grid_pub_.publish(msg)

    def publish_path(self, stamp):
        """
        Publish the pose of every update so far on /grid_path as a nav_msgs/Path in the world
        frame: the sensor's position with the robot's heading
        """
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter('grid_frame').value
        for x, y, heading in self.poses_:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(heading / 2.0)     # rotation about z only
            pose.pose.orientation.w = math.cos(heading / 2.0)
            msg.poses.append(pose)
        self.path_pub_.publish(msg)

    # ---------- saving ----------
    def save_final(self):
        """
        Save the grid as it is now as grid.npz (if save_grid), with the poses and
        ^world T_odom, so two runs can be compared later
        """
        if not self.poses_ or not self.save_grid_:      # no update yet, or not wanted
            return
        path = os.path.join(self.get_parameter('output_dir').value, 'grid.npz')
        try:
            self.grid_.save(path, poses=np.array(self.poses_),
                            world_from_odom=self.world_from_odom_)
            self.get_logger().info(f'Saved {path}')
        except OSError as e:
            self.get_logger().warn(f'Could not save {path}: {e}')


def main(args=None):
    """
    Entry point for the grid node
    Initializes the ROS 2 system, creates the node, and spins it
    Saves the grid on shutdown
    """
    rclpy.init(args=args)
    node = GridNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_final()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
