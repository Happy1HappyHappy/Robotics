"""
Cone Node for the day9pkg package (Project 9: computer vision)
Task 1: turns the robot once around in place, finds the colored traffic cones in the camera
image as it turns and names each one's color, then measures how far each is with the LiDAR
Task 2: in a loop, asks for a color and drives to about an inch from that cone
The camera says which object is the cone and which way it is; the LiDAR says how far it is,
and keeps track of it once it is too close for the camera to see it whole
The tasks run in the main thread and drive with cmd_vel; the ROS callbacks run in a
background thread and only store the latest data (and show the camera window)
"""

from dataclasses import dataclass
import math
import os
import signal
import threading
import time
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from day9pkg import geometry as geo
from day9pkg.vision import (blob_bearings, DEFAULT_HSV, draw_blobs, find_blobs,
                            focal_from_fov, pixel_bearing, ShapeLimits)

SHAPE_PARAMETERS = (    # (name, low, high, description) of the vision.ShapeLimits fields
    ('min_aspect', 0.05, 10.0, 'Smallest cone height / width'),
    ('max_aspect', 0.05, 10.0, 'Largest cone height / width'),
    ('min_taper', 0.0, 1.0, 'Smallest 1 - (top width / bottom width); a cone is narrower '
     'at the top'),
    ('min_symmetry', 0.0, 1.0, 'Smallest left-right symmetry (0-1)'),
    ('min_solidity', 0.0, 1.0, 'Smallest pixel count / convex hull area'),
)

WINDOW = 'Cones'
FULL_TURN = 2.0 * math.pi
CENTER_BAND = 0.5       # Task 1 keeps sightings in the middle half of the image only
TURN_TOLERANCE = math.radians(5.0)  # facing a remembered cone this closely is enough
CLUSTER_DEPTH = 0.15    # m, LiDAR points up to this much behind the nearest are the cone too
TRACK_GATE = 0.2        # m, LiDAR points this close to the cone's last position are the cone
LOST_TIME = 1.0         # s, stop if the LiDAR has not seen the cone for this long
SAME_CONE = 0.3         # m, two colors this close together are probably one cone
HAZARD_TIME = 1.0       # s, a hazard reported this recently stops the robot
# hazards that stop the robot (not OBJECT_PROXIMITY: it fires near the cone on purpose)
STOP_HAZARDS = {
    HazardDetection.BUMP: 'bump',
    HazardDetection.CLIFF: 'cliff',
    HazardDetection.STALL: 'wheel stall',
    HazardDetection.WHEEL_DROP: 'wheel drop',
}


class Aborted(Exception):
    """
    A motion stopped before it finished; the message says why
    """


@dataclass
class RobotPose:
    """
    Robot pose from odometry (odom frame) and when it arrived (node clock, s)
    """
    x: float
    y: float
    yaw: float
    stamp: float


@dataclass
class Frame:
    """
    A camera image (BGR), when it arrived, the robot's pose then, and a counter to tell
    new images from old
    """
    bgr: np.ndarray
    stamp: float
    pose: Optional[RobotPose]
    seq: int


@dataclass
class Scan:
    """
    A LiDAR scan as points (N x 2, base frame, m), when it arrived, the robot's pose then,
    and a counter to tell new scans from old
    """
    points: np.ndarray
    stamp: float
    pose: Optional[RobotPose]
    seq: int


@dataclass
class Sighting:
    """
    Task 1's best view of one color so far
    """
    area: int               # px
    bearing: float          # rad, direction of the cone in the odom frame
    half_width: float       # rad, half the angle the cone covers in the image
    image: np.ndarray       # the camera image with the blobs drawn on it


@dataclass
class Cone:
    """
    A cone found in Task 1 (updated after each drive to it)
    """
    color: str
    bearing: float                  # rad, odom frame, from where the robot turned in Task 1
    position: Optional[np.ndarray]  # (x, y) m, odom frame; None if the LiDAR did not see it
    image: Optional[np.ndarray]     # the best camera image of it


def pose_matrix(p):
    """
    ^odom T_base for a RobotPose
    """
    return geo.transform(p.x, p.y, p.yaw)


class ConeNode(Node):
    """
    Cone Node class for the day9pkg package
    Subscribes to the camera (image + camera_info), LiDAR (scan), odometry (odom) and hazards
    (hazard_detection); publishes velocity commands (cmd_vel)
    """
    def __init__(self):
        super().__init__('cone_node')

        self.declare_parameter('colors', ['orange', 'green', 'blue'])  # the 3 cones' colors
        for name, hsv in DEFAULT_HSV.items():
            # [h_lo, s_lo, v_lo, h_hi, s_hi, v_hi], OpenCV scale (H 0-179, S and V 0-255)
            self.declare_parameter(f'hsv.{name}', hsv)
        self.declare_parameter('min_area', 80)              # px, smaller blobs are not cones
        self.declare_parameter('image_topic', 'oakd/rgb/preview/image_raw')
        self.declare_parameter('camera_info_topic', 'oakd/rgb/preview/camera_info')
        self.declare_parameter('hfov', 69.0)        # deg, camera's field of view if no camera_info
        self.declare_parameter('scan_speed', 0.3)   # rad/s, Task 1's turn and searching for a cone
        self.declare_parameter('center_tolerance', 2.0)  # deg, how centered the cone must be
        self.declare_parameter('max_linear_speed', 0.15)    # m/s (TurtleBot 4 max 0.31)
        self.declare_parameter('max_angular_speed', 0.8)    # rad/s (TurtleBot 4 max 1.9)
        self.declare_parameter('robot_radius', 0.171)   # m, the TurtleBot 4 is 0.342 m across
        self.declare_parameter('stop_gap', 0.025)       # m, gap to leave at the cone's base
        # m, how much farther the cone's base sticks out than its side at the LiDAR's height
        # (the LiDAR sees the cone ~19 cm up, where it is narrower than at the floor)
        self.declare_parameter('cone_base_extra', 0.06)
        self.declare_parameter('bearing_margin', 4.0)   # deg, widen the camera's view of the
        #                                                 cone by this when looking in the scan
        self.declare_parameter('max_cone_range', 4.0)   # m, farther than this is not our cone
        self.declare_parameter('rate_hz', 20.0)         # Hz, control loop
        self.declare_parameter('settle_time', 0.5)      # s, wait after stopping to look again
        self.declare_parameter('data_timeout', 0.5)     # s, stop if odom or scans get older
        self.declare_parameter('move_timeout', 60.0)    # s, give up on one move after this
        self.declare_parameter('use_tf', True)          # LiDAR mounting from TF
        self.declare_parameter('lidar_mount', [-0.04, 0.0, 90.0])  # x m, y m, yaw deg; if no TF
        self.declare_parameter('display', True)         # show the camera in a window
        self.declare_parameter('display_scale', 2)      # window pixels per image pixel
        self.declare_parameter('output_dir', '.')       # where the cone images are saved
        self.declare_parameter('shape_check', True, ParameterDescriptor(
            description='Only count regions shaped like a cone (taller than a sliver, '
                        'narrower at the top, symmetric), so boxes and balls are skipped',
            read_only=True))
        defaults = ShapeLimits()
        for name, low, high, description in SHAPE_PARAMETERS:
            self.declare_parameter(name, getattr(defaults, name), ParameterDescriptor(
                description=description, read_only=True,
                floating_point_range=[FloatingPointRange(from_value=low, to_value=high)]))

        self.colors_ = [c.lower() for c in self.get_parameter('colors').value]
        unknown = [c for c in self.colors_ if c not in DEFAULT_HSV]
        if unknown:
            raise ValueError(f'Unknown color(s) {unknown}; choose from {list(DEFAULT_HSV)}')
        self.hsv_ = {c: list(self.get_parameter(f'hsv.{c}').value) for c in self.colors_}
        for c, hsv in self.hsv_.items():
            if len(hsv) != 6:
                raise ValueError(f'hsv.{c} needs 6 numbers: h_lo s_lo v_lo h_hi s_hi v_hi')
        self.min_area_ = self.get_parameter('min_area').value
        self._shape = None      # ShapeLimits, or None to accept any shape
        if self.get_parameter('shape_check').value:
            self._shape = ShapeLimits(**{name: self.get_parameter(name).value
                                         for name, *_ in SHAPE_PARAMETERS})
            if self._shape.min_aspect > self._shape.max_aspect:
                raise ValueError('min_aspect is above max_aspect')
        self.hfov_ = math.radians(self.get_parameter('hfov').value)
        self.scan_speed_ = self.get_parameter('scan_speed').value
        self.center_tolerance_ = math.radians(self.get_parameter('center_tolerance').value)
        self.limits_ = geo.MotionLimits(
            max_linear=self.get_parameter('max_linear_speed').value,
            max_angular=self.get_parameter('max_angular_speed').value)
        self.robot_radius_ = self.get_parameter('robot_radius').value
        self.stop_gap_ = self.get_parameter('stop_gap').value
        self.base_extra_ = self.get_parameter('cone_base_extra').value
        self.margin_ = math.radians(self.get_parameter('bearing_margin').value)
        self.max_cone_range_ = self.get_parameter('max_cone_range').value
        self.period_ = 1.0 / self.get_parameter('rate_hz').value
        self.settle_time_ = self.get_parameter('settle_time').value
        self.data_timeout_ = self.get_parameter('data_timeout').value
        self.move_timeout_ = self.get_parameter('move_timeout').value
        self.scale_ = max(1, self.get_parameter('display_scale').value)
        self.output_dir_ = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir_, exist_ok=True)

        # latest data, each written by one callback only and read by the main thread
        self.odom_ = None           # RobotPose
        self.frame_ = None          # Frame
        self.scan_ = None           # Scan
        self.camera_ = None         # (fx px, cx px, image width px) from camera_info
        self.hazard_ = None         # (name, when) of the latest hazard that stops the robot
        self.base_frame_ = 'base_link'  # replaced by /odom's child_frame_id
        self.mount_ = None          # ^base T_lidar, looked up once
        self.frame_seq_ = 0
        self.scan_seq_ = 0

        self.cones_ = {}            # color -> Cone
        self.target_ = None         # color being driven to, highlighted in the window

        self.bridge_ = CvBridge()
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.cmd_pub_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        # keep only the newest image, so a slow network does not hand us old ones
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_topic_ = self.get_parameter('image_topic').value
        self.create_subscription(Image, self.image_topic_, self.image_callback, image_qos)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, 'hazard_detection',
                                 self.hazard_callback, qos_profile_sensor_data)

        self.window_open_ = False
        self.shown_seq_ = -1
        # The window runs in a thread of its own, not in the ROS executor: drawing it must
        # never hold up the odometry, scan and image callbacks. If it did for longer than
        # data_timeout, the robot would stop mid-turn as if odometry had stopped arriving.
        self._display_stop = threading.Event()
        self._display_thread = None
        if self.get_parameter('display').value:
            if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
                self._display_thread = threading.Thread(target=self._display_loop,
                                                        daemon=True)
                self._display_thread.start()
            else:
                self.get_logger().warn('No display found; the camera window is off')

    # ---------- callbacks (background thread): store the latest data ----------
    def now(self):
        """
        Node clock in seconds
        """
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg: Odometry):
        """
        Store the latest odometry pose
        """
        p = msg.pose.pose.position
        self.odom_ = RobotPose(p.x, p.y, geo.yaw_from_quaternion(msg.pose.pose.orientation),
                               self.now())
        if msg.child_frame_id:
            self.base_frame_ = msg.child_frame_id

    def image_callback(self, msg: Image):
        """
        Store the latest camera image as an OpenCV BGR array, with the robot's pose
        """
        try:
            bgr = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Cannot convert the image: {e}', throttle_duration_sec=5.0)
            return
        self.frame_seq_ += 1
        self.frame_ = Frame(bgr, self.now(), self.odom_, self.frame_seq_)

    def camera_info_callback(self, msg: CameraInfo):
        """
        Store the camera's focal length and optical center (pixels) from its calibration
        """
        if msg.k[0] > 0.0 and msg.width > 0:
            self.camera_ = (msg.k[0], msg.k[2], msg.width)

    def scan_callback(self, msg: LaserScan):
        """
        Store the latest scan as points in the base frame, with the robot's pose
        Points inside the robot's outline (its own posts) are dropped
        """
        mount = self.mount_transform(msg.header.frame_id)
        if mount is None:
            return
        points = geo.apply(mount, geo.scan_points(msg.ranges, msg.angle_min,
                                                  msg.angle_increment, msg.range_min,
                                                  msg.range_max))
        points = points[np.hypot(points[:, 0], points[:, 1]) > self.robot_radius_]
        self.scan_seq_ += 1
        self.scan_ = Scan(points, self.now(), self.odom_, self.scan_seq_)

    def hazard_callback(self, msg: HazardDetectionVector):
        """
        Remember a bump, cliff, stall or wheel drop, so the main thread stops the robot
        """
        for d in msg.detections:
            if d.type in STOP_HAZARDS:
                self.hazard_ = (STOP_HAZARDS[d.type], self.now())

    def mount_transform(self, lidar_frame):
        """
        Return ^base T_lidar, where the LiDAR sits on the robot, or None until it is known
        From TF when use_tf is true, else from the lidar_mount parameter
        """
        if self.mount_ is not None:
            return self.mount_
        if self.get_parameter('use_tf').value:
            try:
                t = self.tf_buffer_.lookup_transform(self.base_frame_, lidar_frame, Time())
            except TransformException as e:
                self.get_logger().warn(
                    f'Waiting for transform {self.base_frame_} -> {lidar_frame}: {e}',
                    throttle_duration_sec=5.0)
                return None
            tr = t.transform.translation
            self.mount_ = geo.transform(tr.x, tr.y, geo.yaw_from_quaternion(t.transform.rotation))
        else:
            x, y, yaw = self.get_parameter('lidar_mount').value
            self.mount_ = geo.transform(x, y, math.radians(yaw))
        x, y = self.mount_[0, 2], self.mount_[1, 2]
        yaw = math.degrees(math.atan2(self.mount_[1, 0], self.mount_[0, 0]))
        self.get_logger().info(f'LiDAR on the robot: x={x:.3f} m, y={y:.3f} m, yaw={yaw:.1f} deg')
        return self.mount_

    # ---------- camera window (its own thread) ----------
    def _display_loop(self):
        """Show the camera window until close_display() is called (runs in its own thread)."""
        while not self._display_stop.is_set() and self.display_callback():
            time.sleep(0.05)
        if self.window_open_:
            cv2.destroyAllWindows()

    def close_display(self):
        """Stop the camera window's thread, which closes the window."""
        self._display_stop.set()
        if self._display_thread is not None:
            self._display_thread.join(timeout=1.0)

    def display_callback(self):
        """
        Show the latest image with each color's blob boxed and the image's center line.

        cv2.waitKey lets the window draw itself and handle clicks. Return False if the
        window cannot be shown (it is then turned off), True otherwise.
        """
        frame = self.frame_
        try:
            if frame is not None and frame.seq != self.shown_seq_:
                self.shown_seq_ = frame.seq
                blobs = find_blobs(frame.bgr, self.hsv_, self.min_area_, self._shape)
                img = draw_blobs(frame.bgr, blobs, highlight=self.target_)
                _, cx = self.camera_model(img.shape[1])
                cv2.line(img, (int(round(cx)), 0), (int(round(cx)), img.shape[0] - 1),
                         (0, 255, 255), 1)
                img = cv2.resize(img, None, fx=self.scale_, fy=self.scale_,
                                 interpolation=cv2.INTER_NEAREST)
                cv2.imshow(WINDOW, img)
                if not self.window_open_:
                    cv2.setMouseCallback(WINDOW, self.on_click)
                    self.window_open_ = True
            if self.window_open_:
                cv2.waitKey(1)
        except cv2.error as e:
            self.get_logger().warn(f'Cannot show the camera window ({e}); turning it off')
            return False
        return True

    def on_click(self, event, x, y, flags, param):
        """
        Log the HSV values around a clicked pixel, to set the hsv.<color> ranges
        """
        frame = self.frame_
        if event != cv2.EVENT_LBUTTONDOWN or frame is None:
            return
        u, v = x // self.scale_, y // self.scale_
        patch = frame.bgr[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        if patch.size == 0:
            return
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        lo, hi = hsv.min(axis=0), hsv.max(axis=0)
        self.get_logger().info(
            f'Pixel ({u}, {v}) and its 5x5 neighbors: H {lo[0]}-{hi[0]}, '
            f'S {lo[1]}-{hi[1]}, V {lo[2]}-{hi[2]}')

    # ---------- helpers (main thread) ----------
    def camera_model(self, width):
        """
        Focal length and optical center column (pixels) for an image width pixels across
        From camera_info, scaled if its width differs; else from the hfov parameter
        """
        if self.camera_ is not None:
            fx, cx, info_width = self.camera_
            s = width / info_width
            return fx * s, (cx + 0.5) * s - 0.5
        return focal_from_fov(width, self.hfov_), (width - 1) / 2.0

    def pose(self):
        """
        Latest odometry pose; raises Aborted if it is missing or older than data_timeout
        """
        p = self.odom_
        if p is None or self.now() - p.stamp > self.data_timeout_:
            raise Aborted('odometry (odom) stopped arriving')
        return p

    def publish_cmd(self, v, w):
        """
        Publish a velocity command (v m/s forward, w rad/s left), clamped to the limits
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame_
        msg.twist.linear.x = float(max(-self.limits_.max_linear,
                                       min(self.limits_.max_linear, v)))
        msg.twist.angular.z = float(max(-self.limits_.max_angular,
                                        min(self.limits_.max_angular, w)))
        self.cmd_pub_.publish(msg)

    def stop(self):
        """
        Stop the robot: a zero velocity, sent a few times in case one is lost
        Best effort: once ROS itself is shut down it cannot be sent
        """
        try:
            for _ in range(3):
                self.publish_cmd(0.0, 0.0)
                time.sleep(0.02)
        except Exception as e:      # whatever fails here, the robot's own timeout stops it
            self.get_logger().warn(f'Could not send the stop command: {e}')

    def step(self, v, w):
        """
        One control period: check that it is safe to move, send (v, w), wait for the next one
        Raises Aborted on a hazard or old odometry; the caller's finally stops the robot
        """
        hazard = self.hazard_
        if hazard is not None and self.now() - hazard[1] < HAZARD_TIME:
            raise Aborted(f'{hazard[0]} detected')
        self.pose()
        self.publish_cmd(v, w)
        time.sleep(self.period_)

    def wait_for(self, get, timeout, what):
        """
        Wait (robot stopped) until get() returns something other than None, and return it
        Raises Aborted after timeout seconds
        """
        deadline = self.now() + timeout
        while True:
            value = get()
            if value is not None:
                return value
            if self.now() > deadline:
                raise Aborted(f'no {what} for {timeout:.0f} s')
            time.sleep(self.period_)

    @staticmethod
    def newer(item, after):
        """
        item (a Frame or Scan) if it arrived after the time after, with a pose; else None
        """
        if item is not None and item.stamp > after and item.pose is not None:
            return item
        return None

    def still_frame(self):
        """
        A camera image that arrived after the robot stopped and settled
        """
        after = self.now() + self.settle_time_
        return self.wait_for(lambda: self.newer(self.frame_, after), self.settle_time_ + 3.0,
                             'camera image')

    def still_scan(self):
        """
        A LiDAR scan that arrived after the robot stopped and settled
        """
        after = self.now() + self.settle_time_
        return self.wait_for(lambda: self.newer(self.scan_, after), self.settle_time_ + 3.0,
                             'LiDAR scan')

    def find(self, color, bgr):
        """
        The largest blob of one color in an image, or None
        """
        return find_blobs(bgr, {color: self.hsv_[color]}, self.min_area_,
                          self._shape).get(color)

    def wait_for_data(self, timeout=30.0):
        """
        Wait until odometry, LiDAR scans and camera images arrive; True if they all did
        """
        deadline = time.monotonic() + timeout
        next_print = 0.0
        missing = []
        while time.monotonic() < deadline:
            missing = [what for what, item in (
                ('odometry (odom)', self.odom_),
                ('LiDAR scans (scan, and the transform to the LiDAR)', self.scan_),
                (f'camera images ({self.image_topic_})', self.frame_)) if item is None]
            if not missing:
                if self.camera_ is None:
                    self.get_logger().warn('No camera_info yet; using hfov '
                                           f'{math.degrees(self.hfov_):.0f} deg until it comes')
                return True
            if time.monotonic() > next_print:
                print('Waiting for ' + ', '.join(missing), flush=True)
                next_print = time.monotonic() + 3.0
            time.sleep(0.2)
        print(f'Still missing after {timeout:.0f} s: ' + ', '.join(missing) +
              '\nIs the robot connected? Check with: ros2 topic list', flush=True)
        return False

    # ---------- Task 1 ----------
    def find_cones(self):
        """
        Task 1: turn once around in place, keep the best sighting of each color, then find
        each cone in a LiDAR scan to measure how far it is
        A color's best sighting is its largest blob seen in the middle half of the image,
        where the direction from the camera is most accurate
        """
        print('\nTask 1: turning 360 degrees to look for the cones...', flush=True)
        sightings = {}
        start = self.now()
        time_limit = FULL_TURN / self.scan_speed_ * 2.0 + 10.0
        last_yaw = self.pose().yaw
        turned = 0.0
        seq = -1
        try:
            while turned < FULL_TURN:
                if self.now() - start > time_limit:
                    raise Aborted('the turn took too long; is the robot stuck?')
                self.step(0.0, self.scan_speed_)
                yaw = self.pose().yaw
                turned += geo.wrap_angle(yaw - last_yaw)
                last_yaw = yaw
                frame = self.frame_
                if frame is not None and frame.seq != seq and frame.pose is not None:
                    seq = frame.seq
                    self.note_sightings(frame, sightings)
        finally:
            self.stop()

        # where each cone is: the nearest object in its direction in a still scan
        scan = self.still_scan()
        here = pose_matrix(scan.pose)
        self.cones_ = {}
        for color in self.colors_:
            s = sightings.get(color)
            if s is None:
                continue
            cluster = geo.nearest_cluster(scan.points, geo.wrap_angle(s.bearing - scan.pose.yaw),
                                          s.half_width + self.margin_, CLUSTER_DEPTH)
            position = None
            if len(cluster) and np.hypot(*cluster.T).min() <= self.max_cone_range_:
                position = geo.apply(here, cluster).mean(axis=0)
            self.cones_[color] = Cone(color, s.bearing, position, s.image)
        self.report_cones(scan.pose)

    def note_sightings(self, frame, sightings):
        """
        Keep each color's blob in this image if it is the best sighting of that color so far
        """
        blobs = find_blobs(frame.bgr, self.hsv_, self.min_area_, self._shape)
        width = frame.bgr.shape[1]
        fx, cx = self.camera_model(width)
        for color, blob in blobs.items():
            if abs(blob.u - cx) > CENTER_BAND * width / 2.0:
                continue
            if color in sightings and sightings[color].area >= blob.area:
                continue
            if color not in sightings:
                print(f'  saw {color}', flush=True)
            center, left, right = blob_bearings(blob, fx, cx)
            sightings[color] = Sighting(blob.area, geo.wrap_angle(frame.pose.yaw + center),
                                        (left - right) / 2.0,
                                        draw_blobs(frame.bgr, blobs, highlight=color))

    def report_cones(self, pose):
        """
        Print what Task 1 found and save each cone's best camera image
        """
        print('\nCones found (direction from the robot now: 0 = ahead, + = left; distance '
              'from its center to the near side of the cone):')
        for color in self.colors_:
            cone = self.cones_.get(color)
            if cone is None:
                print(f'  {color:<8} not found')
                continue
            if cone.position is not None:
                dx, dy = cone.position[0] - pose.x, cone.position[1] - pose.y
                where = (f'{math.degrees(geo.wrap_angle(math.atan2(dy, dx) - pose.yaw)):+5.0f}'
                         f' deg  {math.hypot(dx, dy):.2f} m')
            else:
                where = (f'{math.degrees(geo.wrap_angle(cone.bearing - pose.yaw)):+5.0f} deg'
                         '  (not seen by the LiDAR)')
            path = os.path.join(self.output_dir_, f'cone_{color}.png')
            saved = cv2.imwrite(path, cone.image)
            print(f'  {color:<8} {where}   image: {path if saved else "could not save"}')
        found = [c for c in self.colors_ if c in self.cones_
                 and self.cones_[c].position is not None]
        for i, a in enumerate(found):
            for b in found[i + 1:]:
                if np.linalg.norm(self.cones_[a].position - self.cones_[b].position) < SAME_CONE:
                    print(f'  Warning: {a} and {b} are in the same place; their HSV ranges '
                          'may overlap')
        missing = [c for c in self.colors_ if c not in self.cones_]
        if missing:
            print(f'  Not found: {", ".join(missing)}. Click on the cone in the camera window '
                  'to see its HSV, fix hsv.<color>, then type s to scan again.')
        print(flush=True)

    # ---------- Task 2 ----------
    def go_to(self, color):
        """
        Task 2: drive to stop_gap from the cone of this color
        1. face where Task 1 (or the last drive to it) put it
        2. turn until it is in the middle of the camera image (searching if it is not in view)
        3. pick it out in a LiDAR scan: the nearest object in the directions it covers
        4. drive to it, following it in the LiDAR scans, and stop stop_gap from it
        """
        print(f'\nDriving to the {color} cone...', flush=True)
        self.target_ = color
        cone = self.cones_.get(color)
        try:
            if cone is not None:
                p = self.pose()
                if cone.position is not None:
                    heading = math.atan2(cone.position[1] - p.y, cone.position[0] - p.x)
                else:
                    heading = cone.bearing      # from where the robot turned in Task 1
                self.turn_to(heading)
            position = self.lock_on(self.center_on(color))
            gap, position = self.approach(position)
            print(f'Arrived: about {(gap - self.base_extra_) * 100.0:.1f} cm from the base of '
                  f'the {color} cone ({gap * 100.0:.1f} cm at the height of the LiDAR)',
                  flush=True)
            # remember where it is for the next drive to it
            if cone is None:
                p = self.pose()
                self.cones_[color] = Cone(color, math.atan2(position[1] - p.y,
                                                            position[0] - p.x), position, None)
            else:
                cone.position = position
        except Aborted as e:
            print(f'Stopped: {e}', flush=True)
        finally:
            self.stop()
            self.target_ = None

    def turn_to(self, heading):
        """
        Turn in place to face heading (rad, odom frame), within TURN_TOLERANCE
        """
        deadline = self.now() + self.move_timeout_
        while True:
            error = geo.wrap_angle(heading - self.pose().yaw)
            if abs(error) < TURN_TOLERANCE:
                break
            if self.now() > deadline:
                raise Aborted('the turn took too long')
            self.step(0.0, geo.turn_command(error, self.limits_))
        self.stop()

    def center_on(self, color):
        """
        Turn until the cone of this color is in the middle of the camera image
        If it is not in view, turn left until it is, for at most one full turn
        Returns the bearings (rad: center, left edge, right edge) of the cone in a still image
        """
        deadline = self.now() + self.move_timeout_
        seq, w = -1, 0.0
        searching = False
        searched, last_yaw = 0.0, self.pose().yaw
        while True:
            if self.now() > deadline:
                raise Aborted(f'could not center the {color} cone in the image')
            frame = self.frame_
            if frame is not None and frame.seq != seq:
                seq = frame.seq
                blob = self.find(color, frame.bgr)
                searching = blob is None
                if searching:
                    w = self.scan_speed_
                else:
                    fx, cx = self.camera_model(frame.bgr.shape[1])
                    error = pixel_bearing(blob.u, fx, cx)
                    if abs(error) < self.center_tolerance_:
                        # stop and check in a still image (no lag, no motion blur)
                        self.stop()
                        still = self.still_frame()
                        blob = self.find(color, still.bgr)
                        if blob is not None:
                            bearings = blob_bearings(blob, fx, cx)
                            if abs(bearings[0]) < 2.0 * self.center_tolerance_:
                                return bearings
                        seq, w = -1, 0.0
                        continue
                    w = geo.turn_command(error, self.limits_)
            yaw = self.pose().yaw
            if searching:
                searched += abs(geo.wrap_angle(yaw - last_yaw))
                if searched > FULL_TURN:
                    raise Aborted(f'no {color} cone in view after a full turn')
            last_yaw = yaw
            self.step(0.0, w)

    def lock_on(self, bearings):
        """
        Pick out the cone in a still LiDAR scan: the nearest object in the directions it
        covers in the camera image (widened by bearing_margin)
        Returns its position (x, y) in the odom frame
        """
        _, left, right = bearings
        scan = self.still_scan()
        cluster = geo.nearest_cluster(scan.points, (left + right) / 2.0,
                                      (left - right) / 2.0 + self.margin_, CLUSTER_DEPTH)
        if len(cluster) == 0:
            raise Aborted('the LiDAR sees nothing where the camera sees the cone '
                          '(is the cone lower than the LiDAR?)')
        distance = float(np.hypot(cluster[:, 0], cluster[:, 1]).min())
        if distance > self.max_cone_range_:
            raise Aborted(f'the nearest thing that way is {distance:.1f} m away, too far '
                          'for a cone')
        print(f'Cone {distance:.2f} m from the center of the robot; driving to it', flush=True)
        return geo.apply(pose_matrix(scan.pose), cluster).mean(axis=0)

    def cone_points(self, scan, position):
        """
        The points of a scan (moved into the odom frame) within TRACK_GATE of position
        """
        points = geo.apply(pose_matrix(scan.pose), scan.points)
        return points[np.hypot(points[:, 0] - position[0], points[:, 1] - position[1])
                      < TRACK_GATE]

    def approach(self, position):
        """
        Drive to the cone at position (odom frame) until the gap to its base is stop_gap
        With each new scan, the points within TRACK_GATE of the cone's last position are the
        cone, and their mean is its new position. Each control period, those points seen from
        where odometry says the robot is now give the direction to steer and the gap left
        The LiDAR measures the gap at its own height, where the cone is cone_base_extra
        narrower than at its base, so it stops when that gap is stop_gap + cone_base_extra
        Returns (gap at the LiDAR's height in a still scan after stopping, cone's position)
        """
        stop_at = self.stop_gap_ + self.base_extra_
        deadline = self.now() + self.move_timeout_
        seq = -1
        points = np.array([position])   # until the first scan
        last_seen = self.now()
        gap = math.inf
        try:
            while True:
                if self.now() > deadline:
                    raise Aborted('took too long to reach the cone')
                scan = self.scan_
                if self.now() - scan.stamp > self.data_timeout_:
                    raise Aborted('LiDAR scans (scan) stopped arriving')
                if scan.seq != seq and scan.pose is not None:
                    seq = scan.seq
                    near = self.cone_points(scan, position)
                    if len(near):
                        points, position, last_seen = near, near.mean(axis=0), self.now()
                    elif self.now() - last_seen > LOST_TIME:
                        raise Aborted('lost the cone in the LiDAR scans')
                base_from_odom = np.linalg.inv(pose_matrix(self.pose()))
                target = geo.apply(base_from_odom, position)[0]
                gap = geo.gap_to(geo.apply(base_from_odom, points), self.robot_radius_)
                v, w = geo.approach_command(math.atan2(target[1], target[0]), gap, stop_at,
                                            self.limits_)
                if v == 0.0 and w == 0.0:
                    break
                self.step(v, w)
        finally:
            self.stop()
        # measure once more, standing still
        scan = self.still_scan()
        near = self.cone_points(scan, position)
        if len(near) == 0:
            return gap, position
        base_from_odom = np.linalg.inv(pose_matrix(scan.pose))
        return geo.gap_to(geo.apply(base_from_odom, near), self.robot_radius_), near.mean(axis=0)

    # ---------- the program ----------
    def run_task1(self):
        """
        Task 1, reporting instead of raising if it stops early
        """
        try:
            self.find_cones()
        except Aborted as e:
            self.stop()
            print(f'Task 1 stopped: {e}. Type s to try again.', flush=True)

    def match_color(self, answer):
        """
        The color named by answer (the whole name or its start), or None
        """
        if answer in self.colors_:
            return answer
        matches = [c for c in self.colors_ if c.startswith(answer)]
        return matches[0] if len(matches) == 1 else None

    def run(self):
        """
        Task 1, then Task 2's loop: ask for a color and drive to that cone, until q
        """
        if not self.wait_for_data():
            return
        self.run_task1()
        prompt = f'Drive to which cone? ({", ".join(self.colors_)}; s = scan again, q = quit) > '
        while True:
            answer = input(prompt).strip().lower()
            if not answer:
                continue
            if answer in ('q', 'quit'):
                break
            if answer in ('s', 'scan'):
                self.run_task1()
                continue
            color = self.match_color(answer)
            if color is None:
                print(f'Unknown color "{answer}"', flush=True)
                continue
            self.go_to(color)


def raise_interrupt(signum, frame):
    """
    Turn SIGTERM / SIGHUP (launch shutdown, closed terminal) into Ctrl-C
    """
    raise KeyboardInterrupt


def main(args=None):
    """
    Entry point for the Cone Node
    The ROS callbacks spin in a background thread; the tasks (and input()) run here
    On every exit (q, Ctrl-C, an error) the robot is stopped before ROS shuts down
    """
    # without rclpy's own handler, Ctrl-C raises KeyboardInterrupt here while ROS is still
    # up, so the finally below can still send the stop command
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, raise_interrupt)
    signal.signal(signal.SIGHUP, raise_interrupt)
    try:
        node = ConeNode()
    except ValueError as e:
        print(f'Bad parameter: {e}')
        rclpy.try_shutdown()
        return
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        node.run()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.stop()
        node.close_display()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
