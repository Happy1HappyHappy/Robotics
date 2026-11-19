# Copyright 2026 Claire Liu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Cone Navigator node: explore, register the cones, and drive to the one the user picks.

A finite state machine runs in the main thread:

- EXPLORE: turn 360 degrees (measured with odometry), fuse each accepted camera detection
  with the LiDAR and add it to the cone registry (unique IDs, same-color cones kept apart).
- SELECT_TARGET: list the cones by ID and ask which one to visit.
- ALIGN: turn toward the cone, center it in the camera image (angular velocity = Kp *
  bearing error), and measure it with the LiDAR. Without a LiDAR distance, no approach.
- APPROACH: drive toward it, following it in the LiDAR scans (linear velocity = Kd *
  distance error), stopping for obstacles or if the cone is lost.
- STOP: stop at the stand-off distance and measure the final gap.
- COMPLETE: report, then back to SELECT_TARGET.

The ROS callbacks run in a background thread and only store the latest data.
"""

from dataclasses import dataclass
import enum
import math
import signal
import threading
import time
from typing import Optional

from cs5335_cone_navigation import control as ctl
from cs5335_cone_navigation import fusion
from cs5335_cone_navigation.parameters import declare
from cs5335_cone_navigation.registry import ConeRegistry, Observation
from geometry_msgs.msg import Twist, TwistStamped
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection2DArray

FULL_TURN = 2.0 * math.pi
ALIGN_FRAMES = 3    # consecutive centered detections (robot stopped) that end ALIGN
# hazards that stop the robot (not OBJECT_PROXIMITY: it fires near the cone on purpose)
STOP_HAZARDS = {
    HazardDetection.BUMP: 'bump',
    HazardDetection.CLIFF: 'cliff',
    HazardDetection.STALL: 'wheel stall',
    HazardDetection.WHEEL_DROP: 'wheel drop',
}

# (name, default, description[, low, high]); units are SI
PARAMETERS = (
    ('detections_topic', 'cone_detections', 'Cone Detector output topic'),
    ('camera_info_topic', 'oakd/rgb/preview/camera_info',
     "Calibration of the Cone Detector's camera (same image size)"),
    ('scan_topic', 'scan', 'LiDAR topic'),
    ('odom_topic', 'odom', 'Odometry topic'),
    ('cmd_vel_topic', 'cmd_vel', 'Velocity command topic'),
    ('hazard_topic', 'hazard_detection', 'Create 3 hazard topic'),
    ('cmd_vel_stamped', True, 'Publish geometry_msgs/TwistStamped (true) or Twist (false)'),
    ('use_tf', True, 'Look up where the camera and LiDAR sit on the robot in TF'),
    ('camera_mount', [0.0, 0.0, 0.0],
     'Camera [x (m), y (m), yaw (rad)] on the robot if use_tf is false; measure it'),
    ('lidar_mount', [0.0, 0.0, 0.0],
     'LiDAR [x (m), y (m), yaw (rad)] on the robot if use_tf is false; measure it'),
    ('camera_yaw_offset', 0.0,
     'Calibration: added to every camera bearing (rad, + = left)', -0.5, 0.5),
    ('hfov', 1.204, 'Camera horizontal field of view (rad) if camera_info has no '
     'calibration', 0.1, 3.0),
    ('scan_speed', 0.25, 'Turn speed while exploring or searching (rad/s)', 0.05, 1.0),
    ('association_distance', 0.25,
     'An observation this close (m) to a cone of the same color is that cone', 0.02, 2.0),
    ('association_angle', 0.2, 'Without LiDAR positions, an observation this close in '
     'direction (rad) to a cone of the same color is that cone', 0.01, 1.0),
    ('min_observations', 3, 'Sightings a cone needs before it gets an ID', 1, 100),
    ('target_gate', 0.35, 'ALIGN only accepts a detection this close (rad) to where the '
     'target should be', 0.05, 1.5),
    ('bearing_margin', 0.05, "Widen the camera's view of a cone by this (rad) when "
     'matching LiDAR points', 0.0, 0.5),
    ('cluster_depth', 0.15,
     'LiDAR points up to this much (m) behind the nearest are the cone too', 0.01, 1.0),
    ('max_cone_range', 3.0, 'LiDAR returns farther than this (m) are not matched to cones',
     0.2, 12.0),
    ('max_linear_velocity', 0.15, 'Top forward speed (m/s)', 0.01, 0.31),
    ('min_linear_velocity', 0.02, 'Slowest forward speed while approaching (m/s)', 0.0, 0.1),
    ('max_angular_velocity', 0.3, 'Top turn speed (rad/s)', 0.05, 1.9),
    ('min_angular_velocity', 0.08, 'Slowest turn speed (rad/s)', 0.0, 0.5),
    ('kp_angular', 1.0, 'Kp: turn speed per rad of bearing error (1/s)', 0.05, 10.0),
    ('kd_linear', 0.5, 'Kd: forward speed per m of distance error (1/s)', 0.05, 5.0),
    ('angular_tolerance', 0.05, 'ALIGN is done within this bearing error (rad)', 0.005, 0.5),
    ('heading_limit', 0.35, 'APPROACH turns in place if the cone is farther off (rad)',
     0.05, 1.5),
    ('robot_radius', 0.171, 'Radius of the robot (m)', 0.05, 0.5),
    ('stop_gap', 0.0254, 'Gap to leave between the robot and the cone (m); one inch',
     0.0, 0.5),
    ('cone_base_offset', 0.05, 'Calibration: how much farther the cone reaches at the floor '
     'than where the LiDAR hits it (m)', 0.0, 0.5),
    ('track_gate', 0.2, "LiDAR points this close (m) to the cone's last position are the "
     'cone', 0.05, 1.0),
    ('lost_time', 1.0, 'The cone is lost if the LiDAR has not seen it for this long (s)',
     0.1, 10.0),
    ('obstacle_clearance', 0.2,
     'APPROACH stops if anything but the cone is this close ahead (m)', 0.0, 2.0),
    ('max_reacquire', 1, 'Times to go back to ALIGN after losing the cone', 0, 10),
    ('rate_hz', 20.0, 'Control loop rate (Hz)', 1.0, 100.0),
    ('settle_time', 0.5, 'Wait after stopping before taking a still scan (s)', 0.0, 5.0),
    ('data_timeout', 0.5, 'Stop if odometry, scans or detections are older than this (s)',
     0.1, 5.0),
    ('move_timeout', 60.0, 'Give up on one motion after this (s)', 1.0, 600.0),
    ('data_wait', 30.0, 'Give up if the sensor data has not all arrived by then (s)',
     1.0, 600.0),
)


class State(enum.Enum):
    """States of the navigator's finite state machine."""

    EXPLORE = enum.auto()
    SELECT_TARGET = enum.auto()
    ALIGN = enum.auto()
    APPROACH = enum.auto()
    STOP = enum.auto()
    COMPLETE = enum.auto()
    QUIT = enum.auto()


class MotionAborted(Exception):
    """A motion stopped early (hazard, obstacle, stale data, timeout); the message says why."""


class TargetLost(MotionAborted):
    """The target cone was lost while approaching it."""


@dataclass
class Pose:
    """Robot pose in the odom frame: x and y in m, yaw in rad."""

    x: float
    y: float
    yaw: float

    def matrix(self):
        """Return t_odom_base, the 3x3 transform of this pose."""
        return ctl.transform(self.x, self.y, self.yaw)


@dataclass
class Item:
    """One accepted cone in an image: centroid column and box width (px), color, score."""

    u: float
    width: float
    color: str
    confidence: float


@dataclass
class Received:
    """
    Sensor data with the robot's pose when it arrived.

    data is the list of Items of a detection set, or the LiDAR points (N x 2, base frame)
    of a scan. arrival is when it arrived (s, node clock), pose the latest odometry pose
    then, and seq a counter to tell new data from old.
    """

    data: object
    arrival: float
    pose: Optional[Pose]
    seq: int


class ConeNavigator(Node):
    """
    Cone Navigator node.

    Subscribes to the Cone Detector, camera_info, the LiDAR, odometry and hazards;
    publishes velocity commands.
    """

    def __init__(self):
        super().__init__('cone_navigator')
        self._p = {spec[0]: declare(self, *spec) for spec in PARAMETERS}
        for name in ('camera_mount', 'lidar_mount'):
            if len(self._p[name]) != 3:
                raise ValueError(f'{name} needs 3 numbers: x (m), y (m), yaw (rad)')
        self._limits = ctl.MotionLimits(
            max_linear=self._p['max_linear_velocity'],
            min_linear=self._p['min_linear_velocity'],
            kd_linear=self._p['kd_linear'],
            max_angular=self._p['max_angular_velocity'],
            min_angular=self._p['min_angular_velocity'],
            kp_angular=self._p['kp_angular'],
            heading_limit=self._p['heading_limit'])
        self._period = 1.0 / self._p['rate_hz']

        self._odom = None           # (Pose, arrival s)
        self._scan = None           # Received with LiDAR points
        self._detections = None     # Received with Items
        self._camera = None         # (k, width, frame_id) from camera_info
        self._hazard = None         # (name, arrival s) of the latest stopping hazard
        self._hazard_seen = False
        self._motion_start = math.inf   # hazards after this time stop the current motion
        self._base_frame = 'base_link'
        self._lidar_mount = None    # t_base_lidar (3x3)
        self._camera_mount = None   # (x, y, yaw) of the camera on the robot
        self._seq = 0
        self._registry = None
        self._target = None         # TrackedCone being visited
        self._track = None          # its latest LiDAR position (odom)
        self._final_gap = None
        self._reacquired = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        stamped = self._p['cmd_vel_stamped']
        self._cmd_pub = self.create_publisher(TwistStamped if stamped else Twist,
                                              self._p['cmd_vel_topic'], 10)
        sensor = qos_profile_sensor_data
        self.create_subscription(Odometry, self._p['odom_topic'], self._odom_callback, sensor)
        self.create_subscription(LaserScan, self._p['scan_topic'], self._scan_callback, sensor)
        self.create_subscription(Detection2DArray, self._p['detections_topic'],
                                 self._detections_callback, sensor)
        self.create_subscription(CameraInfo, self._p['camera_info_topic'],
                                 self._camera_info_callback, sensor)
        self.create_subscription(HazardDetectionVector, self._p['hazard_topic'],
                                 self._hazard_callback, sensor)

    # ---------- callbacks (background thread): store the latest data ----------
    def _now(self):
        """Return the node clock in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def _next_seq(self):
        """Return a new sequence number."""
        self._seq += 1
        return self._seq

    def _latest_pose(self):
        """Return the latest odometry pose, or None (no freshness check)."""
        odom = self._odom
        return odom[0] if odom is not None else None

    def _odom_callback(self, msg):
        """Store the latest odometry pose."""
        p = msg.pose.pose.position
        self._odom = (Pose(p.x, p.y, ctl.yaw_from_quaternion(msg.pose.pose.orientation)),
                      self._now())
        if msg.child_frame_id:
            self._base_frame = msg.child_frame_id

    def _scan_callback(self, msg):
        """Store the latest scan as points in the base frame, minus the robot's own posts."""
        mount = self._lidar_transform(msg.header.frame_id)
        if mount is None:
            return
        points = ctl.apply(mount, ctl.scan_points(msg.ranges, msg.angle_min,
                                                  msg.angle_increment, msg.range_min,
                                                  msg.range_max))
        points = points[np.hypot(points[:, 0], points[:, 1]) > self._p['robot_radius']]
        self._scan = Received(points, self._now(), self._latest_pose(), self._next_seq())

    def _detections_callback(self, msg):
        """Store the accepted cones of the latest image."""
        items = [Item(d.bbox.center.position.x, d.bbox.size_x, d.results[0].hypothesis.class_id,
                      d.results[0].hypothesis.score) for d in msg.detections if d.results]
        self._detections = Received(items, self._now(), self._latest_pose(), self._next_seq())

    def _camera_info_callback(self, msg):
        """Store the camera matrix, image width and camera frame."""
        if msg.width > 0:
            self._camera = (tuple(msg.k), msg.width, msg.header.frame_id)

    def _hazard_callback(self, msg):
        """Remember a bump, cliff, stall or wheel drop, so the main thread stops the robot."""
        self._hazard_seen = True
        for d in msg.detections:
            if d.type in STOP_HAZARDS:
                self._hazard = (STOP_HAZARDS[d.type], self._now())

    def _lookup(self, frame):
        """Return the transform base_link <- frame from TF, or None (with a warning)."""
        try:
            return self._tf_buffer.lookup_transform(self._base_frame, frame, Time()).transform
        except TransformException as e:
            self.get_logger().warn(f'Waiting for the transform {self._base_frame} -> {frame}: '
                                   f'{e}', throttle_duration_sec=5.0)
            return None

    def _lidar_transform(self, frame):
        """Return t_base_lidar, from TF or lidar_mount, or None until it is known."""
        if self._lidar_mount is None:
            if self._p['use_tf']:
                t = self._lookup(frame)
                if t is None:
                    return None
                x, y = t.translation.x, t.translation.y
                yaw = fusion.facing_yaw(t.rotation, optical=False)
            else:
                x, y, yaw = self._p['lidar_mount']
            self._lidar_mount = ctl.transform(x, y, yaw)
            self.get_logger().info(f'LiDAR on the robot: x={x:.3f} m, y={y:.3f} m, '
                                   f'facing {math.degrees(yaw):.1f} deg')
        return self._lidar_mount

    def _camera_transform(self):
        """Return the camera's (x, y, yaw) on the robot, from TF or camera_mount, or None."""
        if self._camera_mount is None and self._camera is not None:
            frame = self._camera[2]
            if self._p['use_tf']:
                t = self._lookup(frame)
                if t is None:
                    return None
                mount = (t.translation.x, t.translation.y,
                         fusion.facing_yaw(t.rotation, optical='optical' in frame))
            else:
                mount = tuple(self._p['camera_mount'])
            self._camera_mount = mount
            self.get_logger().info(f'Camera ({frame}) on the robot: x={mount[0]:.3f} m, '
                                   f'y={mount[1]:.3f} m, facing {math.degrees(mount[2]):.1f} deg')
        return self._camera_mount

    # ---------- helpers (main thread) ----------
    def _pose(self):
        """Return the latest odometry pose; raise MotionAborted if it is missing or old."""
        odom = self._odom
        if odom is None or self._now() - odom[1] > self._p['data_timeout']:
            raise MotionAborted('odometry stopped arriving')
        return odom[0]

    def _fresh(self, received, what):
        """Return received data; raise MotionAborted if it is older than data_timeout."""
        if received is None or self._now() - received.arrival > self._p['data_timeout']:
            raise MotionAborted(f'{what} stopped arriving')
        return received

    def _publish_cmd(self, v, w):
        """Publish a velocity command (v m/s forward, w rad/s left), clamped to the limits."""
        if not (math.isfinite(v) and math.isfinite(w)):   # max/min would let NaN through
            self.get_logger().error(f'Refusing a velocity that is not a number ({v}, {w})')
            v, w = 0.0, 0.0
        twist = Twist()
        twist.linear.x = float(max(-self._limits.max_linear, min(self._limits.max_linear, v)))
        twist.angular.z = float(max(-self._limits.max_angular,
                                    min(self._limits.max_angular, w)))
        if self._p['cmd_vel_stamped']:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._base_frame
            msg.twist = twist
            self._cmd_pub.publish(msg)
        else:
            self._cmd_pub.publish(twist)

    def stop(self):
        """Stop the robot: a zero velocity, sent a few times in case one is lost."""
        try:
            for _ in range(3):
                self._publish_cmd(0.0, 0.0)
                time.sleep(0.02)
        except Exception as e:  # whatever fails here, the robot's own timeout stops it
            self.get_logger().warn(f'Could not send the stop command: {e}')

    def _step(self, v, w):
        """
        Run one control period: check that moving is safe, send (v, w), wait.

        Raise MotionAborted on a hazard reported since the current motion started, or old
        odometry; the caller's finally stops the robot.
        """
        hazard = self._hazard
        if hazard is not None and hazard[1] >= self._motion_start:
            raise MotionAborted(f'{hazard[0]} detected')
        self._pose()
        self._publish_cmd(v, w)
        time.sleep(self._period)

    def _still_scan(self):
        """Return a LiDAR scan that arrived after the robot stopped and settled."""
        after = self._now() + self._p['settle_time']
        deadline = after + 3.0
        while True:
            scan = self._scan
            if scan is not None and scan.arrival > after:
                return scan
            if self._now() > deadline:
                raise MotionAborted('no LiDAR scan after stopping')
            time.sleep(self._period)

    def _wait_for_data(self):
        """Wait until all sensor data and both sensor mountings are known."""
        deadline = self._now() + self._p['data_wait']
        while True:
            missing = [what for what, ok in (
                ('odometry', self._odom is not None),
                ('LiDAR scans (and the LiDAR transform)', self._scan is not None),
                ('camera_info', self._camera is not None),
                ('the camera transform', self._camera_transform() is not None),
                ('detections (is the Cone Detector running?)', self._detections is not None))
                if not ok]
            if not missing:
                return True
            if self._now() > deadline:
                self.get_logger().error(f'Still missing after {self._p["data_wait"]:.0f} s: '
                                        f'{", ".join(missing)}. Check with: ros2 topic list')
                return False
            self.get_logger().info('Waiting for ' + ', '.join(missing),
                                   throttle_duration_sec=3.0)
            time.sleep(0.2)

    def _observe(self, item, detections, scan):
        """
        Fuse one accepted cone in an image with a LiDAR scan, in the odom frame.

        Return its ray (origin, bearing) and the LiDAR points of the cone (empty if no
        return lies along the ray: its distance is then unavailable).
        """
        k, width, _ = self._camera
        fx, cx = fusion.camera_model(k, width, self._p['hfov'])
        bearing_in_camera = fusion.pixel_bearing(item.u, fx, cx) + self._p['camera_yaw_offset']
        origin, bearing = fusion.ray_in_frame(detections.pose.matrix(), self._camera_mount,
                                              bearing_in_camera)
        half = fusion.angular_half_width(item.u, item.width, fx, cx) + self._p['bearing_margin']
        points = ctl.apply(scan.pose.matrix(), scan.data)
        cluster = fusion.cone_points(points, origin, bearing, half, self._p['cluster_depth'],
                                     self._p['max_cone_range'])
        return origin, bearing, cluster

    def _direction_to(self, cone, pose):
        """Return the direction (rad, odom frame) from the robot to a registered cone."""
        if cone.position is not None:
            return math.atan2(cone.position[1] - pose.y, cone.position[0] - pose.x)
        return cone.bearing

    def _cone_points(self, scan, position):
        """Return the points of a scan (odom frame) within track_gate of a position."""
        points = ctl.apply(scan.pose.matrix(), scan.data)
        near = np.hypot(points[:, 0] - position[0], points[:, 1] - position[1])
        return points[near < self._p['track_gate']]

    def _clear_ahead(self, scan, t_base_odom, reach, cone):
        """Return whether no LiDAR point but the cone's is in the strip ahead of the robot."""
        points = ctl.apply(scan.pose.matrix(), scan.data)
        off = np.hypot(points[:, 0] - cone[0], points[:, 1] - cone[1])
        others = ctl.apply(t_base_odom, points[off >= self._p['track_gate']])
        return ctl.path_clear(others, reach, self._p['robot_radius'])

    def _turn_to(self, heading):
        """Turn in place to face heading (rad, odom frame) by odometry."""
        deadline = self._now() + self._p['move_timeout']
        try:
            while True:
                error = ctl.wrap_angle(heading - self._pose().yaw)
                if abs(error) < self._p['angular_tolerance']:
                    return
                if self._now() > deadline:
                    raise MotionAborted('the turn took too long')
                self._step(0.0, ctl.turn_command(error, self._limits))
        finally:
            self.stop()

    def _list_targets(self):
        """Log every registered cone with its ID, color and estimated position."""
        cones = self._registry.confirmed() if self._registry else []
        if not cones:
            self.get_logger().warn('No targets detected. Type r to explore again.')
            return
        pose = self._latest_pose()
        self.get_logger().info('Detected targets (positions in the odom frame):')
        for cone in cones:
            if cone.distance_valid():
                dx, dy = cone.position[0] - pose.x, cone.position[1] - pose.y
                where = (f'at ({cone.position[0]:+.2f}, {cone.position[1]:+.2f}) m, '
                         f'{math.hypot(dx, dy):.2f} m away')
            else:
                where = 'distance unavailable (no LiDAR return)'
            self.get_logger().info(f'  ID {cone.cone_id}: {cone.color.capitalize()} Cone, '
                                   f'{where}, confidence {cone.confidence:.2f}')

    # ---------- states ----------
    def _explore(self):
        """EXPLORE: turn 360 degrees by odometry and register every cone seen."""
        self._registry = ConeRegistry(self._p['association_distance'],
                                      self._p['association_angle'],
                                      self._p['min_observations'])
        start = self._pose()
        self.get_logger().info(f'Initial heading {math.degrees(start.yaw):.0f} deg; turning '
                               '360 degrees...')
        self._motion_start = self._now()
        deadline = self._now() + FULL_TURN / self._p['scan_speed'] * 2.0 + 10.0
        turned, last_yaw, seq = 0.0, start.yaw, -1
        try:
            while turned < FULL_TURN:
                if self._now() > deadline:
                    raise MotionAborted('the turn took too long; is the robot stuck?')
                self._step(0.0, self._p['scan_speed'])
                yaw = self._pose().yaw
                turned += ctl.wrap_angle(yaw - last_yaw)
                last_yaw = yaw
                detections = self._fresh(self._detections, 'camera detections')
                scan = self._fresh(self._scan, 'LiDAR scans')
                if detections.seq == seq or detections.pose is None or scan.pose is None:
                    continue
                seq = detections.seq
                for item in detections.data:
                    _, bearing, cluster = self._observe(item, detections, scan)
                    position = cluster.mean(axis=0) if len(cluster) else None
                    robot = np.array([detections.pose.x, detections.pose.y])
                    had_id = len(self._registry.confirmed())
                    cone = self._registry.add(Observation(item.color, bearing, position,
                                                          robot, item.confidence))
                    if len(self._registry.confirmed()) > had_id:
                        self.get_logger().info(f'  found ID {cone.cone_id}: {cone.color}')
        finally:
            self.stop()
        self._list_targets()
        return State.SELECT_TARGET

    def _select_target(self):
        """SELECT_TARGET: ask for a target ID and check it."""
        try:
            answer = input('Select target ID (r = explore again, q = quit): ').strip().lower()
        except EOFError:
            return State.QUIT
        if answer in ('q', 'quit'):
            return State.QUIT
        if answer in ('r', 'explore'):
            return State.EXPLORE
        if not answer:
            return State.SELECT_TARGET
        try:
            cone = self._registry.get(int(answer)) if self._registry else None
        except ValueError:
            cone = None
        if cone is None:
            self.get_logger().warn(f'"{answer}" is not a target ID')
            self._list_targets()
            return State.SELECT_TARGET
        self._target, self._track, self._reacquired = cone, cone.position, 0
        where = (f'({cone.position[0]:+.2f}, {cone.position[1]:+.2f}) m'
                 if cone.distance_valid() else 'unknown (no LiDAR distance yet)')
        self.get_logger().info(f'Target: ID {cone.cone_id}, {cone.color.capitalize()} Cone, '
                               f'estimated position {where}')
        return State.ALIGN

    def _find_target(self, detections, expected):
        """
        Find the target in a detection set.

        Return (item, bearing) of the detection of the target's color whose ray (odom
        frame) is closest to the expected direction, within target_gate, or None.
        """
        best, best_off = None, self._p['target_gate']
        k, width, _ = self._camera
        fx, cx = fusion.camera_model(k, width, self._p['hfov'])
        for item in detections.data:
            if item.color != self._target.color:
                continue
            bearing_in_camera = (fusion.pixel_bearing(item.u, fx, cx)
                                 + self._p['camera_yaw_offset'])
            _, bearing = fusion.ray_in_frame(detections.pose.matrix(), self._camera_mount,
                                             bearing_in_camera)
            off = abs(ctl.wrap_angle(bearing - expected))
            if off <= best_off:
                best, best_off = (item, bearing), off
        return best

    def _align(self):
        """
        ALIGN: face the target, center it in the image, and measure it with the LiDAR.

        Each detection's ray is compared with the robot's heading now, so the time the
        image took to arrive does not cause overshoot. When the target is centered for
        ALIGN_FRAMES detections in a row with the robot stopped, a still LiDAR scan must
        show it; otherwise its distance is unavailable and there is no approach.
        """
        self._motion_start = self._now()
        expected = self._direction_to(self._target, self._pose())
        self._turn_to(expected)
        deadline = self._now() + self._p['move_timeout']
        seq, centered, w = -1, 0, 0.0
        last_seen, searched, last_yaw = self._now(), 0.0, self._pose().yaw
        found, found_in = None, None
        try:
            while centered < ALIGN_FRAMES:
                if self._now() > deadline:
                    raise MotionAborted('could not center the target')
                detections = self._fresh(self._detections, 'camera detections')
                yaw = self._pose().yaw
                if detections.seq != seq and detections.pose is not None:
                    seq = detections.seq
                    found = self._find_target(detections, expected)
                    if found is not None:
                        found_in, last_seen, searched = detections, self._now(), 0.0
                        error = ctl.wrap_angle(found[1] - yaw)
                        if abs(error) < self._p['angular_tolerance']:
                            centered, w = centered + 1, 0.0
                        else:
                            centered, w = 0, ctl.turn_command(error, self._limits)
                    else:
                        centered, w = 0, 0.0   # not in this image: wait, then search
                if found is None and self._now() - last_seen > self._p['lost_time']:
                    w = self._p['scan_speed']   # target not in view: search, turning left
                    searched += abs(ctl.wrap_angle(yaw - last_yaw))
                    if searched > FULL_TURN:
                        raise MotionAborted('the target is not in view after a full turn')
                last_yaw = yaw
                self._step(0.0, w)
        finally:
            self.stop()
        scan = self._still_scan()
        _, _, cluster = self._observe(found[0], found_in, scan)
        if len(cluster) == 0:
            self.get_logger().warn('No LiDAR return where the camera sees the target, so its '
                                   'distance is unavailable; not approaching. Is the cone '
                                   'lower than the LiDAR?')
            return State.SELECT_TARGET
        self._track = cluster.mean(axis=0)
        pose = self._pose()
        distance = math.hypot(self._track[0] - pose.x, self._track[1] - pose.y)
        self.get_logger().info(f'Aligned; target {distance:.2f} m from the center of the robot')
        return State.APPROACH

    def _approach(self):
        """
        APPROACH: drive to the target with LiDAR feedback until the stand-off distance.

        With each scan, the points within track_gate of the target's last position are the
        target. The gap is measured from the robot's edge to the nearest of them. If the
        newest scan does not show the target, the robot waits (tracking is unreliable);
        after lost_time it is lost. Anything else within obstacle_clearance ahead stops it.
        """
        stop_at = self._p['stop_gap'] + self._p['cone_base_offset']
        self._motion_start = self._now()
        deadline = self._now() + self._p['move_timeout']
        position = self._track
        seq, points, last_seen, seen = -1, np.array([position]), self._now(), True
        try:
            while True:
                if self._now() > deadline:
                    raise MotionAborted('took too long to reach the target')
                scan = self._fresh(self._scan, 'LiDAR scans')
                if scan.seq != seq and scan.pose is not None:
                    seq = scan.seq
                    near = self._cone_points(scan, position)
                    seen = len(near) > 0
                    if seen:
                        points, position, last_seen = near, near.mean(axis=0), self._now()
                        self._track = position
                    elif self._now() - last_seen > self._p['lost_time']:
                        self._target.position = position
                        raise TargetLost('the target left the LiDAR scans')
                pose = self._pose()
                t_base_odom = np.linalg.inv(pose.matrix())
                target = ctl.apply(t_base_odom, position)[0]
                gap = ctl.gap_to(ctl.apply(t_base_odom, points), self._p['robot_radius'])
                v, w = ctl.approach_command(math.atan2(target[1], target[0]), gap, stop_at,
                                            self._limits)
                if v == 0.0 and w == 0.0:
                    break
                if not seen:
                    v, w = 0.0, 0.0
                reach = self._p['robot_radius'] + min(self._p['obstacle_clearance'], gap)
                if v > 0.0 and not self._clear_ahead(scan, t_base_odom, reach, position):
                    raise MotionAborted('obstacle ahead')
                self._step(v, w)
        finally:
            self.stop()
        return State.STOP

    def _stop_state(self):
        """STOP: stay stopped and measure the final gap in a still scan."""
        self.stop()
        scan = self._still_scan()
        near = self._cone_points(scan, self._track)
        self._final_gap = None
        if len(near):
            t_base_odom = np.linalg.inv(scan.pose.matrix())
            self._final_gap = ctl.gap_to(ctl.apply(t_base_odom, near), self._p['robot_radius'])
            self._track = near.mean(axis=0)
        self._target.position = self._track
        return State.COMPLETE

    def _complete(self):
        """COMPLETE: report the result and go back to SELECT_TARGET."""
        name = f'{self._target.color.capitalize()} Cone #{self._target.cone_id}'
        self.get_logger().info(f'Successfully reached {name}.')
        if self._final_gap is not None:
            self.get_logger().info(
                f'  LiDAR gap {self._final_gap * 100.0:.1f} cm where it hits the cone; about '
                f'{(self._final_gap - self._p["cone_base_offset"]) * 100.0:.1f} cm at the '
                'floor with cone_base_offset (an estimate until measured)')
        self.get_logger().info('Select another target?')
        self._list_targets()
        return State.SELECT_TARGET

    def run(self):
        """Run the state machine until the user quits."""
        if not self._wait_for_data():
            return
        if not self._hazard_seen:
            self.get_logger().warn('No hazard_detection message yet: this node may not see '
                                   'bumps or cliffs. Check: ros2 topic echo /hazard_detection')
        handlers = {State.EXPLORE: self._explore, State.SELECT_TARGET: self._select_target,
                    State.ALIGN: self._align, State.APPROACH: self._approach,
                    State.STOP: self._stop_state, State.COMPLETE: self._complete}
        state = State.EXPLORE
        while state is not State.QUIT:
            self.get_logger().info(f'State: {state.name}')
            try:
                state = handlers[state]()
            except TargetLost as e:
                self.stop()
                if self._reacquired < self._p['max_reacquire']:
                    self._reacquired += 1
                    self.get_logger().warn(f'{e}; stopped, trying to find it again')
                    state = State.ALIGN
                else:
                    self.get_logger().error(f'{e}; giving up on this target')
                    state = State.SELECT_TARGET
            except MotionAborted as e:
                self.stop()
                self.get_logger().error(f'Stopped: {e}')
                state = State.SELECT_TARGET


def raise_interrupt(signum, frame):
    """Turn SIGTERM and SIGHUP (launch shutdown, closed terminal) into Ctrl-C."""
    raise KeyboardInterrupt


def main(args=None):
    """
    Run the Cone Navigator node.

    The ROS callbacks spin in a background thread; the state machine (and input()) runs
    here. On every exit the robot is stopped before ROS shuts down.
    """
    # without rclpy's own handler, Ctrl-C raises KeyboardInterrupt here while ROS is still
    # up, so the finally below can still send the stop command
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, raise_interrupt)
    signal.signal(signal.SIGHUP, raise_interrupt)
    try:
        node = ConeNavigator()
    except Exception as e:  # bad parameters
        get_logger('cone_navigator').fatal(f'Cannot start: {e}')
        rclpy.try_shutdown()
        return
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
