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
Cone Mission node: record the first three cones, then visit each and come back between.

Task 1: where it starts (the origin), the robot turns in place and records the first
max_cones cones the Cone Detector reports near the middle of the camera image: each one's
color, its bearing and, from the LiDAR, its distance. Task 2: it drives to about an inch
from cone 1, comes back to the origin, then cone 2 and back, and so on, and stops.

The camera (through the Cone Detector) says which object is a cone and which way it is;
the LiDAR says how far it is and keeps track of it once it is too close for the camera.
The mission runs in the main thread and drives with cmd_vel; the ROS callbacks run in a
background thread and only store the latest data.
"""

from dataclasses import dataclass
import functools
import math
import signal
import threading
import time
from typing import Optional

from cs5335_cone_finder import geometry as geo
from cs5335_cone_finder.cone_recorder import ConeRecorder, Sighting
from cs5335_cone_finder.parameters import declare
from cs5335_cone_finder.pose_history import PlanarPose, PoseHistory
from cs5335_cone_finder.vision import box_bearings, camera_model
from geometry_msgs.msg import TwistStamped
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
UNKNOWN_COLOR = 'unknown'   # the Cone Detector's class_id for a cone with no clear color
POSE_TOLERANCE = 0.1        # s, how far outside the odometry history a stamp may be
HISTORY_AGE = 5.0           # s of odometry kept to look poses up by time
# hazards that stop the robot (not OBJECT_PROXIMITY: it fires near the cone on purpose)
STOP_HAZARDS = {
    HazardDetection.BUMP: 'bump',
    HazardDetection.CLIFF: 'cliff',
    HazardDetection.STALL: 'wheel stall',
    HazardDetection.WHEEL_DROP: 'wheel drop',
}


class MissionAborted(Exception):
    """The whole mission stops: a hazard, stale data, or no safe way back to the origin."""


class StepFailed(Exception):
    """One cone could not be reached; the robot goes back to the origin and goes on."""


@dataclass
class Detection:
    """One cone from the Cone Detector: box center column and width (px), color, score."""

    u: float
    width: float
    color: str
    score: float


@dataclass
class DetectionSet:
    """
    The cones found in one image.

    stamp is the image's time (s, robot clock), arrival when the set arrived (s, node
    clock), and seq a counter to tell new sets from old. lag is how far (s) the stamp was
    behind the newest odometry stamp when it arrived, and arrival_pose the newest odometry
    pose then; both are None if no odometry had arrived yet.
    """

    items: list
    stamp: float
    arrival: float
    seq: int
    lag: Optional[float]
    arrival_pose: Optional[PlanarPose]


@dataclass
class Scan:
    """
    A LiDAR scan as points (N x 2, base frame, m).

    stamp is its time (s, robot clock), arrival when it arrived (s, node clock), and seq a
    counter to tell new scans from old. lag and arrival_pose are as in DetectionSet.
    """

    points: np.ndarray
    stamp: float
    arrival: float
    seq: int
    lag: Optional[float]
    arrival_pose: Optional[PlanarPose]


@dataclass
class Cone:
    """
    A cone recorded in Task 1, numbered from 1 in the order it was first seen.

    bearing is the direction of its center in the odom frame and half_width half the angle
    it covered in the image (rad). position (x, y in m, odom frame) and distance (m, from
    the robot's center to the near side of the cone) come from the LiDAR; they are None if
    the LiDAR did not see it.
    """

    number: int
    color: str
    bearing: float
    half_width: float
    position: Optional[np.ndarray]
    distance: Optional[float]


def stamp_seconds(stamp):
    """Return a builtin_interfaces/Time as seconds."""
    return Time.from_msg(stamp).nanoseconds / 1e9


class ConeMission(Node):
    """
    Cone Mission node.

    Subscribes to the Cone Detector (cone_detections), the camera calibration
    (camera_info_topic), the LiDAR (scan), odometry (odom) and hazards (hazard_detection);
    publishes velocity commands (cmd_vel).
    """

    def __init__(self):
        super().__init__('cone_mission')
        self._declare_parameters()

        self._lock = threading.Lock()   # guards the odometry history
        self._history = PoseHistory(HISTORY_AGE)
        self._odom = None               # (PlanarPose, arrival s) of the latest odometry
        self._odom_stamp = None         # stamp (s, robot clock) of the latest odometry
        self._scan = None               # Scan
        self._detections = None         # DetectionSet
        self._camera = None             # (k, width) from camera_info
        self._hazard = None             # (name, arrival s) of the latest stopping hazard
        self._hazard_seen = False       # whether any hazard_detection message has arrived
        self._mission_start = math.inf  # node clock (s) when the robot first moves
        self._base_frame = 'base_link'  # replaced by /odom's child_frame_id
        self._mount = None              # t_base_lidar, looked up once
        self._scan_seq = 0
        self._detection_seq = 0
        self._origin = None             # PlanarPose where the mission started

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        sensor = qos_profile_sensor_data
        self.create_subscription(Odometry, 'odom', self._odom_callback, sensor)
        self.create_subscription(LaserScan, 'scan', self._scan_callback, sensor)
        self.create_subscription(Detection2DArray, self._detections_topic,
                                 self._detections_callback, sensor)
        self.create_subscription(CameraInfo, self._camera_info_topic,
                                 self._camera_info_callback, sensor)
        self.create_subscription(HazardDetectionVector, 'hazard_detection',
                                 self._hazard_callback, sensor)

    def _declare_parameters(self):
        """Declare and read every parameter; raise ValueError if one is invalid."""
        d = functools.partial(declare, self)
        self._detections_topic = d('detections_topic', 'cone_detections',
                                   "Topic of the Cone Detector's detections")
        self._camera_info_topic = d('camera_info_topic', 'oakd/rgb/preview/camera_info',
                                    'Calibration topic of the camera the detector uses')
        self._hfov = d('hfov', 1.204,
                       'Horizontal field of view (rad) if camera_info has no calibration',
                       0.1, 3.0)
        self._max_cones = d('max_cones', 3, 'How many cones to record and visit', 1, 10)
        self._min_sightings = d('min_sightings', 2,
                                'Sightings a cone needs before it counts', 1, 50)
        self._scan_speed = d('scan_speed', 0.3, 'Turn speed while looking for cones (rad/s)',
                             0.05, 1.0)
        self._scan_band = d('scan_band', 0.3,
                            'Only cones within this angle (rad) of the image center count',
                            0.05, 1.0)
        self._scan_center_tolerance = d(
            'scan_center_tolerance', 0.09,
            'Each recorded cone must be seen this close (rad) to the image center', 0.01, 0.5)
        self._same_cone_angle = d('same_cone_angle', 0.35,
                                  'Sightings this close together (rad) are one cone',
                                  0.05, 1.5)
        self._center_tolerance = d('center_tolerance', 0.035,
                                   'How centered (rad) a cone must be before driving to it',
                                   0.005, 0.3)
        self._turn_tolerance = d('turn_tolerance', 0.087,
                                 'Heading error (rad) at which a turn in place is done',
                                 0.01, 0.5)
        self._limits = geo.MotionLimits(
            max_linear=d('max_linear_speed', 0.15, 'Top forward speed (m/s)', 0.01, 0.31),
            min_linear=d('min_linear_speed', 0.02,
                         'Slowest forward speed while driving (m/s)', 0.0, 0.1),
            linear_gain=d('linear_gain', 0.5, 'Forward speed per meter still to go (1/s)',
                          0.05, 5.0),
            max_angular=d('max_angular_speed', 0.8, 'Top turn speed (rad/s)', 0.05, 1.9),
            min_angular=d('min_angular_speed', 0.1, 'Slowest turn speed (rad/s)', 0.0, 0.5),
            angular_gain=d('angular_gain', 1.5, 'Turn speed per radian of error (1/s)',
                           0.1, 10.0),
            turn_first=d('turn_first', 0.35,
                         'Turn in place before driving if the target is farther off (rad)',
                         0.05, math.pi))
        self._robot_radius = d('robot_radius', 0.171, 'Radius of the robot (m)', 0.05, 0.5)
        self._stop_gap = d('stop_gap', 0.025,
                           'Gap to leave between the robot and the base of a cone (m)',
                           0.0, 0.5)
        self._base_extra = d('cone_base_extra', 0.06,
                             "How much farther a cone's base sticks out than its side at "
                             "the LiDAR's height (m)", 0.0, 0.3)
        self._margin = d('bearing_margin', 0.07,
                         "Widen the camera's view of a cone by this (rad) in the LiDAR scan",
                         0.0, 0.5)
        self._cluster_depth = d('cluster_depth', 0.15,
                                'LiDAR points up to this much (m) behind the nearest are '
                                'the cone too', 0.01, 1.0)
        self._track_gate = d('track_gate', 0.2,
                             "LiDAR points this close (m) to the cone's last position are "
                             'the cone', 0.05, 1.0)
        self._lost_time = d('lost_time', 1.0,
                            'Stop if the LiDAR has not seen the cone for this long (s)',
                            0.1, 10.0)
        self._max_cone_range = d('max_cone_range', 4.0,
                                 'Anything farther than this (m) is not a cone', 0.3, 12.0)
        self._origin_tolerance = d('origin_tolerance', 0.05,
                                   'How close (m) to the origin counts as back', 0.01, 0.5)
        self._front_clearance = d('front_clearance', 0.2,
                                  'On the way back, stop if anything is this close (m) in '
                                  'front of the robot', 0.0, 2.0)
        self._period = 1.0 / d('rate_hz', 20.0, 'Control loop rate (Hz)', 1.0, 100.0)
        self._settle_time = d('settle_time', 0.5,
                              'Wait this long (s) after stopping before looking again',
                              0.0, 5.0)
        self._data_timeout = d('data_timeout', 0.5,
                               'Stop if odometry or scans are older than this (s)', 0.1, 5.0)
        self._move_timeout = d('move_timeout', 60.0, 'Give up on one move after this (s)',
                               1.0, 600.0)
        self._start_delay = d('start_delay', 3.0,
                              'Wait this long (s) once all data is in before moving',
                              0.0, 60.0)
        self._data_wait = d('data_wait', 60.0,
                            'Give up if the sensor data has not all arrived after this (s)',
                            1.0, 600.0)
        self._use_tf = d('use_tf', True, 'Look up where the LiDAR sits on the robot in TF')
        self._lidar_mount = d('lidar_mount', [-0.04, 0.0, math.pi / 2.0],
                              'Where the LiDAR sits on the robot if use_tf is false: '
                              '[x (m), y (m), yaw (rad)]')
        self._max_scan_lag = d('max_scan_lag', 0.5,
                               'A scan whose stamp lags the odometry by more (s) means the '
                               "robot's clocks disagree", 0.05, 5.0)
        self._max_detection_lag = d('max_detection_lag', 1.0,
                                    'Detections whose image stamp lags the odometry by more '
                                    "(s) mean the robot's clocks disagree, or YOLO is slow",
                                    0.05, 10.0)
        if len(self._lidar_mount) != 3:
            raise ValueError('lidar_mount needs 3 numbers: x (m), y (m), yaw (rad)')
        if self._limits.min_linear > self._limits.max_linear:
            raise ValueError('min_linear_speed is above max_linear_speed')

    # ---------- callbacks (background thread): store the latest data ----------
    def _now(self):
        """Return the node clock in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def _odom_callback(self, msg):
        """Store the latest odometry pose and add it to the history."""
        p = msg.pose.pose.position
        pose = PlanarPose(p.x, p.y, geo.yaw_from_quaternion(msg.pose.pose.orientation))
        stamp = stamp_seconds(msg.header.stamp)
        with self._lock:
            self._history.add(stamp, pose)
        self._odom = (pose, self._now())
        self._odom_stamp = stamp
        if msg.child_frame_id:
            self._base_frame = msg.child_frame_id

    def _arrival_info(self, stamp):
        """Return how far a sensor stamp lags the newest odometry stamp, and that pose."""
        odom, odom_stamp = self._odom, self._odom_stamp
        if odom is None or odom_stamp is None:
            return None, None
        return odom_stamp - stamp, odom[0]

    def _scan_callback(self, msg):
        """Store the latest scan as points in the base frame, minus the robot's own posts."""
        mount = self._mount_transform(msg.header.frame_id)
        if mount is None:
            return
        points = geo.apply(mount, geo.scan_points(msg.ranges, msg.angle_min,
                                                  msg.angle_increment, msg.range_min,
                                                  msg.range_max))
        points = points[np.hypot(points[:, 0], points[:, 1]) > self._robot_radius]
        stamp = stamp_seconds(msg.header.stamp)
        self._scan_seq += 1
        self._scan = Scan(points, stamp, self._now(), self._scan_seq,
                          *self._arrival_info(stamp))

    def _detections_callback(self, msg):
        """Store the cones the Cone Detector found in its latest image."""
        items = []
        for d in msg.detections:
            if d.results:
                hypothesis = d.results[0].hypothesis
                items.append(Detection(d.bbox.center.position.x, d.bbox.size_x,
                                       hypothesis.class_id, hypothesis.score))
        stamp = stamp_seconds(msg.header.stamp)
        self._detection_seq += 1
        self._detections = DetectionSet(items, stamp, self._now(), self._detection_seq,
                                        *self._arrival_info(stamp))

    def _camera_info_callback(self, msg):
        """Store the camera matrix and image width."""
        if msg.width > 0:
            self._camera = (tuple(msg.k), msg.width)

    def _hazard_callback(self, msg):
        """Remember a bump, cliff, stall or wheel drop, so the main thread stops the robot."""
        self._hazard_seen = True
        for d in msg.detections:
            if d.type in STOP_HAZARDS:
                self._hazard = (STOP_HAZARDS[d.type], self._now())

    def _mount_transform(self, lidar_frame):
        """
        Return t_base_lidar, where the LiDAR sits on the robot, or None until it is known.

        From TF when use_tf is true, otherwise from the lidar_mount parameter.
        """
        if self._mount is not None:
            return self._mount
        if self._use_tf:
            try:
                t = self._tf_buffer.lookup_transform(self._base_frame, lidar_frame, Time())
            except TransformException as e:
                self.get_logger().warn(
                    f'Waiting for the transform {self._base_frame} -> {lidar_frame}: {e}',
                    throttle_duration_sec=5.0)
                return None
            tr = t.transform.translation
            self._mount = geo.transform(tr.x, tr.y,
                                        geo.yaw_from_quaternion(t.transform.rotation))
        else:
            x, y, yaw = self._lidar_mount
            self._mount = geo.transform(x, y, yaw)
        x, y = self._mount[0, 2], self._mount[1, 2]
        yaw = math.atan2(self._mount[1, 0], self._mount[0, 0])
        self.get_logger().info(
            f'LiDAR on the robot: x={x:.3f} m, y={y:.3f} m, yaw={math.degrees(yaw):.1f} deg')
        return self._mount

    # ---------- helpers (main thread) ----------
    def _pose(self):
        """Return the latest odometry pose; raise MissionAborted if it is missing or old."""
        odom = self._odom
        if odom is None or self._now() - odom[1] > self._data_timeout:
            raise MissionAborted('odometry (odom) stopped arriving')
        return odom[0]

    def _pose_for(self, item, max_lag):
        """
        Return the robot's pose when a scan or image (a Scan or DetectionSet) was taken.

        The pose is looked up in the odometry history at the item's stamp. If the stamp does
        not fit, because it lagged the newest odometry by more than max_lag s or was ahead
        of it (the robot's clocks probably disagree), or it is outside the history, the
        pose when the item arrived is used instead, with a warning.
        """
        pose = None
        if item.lag is not None and -POSE_TOLERANCE <= item.lag <= max_lag:
            with self._lock:
                pose = self._history.pose_at(item.stamp, POSE_TOLERANCE)
        if pose is None:
            lag = 'unknown' if item.lag is None else f'{item.lag:+.2f} s'
            self.get_logger().warn(
                f'A sensor stamp does not fit the odometry stamps (lag {lag}); using the pose '
                "when the data arrived. Check that the robot's clocks agree.",
                throttle_duration_sec=10.0)
            pose = item.arrival_pose if item.arrival_pose is not None else self._pose()
        return pose

    def _scan_pose(self, scan):
        """Return the robot's pose when a scan was taken."""
        return self._pose_for(scan, self._max_scan_lag)

    def _detection_pose(self, detections):
        """Return the robot's pose when the image of a detection set was taken."""
        return self._pose_for(detections, self._max_detection_lag)

    def _camera_model(self):
        """Return the camera's focal length and optical axis column (px)."""
        k, width = self._camera
        return camera_model(k, width, self._hfov)

    def _publish_cmd(self, v, w):
        """Publish a velocity command (v m/s forward, w rad/s left), clamped to the limits."""
        if not (math.isfinite(v) and math.isfinite(w)):   # max/min would let NaN through
            self.get_logger().error(f'Refusing a velocity that is not a number ({v}, {w})')
            v, w = 0.0, 0.0
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.twist.linear.x = float(max(-self._limits.max_linear,
                                       min(self._limits.max_linear, v)))
        msg.twist.angular.z = float(max(-self._limits.max_angular,
                                        min(self._limits.max_angular, w)))
        self._cmd_pub.publish(msg)

    def stop(self):
        """
        Stop the robot: a zero velocity, sent a few times in case one is lost.

        Best effort: once ROS itself is shut down it cannot be sent, and the robot's own
        command timeout stops it.
        """
        try:
            for _ in range(3):
                self._publish_cmd(0.0, 0.0)
                time.sleep(0.02)
        except Exception as e:  # whatever fails here, the robot's own timeout stops it
            self.get_logger().warn(f'Could not send the stop command: {e}')

    def _step(self, v, w):
        """
        Run one control period: check that moving is safe, send (v, w), wait.

        Raise MissionAborted on old odometry, or on any hazard reported since the mission
        started, even while the robot stood still (a bump does not repeat); the caller's
        finally stops the robot.
        """
        hazard = self._hazard
        if hazard is not None and hazard[1] >= self._mission_start:
            raise MissionAborted(f'{hazard[0]} detected')
        self._pose()
        self._publish_cmd(v, w)
        time.sleep(self._period)

    def _wait_for(self, get, timeout, what):
        """
        Wait (robot stopped) until get() returns something other than None, and return it.

        Raise MissionAborted after timeout s.
        """
        deadline = self._now() + timeout
        while True:
            value = get()
            if value is not None:
                return value
            if self._now() > deadline:
                raise MissionAborted(f'no {what} for {timeout:.0f} s')
            time.sleep(self._period)

    @staticmethod
    def _newer(item, after):
        """Return item (a Scan or DetectionSet) if it arrived after the time after."""
        return item if item is not None and item.arrival > after else None

    def _still_scan(self):
        """Return a LiDAR scan that arrived after the robot stopped and settled."""
        after = self._now() + self._settle_time
        return self._wait_for(lambda: self._newer(self._scan, after),
                              self._settle_time + 3.0, 'LiDAR scan')

    def _still_detections(self):
        """
        Return detections from an image taken after the robot stopped and settled.

        YOLO takes a while per image, so the first set to arrive after settling may come
        from an image taken before; the one after it cannot.
        """
        after = self._now() + self._settle_time
        first = self._wait_for(lambda: self._newer(self._detections, after),
                               self._settle_time + 5.0, 'detections from the Cone Detector')
        return self._wait_for(lambda: self._newer(self._detections, first.arrival), 5.0,
                              'detections from the Cone Detector')

    def _wait_for_data(self):
        """Wait until all sensor data has arrived; return whether it did."""
        deadline = self._now() + self._data_wait
        while True:
            missing = [what for what, item in (
                ('odometry (odom)', self._odom),
                ('LiDAR scans (scan, and the transform to the LiDAR)', self._scan),
                (f'camera calibration ({self._camera_info_topic})', self._camera),
                (f'detections ({self._detections_topic}; is the Cone Detector running?)',
                 self._detections)) if item is None]
            if not missing:
                return True
            if self._now() > deadline:
                self.get_logger().error(
                    f'Still missing after {self._data_wait:.0f} s: {", ".join(missing)}. '
                    'Is the robot connected? Check with: ros2 topic list')
                return False
            self.get_logger().info('Waiting for ' + ', '.join(missing),
                                   throttle_duration_sec=3.0)
            time.sleep(0.2)

    def _turn_to(self, heading):
        """Turn in place to face heading (rad, odom frame); raise StepFailed if too slow."""
        deadline = self._now() + self._move_timeout
        try:
            while True:
                error = geo.wrap_angle(heading - self._pose().yaw)
                if abs(error) < self._turn_tolerance:
                    return
                if self._now() > deadline:
                    raise StepFailed('the turn took too long')
                self._step(0.0, geo.turn_command(error, self._limits))
        finally:
            self.stop()

    # ---------- Task 1: record the first cones ----------
    def _find_cones(self):
        """
        Turn in place, record the first max_cones cones, and measure them with the LiDAR.

        The turn ends when every recorded cone has been seen near the image center, or
        after a full turn (plus scan_band, to center a cone that was just right of the
        center at the start). Return the cones (a list of Cone).
        """
        self.get_logger().info(
            f'Task 1: turning in place to find the first {self._max_cones} cones...')
        recorder = ConeRecorder(self._max_cones, self._same_cone_angle, self._scan_band,
                                self._scan_center_tolerance, self._min_sightings)
        max_turn = FULL_TURN + self._scan_band
        deadline = self._now() + max_turn / self._scan_speed * 2.0 + 10.0
        turned, last_yaw, seq = 0.0, self._pose().yaw, -1
        try:
            while not recorder.complete() and turned < max_turn:
                if self._now() > deadline:
                    raise MissionAborted('the turn took too long; is the robot stuck?')
                self._step(0.0, self._scan_speed)
                yaw = self._pose().yaw
                turned += abs(geo.wrap_angle(yaw - last_yaw))
                last_yaw = yaw
                detections = self._detections
                if detections.seq != seq:
                    seq = detections.seq
                    self._note_sightings(detections, recorder)
        finally:
            self.stop()

        found = recorder.cones()
        if len(found) < self._max_cones:
            self.get_logger().warn(
                f'Found only {len(found)} of {self._max_cones} cones in a full turn')
        scan = self._still_scan()
        pose = self._scan_pose(scan)
        t_odom_base = geo.transform(pose.x, pose.y, pose.yaw)
        cones = []
        for number, candidate in enumerate(found, start=1):
            best = candidate.best
            cluster = geo.nearest_cluster(scan.points, geo.wrap_angle(best.bearing - pose.yaw),
                                          best.half_width + self._margin, self._cluster_depth)
            position = distance = None
            if len(cluster):
                nearest = float(np.hypot(cluster[:, 0], cluster[:, 1]).min())
                if nearest <= self._max_cone_range:
                    position = geo.apply(t_odom_base, cluster).mean(axis=0)
                    distance = nearest
            cones.append(Cone(number, candidate.color(), best.bearing, best.half_width,
                              position, distance))
        self._report_cones(cones)
        return cones

    def _note_sightings(self, detections, recorder):
        """Add the colored cones of one detection set to the recorder."""
        fx, cx = self._camera_model()
        pose = self._detection_pose(detections)
        for d in detections.items:
            if d.color == UNKNOWN_COLOR:
                continue
            center, left, right = box_bearings(d.u, d.width, fx, cx)
            before = len(recorder.cones())
            recorder.add(Sighting(geo.wrap_angle(pose.yaw + center), center,
                                  (left - right) / 2.0, d.color))
            if len(recorder.cones()) > before:
                self.get_logger().info(f'  saw cone {len(recorder.cones())}: {d.color}')

    def _report_cones(self, cones):
        """Log each cone's color, bearing from the start heading and LiDAR distance."""
        o = self._origin
        self.get_logger().info(
            'Cones found (bearing from the start heading: 0 = ahead, + = left; distance '
            "from the robot's center to the near side of the cone, by LiDAR):")
        for cone in cones:
            if cone.position is not None:
                bearing = math.atan2(cone.position[1] - o.y, cone.position[0] - o.x) - o.yaw
                distance = f'{cone.distance:.2f} m'
            else:
                bearing = cone.bearing - o.yaw
                distance = 'not seen by the LiDAR (is it lower than the LiDAR?)'
            self.get_logger().info(
                f'  cone {cone.number}  {cone.color:<7} '
                f'{math.degrees(geo.wrap_angle(bearing)):+5.0f} deg  {distance}')

    # ---------- Task 2: visit each cone and come back ----------
    def _visit_all(self, cones):
        """Drive to each cone in order, coming back to the origin after each."""
        results = []
        for cone in cones:
            try:
                gap = self._go_to_cone(cone)
                results.append(f'cone {cone.number} ({cone.color}): reached, about '
                               f'{gap * 100.0:.1f} cm from its base')
            except StepFailed as e:
                self.stop()
                self.get_logger().warn(f'Cone {cone.number} not reached: {e}')
                results.append(f'cone {cone.number} ({cone.color}): not reached ({e})')
            self._return_to_origin()
        self.get_logger().info('Mission done:')
        for line in results:
            self.get_logger().info(f'  {line}')

    def _go_to_cone(self, cone):
        """
        Drive to stop_gap from a cone and return the gap left at its base (m).

        Face where Task 1 put it, center it in the camera image, pick it out in a LiDAR
        scan, then drive to it following it in the scans.
        """
        self.get_logger().info(f'Task 2: driving to cone {cone.number} ({cone.color})...')
        pose = self._pose()
        if cone.position is not None:
            expected = math.atan2(cone.position[1] - pose.y, cone.position[0] - pose.x)
        else:
            expected = cone.bearing
        self._turn_to(expected)
        bearings = self._center_on(cone, expected)
        position = self._lock_on(bearings)
        gap, position = self._approach(position)
        cone.position = position
        self.get_logger().info(
            f'Arrived: about {(gap - self._base_extra) * 100.0:.1f} cm from the base of cone '
            f'{cone.number} ({gap * 100.0:.1f} cm at the height of the LiDAR)')
        return gap - self._base_extra

    def _pick(self, detections, cone, expected):
        """
        Find a cone in a detection set.

        Only detections within same_cone_angle of the expected direction (odom frame)
        count; of those, one of the cone's color comes first, then the closest. Return its
        bearings in the image (rad: center, left edge, right edge) and the direction of its
        center in the odom frame (rad), or None if there is none.
        """
        fx, cx = self._camera_model()
        pose = self._detection_pose(detections)
        best, best_key = None, None
        for d in detections.items:
            bearings = box_bearings(d.u, d.width, fx, cx)
            heading = geo.wrap_angle(pose.yaw + bearings[0])
            off = abs(geo.wrap_angle(heading - expected))
            if off > self._same_cone_angle:
                continue
            key = (d.color != cone.color, off)
            if best_key is None or key < best_key:
                best, best_key = (bearings, heading), key
        return best

    def _center_on(self, cone, expected):
        """
        Turn until the cone is in the middle of the camera image.

        Each detection gives the cone's direction in the odom frame (from the robot's pose
        when the image was taken), and the robot turns toward it by odometry, so YOLO's
        delay does not make it overshoot. If the cone is not in view, the robot turns left
        until it is, for at most one full turn. Return its bearings (rad: center, left
        edge, right edge) in a still image.
        """
        deadline = self._now() + self._move_timeout
        seq, target = -1, None
        searched, last_yaw = 0.0, self._pose().yaw
        try:
            while True:
                if self._now() > deadline:
                    raise StepFailed('could not center the cone in the image')
                detections = self._detections
                if detections.seq != seq:
                    seq = detections.seq
                    picked = self._pick(detections, cone, expected)
                    if picked is not None:
                        target = picked[1]
                yaw = self._pose().yaw
                if target is None:
                    searched += abs(geo.wrap_angle(yaw - last_yaw))
                    last_yaw = yaw
                    if searched > FULL_TURN:
                        raise StepFailed('the cone was not in view after a full turn')
                    self._step(0.0, self._scan_speed)
                    continue
                error = geo.wrap_angle(target - yaw)
                if abs(error) >= self._center_tolerance:
                    self._step(0.0, geo.turn_command(error, self._limits))
                    continue
                # stop and check in a still image (no lag, no motion blur)
                self.stop()
                still = self._still_detections()
                picked = self._pick(still, cone, expected)
                if picked is not None and abs(picked[0][0]) < 2.0 * self._center_tolerance:
                    return picked[0]
                seq, target = still.seq, (picked[1] if picked is not None else None)
                last_yaw = self._pose().yaw
        finally:
            self.stop()

    def _lock_on(self, bearings):
        """
        Pick out the cone in a still LiDAR scan and return its position (odom frame).

        It is the nearest object in the directions it covers in the camera image, widened
        by bearing_margin.
        """
        _, left, right = bearings
        scan = self._still_scan()
        cluster = geo.nearest_cluster(scan.points, (left + right) / 2.0,
                                      (left - right) / 2.0 + self._margin, self._cluster_depth)
        if len(cluster) == 0:
            raise StepFailed('the LiDAR sees nothing where the camera sees the cone '
                             '(is the cone lower than the LiDAR?)')
        distance = float(np.hypot(cluster[:, 0], cluster[:, 1]).min())
        if distance > self._max_cone_range:
            raise StepFailed(f'the nearest thing that way is {distance:.1f} m away, too far '
                             'for a cone')
        self.get_logger().info(f'Cone {distance:.2f} m from the center of the robot; '
                               'driving to it')
        pose = self._scan_pose(scan)
        return geo.apply(geo.transform(pose.x, pose.y, pose.yaw), cluster).mean(axis=0)

    def _cone_points(self, scan, position):
        """Return the points of a scan (in the odom frame) within track_gate of position."""
        pose = self._scan_pose(scan)
        points = geo.apply(geo.transform(pose.x, pose.y, pose.yaw), scan.points)
        near = np.hypot(points[:, 0] - position[0], points[:, 1] - position[1])
        return points[near < self._track_gate]

    def _clear_ahead(self, scan, t_base_odom, reach, cone=None):
        """
        Return whether no LiDAR point of a scan is in the strip ahead of the robot.

        t_base_odom is the robot's pose now (inverted). The strip runs reach m forward from
        the robot's center and is as wide as the robot. If cone (x, y in the odom frame) is
        given, the points within track_gate of it are the cone and do not count.
        """
        pose = self._scan_pose(scan)
        points = geo.apply(geo.transform(pose.x, pose.y, pose.yaw), scan.points)
        if cone is not None:
            off = np.hypot(points[:, 0] - cone[0], points[:, 1] - cone[1])
            points = points[off >= self._track_gate]
        return geo.path_clear(geo.apply(t_base_odom, points), reach, self._robot_radius)

    def _approach(self, position):
        """
        Drive to the cone at position (odom frame) until the gap at its base is stop_gap.

        With each new scan, the points within track_gate of the cone's last position are
        the cone, and their mean is its new position. Each control period, those points
        seen from where odometry says the robot is now give the direction to steer and the
        gap left. The LiDAR measures the gap at its own height, where the cone is
        cone_base_extra narrower than at its base, so the robot stops when that gap is
        stop_gap + cone_base_extra. If the newest scan does not show the cone, the robot
        waits (for at most lost_time) instead of driving on old data; if anything other
        than the cone is within front_clearance ahead, it gives up on this cone. Return (gap
        at the LiDAR's height in a still scan after stopping, the cone's position).
        """
        stop_at = self._stop_gap + self._base_extra
        deadline = self._now() + self._move_timeout
        seq, points, last_seen, gap = -1, np.array([position]), self._now(), math.inf
        seen = True     # whether the newest scan showed the cone
        try:
            while True:
                if self._now() > deadline:
                    raise StepFailed('took too long to reach the cone')
                scan = self._scan
                if self._now() - scan.arrival > self._data_timeout:
                    raise MissionAborted('LiDAR scans (scan) stopped arriving')
                if scan.seq != seq:
                    seq = scan.seq
                    near = self._cone_points(scan, position)
                    seen = len(near) > 0
                    if seen:
                        points, position, last_seen = near, near.mean(axis=0), self._now()
                    elif self._now() - last_seen > self._lost_time:
                        raise StepFailed('lost the cone in the LiDAR scans')
                pose = self._pose()
                t_base_odom = np.linalg.inv(geo.transform(pose.x, pose.y, pose.yaw))
                target = geo.apply(t_base_odom, position)[0]
                gap = geo.gap_to(geo.apply(t_base_odom, points), self._robot_radius)
                v, w = geo.approach_command(math.atan2(target[1], target[0]), gap, stop_at,
                                            self._limits)
                if v == 0.0 and w == 0.0:
                    break
                if not seen:
                    v, w = 0.0, 0.0
                reach = self._robot_radius + min(self._front_clearance, gap)
                if v > 0.0 and not self._clear_ahead(scan, t_base_odom, reach, position):
                    raise StepFailed('something other than the cone is in the way')
                self._step(v, w)
        finally:
            self.stop()
        scan = self._still_scan()   # measure once more, standing still
        near = self._cone_points(scan, position)
        if len(near) == 0:
            return gap, position
        pose = self._scan_pose(scan)
        t_base_odom = np.linalg.inv(geo.transform(pose.x, pose.y, pose.yaw))
        return geo.gap_to(geo.apply(t_base_odom, near), self._robot_radius), near.mean(axis=0)

    def _return_to_origin(self):
        """
        Drive back to the origin and face the start heading.

        Raise MissionAborted if it cannot: the rest of the mission starts from there.
        """
        o = self._origin
        self.get_logger().info('Going back to the origin...')
        try:
            pose = self._pose()
            if math.hypot(o.x - pose.x, o.y - pose.y) > self._origin_tolerance:
                self._turn_to(math.atan2(o.y - pose.y, o.x - pose.x))
                self._drive_to_origin()
            self._turn_to(o.yaw)
        except StepFailed as e:
            raise MissionAborted(f'could not get back to the origin: {e}')
        pose = self._pose()
        self.get_logger().info(
            f'Back at the origin ({math.hypot(o.x - pose.x, o.y - pose.y) * 100.0:.1f} cm '
            'from it, by odometry)')

    def _drive_to_origin(self):
        """
        Drive to the origin by odometry, stopping if anything is in the way.

        While driving forward, no LiDAR point may be within front_clearance in front of
        the robot (or before the origin, if that is closer).
        """
        o = self._origin
        deadline = self._now() + self._move_timeout
        try:
            while True:
                if self._now() > deadline:
                    raise StepFailed('took too long')
                scan = self._scan
                if self._now() - scan.arrival > self._data_timeout:
                    raise MissionAborted('LiDAR scans (scan) stopped arriving')
                pose = self._pose()
                t_base_odom = np.linalg.inv(geo.transform(pose.x, pose.y, pose.yaw))
                target = geo.apply(t_base_odom, [o.x, o.y])[0]
                distance = math.hypot(target[0], target[1])
                v, w = geo.approach_command(math.atan2(target[1], target[0]), distance,
                                            self._origin_tolerance, self._limits)
                if v == 0.0 and w == 0.0:
                    return
                reach = self._robot_radius + min(self._front_clearance, distance)
                if v > 0.0 and not self._clear_ahead(scan, t_base_odom, reach):
                    raise MissionAborted('something is in the way back to the origin')
                self._step(v, w)
        finally:
            self.stop()

    # ---------- the mission ----------
    def run(self):
        """Wait for the data, record the origin, then run Task 1 and Task 2."""
        if not self._wait_for_data():
            return
        scan, detections = self._scan, self._detections
        self.get_logger().info(
            'Sensor stamps behind the newest odometry: scan '
            f'{scan.lag if scan.lag is not None else math.nan:+.2f} s, detections '
            f'{detections.lag if detections.lag is not None else math.nan:+.2f} s (a few '
            "tenths of a second is normal; much more means the robot's clocks disagree)")
        if not self._hazard_seen:
            self.get_logger().warn(
                'No hazard_detection message yet: this node may not see bumps or cliffs. '
                'Check with: timeout 5 ros2 topic echo /hazard_detection --once')
        if self._start_delay > 0.0:
            self.get_logger().info(f'All data in. Starting in {self._start_delay:.0f} s '
                                   '(Ctrl-C to cancel)...')
            time.sleep(self._start_delay)
        try:
            self._mission_start = self._now()
            self._origin = self._pose()
            self.get_logger().info(
                f'Origin: x={self._origin.x:.2f} m, y={self._origin.y:.2f} m, '
                f'heading {math.degrees(self._origin.yaw):.0f} deg (odom frame)')
            cones = self._find_cones()
            if not cones:
                self.get_logger().error('No cones found; nothing to visit')
                return
            self._visit_all(cones)
        except MissionAborted as e:
            self.stop()
            self.get_logger().error(f'Mission stopped: {e}')


def raise_interrupt(signum, frame):
    """Turn SIGTERM and SIGHUP (launch shutdown, closed terminal) into Ctrl-C."""
    raise KeyboardInterrupt


def main(args=None):
    """
    Run the Cone Mission node.

    The ROS callbacks spin in a background thread and the mission runs here. On every exit
    (done, Ctrl-C, an error) the robot is stopped before ROS shuts down.
    """
    # without rclpy's own handler, Ctrl-C raises KeyboardInterrupt here while ROS is still
    # up, so the finally below can still send the stop command
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, raise_interrupt)
    signal.signal(signal.SIGHUP, raise_interrupt)
    try:
        node = ConeMission()
    except Exception as e:  # bad parameters
        get_logger('cone_mission').fatal(f'Cannot start: {e}')
        rclpy.try_shutdown()
        return
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
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
