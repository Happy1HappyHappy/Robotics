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
Cone driver node for Project 9.

Task 1: turn 360 degrees in place, find each colored cone in the camera image as the
robot turns, and log each cone's color, direction and distance. Task 2 lets the user
choose a cone and drives toward it using the camera and LiDAR.

The callbacks only store the latest data. Task 1 runs in the main thread and reads it,
while the executor spins in a background thread.
"""

from dataclasses import dataclass
import math
import threading
import time

from cs5335_cones.sensing import (DEFAULT_HSV, find_blob, front_range, nearest_ahead,
                                  wrap_angle, yaw_from_quaternion)
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, LaserScan

FULL_TURN = 2.0 * math.pi       # rad
CONTROL_PERIOD = 0.02           # s, the motion loops decide every 20 ms (50 Hz)
COMMAND_PERIOD = 0.05           # s, cmd_vel goes out every 50 ms (20 Hz); faster only loads WiFi
HAZARD_HOLD = 1.0               # s, a bump or cliff this recent stops the robot
MAX_LINEAR_SPEED = 0.15         # m/s (the TurtleBot 4 can do 0.31)
MAX_ANGULAR_SPEED = 0.8         # rad/s (the TurtleBot 4 can do 1.9)
TASK2_LINEAR_SPEED = 0.08       # m/s, keep the approach slow
TASK2_TURN_SPEED = 0.25         # rad/s, slow alignment and heading turns
TASK2_ALIGNMENT_DISTANCE = 1.0  # m, odometry distance before the camera realignment
RETURN_PATH_SAMPLE = 0.10       # m, record an odometry point this often on the way out
# rad, the heartbeat also logs the nearest LiDAR point this far either side of ahead, to
# show a cone that is close but just outside the narrow front window used for stopping
HEARTBEAT_HALF_WIDTH = math.radians(30.0)

# Hazards that stop the robot, with the name to log
STOP_HAZARDS = {
    HazardDetection.BUMP: 'bump',
    HazardDetection.CLIFF: 'cliff',
}


class Abort(Exception):
    """A motion had to stop early; the message says why."""


@dataclass
class Sighting:
    """Task 1's best view of one cone so far."""

    offset: float       # px, from the blob's center to the image's center column
    yaw: float          # rad, odom frame, which way the robot faced then
    distance: float     # m, the LiDAR's front range then (math.inf if no reading)


class ConeDriver(Node):
    """Find the colored cones with the camera, odometry and LiDAR (Task 1)."""

    def __init__(self):
        super().__init__('cone_driver')

        self._colors = list(self._declare(
            'colors', ['orange', 'green', 'blue'],
            'Colors of the three cones, from: ' + ', '.join(DEFAULT_HSV)))
        unknown = [c for c in self._colors if c not in DEFAULT_HSV]
        if unknown:
            raise ValueError(f'unknown color(s) {unknown}; choose from {list(DEFAULT_HSV)}')
        self._hsv = {}
        for color in self._colors:
            hsv = list(self._declare(
                f'hsv.{color}', DEFAULT_HSV[color],
                f'HSV range of {color}: [h_lo, s_lo, v_lo, h_hi, s_hi, v_hi], OpenCV scale '
                '(H 0-179, S and V 0-255); h_lo > h_hi wraps through 0'))
            if len(hsv) != 6:
                raise ValueError(f'hsv.{color} needs 6 numbers')
            self._hsv[color] = hsv
        self._min_area = self._declare(
            'min_area', 50, 'Smallest blob that can be a cone (px)')
        self._turn_speed = self._declare(
            'turn_speed', 0.3, 'Turn speed of the 360 degree turn (rad/s)', 0.05,
            MAX_ANGULAR_SPEED)
        self._lidar_yaw = self._declare(
            'lidar_yaw', math.pi / 2.0,
            'How far the LiDAR is turned on the robot, counter-clockwise (rad); from '
            'ros2 run tf2_ros tf2_echo base_link rplidar_link', -math.pi, math.pi)
        self._front_half_width = self._declare(
            'front_half_width', math.radians(5.0),
            'Half the width of the LiDAR window straight ahead (rad)', 0.01, 0.5)
        self._data_timeout = self._declare(
            'data_timeout', 0.5,
            'Hold still if odometry, LiDAR or camera data is older than this (s)', 0.1, 5.0)
        self._stale_wait = self._declare(
            'stale_wait', 2.0,
            'How long to hold still waiting for stale data to come back before giving up (s); '
            'a WiFi stall delays every topic at once for a moment', 0.0, 10.0)
        self._stop_range = self._declare(
            'stop_range', 0.20,
            'LiDAR range at which to stop before the cone (m); tune to leave about 1 inch',
            0.15, 0.35)
        self._center_tolerance = self._declare(
            'center_tolerance', 0.06,
            'Allow this fraction of the image half-width from center before driving',
            0.01, 0.25)

        self._bridge = CvBridge()

        # The latest data, and when it arrived (node clock). Each time stays None until
        # the first message. A callback sets the value first and the time second, so data
        # is never older than its time says.
        self._image = None          # BGR image, a NumPy array of rows x columns x 3
        self._image_time = None
        self._front = None          # m, nearest LiDAR range straight ahead (math.inf: none)
        self._wide = (math.inf, None)   # (m, rad) nearest point within HEARTBEAT_HALF_WIDTH
        self._front_time = None
        self._x = None               # m, odom frame
        self._y = None               # m, odom frame
        self._yaw = None            # rad, odom frame, counter-clockwise positive
        self._yaw_time = None
        self._origin = None         # (x, y) m, odom frame, captured before Task 1
        self._approach_path = []    # odometry points recorded while approaching the cone
        self._hazard = None         # name of the latest bump or cliff
        self._hazard_time = None

        # Counters for the low-rate health heartbeat. These show what this node has
        # received and published; they do not prove the robot controller received it.
        self._cmd_count = 0
        self._cmd_linear = None
        self._cmd_angular = None
        self._cmd_time = None
        self._target_linear = 0.0
        self._target_angular = 0.0
        self._motion_active = False
        self._command_lock = threading.Lock()

        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        # While moving, this timer is the only regular sender of cmd_vel: it keeps the
        # latest setpoint flowing at a steady rate, even while the main thread processes
        # camera images between control decisions.
        self._command_timer = self.create_timer(COMMAND_PERIOD, self._repeat_command)
        self.create_subscription(Image, 'oakd/rgb/preview/image_raw',
                                 self._image_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan',
                                 self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom',
                                 self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, 'hazard_detection',
                                 self._hazard_callback, qos_profile_sensor_data)
        self._health_timer = self.create_timer(1.0, self._health_callback)

    def _declare(self, name, default, description, low=None, high=None):
        """Declare a read-only parameter, with a range if low and high are given."""
        descriptor = ParameterDescriptor(description=description, read_only=True)
        if low is not None:
            descriptor.floating_point_range = [
                FloatingPointRange(from_value=low, to_value=high)]
        return self.declare_parameter(name, default, descriptor).value

    # ---------- callbacks (background thread): store the latest data ----------

    def _image_callback(self, msg):
        """Store the latest camera image as an OpenCV BGR array."""
        try:
            self._image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Cannot convert the image: {e}',
                                   throttle_duration_sec=5.0)
            return
        self._image_time = self.get_clock().now()

    def _scan_callback(self, msg):
        """Store the nearest LiDAR range straight ahead, and the nearest point for the log."""
        self._wide = nearest_ahead(msg, self._lidar_yaw, HEARTBEAT_HALF_WIDTH)
        self._front = front_range(msg, self._lidar_yaw, self._front_half_width)
        self._front_time = self.get_clock().now()

    def _odom_callback(self, msg):
        """Store the robot's latest planar position and yaw from odometry."""
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._yaw_time = self.get_clock().now()

    def _hazard_callback(self, msg):
        """Store the latest bump or cliff, so the robot can stop for it."""
        for detection in msg.detections:
            if detection.type in STOP_HAZARDS:
                self._hazard = STOP_HAZARDS[detection.type]
                self._hazard_time = self.get_clock().now()

    # ---------- helpers (main thread) ----------

    def _now(self):
        """Return the node clock's time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def _age(self, stamp):
        """Return how many seconds ago stamp (node clock) was, or None if it is None."""
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _health_callback(self):
        """Log sensor freshness and this node's cmd_vel publishing activity every second."""
        sensors = (
            ('image', self._image_time),
            ('scan', self._front_time),
            ('odom', self._yaw_time),
        )
        statuses = []
        stale = []
        for name, stamp in sensors:
            age = self._age(stamp)
            if age is None:
                statuses.append(f'{name}=missing')
                stale.append(name)
            else:
                statuses.append(f'{name}={age:.2f}s')
                if age > self._data_timeout:
                    stale.append(name)

        cmd_age = self._age(self._cmd_time)
        if cmd_age is None:
            command = f'cmd_vel=never published ({self._cmd_count} msgs/s)'
        else:
            command = (f'cmd_vel={self._cmd_linear:+.2f} m/s, '
                       f'{self._cmd_angular:+.2f} rad/s, age={cmd_age:.2f}s '
                       f'({self._cmd_count} msgs/s)')
        self._cmd_count = 0

        if self._front is None:
            lidar = 'lidar=no scan yet'
        else:
            # front is what the stop decision uses; nearest shows what else is close
            front = f'{self._front:.2f} m' if math.isfinite(self._front) else 'none'
            wide_range, wide_angle = self._wide
            if wide_angle is None:
                nearest = 'none'
            else:
                nearest = f'{wide_range:.2f} m at {math.degrees(wide_angle):+.0f} deg'
            lidar = (f'lidar front(+/-{math.degrees(self._front_half_width):.0f} deg)={front}, '
                     f'nearest(+/-{math.degrees(HEARTBEAT_HALF_WIDTH):.0f} deg)={nearest}, '
                     f'stop at {self._stop_range:.2f} m')

        message = 'Health: ' + ', '.join(statuses) + '; ' + lidar + '; ' + command
        if stale:
            self.get_logger().warn(message + '; unavailable/stale: ' + ', '.join(stale))
        else:
            self.get_logger().info(message)

    def _is_fresh(self, stamp):
        """Return whether data that arrived at stamp (node clock) is recent enough."""
        age = self._age(stamp)
        return age is not None and age <= self._data_timeout

    def _await_fresh(self, name, stamp_of):
        """
        Return once the data named name is fresh, holding the robot still while it is not.

        stamp_of returns when that data last arrived. A short WiFi stall delays every topic
        at once, so instead of aborting the task at once, stop and wait up to stale_wait for
        the data to come back. Raise Abort if it does not.
        """
        if self._is_fresh(stamp_of()):
            return
        self.stop()
        self.get_logger().warn(f'{name} is stale; holding still for up to '
                               f'{self._stale_wait:.1f} s for it to come back')
        deadline = self._now() + self._stale_wait
        while self._now() < deadline:
            time.sleep(CONTROL_PERIOD)
            if self._is_fresh(stamp_of()):
                self.get_logger().info(f'{name} is back; continuing')
                return
        raise Abort(f'{name} stopped arriving for over {self._stale_wait:.1f} s')

    def _fresh_yaw(self):
        """Return the latest yaw, waiting out a short odometry stall (see _await_fresh)."""
        self._await_fresh('odometry (odom)', lambda: self._yaw_time)
        return self._yaw

    def _fresh_front(self):
        """
        Return the LiDAR's front range, waiting out a short scan stall (see _await_fresh).

        The result is math.inf if no valid range is straight ahead in the latest scan.
        """
        self._await_fresh('LiDAR scans (scan)', lambda: self._front_time)
        return self._front

    def _front_if_fresh(self):
        """Return the LiDAR's front range without waiting, or math.inf if scans are stale."""
        return self._front if self._is_fresh(self._front_time) else math.inf

    def _fresh_image(self):
        """Return the latest camera image, waiting out a short image stall."""
        self._await_fresh('camera images', lambda: self._image_time)
        return self._image

    def _set_target(self, linear, angular):
        """Set the velocity setpoint (m/s forward, rad/s left), clamped to the limits."""
        self._target_linear = max(-MAX_LINEAR_SPEED,
                                  min(MAX_LINEAR_SPEED, float(linear)))
        self._target_angular = max(-MAX_ANGULAR_SPEED,
                                   min(MAX_ANGULAR_SPEED, float(angular)))

    def _publish(self):
        """Publish the current velocity setpoint on cmd_vel."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self._target_linear
        msg.twist.angular.z = self._target_angular
        self._cmd_pub.publish(msg)
        self._cmd_count += 1
        self._cmd_linear = msg.twist.linear.x
        self._cmd_angular = msg.twist.angular.z
        self._cmd_time = self.get_clock().now()

    def _repeat_command(self):
        """Keep active motion alive; stay quiet while stopped or awaiting user input."""
        with self._command_lock:
            if self._motion_active:
                self._publish()

    def _step(self, linear, angular):
        """
        Set the velocity setpoint and wait one control period.

        The command timer sends the setpoint; only the first command of a motion goes out
        here at once, so the robot starts without waiting for the timer. On a recent bump
        or cliff, or when odometry stays stale past stale_wait, raise Abort instead; the
        caller's finally stops the robot.
        """
        age = self._age(self._hazard_time)
        if age is not None and age < HAZARD_HOLD:
            raise Abort(f'{self._hazard} detected')
        self._fresh_yaw()
        with self._command_lock:
            starting = not self._motion_active
            self._set_target(linear, angular)
            self._motion_active = True
            if starting:
                self._publish()
        time.sleep(CONTROL_PERIOD)

    def stop(self):
        """Stop the robot: send a zero velocity a few times, in case one is lost."""
        with self._command_lock:
            self._motion_active = False
            self._set_target(0.0, 0.0)
            self._publish()
        for _ in range(2):
            self._publish()
            time.sleep(0.02)

    def _wait_for_data(self, timeout=30.0):
        """Wait until odometry, LiDAR scans and camera images arrive; return whether they did."""
        deadline = self._now() + timeout
        while self._now() < deadline:
            missing = [name for name, stamp in (('odometry (odom)', self._yaw_time),
                                                ('LiDAR scans (scan)', self._front_time),
                                                ('camera images', self._image_time))
                       if stamp is None]
            if not missing:
                rows, columns = self._image.shape[:2]
                self.get_logger().info(
                    f'Data arriving: image {columns}x{rows}, front range '
                    f'{self._front:.2f} m, yaw {math.degrees(self._yaw):+.1f} deg')
                return True
            self.get_logger().info('Waiting for ' + ', '.join(missing),
                                   throttle_duration_sec=3.0)
            time.sleep(0.2)
        self.get_logger().error(f'Still missing after {timeout:.0f} s: ' + ', '.join(missing)
                                + '. Is the robot connected? Check with ros2 topic list')
        return False

    # ---------- Task 1 ----------

    def find_cones(self):
        """
        Turn once around in place and return each color's best sighting.

        A color's best sighting is the image in which its blob was closest to the image's
        center column, so the robot was facing that cone then.
        """
        self.get_logger().info('Task 1: turning 360 degrees to look for the cones')
        sightings = {}
        deadline = self._now() + FULL_TURN / self._turn_speed * 2.0 + 10.0
        last_yaw = self._fresh_yaw()
        turned = 0.0
        last_image_time = None
        try:
            while turned < FULL_TURN:
                if self._now() > deadline:
                    raise Abort('the turn took too long; is the robot stuck?')
                self._step(0.0, self._turn_speed)
                yaw = self._fresh_yaw()
                turned += wrap_angle(yaw - last_yaw)
                last_yaw = yaw
                # each image gets a new time object; `is not` also works while one is
                # None (comparing an rclpy Time with None raises TypeError)
                if self._image_time is not last_image_time:
                    last_image_time = self._image_time
                    self._look(self._image, yaw, sightings)
        finally:
            self.stop()
        return sightings

    def _look(self, image, yaw, sightings):
        """Update sightings with each color's blob in one camera image."""
        center = (image.shape[1] - 1) / 2.0
        # Task 1 only logs this distance, so do not hold the turn still for it
        distance = self._front_if_fresh()
        for color in self._colors:
            blob = find_blob(image, self._hsv[color], self._min_area)
            if blob is None:
                continue
            offset = abs(blob.u - center)
            if color in sightings and sightings[color].offset <= offset:
                continue
            if color not in sightings:
                self.get_logger().info(f'  saw {color}')
            sightings[color] = Sighting(offset, yaw, distance)

    def _report(self, sightings):
        """Log each color's direction from where the robot faces now, and its distance."""
        yaw = self._yaw
        self.get_logger().info('Cones found (direction: 0 = ahead, + = left; distance '
                               'from the LiDAR):')
        for color in self._colors:
            s = sightings.get(color)
            if s is None:
                self.get_logger().info(f'  {color:<8} not found')
                continue
            direction = math.degrees(wrap_angle(s.yaw - yaw))
            distance = f'{s.distance:.2f} m' if math.isfinite(s.distance) else 'no LiDAR reading'
            self.get_logger().info(f'  {color:<8} {direction:+5.0f} deg  {distance}')

    # ---------- Task 2 ----------

    def _turn_to(self, target_yaw):
        """Turn in place toward a cone's remembered odometry heading."""
        deadline = self._now() + 30.0
        try:
            while True:
                error = wrap_angle(target_yaw - self._fresh_yaw())
                if abs(error) < math.radians(3.0):
                    return
                if self._now() > deadline:
                    raise Abort('timed out turning toward the cone')
                angular = math.copysign(min(TASK2_TURN_SPEED, max(0.10, abs(error))), error)
                self._step(0.0, angular)
        finally:
            self.stop()

    def _record_approach_position(self, force=False):
        """Save the current odometry position when far enough from the last path point."""
        if not self._approach_path:
            self._approach_path.append((self._x, self._y))
            return
        last_x, last_y = self._approach_path[-1]
        distance = math.hypot(self._x - last_x, self._y - last_y)
        if distance >= RETURN_PATH_SAMPLE or (force and distance > 0.01):
            self._approach_path.append((self._x, self._y))

    def drive_to_cone(self, color, sighting):
        """Drive 1 m by odometry, realign once, then approach to the LiDAR stop range."""
        self.get_logger().info(f'Task 2: approaching {color}; target gap about 1 inch')
        self._turn_to(sighting.yaw)
        deadline = self._now() + 60.0
        start_x, start_y = self._x, self._y
        self._approach_path = [(start_x, start_y)]
        try:
            # Drive the first meter along the remembered cone heading. Odometry holds
            # that heading; the camera is used for the planned realignment at 1 m.
            while self._now() <= deadline:
                front = self._fresh_front()
                if front <= self._stop_range:
                    self._record_approach_position(force=True)
                    self.get_logger().info(
                        f'Reached {color}: front range {front:.2f} m (stop threshold '
                        f'{self._stop_range:.2f} m)')
                    return
                if not math.isfinite(front):
                    raise Abort('no valid LiDAR range ahead; refusing to drive')
                distance = math.hypot(self._x - start_x, self._y - start_y)
                if distance >= TASK2_ALIGNMENT_DISTANCE:
                    break
                heading_error = wrap_angle(sighting.yaw - self._fresh_yaw())
                angular = max(-0.15, min(0.15, heading_error * 1.5))
                self._step(TASK2_LINEAR_SPEED, angular)
                self._record_approach_position()
            else:
                raise Abort(f'timed out driving the first {TASK2_ALIGNMENT_DISTANCE:.1f} m')

            self.stop()
            self.get_logger().info(
                f'Drove {TASK2_ALIGNMENT_DISTANCE:.1f} m by odometry; realigning with camera')
            # Stop and use the camera to center the cone once. If the cone is not yet
            # visible, rotate slowly until it appears; abort rather than drive blind.
            align_deadline = min(deadline, self._now() + 20.0)
            while self._now() <= align_deadline:
                image = self._fresh_image()
                blob = find_blob(image, self._hsv[color], self._min_area)
                if blob is None:
                    self._step(0.0, TASK2_TURN_SPEED)
                    continue
                center = (image.shape[1] - 1) / 2.0
                error = (blob.u - center) / max(1.0, center)
                if abs(error) <= self._center_tolerance:
                    break
                angular = max(-TASK2_TURN_SPEED,
                              min(TASK2_TURN_SPEED, -error * TASK2_TURN_SPEED))
                self._step(0.0, angular)
            else:
                raise Abort(f'timed out realigning with {color}')
            self.stop()

            # After this one camera correction, keep the corrected odometry heading
            # and use LiDAR for the stopping decision.
            approach_yaw = self._fresh_yaw()
            while self._now() <= deadline:
                front = self._fresh_front()
                if front <= self._stop_range:
                    self._record_approach_position(force=True)
                    self.get_logger().info(
                        f'Reached {color}: front range {front:.2f} m (stop threshold '
                        f'{self._stop_range:.2f} m)')
                    return
                if not math.isfinite(front):
                    raise Abort('no valid LiDAR range ahead; refusing to drive')
                heading_error = wrap_angle(approach_yaw - self._fresh_yaw())
                angular = max(-0.15, min(0.15, heading_error * 1.5))
                self._step(TASK2_LINEAR_SPEED, angular)
                self._record_approach_position()
            raise Abort(f'timed out approaching {color}')
        finally:
            self.stop()

    def return_to_origin(self):
        """Follow the recorded approach path in reverse back to the starting position."""
        if self._origin is None:
            raise Abort('starting position was not recorded')
        self._record_approach_position(force=True)
        path = list(self._approach_path)
        if len(path) < 2:
            distance = math.hypot(self._origin[0] - self._x, self._origin[1] - self._y)
            if distance <= 0.08:
                self.get_logger().info('Back at the starting position')
                return
            path = [self._origin, (self._x, self._y)]
        # The route should finish at the position captured before the initial scan.
        path[0] = self._origin
        waypoint = len(path) - 2
        self.get_logger().info('Following the recorded path back to the starting position')
        deadline = self._now() + 60.0
        try:
            # Turn around once to face the last recorded part of the outbound path.
            first_x, first_y = path[waypoint]
            bearing = math.atan2(first_y - self._y, first_x - self._x)
            self._turn_to(bearing)
            while self._now() <= deadline and waypoint >= 0:
                self._fresh_yaw()
                target_x, target_y = path[waypoint]
                dx = target_x - self._x
                dy = target_y - self._y
                distance = math.hypot(dx, dy)
                if distance <= 0.08:
                    waypoint -= 1
                    continue
                front = self._fresh_front()
                if not math.isfinite(front) or front < 0.25:
                    raise Abort('blocked or invalid LiDAR range on the return path')
                bearing = math.atan2(dy, dx)
                heading_error = wrap_angle(bearing - self._yaw)
                # Keep moving between recorded points; slow down and steer more on
                # bends instead of stopping to turn at every waypoint.
                speed = min(TASK2_LINEAR_SPEED, max(0.03, distance * 0.5))
                if abs(heading_error) > math.radians(35.0):
                    speed = min(speed, 0.04)
                angular = max(-TASK2_TURN_SPEED,
                              min(TASK2_TURN_SPEED, heading_error * 1.5))
                self._step(speed, angular)
            if waypoint < 0:
                self.get_logger().info('Back at the starting position')
                return
            raise Abort('timed out returning along the recorded path')
        finally:
            self.stop()

    def choose_and_approach(self, sightings):
        """Ask for cone colors repeatedly; q exits the selection loop."""
        while True:
            try:
                choice = input(f'Choose a cone color ({", ".join(self._colors)}) or q to quit: ')
            except EOFError:
                return
            color = choice.strip().lower()
            if color in ('q', 'quit', 'exit'):
                self.get_logger().info('Leaving Task 2')
                return
            if color not in self._colors:
                self.get_logger().warn(f'Unknown color {color!r}; choose from {self._colors}')
                continue
            sighting = sightings.get(color)
            if sighting is None:
                self.get_logger().warn(f'{color} was not detected during Task 1')
                continue
            try:
                self.drive_to_cone(color, sighting)
            except Abort as e:
                self.get_logger().error(f'Approach stopped: {e}')
                return
            try:
                self.return_to_origin()
            except Abort as e:
                self.get_logger().error(f'Return stopped: {e}; not asking for another cone')
                return

    def run(self):
        """Wait for the robot's data, then run both tasks."""
        if not self._wait_for_data():
            return
        self._origin = (self._x, self._y)
        self.get_logger().info(f'Starting position recorded: x={self._x:.2f} m, '
                               f'y={self._y:.2f} m (odom)')
        try:
            sightings = self.find_cones()
        except Abort as e:
            self.get_logger().error(f'Task 1 stopped: {e}')
            return
        self._report(sightings)
        self.choose_and_approach(sightings)


def main(args=None):
    """Run Task 1, stopping the robot on every exit."""
    # Without rclpy's own signal handler, Ctrl-C raises KeyboardInterrupt in the main
    # thread while ROS is still up, so the finally below can still send the stop command
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = ConeDriver()
    except ValueError as e:
        get_logger('cone_driver').error(f'Bad parameter: {e}')
        rclpy.try_shutdown()
        return
    # The callbacks run in this thread, so they keep storing data while node.run() blocks
    # (see "Sync vs async" in the ROS 2 docs)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        # Let spin() return before the node and ROS are torn down; tearing them down under
        # a still-spinning thread aborts the process ("terminate called without an active
        # exception")
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
