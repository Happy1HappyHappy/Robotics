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
robot turns, and log each cone's color, direction and distance. Task 2 is added later.

The callbacks only store the latest data. Task 1 runs in the main thread and reads it,
while the executor spins in a background thread.
"""

from dataclasses import dataclass
import math
import threading
import time

from cs5335_cones.sensing import (DEFAULT_HSV, find_blob, front_range, wrap_angle,
                                  yaw_from_quaternion)
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
CONTROL_PERIOD = 0.05           # s, one velocity command every 50 ms (20 Hz)
HAZARD_HOLD = 1.0               # s, a bump or cliff this recent stops the robot
MAX_LINEAR_SPEED = 0.15         # m/s (the TurtleBot 4 can do 0.31)
MAX_ANGULAR_SPEED = 0.8         # rad/s (the TurtleBot 4 can do 1.9)

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
            'data_timeout', 0.5, 'Stop if odometry is older than this (s)', 0.1, 5.0)

        self._bridge = CvBridge()

        # The latest data, and when it arrived (node clock). Each time stays None until
        # the first message. A callback sets the value first and the time second, so data
        # is never older than its time says.
        self._image = None          # BGR image, a NumPy array of rows x columns x 3
        self._image_time = None
        self._front = None          # m, nearest LiDAR range straight ahead (math.inf: none)
        self._front_time = None
        self._yaw = None            # rad, odom frame, counter-clockwise positive
        self._yaw_time = None
        self._hazard = None         # name of the latest bump or cliff
        self._hazard_time = None

        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.create_subscription(Image, 'oakd/rgb/preview/image_raw',
                                 self._image_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan',
                                 self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom',
                                 self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, 'hazard_detection',
                                 self._hazard_callback, qos_profile_sensor_data)

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
        """Store the nearest LiDAR range straight ahead."""
        self._front = front_range(msg, self._lidar_yaw, self._front_half_width)
        self._front_time = self.get_clock().now()

    def _odom_callback(self, msg):
        """Store the robot's latest yaw from odometry."""
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

    def _fresh_yaw(self):
        """Return the latest yaw; raise Abort if odometry stopped arriving."""
        age = self._age(self._yaw_time)
        if age is None or age > self._data_timeout:
            raise Abort('odometry (odom) stopped arriving')
        return self._yaw

    def _fresh_front(self):
        """Return the LiDAR's front range, or math.inf if scans stopped arriving."""
        age = self._age(self._front_time)
        if age is None or age > self._data_timeout:
            return math.inf
        return self._front

    def _publish(self, linear, angular):
        """Publish a velocity command (m/s forward, rad/s left), clamped to the limits."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, float(linear)))
        msg.twist.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, float(angular)))
        self._cmd_pub.publish(msg)

    def _step(self, linear, angular):
        """
        Send one velocity command and wait one control period.

        On a recent bump or cliff, or when odometry stopped arriving, raise Abort instead;
        the caller's finally stops the robot.
        """
        age = self._age(self._hazard_time)
        if age is not None and age < HAZARD_HOLD:
            raise Abort(f'{self._hazard} detected')
        self._fresh_yaw()
        self._publish(linear, angular)
        time.sleep(CONTROL_PERIOD)

    def stop(self):
        """Stop the robot: send a zero velocity a few times, in case one is lost."""
        for _ in range(3):
            self._publish(0.0, 0.0)
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
        distance = self._fresh_front()
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

    def run(self):
        """Wait for the robot's data, then do Task 1."""
        if not self._wait_for_data():
            return
        try:
            sightings = self.find_cones()
        except Abort as e:
            self.get_logger().error(f'Task 1 stopped: {e}')
            return
        self._report(sightings)


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
