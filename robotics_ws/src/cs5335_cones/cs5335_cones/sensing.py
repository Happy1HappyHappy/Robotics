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
Pure functions for the cone driver.

No ROS imports, so they can be tested without ROS or the robot. Angles are in radians,
counter-clockwise positive (REP 103).
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

# Starting HSV ranges, on OpenCV's scale: H 0-179 (degrees / 2), S and V 0-255. Each is
# [h_lo, s_lo, v_lo, h_hi, s_hi, v_hi]; h_lo > h_hi means the hue range wraps through 0,
# which red needs. Tune them under the lab's lights.
DEFAULT_HSV = {
    'red': [170, 120, 70, 4, 255, 255],
    'orange': [5, 120, 100, 18, 255, 255],
    'yellow': [20, 100, 100, 35, 255, 255],
    'green': [40, 80, 50, 85, 255, 255],
    'blue': [95, 120, 50, 130, 255, 255],
    'purple': [131, 60, 50, 155, 255, 255],
    'pink': [156, 60, 120, 169, 255, 255],
}

# An opening (erode, then dilate) with this kernel removes specks of noise from a mask
OPEN_KERNEL = np.ones((3, 3), np.uint8)


@dataclass
class Blob:
    """A region of one color in an image: its center column and its area, in pixels."""

    u: float
    area: float


def wrap_angle(angle):
    """Return angle (rad) wrapped to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    """
    Return the yaw (rotation about z, rad) of a quaternion.

    q is anything with x, y, z and w attributes, such as a geometry_msgs/Quaternion.
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def front_range(scan, lidar_yaw, half_width):
    """
    Return the nearest valid LiDAR range (m) within half_width rad of straight ahead.

    scan is anything with the fields of a sensor_msgs/LaserScan (ranges, angle_min,
    angle_increment, range_min, range_max). lidar_yaw is how far the LiDAR is turned on
    the robot: LiDAR angle a points to a + lidar_yaw in the robot's frame. Ranges outside
    [range_min, range_max], including inf and NaN, are skipped. If no valid range is in
    that direction, the result is math.inf.
    """
    nearest = math.inf
    for i, r in enumerate(scan.ranges):
        if not scan.range_min <= r <= scan.range_max:
            continue
        angle = wrap_angle(scan.angle_min + i * scan.angle_increment + lidar_yaw)
        if abs(angle) <= half_width:
            nearest = min(nearest, r)
    return nearest


def find_blob(bgr, hsv_range, min_area):
    """
    Return the largest region of one color in a BGR image as a Blob, or None.

    hsv_range is [h_lo, s_lo, v_lo, h_hi, s_hi, v_hi] on OpenCV's scale (see DEFAULT_HSV).
    Regions smaller than min_area pixels are ignored. A cone's white stripe may split it
    into two regions; the larger one is still in the cone's columns, which is all the
    direction needs.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h_lo, s_lo, v_lo, h_hi, s_hi, v_hi = (int(x) for x in hsv_range)
    if h_lo <= h_hi:
        mask = cv2.inRange(hsv, np.array([h_lo, s_lo, v_lo], np.uint8),
                           np.array([h_hi, s_hi, v_hi], np.uint8))
    else:   # wraps through 0: hues h_lo..179 and 0..h_hi
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([h_lo, s_lo, v_lo], np.uint8),
                        np.array([179, s_hi, v_hi], np.uint8)),
            cv2.inRange(hsv, np.array([0, s_lo, v_lo], np.uint8),
                        np.array([h_hi, s_hi, v_hi], np.uint8)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    moments = cv2.moments(largest)
    if moments['m00'] <= 0.0 or moments['m00'] < min_area:
        return None
    return Blob(u=moments['m10'] / moments['m00'], area=moments['m00'])
