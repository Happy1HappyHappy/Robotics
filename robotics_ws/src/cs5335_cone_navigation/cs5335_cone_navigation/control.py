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
Plane geometry and proportional velocity commands.

Frames follow REP 103 and REP 105: x forward, y left, angles counter-clockwise in radians,
lengths in meters; base is the robot (base_link) and odom the odometry frame. A 3x3 matrix
t_a_b maps points of frame b into frame a. No ROS imports.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class MotionLimits:
    """
    Gains and limits of the ALIGN and APPROACH controllers.

    kp_angular (1/s) turns a heading error (rad) into a turn speed and kd_linear (1/s) a
    distance error (m) into a forward speed. Speeds stay within max_linear (m/s) and
    max_angular (rad/s), and above min_linear and min_angular so the robot does not stall.
    A target more than heading_limit (rad) off to the side makes the robot turn in place
    before driving.
    """

    max_linear: float = 0.15
    min_linear: float = 0.02
    kd_linear: float = 0.5
    max_angular: float = 0.3
    min_angular: float = 0.08
    kp_angular: float = 1.0
    heading_limit: float = 0.35


def wrap_angle(angle):
    """Wrap an angle (rad) to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    """Return the yaw (rad, rotation about z) of a quaternion with fields x, y, z and w."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform(x, y, yaw):
    """Return the homogeneous 3x3 matrix of a 2D pose (m, m, rad)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def apply(t, points):
    """Return points (N x 2) moved by the 3x3 transform t."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return points @ t[:2, :2].T + t[:2, 2]


def scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    """
    Return the valid returns of a LiDAR scan as points (N x 2) in the LiDAR's frame.

    Returns outside [range_min, range_max], including inf and NaN, are invalid and dropped.
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])


def gap_to(points, robot_radius):
    """
    Return the gap (m) between the robot's edge and the nearest of points (base frame).

    The TurtleBot 4 is round, so this is the distance from its center minus its radius.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return float(np.hypot(points[:, 0], points[:, 1]).min()) - robot_radius


def turn_command(error, limits):
    """
    Return the turn speed (rad/s, + = left) for a heading error (rad, + = left).

    angular_velocity = kp_angular * error, clamped to max_angular and at least min_angular.
    """
    w = max(-limits.max_angular, min(limits.max_angular, limits.kp_angular * error))
    if abs(w) < limits.min_angular:
        w = math.copysign(limits.min_angular, error)
    return w


def approach_command(bearing, gap, stop_gap, limits):
    """
    Return the velocity (v m/s, w rad/s) toward a target bearing rad away (+ = left).

    gap is the distance (m) to the target's near side; the result is (0, 0) once gap <=
    stop_gap. If the target is more than heading_limit off to the side, the robot turns in
    place. Otherwise linear_velocity = kd_linear * (gap - stop_gap), within [min_linear,
    max_linear], so it slows down near the target, while it steers with kp_angular.
    """
    if gap <= stop_gap:
        return 0.0, 0.0
    if abs(bearing) > limits.heading_limit:
        return 0.0, turn_command(bearing, limits)
    v = max(limits.min_linear, min(limits.max_linear, limits.kd_linear * (gap - stop_gap)))
    w = max(-limits.max_angular, min(limits.max_angular, limits.kp_angular * bearing))
    return v, w


def path_clear(points, reach, half_width):
    """
    Return whether none of points (N x 2, base frame) is in the strip ahead of the robot.

    The strip runs from the robot's center reach m forward and half_width m to each side.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    ahead = ((points[:, 0] > 0.0) & (points[:, 0] < reach)
             & (np.abs(points[:, 1]) < half_width))
    return not bool(ahead.any())
