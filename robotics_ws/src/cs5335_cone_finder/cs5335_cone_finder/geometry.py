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
Plane geometry and velocity commands for driving to cones and back.

Pure NumPy (no ROS), so it can be tested without the robot. Frames follow REP 103 and
REP 105: x forward, y left, angles counter-clockwise in radians, lengths in meters. The
base frame is the robot (base_link, at its center) and odom the odometry frame. A 3x3
matrix t_a_b maps points of frame b into frame a.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class MotionLimits:
    """
    Speeds and gains of the turn and drive commands.

    max_linear and min_linear are in m/s, max_angular and min_angular in rad/s, the gains in
    1/s, and turn_first in rad: a target farther off to the side than that makes the robot
    turn in place before it drives.
    """

    max_linear: float = 0.15
    min_linear: float = 0.02
    linear_gain: float = 0.5
    max_angular: float = 0.8
    min_angular: float = 0.1
    angular_gain: float = 1.5
    turn_first: float = 0.35


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

    Returns outside [range_min, range_max], including inf and NaN, are dropped.
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])


def nearest_cluster(points, bearing, half_width, depth):
    """
    Return the points (N x 2, base frame) of the nearest object in a direction.

    Of the points within half_width rad of bearing, these are the nearest one and every
    other one at most depth m farther. The result is empty (0 x 2) if no point is in that
    direction. With the cones on open floor, the nearest thing where the camera sees a cone
    is the cone, and the depth limit keeps the wall behind it out.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    off = np.abs((np.arctan2(points[:, 1], points[:, 0]) - bearing + np.pi)
                 % (2.0 * np.pi) - np.pi)
    inside = points[off <= half_width]
    if len(inside) == 0:
        return inside
    r = np.hypot(inside[:, 0], inside[:, 1])
    return inside[r <= r.min() + depth]


def gap_to(points, robot_radius):
    """
    Return the gap (m) between the robot's edge and the nearest of points (base frame).

    The TurtleBot 4 is round, so this is the distance from its center minus its radius, in
    whatever direction the points are.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return float(np.hypot(points[:, 0], points[:, 1]).min()) - robot_radius


def turn_command(error, limits):
    """
    Return the turn speed (rad/s, + = left) that reduces a heading error (rad, + = left).

    Proportional, at most max_angular, and at least min_angular so the robot does not stall.
    The caller stops once the error is small enough.
    """
    w = max(-limits.max_angular, min(limits.max_angular, limits.angular_gain * error))
    if abs(w) < limits.min_angular:
        w = math.copysign(limits.min_angular, error)
    return w


def approach_command(bearing, gap, stop_gap, limits):
    """
    Return the velocity (v m/s, w rad/s) toward a target bearing rad away (+ = left).

    gap is the distance (m) still to go; the result is (0, 0) once gap <= stop_gap. The
    robot turns in place while the target is more than turn_first rad off to the side.
    Otherwise it drives and steers at once, slowing in proportion to the distance still to
    go (times cos(bearing), so it slows while it steers), but not below min_linear so it
    does not stall.
    """
    if gap <= stop_gap:
        return 0.0, 0.0
    if abs(bearing) > limits.turn_first:
        return 0.0, turn_command(bearing, limits)
    w = max(-limits.max_angular, min(limits.max_angular, limits.angular_gain * bearing))
    v = limits.linear_gain * (gap - stop_gap) * math.cos(bearing)
    v = max(limits.min_linear, min(limits.max_linear, v))
    return v, w


def path_clear(points, reach, half_width):
    """
    Return whether none of points (N x 2, base frame) is in the strip ahead of the robot.

    The strip starts at the robot's center and runs reach m forward, half_width m to each
    side.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    ahead = ((points[:, 0] > 0.0) & (points[:, 0] < reach)
             & (np.abs(points[:, 1]) < half_width))
    return not bool(ahead.any())
