"""
Geometry and motion commands for driving to a cone (Project 9)
Pure NumPy (no ROS), so it can be tested without the robot
Frames follow REP 103/105: x forward, y left, angles counter-clockwise in radians, lengths in
meters; base is the robot (base_link, at its center), odom the odometry frame. A 3x3 matrix
^A T_B maps points of frame B into frame A
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class MotionLimits:
    """
    Speeds and gains of the turn and approach commands
    """
    max_linear: float = 0.15    # m/s
    min_linear: float = 0.02    # m/s, slower and the robot may not move
    linear_gain: float = 0.5    # 1/s, forward speed per meter still to go
    max_angular: float = 0.8    # rad/s
    min_angular: float = 0.1    # rad/s, slower and the robot may not turn
    angular_gain: float = 1.5   # 1/s, turn speed per radian of error
    turn_first: float = 0.35    # rad, turn in place before driving if the target is farther off


def wrap_angle(a):
    """
    Wrap an angle to [-pi, pi)
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


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


def apply(T, points):
    """
    Points (N x 2) moved by the 3x3 transform T
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return points @ T[:2, :2].T + T[:2, 2]


def scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    """
    The valid LiDAR returns of a scan as points (N x 2) in the LiDAR's own frame
    Returns outside [range_min, range_max] (including inf and NaN) are dropped
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])


def nearest_cluster(points, bearing, half_width, depth):
    """
    The points (N x 2, base frame) of the nearest object in a direction: of the points within
    half_width rad of bearing, the nearest one and every other one at most depth m farther
    Empty (0 x 2) if no point is in that direction
    With the cones on open floor, the nearest thing where the camera sees a cone is the cone,
    and the depth limit keeps the wall behind it out
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
    Gap (m) between the robot's edge and the nearest of points (N x 2, base frame)
    The TurtleBot 4 is round, so this is the distance from its center minus its radius,
    in whatever direction the points are
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return float(np.hypot(points[:, 0], points[:, 1]).min()) - robot_radius


def turn_command(error, limits):
    """
    Turn speed (rad/s, + = left) to reduce a heading error (rad, + = target to the left)
    Proportional, at most max_angular, at least min_angular so it does not stall
    The caller stops once the error is small enough
    """
    w = max(-limits.max_angular, min(limits.max_angular, limits.angular_gain * error))
    if abs(w) < limits.min_angular:
        w = math.copysign(limits.min_angular, error)
    return w


def approach_command(bearing, gap, stop_gap, limits):
    """
    Velocity (v m/s, w rad/s) toward a target bearing rad away (+ = left) whose nearest point
    is gap m from the robot's edge; (0, 0) once gap <= stop_gap
    Turns in place while the target is more than turn_first rad off to the side; otherwise
    drives and steers at once, slowing in proportion to the distance still to go (times
    cos(bearing), so it slows while it steers), but not below min_linear so it does not stall
    """
    if gap <= stop_gap:
        return 0.0, 0.0
    w = max(-limits.max_angular, min(limits.max_angular, limits.angular_gain * bearing))
    if abs(bearing) > limits.turn_first:
        return 0.0, turn_command(bearing, limits)
    v = limits.linear_gain * (gap - stop_gap) * math.cos(bearing)
    v = max(limits.min_linear, min(limits.max_linear, v))
    return v, w
