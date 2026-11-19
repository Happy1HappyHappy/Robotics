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
Camera and LiDAR fusion: which LiDAR points belong to the cone the camera sees.

The camera gives a ray from the camera through the cone's centroid, and the cone's
angular width; the LiDAR gives points. The two sensors sit at different places on the
robot, so both are put in one frame (with their mounting transforms) and the LiDAR points
are compared with the ray as seen from the camera's position. If no LiDAR point lies along
the ray, the cone's distance is unavailable; nothing is made up. No ROS imports.
"""

import math

from cs5335_cone_navigation.control import wrap_angle
import numpy as np


def pixel_bearing(u, fx, cx):
    """
    Return the bearing (rad, + = left, REP 103) of the ray through pixel column u.

    Pinhole model: a point x to the right of the optical axis at depth z lands on column
    u = cx + fx x / z, so the ray is atan2(u - cx, fx) to the right, which is
    atan2(cx - u, fx) to the left. fx and cx are the focal length and the optical axis
    column (px) from camera_info.
    """
    return math.atan2(cx - u, fx)


def camera_model(k, width, hfov):
    """
    Return the focal length fx and optical axis column cx (px) of a camera.

    k is the row-major 3x3 camera matrix from camera_info and width the image width (px).
    If k holds no calibration (k[0] is not positive), they come from the horizontal field
    of view hfov (rad) instead.
    """
    if k[0] > 0.0:
        return float(k[0]), float(k[2])
    return (width / 2.0) / math.tan(hfov / 2.0), (width - 1) / 2.0


def facing_yaw(q, optical):
    """
    Return the direction (rad, yaw) a sensor faces, from its orientation quaternion.

    An optical frame (z forward, x right, y down, used for camera images) faces along its
    z axis; any other frame faces along its x axis (REP 103). q has fields x, y, z and w.
    """
    if optical:
        ax, ay = 2.0 * (q.x * q.z + q.w * q.y), 2.0 * (q.y * q.z - q.w * q.x)
    else:
        ax, ay = 1.0 - 2.0 * (q.y * q.y + q.z * q.z), 2.0 * (q.x * q.y + q.w * q.z)
    return math.atan2(ay, ax)


def angular_half_width(center_u, size_u, fx, cx):
    """Return half the angle (rad) a box covers, from its center column and width (px)."""
    half = size_u / 2.0
    return abs(pixel_bearing(center_u - half, fx, cx)
               - pixel_bearing(center_u + half, fx, cx)) / 2.0


def cone_points(points, origin, bearing, half_width, depth, max_range):
    """
    Return the LiDAR points (N x 2) of the nearest object along a camera ray.

    points and origin (the camera's position) are in one frame, and bearing is the ray's
    direction in that frame. Of the points within half_width rad of the ray as seen from
    origin, these are the nearest one and every other one at most depth m farther; several
    objects in the same direction are told apart this way, and the nearest is the one the
    camera sees. The result is empty (0 x 2) if no point lies along the ray within
    max_range m.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    rel = points - np.asarray(origin, dtype=float)
    off = np.abs((np.arctan2(rel[:, 1], rel[:, 0]) - bearing + np.pi) % (2.0 * np.pi)
                 - np.pi)
    inside = off <= half_width
    if not inside.any():
        return np.zeros((0, 2))
    r = np.hypot(rel[inside, 0], rel[inside, 1])
    if r.min() > max_range:
        return np.zeros((0, 2))
    return points[inside][r <= r.min() + depth]


def ray_in_frame(t_frame_base, camera_mount, bearing_in_camera):
    """
    Return a camera ray's origin (x, y) and direction (rad) in another frame.

    t_frame_base is the robot's pose in that frame (3x3), camera_mount the camera's
    (x, y, yaw) on the robot, and bearing_in_camera the ray's bearing from the camera's
    forward direction.
    """
    x, y, yaw = camera_mount
    origin = t_frame_base[:2, :2] @ np.array([x, y]) + t_frame_base[:2, 2]
    robot_yaw = math.atan2(t_frame_base[1, 0], t_frame_base[0, 0])
    return origin, wrap_angle(robot_yaw + yaw + bearing_in_camera)
