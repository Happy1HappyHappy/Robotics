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
Camera geometry and drawing for cone detections.

Uses the pinhole camera model with pixel (i, j) centered at (i, j), the convention of
sensor_msgs/CameraInfo. Bearings follow REP 103: radians, 0 = straight ahead, positive =
to the left. No ROS imports, so it can be tested without the robot.
"""

import math

import cv2

BOX_COLOR = (255, 255, 255)     # BGR
OUTLINE_COLOR = (0, 0, 0)
CENTER_LINE_COLOR = (0, 255, 255)


def pixel_bearing(u, fx, cx):
    """
    Return the bearing (rad, + = left) of the ray through pixel column u.

    For a camera looking straight ahead along the robot's x axis, a point x to the right of
    the optical axis at depth z lands on column u = cx + fx x / z, so the ray is
    atan((cx - u) / fx) to the left of straight ahead. fx is the focal length and cx the
    column of the optical axis, both in pixels.
    """
    return math.atan2(cx - u, fx)


def focal_from_fov(width, hfov):
    """Return the focal length (px) of a camera whose image is width px across hfov rad."""
    return (width / 2.0) / math.tan(hfov / 2.0)


def camera_model(k, width, hfov):
    """
    Return the focal length and the optical axis column (px) of a camera.

    k is the row-major 3x3 camera matrix from camera_info and width the image width (px).
    If k holds no calibration (k[0] is not positive), they come from the horizontal field
    of view hfov (rad) instead.
    """
    if k[0] > 0.0:
        return float(k[0]), float(k[2])
    return focal_from_fov(width, hfov), (width - 1) / 2.0


def box_bearings(center_u, size_u, fx, cx):
    """
    Return the bearings (rad, + = left) of a box's center, left edge and right edge.

    center_u is the column of the box's center and size_u its width, both in pixels.
    """
    half = size_u / 2.0
    return (pixel_bearing(center_u, fx, cx),
            pixel_bearing(center_u - half, fx, cx),
            pixel_bearing(center_u + half, fx, cx))


def draw_cones(bgr, cones):
    """
    Return a copy of a BGR image with each cone's box and label drawn on it.

    cones holds objects with box edges x1, y1, x2, y2 (px), a color name (or None) and a
    confidence, such as the ConeBox results of YoloConeDetector. A yellow line marks the
    image's center column.
    """
    out = bgr.copy()
    height, width = out.shape[:2]
    center = width // 2
    cv2.line(out, (center, 0), (center, height - 1), CENTER_LINE_COLOR, 1)
    for cone in cones:
        top_left = (int(cone.x1), int(cone.y1))
        bottom_right = (int(cone.x2) - 1, int(cone.y2) - 1)
        cv2.rectangle(out, top_left, bottom_right, OUTLINE_COLOR, 3)
        cv2.rectangle(out, top_left, bottom_right, BOX_COLOR, 1)
        label = f'{cone.color or "unknown"} {cone.confidence:.2f}'
        org = (int(cone.x1), max(10, int(cone.y1) - 3))
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, OUTLINE_COLOR, 3,
                    cv2.LINE_AA)
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, BOX_COLOR, 1,
                    cv2.LINE_AA)
    return out
