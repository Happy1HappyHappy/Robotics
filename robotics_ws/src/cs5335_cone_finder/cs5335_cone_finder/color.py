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
Name the color of a traffic cone from the pixels inside its box.

YOLO says where a cone is but not what color it is. The color is the hue range that most
of the strongly colored pixels in the middle of the box fall in. Hue is used rather than
RGB because it stays about the same in light and shadow. OpenCV's hue runs 0-179
(degrees / 2), so red, at both ends, has a range that wraps through 0. No ROS imports.
"""

from dataclasses import dataclass, field
import math

import cv2
import numpy as np

# Starting hue ranges (low, high) on OpenCV's scale; low > high wraps through 0 (red).
# Tune them under the lab's lights.
DEFAULT_HUE_RANGES = {
    'red': (170, 4),
    'orange': (5, 18),
    'yellow': (20, 35),
    'green': (40, 85),
    'blue': (95, 130),
    'purple': (131, 155),
    'pink': (156, 169),
}


@dataclass
class ColorSettings:
    """
    How cone colors are told apart.

    hue_ranges maps each color name to its hue range (low, high) on OpenCV's scale; low >
    high wraps through 0. A pixel counts toward a color only if its saturation is at least
    min_saturation and its value at least min_value (0-255), which leaves out the gray
    floor, white walls and the cone's white stripe. A color is named only if at least
    min_fraction of the pixels looked at have it. region is the fraction of the box's width
    and height, centered, whose pixels are looked at.
    """

    hue_ranges: dict = field(default_factory=lambda: dict(DEFAULT_HUE_RANGES))
    min_saturation: int = 80
    min_value: int = 60
    min_fraction: float = 0.15
    region: float = 0.6


def hue_in_range(hue, low, high):
    """
    Return which hues (an array, OpenCV scale) are within [low, high].

    low > high means the range wraps through 0, as red's does.
    """
    hue = np.asarray(hue)
    if low <= high:
        return (hue >= low) & (hue <= high)
    return (hue >= low) | (hue <= high)


def inner_region(box, region, width, height):
    """
    Return the centered part of a box as integer pixel bounds (left, top, right, bottom).

    box is (x1, y1, x2, y2) in pixels. The result covers the fraction region of the box's
    width and height, clipped to an image width x height px; right and bottom are one past
    the last pixel, so the region may be empty.
    """
    x1, y1, x2, y2 = box
    margin_x = (x2 - x1) * (1.0 - region) / 2.0
    margin_y = (y2 - y1) * (1.0 - region) / 2.0
    left = max(0, int(math.floor(x1 + margin_x)))
    top = max(0, int(math.floor(y1 + margin_y)))
    right = min(width, max(left, int(math.ceil(x2 - margin_x))))
    bottom = min(height, max(top, int(math.ceil(y2 - margin_y))))
    return left, top, right, bottom


def classify_color(bgr, settings):
    """
    Return the color name of a BGR image patch and the fraction of its pixels that have it.

    The name is None if no color reaches settings.min_fraction (or the patch is empty).
    """
    if bgr.size == 0:
        return None, 0.0
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hue, saturation, value = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    strong = (saturation >= settings.min_saturation) & (value >= settings.min_value)
    best, best_count = None, 0
    for name, (low, high) in settings.hue_ranges.items():
        count = int(np.count_nonzero(strong & hue_in_range(hue, low, high)))
        if count > best_count:
            best, best_count = name, count
    fraction = best_count / hue.size
    if best is None or fraction < settings.min_fraction:
        return None, fraction
    return best, fraction
