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

"""Tests for cs5335_cones.sensing, on made-up quaternions, scans and images."""

import math
from types import SimpleNamespace

from cs5335_cones.sensing import (DEFAULT_HSV, find_blob, front_range, front_window,
                                  is_narrow, wrap_angle, yaw_from_quaternion)
import cv2
import numpy as np
import pytest

# BGR colors that fall in the default HSV ranges
ORANGE = (0, 128, 255)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
GRAY = (128, 128, 128)


def quaternion_from_yaw(yaw):
    """Return a quaternion (x, y, z, w attributes) for a rotation of yaw rad about z."""
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def make_scan(ranges):
    """Return a LaserScan-like object with one range per degree, starting at -pi."""
    return SimpleNamespace(ranges=ranges, angle_min=-math.pi,
                           angle_increment=math.radians(1.0), range_min=0.15,
                           range_max=12.0)


def gray_image():
    """Return a gray BGR image, 100 rows by 200 columns."""
    return np.full((100, 200, 3), GRAY, np.uint8)


@pytest.mark.parametrize('angle, wrapped', [
    (0.0, 0.0), (math.pi / 2.0, math.pi / 2.0), (3.0 * math.pi / 2.0, -math.pi / 2.0),
    (-3.0 * math.pi / 2.0, math.pi / 2.0), (math.pi, -math.pi)])
def test_wrap_angle(angle, wrapped):
    assert wrap_angle(angle) == pytest.approx(wrapped)


@pytest.mark.parametrize('yaw', [0.0, 0.5, -0.5, math.pi / 2.0, 3.0, -3.0])
def test_yaw_from_quaternion_round_trip(yaw):
    assert yaw_from_quaternion(quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_yaw_is_positive_to_the_left():
    # a quarter turn counter-clockwise (to the left) is +pi/2
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sqrt(0.5), w=math.sqrt(0.5))
    assert yaw_from_quaternion(q) == pytest.approx(math.pi / 2.0)


def test_front_range_with_the_lidar_turned_a_quarter_left():
    # with lidar_yaw = +pi/2, straight ahead of the robot is LiDAR angle -pi/2 (index 90)
    ranges = [2.0] * 360
    ranges[90] = 1.0        # straight ahead of the robot
    ranges[180] = 0.5       # LiDAR angle 0: the robot's left
    assert front_range(make_scan(ranges), math.pi / 2.0, math.radians(5.0)) == 1.0


def test_front_range_with_the_lidar_facing_forward():
    ranges = [2.0] * 360
    ranges[182] = 0.8       # 2 degrees left of straight ahead
    assert front_range(make_scan(ranges), 0.0, math.radians(5.0)) == 0.8


def test_front_range_skips_invalid_ranges():
    ranges = [2.0] * 360
    ranges[178:183] = [math.inf, math.nan, 0.05, 20.0, math.inf]
    assert front_range(make_scan(ranges), 0.0, math.radians(2.0)) == math.inf


def test_front_window_marks_invalid_ranges_as_no_return():
    ranges = [2.0] * 360
    ranges[178:183] = [math.inf, math.nan, 0.05, 1.0, 20.0]
    window = front_window(make_scan(ranges), 0.0, math.radians(2.5))
    assert window == [math.inf, math.inf, math.inf, 1.0, math.inf]


def test_a_cone_tip_in_front_of_a_wall_is_narrow():
    # the beam grazes the cone's tip in 2 of 11 directions; the rest reach the wall
    ranges = [3.0] * 360
    ranges[90:92] = [1.6, 1.62]
    window = front_window(make_scan(ranges), math.pi / 2.0, math.radians(5.5))
    assert len(window) == 11
    assert is_narrow(window, 0.15)


def test_a_cone_tip_with_nothing_behind_it_is_narrow():
    ranges = [math.inf] * 360
    ranges[90] = 1.8
    assert is_narrow(front_window(make_scan(ranges), math.pi / 2.0, math.radians(5.0)), 0.15)


def test_a_wall_is_not_narrow():
    # a wall seen at a slant: the ranges change a little across the window, not past depth
    ranges = [1.6 + 0.01 * abs(i - 90) for i in range(360)]
    assert not is_narrow(front_window(make_scan(ranges), math.pi / 2.0, math.radians(5.5)),
                         0.15)


def test_nothing_ahead_is_not_narrow():
    assert not is_narrow([math.inf] * 11, 0.15)
    assert not is_narrow([], 0.15)


def test_find_blob_center_column():
    image = gray_image()
    cv2.rectangle(image, (140, 30), (159, 79), ORANGE, -1)
    blob = find_blob(image, DEFAULT_HSV['orange'], 50)
    assert blob is not None
    assert blob.u == pytest.approx(149.5, abs=0.5)


def test_find_blob_picks_the_largest_region():
    image = gray_image()
    cv2.rectangle(image, (10, 10), (19, 19), BLUE, -1)
    cv2.rectangle(image, (100, 10), (139, 59), BLUE, -1)
    blob = find_blob(image, DEFAULT_HSV['blue'], 50)
    assert blob.u == pytest.approx(119.5, abs=0.5)


def test_find_blob_red_wraps_through_hue_zero():
    image = gray_image()
    cv2.rectangle(image, (40, 30), (59, 79), RED, -1)
    blob = find_blob(image, DEFAULT_HSV['red'], 50)
    assert blob.u == pytest.approx(49.5, abs=0.5)


def test_find_blob_ignores_other_colors_and_small_regions():
    image = gray_image()
    cv2.rectangle(image, (40, 30), (59, 79), BLUE, -1)      # not orange
    cv2.rectangle(image, (150, 50), (152, 52), ORANGE, -1)  # orange, but 3 x 3 px
    assert find_blob(image, DEFAULT_HSV['orange'], 50) is None
