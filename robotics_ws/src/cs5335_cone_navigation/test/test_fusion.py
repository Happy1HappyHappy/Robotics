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

"""Tests of the fusion module (no ROS needed)."""

import math
from types import SimpleNamespace

from cs5335_cone_navigation import fusion
from cs5335_cone_navigation.control import transform
import numpy as np
import pytest


def test_pixel_bearing_is_positive_to_the_left():
    assert fusion.pixel_bearing(100.0, 200.0, 100.0) == pytest.approx(0.0)
    assert fusion.pixel_bearing(0.0, 200.0, 100.0) > 0.0
    assert fusion.pixel_bearing(300.0, 200.0, 100.0) == pytest.approx(-math.pi / 4.0)


def test_camera_model_with_and_without_calibration():
    assert fusion.camera_model([250.0, 0, 124.5, 0, 250.0, 124.5, 0, 0, 1], 250, 1.2) == \
        pytest.approx((250.0, 124.5))
    fx, cx = fusion.camera_model([0.0] * 9, 250, math.pi / 2.0)
    assert (fx, cx) == pytest.approx((125.0, 124.5))


def test_facing_yaw_of_optical_and_body_frames():
    # base_link -> optical frame of a forward camera: roll -90 deg, yaw -90 deg
    optical = SimpleNamespace(x=-0.5, y=0.5, z=-0.5, w=0.5)
    assert fusion.facing_yaw(optical, optical=True) == pytest.approx(0.0, abs=1e-9)
    turned = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.4), w=math.cos(0.4))
    assert fusion.facing_yaw(turned, optical=False) == pytest.approx(0.8)


def test_angular_half_width():
    assert fusion.angular_half_width(100.0, 0.0, 200.0, 100.0) == pytest.approx(0.0)
    assert fusion.angular_half_width(100.0, 40.0, 200.0, 100.0) == \
        pytest.approx(math.atan2(20.0, 200.0))


def test_cone_points_takes_the_nearest_object_along_the_ray():
    cone = [[1.0, -0.02], [1.0, 0.0], [1.02, 0.02]]
    wall = [[2.0, y] for y in np.linspace(-0.2, 0.2, 5)]
    elsewhere = [[0.3, 0.8]]
    cluster = fusion.cone_points(cone + wall + elsewhere, (0.0, 0.0), 0.0, 0.1, 0.15, 3.0)
    assert len(cluster) == 3


def test_cone_points_is_empty_without_a_return():
    assert fusion.cone_points([[1.0, 1.0]], (0.0, 0.0), 0.0, 0.05, 0.15, 3.0).shape == (0, 2)
    assert fusion.cone_points([[5.0, 0.0]], (0.0, 0.0), 0.0, 0.05, 0.15, 3.0).shape == (0, 2)


def test_cone_points_accounts_for_the_camera_position():
    # the camera sits 0.1 m to the left; a cone straight ahead of it is at y = 0.1
    cluster = fusion.cone_points([[1.0, 0.1], [1.0, -0.1]], (0.0, 0.1), 0.0, 0.03, 0.1, 3.0)
    assert cluster.tolist() == [[1.0, 0.1]]


def test_ray_in_frame():
    origin, bearing = fusion.ray_in_frame(transform(1.0, 2.0, math.pi / 2.0),
                                          (0.1, 0.0, 0.0), 0.2)
    assert origin == pytest.approx([1.0, 2.1])
    assert bearing == pytest.approx(math.pi / 2.0 + 0.2)
