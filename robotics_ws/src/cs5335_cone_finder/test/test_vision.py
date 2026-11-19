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

"""Tests of the vision module (no ROS needed)."""

import math
from types import SimpleNamespace

from cs5335_cone_finder import vision
import numpy as np
import pytest


def test_pixel_bearing_signs():
    assert vision.pixel_bearing(100.0, 200.0, 100.0) == pytest.approx(0.0)
    assert vision.pixel_bearing(0.0, 200.0, 100.0) > 0.0     # left of center = left
    assert vision.pixel_bearing(200.0, 200.0, 100.0) < 0.0
    assert vision.pixel_bearing(300.0, 200.0, 100.0) == pytest.approx(-math.pi / 4.0)


def test_focal_from_fov():
    assert vision.focal_from_fov(200, math.pi / 2.0) == pytest.approx(100.0)


def test_camera_model_uses_calibration_when_there_is_one():
    k = [250.0, 0.0, 124.5, 0.0, 250.0, 124.5, 0.0, 0.0, 1.0]
    assert vision.camera_model(k, 250, 1.2) == pytest.approx((250.0, 124.5))


def test_camera_model_falls_back_to_the_field_of_view():
    fx, cx = vision.camera_model([0.0] * 9, 250, math.pi / 2.0)
    assert fx == pytest.approx(125.0)
    assert cx == pytest.approx(124.5)


def test_box_bearings_are_ordered_left_to_right():
    center, left, right = vision.box_bearings(100.0, 40.0, 200.0, 100.0)
    assert center == pytest.approx(0.0)
    assert left > center > right
    assert left == pytest.approx(-right)


def test_draw_cones_returns_a_copy_with_boxes():
    image = np.zeros((60, 80, 3), np.uint8)
    cone = SimpleNamespace(x1=10.0, y1=20.0, x2=30.0, y2=50.0, color=None, confidence=0.5)
    out = vision.draw_cones(image, [cone])
    assert out.shape == image.shape
    assert not image.any()                   # the input is untouched
    assert out[20, 10:30].any()              # the box's top edge is drawn
    assert out[:, 40].any()                  # the center line is drawn
