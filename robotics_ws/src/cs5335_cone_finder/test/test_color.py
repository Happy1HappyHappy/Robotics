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

"""Tests of the color module (no ROS needed)."""

from cs5335_cone_finder.color import (classify_color, ColorSettings, DEFAULT_HUE_RANGES,
                                      hue_in_range, inner_region)
import cv2
import numpy as np
import pytest

SETTINGS = ColorSettings()


def patch(h, s, v, size=(20, 20)):
    """Return a BGR patch of one HSV color (OpenCV scale)."""
    hsv = np.full((size[0], size[1], 3), (h, s, v), np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def test_default_ranges_do_not_overlap():
    hues = np.arange(180)
    counts = sum(hue_in_range(hues, lo, hi).astype(int) for lo, hi in DEFAULT_HUE_RANGES.values())
    assert counts.max() == 1


def test_hue_in_range_plain_and_wrapping():
    assert list(hue_in_range([4, 5, 18, 19], 5, 18)) == [False, True, True, False]
    assert list(hue_in_range([169, 170, 179, 0, 4, 5], 170, 4)) == [
        False, True, True, True, True, False]


@pytest.mark.parametrize('hue, name', [
    (12, 'orange'), (60, 'green'), (110, 'blue'), (28, 'yellow'), (2, 'red'), (177, 'red')])
def test_classify_color_of_a_strong_color(hue, name):
    color, fraction = classify_color(patch(hue, 220, 200), SETTINGS)
    assert color == name
    assert fraction == pytest.approx(1.0)


def test_classify_color_ignores_gray_and_dark_pixels():
    assert classify_color(patch(12, 20, 200), SETTINGS)[0] is None    # gray: low saturation
    assert classify_color(patch(12, 220, 20), SETTINGS)[0] is None    # too dark


def test_classify_color_with_a_white_stripe():
    cone = np.vstack([patch(12, 220, 200, (8, 20)), patch(0, 0, 240, (4, 20)),
                      patch(12, 220, 200, (8, 20))])
    color, fraction = classify_color(cone, SETTINGS)
    assert color == 'orange'
    assert fraction == pytest.approx(0.8)


def test_classify_color_of_an_empty_patch():
    assert classify_color(np.zeros((0, 0, 3), np.uint8), SETTINGS) == (None, 0.0)


def test_inner_region_is_centered_and_clipped():
    assert inner_region((10.0, 20.0, 30.0, 60.0), 0.5, 100, 100) == (15, 30, 25, 50)
    left, top, right, bottom = inner_region((-10.0, -10.0, 110.0, 110.0), 1.0, 100, 80)
    assert (left, top, right, bottom) == (0, 0, 100, 80)


def test_inner_region_of_a_box_outside_the_image_is_empty():
    left, top, right, bottom = inner_region((120.0, 10.0, 140.0, 20.0), 1.0, 100, 100)
    assert right - left <= 0
