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

"""Tests of the cone_detection module on synthetic images (no ROS needed)."""

from cs5335_cone_navigation.cone_detection import (clean_mask, color_fraction, color_mask,
                                                   DEFAULT_HSV, detect_cones,
                                                   DetectionSettings, draw_candidates,
                                                   parse_ranges, score_candidate,
                                                   shape_features, target_mask, touches_edge)
import cv2
import numpy as np
import pytest

SETTINGS = DetectionSettings()
TARGETS = {c: parse_ranges(DEFAULT_HSV[c]) for c in ('red', 'blue', 'green')}


def bgr(h, s, v):
    """Return the BGR color of an HSV color (OpenCV scale)."""
    return tuple(int(c) for c in cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0])


RED, BLUE, GREEN = bgr(2, 230, 200), bgr(110, 220, 180), bgr(60, 200, 160)
ORANGE, YELLOW = bgr(12, 230, 220), bgr(28, 220, 220)


def scene():
    """Return a gray image."""
    return np.full((240, 320, 3), 120, np.uint8)


def draw_cone(img, x, y, w, h, color):
    """Draw a training cone (a trapezoid on a flat base) with its top center at (x, y)."""
    pts = np.array([[x - w * 0.125, y], [x + w * 0.125, y], [x + w * 0.4, y + h * 0.85],
                    [x - w * 0.4, y + h * 0.85]], np.int32)
    cv2.fillPoly(img, [pts], color)
    cv2.rectangle(img, (int(x - w / 2), int(y + h * 0.85)), (int(x + w / 2), int(y + h)),
                  color, -1)


def results(img):
    """Return (color, accepted) of each candidate, left to right."""
    found = detect_cones(img, TARGETS, SETTINGS)
    return [(c.color, c.accepted()) for c in sorted(found, key=lambda c: c.left)]


def test_parse_ranges():
    assert len(parse_ranges(DEFAULT_HSV['red'])) == 2
    with pytest.raises(ValueError):
        parse_ranges([0, 1, 2])
    with pytest.raises(ValueError):
        parse_ranges([170, 100, 70, 5, 255, 255])     # wraps: needs two ranges
    with pytest.raises(ValueError):
        parse_ranges([])


def test_red_mask_covers_both_ends_of_the_hue_scale():
    hsv = np.uint8([[[2, 230, 200], [175, 230, 200], [12, 230, 200]]])
    assert list(color_mask(hsv, TARGETS['red'])[0]) == [255, 255, 0]


def test_clean_mask_removes_specks_and_fills_holes():
    mask = np.zeros((40, 40), np.uint8)
    mask[10:30, 10:30] = 255
    mask[20, 20] = 0            # hole
    mask[2, 2] = 255            # speck
    cleaned = clean_mask(mask, SETTINGS)
    assert cleaned[20, 20] == 255
    assert cleaned[2, 2] == 0


def test_two_cones_of_the_same_color_are_both_accepted():
    img = scene()
    draw_cone(img, 60, 100, 50, 45, RED)
    draw_cone(img, 160, 100, 50, 45, RED)
    assert results(img) == [('red', True), ('red', True)]


def test_each_target_color_is_accepted():
    img = scene()
    for x, color in ((60, RED), (160, GREEN), (260, BLUE)):
        draw_cone(img, x, 100, 50, 45, color)
    assert results(img) == [('red', True), ('green', True), ('blue', True)]


def test_cones_of_other_colors_are_not_candidates():
    img = scene()
    draw_cone(img, 100, 100, 50, 45, YELLOW)
    draw_cone(img, 200, 100, 50, 45, ORANGE)
    assert results(img) == []


def test_target_colored_objects_that_are_not_cones_are_rejected():
    img = scene()
    cv2.rectangle(img, (40, 60), (90, 130), RED, -1)
    cv2.circle(img, (200, 100), 30, BLUE, -1)
    found = detect_cones(img, TARGETS, SETTINGS)
    assert len(found) == 2
    assert not any(c.accepted() for c in found)
    assert all(c.reason.startswith('taper') for c in found)


def test_orange_cone_partly_in_the_red_range_is_rejected_by_color():
    img = scene()
    draw_cone(img, 160, 100, 60, 55, ORANGE)
    cv2.rectangle(img, (150, 135), (170, 150), RED, -1)   # a reddish patch on it
    assert not any(c.accepted() for c in detect_cones(img, TARGETS, SETTINGS))


def test_cone_cut_off_by_the_image_edge_is_rejected():
    img = scene()
    draw_cone(img, 10, 100, 50, 45, RED)
    found = detect_cones(img, TARGETS, SETTINGS)
    assert found and not found[0].accepted()


def test_shape_features_of_a_triangle():
    mask = np.zeros((100, 100), np.uint8)
    cv2.fillPoly(mask, [np.array([[50, 10], [80, 90], [20, 90]], np.int32)], 255)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    features = shape_features(contours[0], SETTINGS)
    assert features['taper'] > 0.6
    assert features['symmetry'] > 0.9
    assert features['solidity'] > 0.95
    assert features['vertices'] == 3


def test_color_fraction_counts_only_strong_colors():
    hsv = np.zeros((10, 10, 3), np.uint8)
    hsv[:, :5] = (2, 230, 200)      # red
    hsv[:, 5:] = (12, 230, 200)     # orange
    assert color_fraction(hsv, TARGETS['red'], (0, 0, 10, 10), 0.0) == pytest.approx(0.5)
    gray = np.zeros((10, 10, 3), np.uint8)
    assert color_fraction(gray, TARGETS['red'], (0, 0, 10, 10), 0.0) == 0.0


def test_score_candidate_weights_and_gates():
    good = {'aspect': 1.45, 'taper': 0.6, 'symmetry': 0.95, 'base': 1.0, 'solidity': 0.95,
            'vertices': 5}
    scores, confidence, reason = score_candidate(good, 0.95, SETTINGS)
    assert reason == ''
    assert scores['aspect'] == pytest.approx(1.0)
    assert 0.9 < confidence <= 1.0
    flat = dict(good, taper=0.0)
    assert score_candidate(flat, 0.95, SETTINGS)[2].startswith('taper')


def test_touches_edge():
    assert touches_edge((0, 50, 20, 80), 320, 240, 2)
    assert not touches_edge((10, 50, 30, 80), 320, 240, 2)


def test_debug_drawing_and_mask():
    img = scene()
    draw_cone(img, 160, 100, 50, 45, RED)
    cv2.rectangle(img, (20, 20), (60, 90), RED, -1)
    out = draw_candidates(img, detect_cones(img, TARGETS, SETTINGS))
    assert out.shape == img.shape and not np.array_equal(out, img)
    assert target_mask(img, TARGETS, SETTINGS)[120, 160] == 255
