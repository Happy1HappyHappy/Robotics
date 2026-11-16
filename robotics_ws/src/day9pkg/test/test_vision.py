"""
Tests of the cone finding in images (no ROS, no robot), from this package's directory:
python3 -m pytest test/test_vision.py
"""

import math

import cv2
import numpy as np
import pytest

from day9pkg.vision import (blob_bearings, color_mask, DEFAULT_HSV, draw_blobs, find_blobs,
                            focal_from_fov, largest_blob, pixel_bearing)

# a typical BGR value of each color, as a camera might see a cone of it
SAMPLES = {
    'red': (30, 20, 210),
    'orange': (0, 128, 255),
    'yellow': (0, 210, 230),
    'green': (40, 180, 40),
    'blue': (200, 80, 20),
    'purple': (160, 40, 120),
    'pink': (180, 105, 255),
}
GRAY = (128, 128, 128)


def cone_image(color_bgr, u=125, size=250, height=60, base=30, stripe=False):
    """
    A gray size x size image with a cone (filled triangle) centered on column u
    With stripe, a white band across the middle splits the cone in two, like a real one
    """
    img = np.full((size, size, 3), GRAY, np.uint8)
    bottom = size // 2 + height // 2
    pts = np.array([[u - base // 2, bottom], [u + base // 2, bottom], [u, bottom - height]],
                   np.int32)
    cv2.fillPoly(img, [pts], color_bgr)
    if stripe:
        y = bottom - height // 2
        img[y - 4:y + 4, u - base:u + base + 1] = (255, 255, 255)
    return img


def hsv_of(bgr):
    """
    A 1x1 HSV image of one BGR color
    """
    return cv2.cvtColor(np.array([[bgr]], np.uint8), cv2.COLOR_BGR2HSV)


@pytest.mark.parametrize('name', list(SAMPLES))
def test_each_sample_is_in_its_own_range_only(name):
    hsv = hsv_of(SAMPLES[name])
    for other, hsv_range in DEFAULT_HSV.items():
        inside = color_mask(hsv, hsv_range)[0, 0] == 255
        assert inside == (other == name), f'{name} sample, {other} range'


def test_gray_white_and_black_are_no_color():
    for bgr in (GRAY, (255, 255, 255), (0, 0, 0), (60, 60, 60)):
        hsv = hsv_of(bgr)
        for hsv_range in DEFAULT_HSV.values():
            assert color_mask(hsv, hsv_range)[0, 0] == 0


def test_red_range_wraps_through_hue_zero():
    hsv = np.array([[[175, 200, 200], [2, 200, 200], [10, 200, 200]]], np.uint8)
    assert list(color_mask(hsv, DEFAULT_HSV['red'])[0]) == [255, 255, 0]


def test_finds_the_cone_and_its_column():
    img = cone_image(SAMPLES['orange'], u=60)
    blobs = find_blobs(img, {c: DEFAULT_HSV[c] for c in ('orange', 'green', 'blue')}, 80)
    assert list(blobs) == ['orange']
    b = blobs['orange']
    assert b.u == pytest.approx(60, abs=1.0)
    assert b.left <= 60 - 14 and b.right >= 60 + 14
    assert 600 < b.area < 1100      # triangle 30 x 60 = 900 px


def test_a_white_stripe_does_not_split_the_cone():
    b = find_blobs(cone_image(SAMPLES['blue'], u=180, stripe=True),
                   {'blue': DEFAULT_HSV['blue']}, 80)['blue']
    assert b.u == pytest.approx(180, abs=1.5)
    # apex (row 95; the opening trims its thinnest few rows) to base (row 155): both pieces
    assert b.top <= 100 and b.bottom >= 155


def test_a_blob_in_other_columns_is_not_part_of_the_cone():
    img = cone_image(SAMPLES['blue'], u=60)
    img[20:40, 200:230] = SAMPLES['blue']           # 600 px, elsewhere in the image
    b = find_blobs(img, {'blue': DEFAULT_HSV['blue']}, 80)['blue']
    assert b.right < 100 and b.u == pytest.approx(60, abs=1.0)


def test_three_cones_in_one_image():
    img = np.full((250, 250, 3), GRAY, np.uint8)
    for color, u in (('orange', 40), ('green', 125), ('blue', 210)):
        cone = cone_image(SAMPLES[color], u=u)
        drawn = np.any(cone != GRAY, axis=2)
        img[drawn] = cone[drawn]
    blobs = find_blobs(img, {c: DEFAULT_HSV[c] for c in ('orange', 'green', 'blue')}, 80)
    assert {c: b.u for c, b in blobs.items()} == pytest.approx(
        {'orange': 40, 'green': 125, 'blue': 210}, abs=1.0)


def test_specks_and_small_blobs_are_ignored():
    img = np.full((250, 250, 3), GRAY, np.uint8)
    img[10:12, 10:12] = SAMPLES['orange']           # 4 px speck: removed by opening
    img[100:107, 100:107] = SAMPLES['orange']       # 49 px: below min_area
    assert largest_blob(color_mask(cv2.cvtColor(img, cv2.COLOR_BGR2HSV),
                                   DEFAULT_HSV['orange']), 80) is None


def test_bearing_is_positive_to_the_left_and_hfov_at_the_edge():
    fx = focal_from_fov(250, math.radians(60.0))
    cx = 124.5
    assert pixel_bearing(cx, fx, cx) == pytest.approx(0.0)
    assert pixel_bearing(0.0, fx, cx) > 0.0                     # left of center
    assert pixel_bearing(249.0, fx, cx) < 0.0                   # right of center
    assert pixel_bearing(-0.5, fx, cx) == pytest.approx(math.radians(30.0))


def test_blob_edges_are_on_either_side_of_its_center():
    b = find_blobs(cone_image(SAMPLES['green'], u=90), {'green': DEFAULT_HSV['green']},
                   80)['green']
    center, left, right = blob_bearings(b, 300.0, 124.5)
    assert left > center > right > 0.0      # all left of the image center at column 124.5


def test_draw_keeps_the_input_unchanged():
    img = cone_image(SAMPLES['orange'])
    copy = img.copy()
    out = draw_blobs(img, find_blobs(img, {'orange': DEFAULT_HSV['orange']}, 80), 'orange')
    assert np.array_equal(img, copy)
    assert out.shape == img.shape and not np.array_equal(out, img)
