"""
Tests of the cone finding in images (no ROS, no robot), from this package's directory:
python3 -m pytest test/test_vision.py
"""

import math

import cv2
import numpy as np
import pytest

from day9pkg.vision import (blob_bearings, color_mask, cone_shaped, DEFAULT_HSV, draw_blobs,
                            find_blobs, focal_from_fov, largest_blob, pixel_bearing,
                            shape_features, ShapeLimits)

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


SHAPE = ShapeLimits()


def region(draw):
    """Return the painted shape, cropped to its bounding box, as a 2D boolean array."""
    mask = np.zeros((120, 120), np.uint8)
    draw(mask)
    rows, cols = np.nonzero(mask)
    return mask[rows.min():rows.max() + 1, cols.min():cols.max() + 1] > 0


def triangle(mask):
    """Paint a cone: a triangle, narrow at the top."""
    cv2.fillPoly(mask, [np.array([[60, 10], [85, 90], [35, 90]], np.int32)], 255)


def training_cone(mask):
    """Paint a short training cone: a trapezoid on a wide flat base."""
    cv2.fillPoly(mask, [np.array([[52, 40], [68, 40], [80, 80], [40, 80]], np.int32)], 255)
    cv2.rectangle(mask, (32, 80), (88, 88), 255, -1)


def test_shape_features_of_a_cone():
    f = shape_features(region(triangle))
    assert f['taper'] > 0.6
    assert f['symmetry'] > 0.9
    assert f['solidity'] > 0.9
    assert f['aspect'] == pytest.approx(81 / 51)


@pytest.mark.parametrize('draw', [triangle, training_cone])
def test_cones_pass_the_shape_checks(draw):
    assert cone_shaped(region(draw), SHAPE)


@pytest.mark.parametrize('draw', [
    lambda m: cv2.rectangle(m, (30, 20), (80, 90), 255, -1),    # a box
    lambda m: cv2.circle(m, (60, 60), 30, 255, -1),              # a ball
    lambda m: cv2.rectangle(m, (10, 50), (110, 56), 255, -1),    # a flat sliver
])
def test_boxes_balls_and_slivers_fail_the_shape_checks(draw):
    assert not cone_shaped(region(draw), SHAPE)


def test_a_striped_cone_passes_the_shape_checks():
    img = cone_image(SAMPLES['orange'], stripe=True)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blob = largest_blob(color_mask(hsv, DEFAULT_HSV['orange']), 80, SHAPE)
    assert blob is not None
    assert blob.bottom - blob.top > 50     # both pieces, above and below the stripe


def test_a_larger_box_of_the_same_color_is_skipped():
    img = cone_image(SAMPLES['red'], u=180)
    img[60:190, 20:100] = SAMPLES['red']                    # a big red box on the left
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = color_mask(hsv, DEFAULT_HSV['red'])
    assert largest_blob(mask, 80).u < 100                   # without shape: the box
    assert largest_blob(mask, 80, SHAPE).u == pytest.approx(180, abs=3)


def test_a_ball_of_a_cone_color_is_not_a_cone():
    img = np.full((250, 250, 3), GRAY, np.uint8)
    cv2.circle(img, (125, 125), 30, SAMPLES['blue'], -1)
    assert 'blue' in find_blobs(img, {'blue': DEFAULT_HSV['blue']}, 80)
    assert find_blobs(img, {'blue': DEFAULT_HSV['blue']}, 80, SHAPE) == {}
