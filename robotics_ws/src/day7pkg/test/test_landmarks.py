"""
Tests of the landmarks (no ROS, no robot), from this package's directory:
PYTHONPATH=../day6pkg:. python3 -m pytest test
"""

import math

import numpy as np
import pytest

from day7pkg.landmarks import (align, compose, find_corners, fit_line, invert,
                               occupied_points, ransac_segments, rigid_fit, Segment,
                               transform_points, wrap_angle)
import synthetic


def line_points(p0, p1, spacing=0.05):
    """
    Points every spacing m from p0 to p1
    """
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    n = int(round(np.linalg.norm(p1 - p0) / spacing)) + 1
    return p0 + np.linspace(0.0, 1.0, n)[:, None] * (p1 - p0)


def test_fit_line_handles_vertical_lines():
    center, direction = fit_line(line_points((1.0, 0.0), (1.0, 1.0)))
    assert center == pytest.approx((1.0, 0.5))
    assert abs(direction[0]) < 1e-9 and abs(direction[1]) == pytest.approx(1.0)


def test_ransac_finds_the_two_walls_of_an_l_and_their_corner():
    rng = np.random.default_rng(1)
    points = np.vstack([line_points((0.0, 0.0), (1.5, 0.0)),
                        line_points((0.0, 0.05), (0.0, 1.2)),
                        rng.uniform(-1.0, 3.0, size=(6, 2))])      # a few stray hits
    segments = ransac_segments(points)
    # the wall found first also takes the other wall's points within inlier_distance of it
    # (up to two cells at the corner), but the corner itself comes from the two lines
    lengths = sorted(s.length for s in segments if s.length > 1.0)
    assert lengths == pytest.approx([1.15, 1.5], abs=0.11)
    corners = find_corners(segments)
    assert len(corners) == 1
    assert np.linalg.norm(corners[0]) < 0.03


def test_a_gap_splits_a_line_into_two_segments():
    points = np.vstack([line_points((0.0, 0.0), (1.0, 0.0)), line_points((1.5, 0.0), (2.5, 0.0))])
    lengths = sorted(round(s.length, 2) for s in ransac_segments(points))
    assert lengths == [1.0, 1.0]


def test_parallel_segments_make_no_corner():
    segments = [Segment(np.array([0.0, 0.0]), np.array([1.0, 0.0]), 20),
                Segment(np.array([0.0, 0.5]), np.array([1.0, 0.5]), 20)]
    assert len(find_corners(segments)) == 0


def test_a_corner_needs_data_up_to_the_intersection():
    # an X: the lines cross at (0.5, 0.5), far from the ends of both segments
    segments = [Segment(np.array([0.0, 0.0]), np.array([1.0, 1.0]), 20),
                Segment(np.array([0.0, 1.0]), np.array([1.0, 0.0]), 20)]
    assert len(find_corners(segments)) == 0


def test_transforms_invert_and_compose():
    t = (1.0, -2.0, 0.7)
    p = np.array([[0.3, 0.4], [-1.0, 2.0]])
    assert np.allclose(transform_points(invert(t), transform_points(t, p)), p)
    assert np.allclose(compose(t, invert(t)), (0.0, 0.0, 0.0), atol=1e-12)
    assert wrap_angle(math.radians(270.0)) == pytest.approx(math.radians(-90.0))


def test_rigid_fit_recovers_a_transform():
    src = np.array([[0.0, 0.0], [1.0, 0.5], [2.0, -1.0], [-1.5, 2.0]])
    t = (0.5, -1.2, math.radians(170.0))
    fit = rigid_fit(src, transform_points(t, src))
    assert fit[:2] == pytest.approx(t[:2])
    assert wrap_angle(fit[2] - t[2]) == pytest.approx(0.0, abs=1e-12)


def test_align_finds_the_transform_despite_missing_and_extra_landmarks():
    rng = np.random.default_rng(3)
    a = np.array([[0.0, 0.0], [1.0, 0.5], [2.0, -1.0], [-1.5, 2.0], [3.0, 1.0],
                  [-2.0, -2.0], [0.5, 2.5], [2.5, -2.5]])
    t = (3.3, -0.05, math.radians(181.5))
    # B saw all of A's landmarks but the last, with a little noise, plus one A did not see
    b = transform_points(invert(t), a[:7]) + rng.normal(0.0, 0.01, (7, 2))
    b = np.vstack([b, [[10.0, 10.0]]])
    found, pairs = align(a, b)
    assert math.hypot(found[0] - t[0], found[1] - t[1]) < 0.03
    assert abs(math.degrees(wrap_angle(found[2] - t[2]))) < 1.0
    assert sorted((i, j) for i, j, _ in pairs) == [(k, k) for k in range(7)]


def test_prior_chooses_which_way_round_a_symmetric_room_is():
    # a rectangle's corners match just as well turned 180 degrees about its center (1.7, 0.1)
    a = np.array([[-1.0, -1.4], [-1.0, 1.6], [4.4, -1.4], [4.4, 1.6]])
    t = (3.28, 0.0, math.pi)
    b = transform_points(invert(t), a)
    found, _ = align(a, b, prior=(3.2, 0.1, math.radians(178.0)))
    assert found[:2] == pytest.approx(t[:2], abs=1e-6)
    twin, _ = align(a, b, prior=(0.0, 0.0, 0.0))
    assert twin[:2] == pytest.approx((0.12, 0.2), abs=1e-6)
    assert wrap_angle(twin[2]) == pytest.approx(0.0, abs=1e-9)


def test_align_gives_up_with_fewer_than_two_matches():
    found, pairs = align(np.array([[0.0, 0.0]]), np.array([[5.0, 5.0]]))
    assert found is None and pairs == []


@pytest.fixture(scope='module')
def corners():
    """
    Corner landmarks of the two synthetic runs, and B's true frame in A's frame
    """
    grid_a, _, grid_b, _, true_b = synthetic.run_pair()
    ca = find_corners(ransac_segments(occupied_points(grid_a)))
    cb = find_corners(ransac_segments(occupied_points(grid_b)))
    return ca, cb, true_b


def test_synthetic_corners_are_real_corners(corners):
    ca, cb, true_b = corners
    assert len(ca) >= 4 and len(cb) >= 4
    for c in np.vstack([ca, transform_points(true_b, cb)]):
        assert np.min(np.linalg.norm(synthetic.CORNERS - c, axis=1)) < 0.1


def test_synthetic_runs_line_up_with_the_true_transform(corners):
    ca, cb, true_b = corners
    found, pairs = align(ca, cb, prior=synthetic.ODOM_B_IN_A)
    assert len(pairs) >= 4
    assert math.hypot(found[0] - true_b[0], found[1] - true_b[1]) < 0.05
    assert abs(math.degrees(wrap_angle(found[2] - true_b[2]))) < 1.0
    assert np.mean([d for _, _, d in pairs]) < 0.05
