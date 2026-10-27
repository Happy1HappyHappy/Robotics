"""
Tests of the landmark search (no ROS, no robot): python3 -m pytest test/test_landmarks.py
The RANSAC tests need scikit-image and are skipped without it
"""

import numpy as np
import pytest

from day6pkg.helpers.occupancy_grid import OccupancyGrid
from day6pkg.helpers.landmarks import find_corners, find_segments, occupied_points


def wall(x0, y0, x1, y1, step=0.05):
    """
    Points every step m from (x0, y0) to (x1, y1)
    """
    n = int(round(np.hypot(x1 - x0, y1 - y0) / step)) + 1
    return np.column_stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n)])


def test_occupied_points_are_the_centers_of_cells_above_the_threshold():
    g = OccupancyGrid(300, 0.05)
    g.log_odds[g.world_to_cell(1.0, 0.5)] = 1.908      # two hits
    g.log_odds[g.world_to_cell(2.0, 0.0)] = 0.954      # one hit: below 1.0
    points = occupied_points(g)
    assert points == pytest.approx(np.array([[1.025, 0.525]]))


def test_corner_where_two_segments_end_at_their_crossing():
    segments = [(np.array([0.0, 1.0]), np.array([2.0, 1.0])),      # along x, ends at x = 2
                (np.array([2.02, 0.98]), np.array([2.02, -1.0]))]  # along y, starts at y = 1
    assert find_corners(segments) == pytest.approx(np.array([[2.02, 1.0]]))


def test_no_corner_for_parallel_segments_or_far_ends():
    parallel = [(np.array([0.0, 0.0]), np.array([1.0, 0.0])),
                (np.array([0.0, 0.5]), np.array([1.0, 0.5]))]
    t_junction = [(np.array([0.0, 0.0]), np.array([2.0, 0.0])),    # passes the crossing
                  (np.array([1.0, 0.05]), np.array([1.0, 1.0]))]
    assert len(find_corners(parallel)) == 0
    assert len(find_corners(t_junction)) == 0


def test_close_corners_merge():
    segments = [(np.array([0.0, 1.0]), np.array([2.0, 1.0])),
                (np.array([2.0, 1.0]), np.array([2.0, -1.0])),
                (np.array([2.03, 1.0]), np.array([2.03, 0.0]))]   # a second, thick-wall copy
    assert len(find_corners(segments)) == 1


def test_ransac_finds_the_two_walls_of_an_l_and_its_corner():
    pytest.importorskip('skimage')
    rng = np.random.default_rng(1)
    points = np.vstack([wall(0.0, 1.0, 2.0, 1.0), wall(2.0, 0.95, 2.0, -1.0),
                        rng.uniform(-0.5, 1.5, size=(10, 2))])  # two walls and some noise
    segments = find_segments(points)
    assert len(segments) == 2
    corners = find_corners(segments)
    assert corners == pytest.approx(np.array([[2.0, 1.0]]), abs=0.05)


def test_ransac_splits_a_wall_at_a_doorway():
    pytest.importorskip('skimage')
    points = np.vstack([wall(0.0, 0.0, 1.0, 0.0), wall(1.8, 0.0, 3.0, 0.0)])   # 0.8 m gap
    segments = find_segments(points)
    assert len(segments) == 2
    ends = sorted(sorted([s[0][0], s[1][0]]) for s in segments)
    assert ends == pytest.approx([[0.0, 1.0], [1.8, 3.0]], abs=0.06)
