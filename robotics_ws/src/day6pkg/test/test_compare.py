"""
Tests of comparing two runs' landmarks (no ROS, no robot): python3 -m pytest test/test_compare.py
The test of the whole tool needs OpenCV and is skipped without it
"""

import math

import numpy as np
import pytest

from day6pkg.helpers.compare import match_landmarks, transform_points, world_from_world
from day6pkg.helpers.files import save_npz
from day6pkg.helpers.occupancy_grid import OccupancyGrid


def transform(x, y, theta):
    """
    Homogeneous 3x3 matrix for a 2D pose (meters, radians)
    """
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


# run A starts somewhere in the odometry frame; run B 3 m ahead of it, turned around
ODOM_T_A = transform(1.0, 2.0, math.radians(30.0))
ODOM_T_B = ODOM_T_A @ transform(3.0, 0.0, math.pi)


def test_points_of_the_way_back_land_where_they_are_on_the_way_there():
    a_T_b = world_from_world(np.linalg.inv(ODOM_T_A), np.linalg.inv(ODOM_T_B))
    # 2 m ahead and 1 m left of A's start is 1 m ahead and 1 m right of B's start
    assert transform_points(a_T_b, [[1.0, -1.0]]) == pytest.approx(np.array([[2.0, 1.0]]))


def test_landmarks_pair_with_their_nearest_within_the_distance():
    a = [[0.0, 0.0], [1.0, 0.0], [5.0, 5.0]]
    b = [[1.1, 0.0], [9.0, 9.0], [0.05, 0.0]]
    pairs = match_landmarks(a, b, max_distance=0.3)
    assert [(i, j) for i, j, _ in pairs] == [(0, 2), (1, 0)]
    assert [d for _, _, d in pairs] == pytest.approx([0.05, 0.1])


def test_each_landmark_is_in_one_pair_only():
    a = [[0.0, 0.0], [0.1, 0.0]]        # both nearest to b[0]: only the nearer one pairs
    b = [[0.08, 0.0]]
    assert [(i, j) for i, j, _ in match_landmarks(a, b)] == [(1, 0)]


def test_no_pairs_without_landmarks():
    assert match_landmarks(np.zeros((0, 2)), [[1.0, 1.0]]) == []
    assert match_landmarks([[1.0, 1.0]], np.zeros((0, 2))) == []


def save_run(log_dir, run, odom_T_world, corners):
    """
    Save the files the Grid Node and the Landmark Node save for a run, as on Ctrl-C
    """
    grid = OccupancyGrid(100, 0.05)
    grid.save(str(log_dir / f'{run}_grid_final.npz'), poses=np.zeros((1, 3)),
              world_from_odom=np.linalg.inv(odom_T_world))
    save_npz(str(log_dir / f'{run}_landmarks_final.npz'), corners=np.array(corners),
             segments=np.zeros((0, 2, 2)), resolution=0.05, frame='world')


def test_the_tool_prints_the_mean_difference_and_saves_the_image(tmp_path, capsys):
    pytest.importorskip('cv2')
    from day6pkg.tools.compare_landmarks import main
    save_run(tmp_path, 'there', ODOM_T_A, [[2.0, 1.0], [2.0, -1.0]])
    # the way back sees the same two corners, 0.1 m and 0.2 m off (the odometry drifted)
    save_run(tmp_path, 'back', ODOM_T_B, [[1.0, -0.9], [1.0, 1.2]])
    main(['there', 'back', '--log-dir', str(tmp_path), '--result-dir', str(tmp_path)])
    out = capsys.readouterr().out
    assert '2 found in both' in out
    assert 'mean difference 0.150 m' in out
    assert (tmp_path / 'grid' / 'there_vs_back_landmarks.png').exists()
