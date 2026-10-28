"""
Comparing the landmarks of two runs, e.g. the way there and the way back (Project 7)
Each run's world frame starts at the robot's start pose of that run, so the two runs' points
are first put in one frame through the odometry frame both runs share: the Grid Node saves
^world T_odom with each grid
Pure NumPy (no ROS)
"""

import numpy as np


def world_from_world(a_world_from_odom, b_world_from_odom):
    """
    ^A T_B = ^A T_odom . (^B T_odom)^-1: maps points from run B's world frame into run A's
    (3 x 3 homogeneous matrices); right only if the odometry was not reset between the runs
    """
    return a_world_from_odom @ np.linalg.inv(b_world_from_odom)


def transform_points(T, points):
    """
    Points (K x 2, m) mapped by the 3 x 3 homogeneous matrix T
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return points @ T[:2, :2].T + T[:2, 2]


def match_landmarks(a, b, max_distance=0.3):
    """
    The landmarks found in both: pairs (i, j, d) of a[i] and b[j] (K x 2 each, m, in one
    frame) that are each other's nearest landmark and at most max_distance (m) apart, d
    their distance, in the order of a; each landmark is in at most one pair
    """
    a = np.asarray(a, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1, 2)
    if not len(a) or not len(b):
        return []
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)    # d[i, j]: a[i] to b[j]
    nearest_b, nearest_a = d.argmin(axis=1), d.argmin(axis=0)
    return [(i, int(j), float(d[i, j])) for i, j in enumerate(nearest_b)
            if nearest_a[j] == i and d[i, j] <= max_distance]
