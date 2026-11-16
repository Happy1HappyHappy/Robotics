"""
Tests of the geometry and motion commands (no ROS, no robot), from this package's directory:
python3 -m pytest test/test_geometry.py
"""

import math

import numpy as np
import pytest

from day9pkg.geometry import (apply, approach_command, gap_to, MotionLimits, nearest_cluster,
                              scan_points, transform, turn_command, wrap_angle)

RADIUS = 0.171      # m, TurtleBot 4


def arc_points(center, radius, toward, spacing=0.01):
    """
    Points (N x 2) on the side of a circle (a cone at the LiDAR's height) facing toward
    """
    cx, cy = center
    facing = math.atan2(toward[1] - cy, toward[0] - cx)
    a = facing + np.arange(-1.2, 1.2, spacing / radius)
    return np.column_stack([cx + radius * np.cos(a), cy + radius * np.sin(a)])


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(3.0 * math.pi / 2.0) == pytest.approx(-math.pi / 2.0)
    assert wrap_angle(-3.0 * math.pi / 2.0) == pytest.approx(math.pi / 2.0)
    assert -math.pi <= wrap_angle(math.pi) < math.pi


def test_transform_moves_points_and_inverts():
    T = transform(1.0, 2.0, math.pi / 2.0)
    assert apply(T, [[1.0, 0.0]])[0] == pytest.approx([1.0, 3.0])   # x axis turns to +y
    back = apply(np.linalg.inv(T), apply(T, [[0.3, -0.7], [2.0, 1.0]]))
    assert back == pytest.approx(np.array([[0.3, -0.7], [2.0, 1.0]]))


def test_scan_points_drop_invalid_returns():
    ranges = [1.0, math.inf, math.nan, 0.05, 2.0, 20.0]
    pts = scan_points(ranges, 0.0, math.pi / 2.0, 0.15, 12.0)
    # kept: 1.0 m at 0 rad (+x), 2.0 m at 4 * 90 deg = 360 deg (+x again)
    assert pts == pytest.approx(np.array([[1.0, 0.0], [2.0, 0.0]]), abs=1e-9)


def test_nearest_cluster_is_the_cone_not_the_wall_behind_it():
    cone = arc_points((1.5, 0.3), 0.05, (0.0, 0.0))
    wall = np.column_stack([np.full(200, 3.0), np.linspace(-2.0, 2.0, 200)])
    pts = np.vstack([wall, cone])
    bearing = math.atan2(0.3, 1.5)
    found = nearest_cluster(pts, bearing, math.radians(5.0), 0.15)
    assert len(found) > 0
    assert np.all(np.hypot(found[:, 0] - 1.5, found[:, 1] - 0.3) < 0.051)


def test_nearest_cluster_window_wraps_behind_the_robot():
    pts = np.array([[-2.0, 0.01], [-2.0, -0.01], [2.0, 0.0]])
    found = nearest_cluster(pts, math.pi, math.radians(3.0), 0.15)
    assert len(found) == 2 and np.all(found[:, 0] < 0.0)


def test_nearest_cluster_empty_when_nothing_that_way():
    found = nearest_cluster(np.array([[1.0, 0.0]]), math.pi / 2.0, 0.1, 0.15)
    assert found.shape == (0, 2)


def test_gap_is_from_the_robot_edge_in_any_direction():
    assert gap_to([[0.3, 0.0], [1.0, 1.0]], RADIUS) == pytest.approx(0.3 - RADIUS)
    assert gap_to([[0.0, -0.25]], RADIUS) == pytest.approx(0.25 - RADIUS)


def test_turn_command_sign_and_limits():
    lim = MotionLimits()
    assert turn_command(0.5, lim) > 0.0 and turn_command(-0.5, lim) < 0.0     # + = left
    assert turn_command(3.0, lim) == pytest.approx(lim.max_angular)
    assert turn_command(0.001, lim) == pytest.approx(lim.min_angular)


def test_approach_command():
    lim = MotionLimits()
    assert approach_command(0.0, 0.02, 0.03, lim) == (0.0, 0.0)            # close enough
    v, w = approach_command(math.radians(40.0), 1.0, 0.03, lim)
    assert v == 0.0 and w > 0.0                                           # turn first
    v, w = approach_command(0.0, 2.0, 0.03, lim)
    assert v == pytest.approx(lim.max_linear) and w == 0.0                # far: full speed
    v, w = approach_command(-0.1, 0.035, 0.03, lim)
    assert v == pytest.approx(lim.min_linear) and w < 0.0                 # near: crawl


@pytest.mark.parametrize('start_yaw, cone', [(0.0, (1.5, 0.0)), (0.2, (1.5, -0.3)),
                                             (-0.5, (1.2, 0.6)), (0.0, (0.4, 0.05))])
def test_simulated_approach_stops_at_the_gap_without_touching(start_yaw, cone):
    """
    Drive a simulated round robot with approach_command, with the cone's LiDAR points one
    period late (like a real scan), and check where it stops
    """
    lim = MotionLimits()
    stop_gap, cone_radius, dt = 0.03, 0.05, 0.05
    x, y, yaw = 0.0, 0.0, start_yaw
    late = None                 # the cone's points one period ago (odom frame)
    for _ in range(2000):
        seen_now = arc_points(cone, cone_radius, (x, y))
        used = seen_now if late is None else late
        late = seen_now
        # as in the node: the (late) points, seen from where the robot is now
        pts = apply(np.linalg.inv(transform(x, y, yaw)), used)
        target = pts.mean(axis=0)
        v, w = approach_command(math.atan2(target[1], target[0]),
                                gap_to(pts, RADIUS), stop_gap, lim)
        if v == 0.0 and w == 0.0:
            break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        true_gap = math.hypot(cone[0] - x, cone[1] - y) - cone_radius - RADIUS
        assert true_gap > 0.0, 'touched the cone'
    else:
        pytest.fail('never stopped')
    true_gap = math.hypot(cone[0] - x, cone[1] - y) - cone_radius - RADIUS
    assert stop_gap - 0.005 <= true_gap <= stop_gap + 0.002
