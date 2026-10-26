"""
Tests of the A* planner (no ROS, no robot), from this package's directory:
PYTHONPATH=../day6pkg:. python3 -m pytest test
"""

import math

import numpy as np
import pytest

from day6pkg.grid import OccupancyGrid
from day7pkg.planner import (astar, blocked_cells, drive_commands, line_of_sight, path_length,
                             plan_path, shorten)
import synthetic


def open_grid(size=40):
    """
    A grid of size x size cells of 5 cm, all known to be free
    """
    g = OccupancyGrid(size, 0.05)
    g.log_odds[:] = -3.0
    return g


def test_blocked_cells_grow_obstacles_and_block_unknown_cells():
    g = open_grid()
    g.log_odds[20, 20] = 3.0        # obstacle
    g.log_odds[5, 5] = 0.0          # never seen
    blocked = blocked_cells(g.log_odds, g.resolution, inflate=0.11)
    assert blocked[20, 20] and blocked[20, 22] and blocked[21, 21]     # within 0.11 m
    assert not blocked[20, 23]                                         # 0.15 m away
    assert blocked[5, 5] and not blocked[5, 7]                         # unknown is not grown
    assert not blocked[0, 0]


def test_astar_goes_straight_on_an_open_grid():
    blocked = np.zeros((40, 40), dtype=bool)
    path, _ = astar(blocked, (5, 5), (5, 30))
    assert path == [(5, c) for c in range(5, 31)]
    path, _ = astar(blocked, (5, 5), (25, 25))
    assert path == [(k, k) for k in range(5, 26)]


def wall_with_gap():
    """
    Blocked cells: a wall down column 20 with a gap in rows 30 to 32
    """
    blocked = np.zeros((40, 40), dtype=bool)
    blocked[:, 20] = True
    blocked[30:33, 20] = False
    return blocked


def test_astar_goes_through_the_gap_in_a_wall():
    blocked = wall_with_gap()
    path, expanded = astar(blocked, (10, 5), (10, 35))
    assert path[0] == (10, 5) and path[-1] == (10, 35)
    assert not any(blocked[c] for c in path)
    assert all(30 <= r <= 32 for r, c in path if c == 20)
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        assert max(abs(r1 - r0), abs(c1 - c0)) == 1     # each step goes to a neighbor
    assert expanded < 40 * 40


def test_astar_reports_when_there_is_no_path():
    blocked = np.zeros((20, 20), dtype=bool)
    blocked[:, 10] = True
    path, _ = astar(blocked, (5, 5), (5, 15))
    assert path is None


def test_diagonal_steps_do_not_cut_corners():
    blocked = np.zeros((20, 20), dtype=bool)
    blocked[10, 11] = blocked[11, 10] = True
    path, _ = astar(blocked, (10, 10), (11, 11))
    assert len(path) > 2
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        if r1 != r0 and c1 != c0:
            assert not blocked[r1, c0] and not blocked[r0, c1]


def test_shorten_keeps_the_ends_and_a_clear_line_between_waypoints():
    blocked = wall_with_gap()
    path, _ = astar(blocked, (10, 5), (10, 35))
    waypoints = shorten(blocked, path)
    assert waypoints[0] == path[0] and waypoints[-1] == path[-1]
    assert 3 <= len(waypoints) < len(path)
    for a, b in zip(waypoints, waypoints[1:]):
        assert line_of_sight(blocked, a, b)


def test_drive_commands_turn_then_drive():
    commands = drive_commands([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    assert commands == [pytest.approx((0.0, 1.0)), pytest.approx((90.0, 1.0)),
                        pytest.approx((90.0, 1.0))]
    assert drive_commands([(0.0, 0.0), (1.0, 0.0)], heading=math.pi / 2)[0][0] == \
        pytest.approx(-90.0)


def distance_to_segments(p, segments):
    """
    Distance (m) from point p to the nearest of segments
    """
    best = math.inf
    for a, b in segments:
        a, b = np.asarray(a), np.asarray(b)
        t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(p - (a + t * (b - a)))))
    return best


def test_synthetic_path_bends_around_the_box_and_keeps_clear_of_it():
    grid = synthetic.run_pair()[0]
    start, goal = (0.0, 0.0), (2.2, 0.7)       # the left box is in the way
    plan = plan_path(grid, start, goal, inflate=0.22)
    assert plan.direct_blocked
    assert len(plan.points) >= 3                # at least one bend
    assert plan.points[0] == start and plan.points[-1] == goal
    assert path_length(plan.points) > math.hypot(*goal)
    # every point along the straight legs keeps the robot (radius 0.17 m) off the walls
    for p, q in zip(plan.points, plan.points[1:]):
        for t in np.linspace(0.0, 1.0, 50):
            point = np.asarray(p) + t * (np.asarray(q) - np.asarray(p))
            assert distance_to_segments(point, synthetic.WORLD) > 0.17


def test_plan_path_rejects_goals_it_cannot_use():
    grid = synthetic.run_pair()[0]
    with pytest.raises(ValueError, match='blocked'):
        plan_path(grid, (0.0, 0.0), (1.45, 0.75), inflate=0.22)      # inside the left box
    with pytest.raises(ValueError, match='outside'):
        plan_path(grid, (0.0, 0.0), (20.0, 0.0), inflate=0.22)
