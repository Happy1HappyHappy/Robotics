"""
Tests of the A* planner (no ROS, no robot): python3 -m pytest test/test_astar.py
"""

import heapq
import math

import numpy as np
import pytest

from day6pkg.helpers.astar import astar, grow, line_is_free, octile, plan, STEPS
from day6pkg.helpers.occupancy_grid import OccupancyGrid

HIT = math.log10(9.0)   # log-odds of a cell after one LiDAR hit (0.95)
ROBOT = 0.171           # m, robot radius
MARGIN = 0.05           # m, safety margin
CLEARANCE = ROBOT + MARGIN


def set_rect(grid, x0, y0, x1, y1, value):
    """
    Set the log-odds of the cells with centers in [x0, x1] x [y0, y1] (m) to value
    """
    rows, cols = np.indices(grid.log_odds.shape)
    x, y = grid.cell_to_world(rows, cols)
    grid.log_odds[(x0 <= x) & (x <= x1) & (y0 <= y) & (y <= y1)] = value


def seen_grid(size=120, resolution=0.05):
    """
    Grid of 6 m by default, seen free everywhere (one ray through every cell)
    """
    grid = OccupancyGrid(size, resolution)
    grid.log_odds[:] = -HIT
    return grid


def path_cost(cells):
    """
    Length of a path of cells, in cells
    """
    return sum(math.hypot(r1 - r0, c1 - c0) for (r0, c0), (r1, c1) in zip(cells, cells[1:]))


def dijkstra_cost(free, start, goal):
    """
    Cost of the shortest path by Dijkstra's algorithm with the same steps as astar(), to
    check that A* finds a shortest one; None if there is no path
    """
    cost = {start: 0.0}
    frontier = [(0.0, start)]
    while frontier:
        g, (r, c) = heapq.heappop(frontier)
        if (r, c) == goal:
            return g
        if g > cost[(r, c)]:
            continue
        for dr, dc, step in STEPS:
            n = (r + dr, c + dc)
            if not (0 <= n[0] < free.shape[0] and 0 <= n[1] < free.shape[1]) or not free[n]:
                continue
            if dr and dc and not (free[r + dr, c] and free[r, c + dc]):
                continue
            if g + step < cost.get(n, math.inf):
                cost[n] = g + step
                heapq.heappush(frontier, (g + step, n))
    return None


def clearance_of(p, grid, occupied=0.5):
    """
    Smallest distance (m) from a point of the plan's path to the center of an obstacle cell
    """
    rows, cols = np.nonzero(grid.log_odds > occupied)
    obstacles = np.column_stack(grid.cell_to_world(rows, cols))
    return np.min(np.linalg.norm(p.points[:, None, :] - obstacles[None, :, :], axis=2))


# ---------- grow ----------
def test_grow_is_a_disk():
    mask = np.zeros((9, 9), dtype=bool)
    mask[4, 4] = True
    grown = grow(mask, 2.0)
    assert grown.sum() == 13                # the cells with dr^2 + dc^2 <= 4
    assert grown[4, 6] and grown[5, 5] and not grown[6, 5]


def test_grow_does_not_wrap_around_the_edges():
    mask = np.zeros((5, 5), dtype=bool)
    mask[0, 0] = True
    assert set(zip(*np.nonzero(grow(mask, 1.0)))) == {(0, 0), (0, 1), (1, 0)}


# ---------- A* ----------
def test_astar_in_the_open_has_the_octile_cost():
    free = np.ones((20, 20), dtype=bool)
    path = astar(free, (2, 3), (15, 8))
    assert path[0] == (2, 3) and path[-1] == (15, 8)
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        assert max(abs(r1 - r0), abs(c1 - c0)) == 1     # each step goes to a neighbor
    assert path_cost(path) == pytest.approx(octile((2, 3), (15, 8)))


def test_astar_goes_through_the_gap_in_a_wall():
    free = np.ones((10, 10), dtype=bool)
    free[5, :] = False
    free[5, 8] = True
    path = astar(free, (0, 0), (9, 0))
    assert (5, 8) in path
    assert all(free[cell] for cell in path)


def test_astar_never_cuts_a_blocked_corner():
    free = np.array([[True, False], [False, True]])
    assert astar(free, (0, 0), (1, 1)) is None


def test_astar_none_if_walled_off_or_blocked():
    free = np.ones((10, 10), dtype=bool)
    free[5, :] = False
    assert astar(free, (0, 0), (9, 0)) is None
    assert astar(free, (5, 0), (0, 0)) is None          # the start is blocked


@pytest.mark.parametrize('seed', range(5))
def test_astar_finds_a_shortest_path(seed):
    rng = np.random.default_rng(seed)
    free = rng.random((30, 30)) > 0.25
    free[0, 0] = free[29, 29] = True
    path = astar(free, (0, 0), (29, 29))
    best = dijkstra_cost(free, (0, 0), (29, 29))
    assert (path is None) == (best is None)
    if path is not None:
        assert path_cost(path) == pytest.approx(best)


def test_line_is_free():
    free = np.ones((10, 10), dtype=bool)
    assert line_is_free(free, (0, 0), (9, 4))
    free[4, 2] = False
    assert not line_is_free(free, (0, 0), (9, 4))


# ---------- planning on a grid ----------
def test_plan_in_the_open_is_straight():
    p = plan(seen_grid(), (0.0, 0.0), (2.5, 0.0), ROBOT, MARGIN)
    assert p.problem == ''
    assert p.straight
    assert p.length == pytest.approx(2.5, abs=0.05)


def test_plan_bends_around_a_box_and_keeps_clear_of_it():
    grid = seen_grid()
    set_rect(grid, 1.0, -0.3, 1.2, 0.3, HIT)        # a box 1 m ahead, 0.6 m wide
    p = plan(grid, (0.0, 0.0), (2.5, 0.0), ROBOT, MARGIN)
    assert p.problem == ''
    assert not p.straight
    assert p.points[0] == pytest.approx([0.0, 0.0], abs=0.036)    # within half a cell
    assert p.points[-1] == pytest.approx([2.5, 0.0], abs=0.036)
    assert np.max(np.abs(p.points[:, 1])) >= 0.3 + ROBOT         # it goes around the box
    assert clearance_of(p, grid) >= CLEARANCE


def test_plan_fits_through_a_wide_gap_but_not_a_narrow_one():
    wide = seen_grid()
    set_rect(wide, 1.0, -10.0, 1.1, -0.3, HIT)      # a wall across the grid, 0.6 m gap
    set_rect(wide, 1.0, 0.3, 1.1, 10.0, HIT)
    p = plan(wide, (0.0, 0.0), (2.5, 0.0), ROBOT, MARGIN)
    assert p.problem == ''
    assert clearance_of(p, wide) >= CLEARANCE

    narrow = seen_grid()                            # 0.3 m gap: narrower than the robot
    set_rect(narrow, 1.0, -10.0, 1.1, -0.15, HIT)
    set_rect(narrow, 1.0, 0.15, 1.1, 10.0, HIT)
    assert plan(narrow, (0.0, 0.0), (2.5, 0.0), ROBOT, MARGIN).problem.startswith('no path')


def test_plan_goal_in_unknown_space():
    grid = seen_grid()
    set_rect(grid, 2.2, -0.3, 2.8, 0.3, 0.0)        # never seen
    p = plan(grid, (0.0, 0.0), (2.5, 0.0), ROBOT, MARGIN)
    assert 'unknown' in p.problem
    assert plan(grid, (0.0, 0.0), (2.5, 0.0), ROBOT, MARGIN, unknown_is_free=True).problem == ''


def test_plan_ignores_what_is_seen_under_the_robot():
    grid = seen_grid()
    grid.log_odds[grid.world_to_cell(0.1, 0.0)] = HIT      # within the robot's radius
    grid.log_odds[grid.world_to_cell(-0.1, 0.05)] = HIT
    assert plan(grid, (0.0, 0.0), (2.5, 0.0), ROBOT, MARGIN).problem == ''
    grid.log_odds[grid.world_to_cell(0.2, 0.0)] = HIT      # beyond it, within the clearance
    assert plan(grid, (0.0, 0.0), (2.5, 0.0), ROBOT, MARGIN).problem.startswith('the start')


def test_plan_outside_the_grid():
    assert plan(seen_grid(), (0.0, 0.0), (5.0, 0.0)).problem == 'the goal is outside the grid'


def test_plan_on_one_scan_sent_as_a_ros_grid():
    # one scan from the origin: a box face 1.2 m ahead, 0.8 m wide; a round wall at 2.8 m
    n = 720
    angles = -math.pi + np.arange(n) * 2.0 * math.pi / n
    ranges = [1.2 / math.cos(a) if math.cos(a) > 0.0 and abs(1.2 * math.tan(a)) <= 0.4
              else 2.8 for a in angles]
    sensed = OccupancyGrid(120, 0.05)
    sensed.update((0.0, 0.0, 0.0), ranges, -math.pi, 2.0 * math.pi / n, 0.15, 12.0)
    values, _ = sensed.ros_layout()
    grid = OccupancyGrid.from_ros_layout(values.ravel(), 0.05)  # as the planner node gets it

    goal = (2.0, 1.0)       # 2.24 m away, seen past the box's left edge
    p = plan(grid, (0.0, 0.0), goal, ROBOT, MARGIN)
    assert p.problem == ''
    assert not p.straight   # the straight line passes 0.18 m from the box's corner
    assert clearance_of(p, grid) >= CLEARANCE
    assert not p.unknown[tuple(zip(*p.cells))].any()  # the path is all in seen space
