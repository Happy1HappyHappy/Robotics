"""
A* path planning on an occupancy grid (Project 7, Task 3)
The robot is a disk, so the obstacles are first grown by its radius plus a safety margin
(configuration space): wherever its center is in a free cell of the grown grid, its body is
clear of the obstacles, and A* can plan for that one point; the planner node runs this
Pure NumPy (no ROS), so it can be tested and used without the robot
Cells are (row, col) of OccupancyGrid.log_odds; points are world (x, y) in meters
"""

import heapq
import math
from typing import NamedTuple

import numpy as np

from day6pkg.helpers.occupancy_grid import bresenham

SQRT2 = math.sqrt(2.0)
# the 8 neighbors of a cell: (row step, col step, cost in cells)
STEPS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


def grow(mask, radius):
    """
    Cells whose center is within radius (in cells) of the center of a True cell of mask,
    as a new mask (a dilation by a disk); radius 0 returns a copy
    """
    grown = mask.copy()
    rows, cols = mask.shape
    r = int(math.floor(radius))
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr * dr + dc * dc > radius * radius or abs(dr) >= rows or abs(dc) >= cols:
                continue
            # grown[i + dr, j + dc] |= mask[i, j], wherever both cells are in the grid
            to = (slice(max(dr, 0), rows + min(dr, 0)), slice(max(dc, 0), cols + min(dc, 0)))
            frm = (slice(max(-dr, 0), rows + min(-dr, 0)),
                   slice(max(-dc, 0), cols + min(-dc, 0)))
            grown[to] |= mask[frm]
    return grown


def octile(a, b):
    """
    Cost (in cells) of the shortest 8-connected path from cell a to cell b if nothing is in
    the way: the A* heuristic; it never overestimates, so A* finds a shortest path
    """
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc)


def astar(free, start, goal):
    """
    Shortest path through the free cells (mask) from cell start to cell goal, both
    included, as a list of (row, col); None if there is none
    A step goes to one of the 8 neighbors: sideways it costs 1, diagonally sqrt(2); a
    diagonal step needs both cells beside it free, so the path never cuts a blocked corner
    """
    rows, cols = free.shape
    if not (free[start] and free[goal]):
        return None
    cost = np.full(free.shape, np.inf)          # g: cost of the best path found to each cell
    done = np.zeros(free.shape, dtype=bool)     # closed set: its best path is final
    parent = {}                                 # cell -> the cell before it on that path
    cost[start] = 0.0
    # open set by f = g + h, ties to the cell nearer the goal (smaller h)
    frontier = [(octile(start, goal), octile(start, goal), start)]
    while frontier:
        _, _, cell = heapq.heappop(frontier)
        if done[cell]:
            continue                # an older entry of a cell already expanded
        if cell == goal:
            path = [cell]
            while cell != start:
                cell = parent[cell]
                path.append(cell)
            return path[::-1]
        done[cell] = True
        r, c = cell
        for dr, dc, step in STEPS:
            n = (r + dr, c + dc)
            if not (0 <= n[0] < rows and 0 <= n[1] < cols) or done[n] or not free[n]:
                continue
            if dr and dc and not (free[r + dr, c] and free[r, c + dc]):
                continue            # diagonal past a blocked corner
            g = cost[cell] + step
            if g < cost[n]:
                cost[n] = g
                parent[n] = cell
                h = octile(n, goal)
                heapq.heappush(frontier, (g + h, h, n))
    return None


def line_is_free(free, start, goal):
    """
    Check that every cell on the straight line from cell start to cell goal is free: then
    the robot could drive straight there, and the path need not bend
    """
    return all(free[cell] for cell in bresenham(*start, *goal) + [goal])


class Plan(NamedTuple):
    """
    What plan() found
    """
    cells: list            # (row, col) of the path, start first; empty if there is none
    points: np.ndarray      # world (x, y) of their centers, N x 2, m
    length: float           # m, along the path
    blocked: np.ndarray     # mask: cells the robot's center must not enter (grown obstacles)
    unknown: np.ndarray     # mask: cells never seen
    straight: bool          # the straight line from start to goal is free (no bend needed)
    problem: str            # why there is no path; '' if there is one


def plan(grid, start, goal, robot_radius=0.171, margin=0.05, occupied=0.5,
         unknown_is_free=False):
    """
    Plan a path for the robot's center from world point start to world point goal (x, y, m)
    on grid (OccupancyGrid)
    A cell with log-odds above occupied is an obstacle, below -occupied it is free, and in
    between it was never seen (unknown); one LiDAR hit adds 0.95, so 0.5 makes a cell hit
    once more than passed through an obstacle, even in a grid of one scan
    The obstacles are grown by robot_radius + margin (m): the robot's center keeps at least
    that far from the center of every obstacle cell; margin covers the cell size (an
    obstacle may be up to half a cell diagonal, 3.5 cm, nearer than its cell's center) and
    driving errors. Unknown cells are blocked unless unknown_is_free, but not grown: one
    cell missed between two LiDAR rays should not close a passage; the robot's side may
    still reach into unseen space, e.g. behind an obstacle
    The cells within robot_radius of start count as free: the robot stands there, so an
    obstacle seen there is the robot itself or noise
    """
    res = grid.resolution
    s, g = grid.world_to_cell(*start), grid.world_to_cell(*goal)
    obstacles = grid.log_odds > occupied
    unknown = np.abs(grid.log_odds) <= occupied
    none = np.zeros((0, 2))
    if not grid.in_bounds(*s):
        return Plan([], none, 0.0, obstacles, unknown, False, 'the start is outside the grid')
    if not grid.in_bounds(*g):
        return Plan([], none, 0.0, obstacles, unknown, False, 'the goal is outside the grid')

    rows, cols = np.indices(obstacles.shape)
    under_robot = (rows - s[0]) ** 2 + (cols - s[1]) ** 2 <= (robot_radius / res) ** 2
    obstacles[under_robot] = unknown[under_robot] = False
    clearance = robot_radius + margin
    blocked = grow(obstacles, clearance / res)
    free = ~blocked if unknown_is_free else ~blocked & ~unknown

    problem = ''
    if blocked[s]:
        problem = f'the start is within {clearance:.2f} m of an obstacle'
    elif blocked[g]:
        problem = f'the goal is within {clearance:.2f} m of an obstacle'
    elif not free[g]:
        problem = 'the goal is in unknown space (never seen)'
    if problem:
        return Plan([], none, 0.0, blocked, unknown, False, problem)
    cells = astar(free, s, g)
    if cells is None:
        return Plan([], none, 0.0, blocked, unknown, False,
                    f'no path: the obstacles, grown by {clearance:.2f} m, close every way')
    points = np.array([grid.cell_to_world(r, c) for r, c in cells])
    length = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    return Plan(cells, points, length, blocked, unknown, line_is_free(free, s, g), '')
