"""
Path planning on an occupancy grid with A* (Project 7, Task 3): grow the obstacles by the
robot's radius, search the free cells with A*, then shorten the path to straight legs
between the farthest cells it can reach without hitting anything
Pure NumPy and OpenCV (no ROS), so it can be tested and used without the robot
Cells are (row, col) as in day6pkg's OccupancyGrid; points are world (x, y) in meters
"""

from dataclasses import dataclass, field
import heapq
import math

import cv2
import numpy as np

from day6pkg.grid import bresenham

# (row step, col step, cost): a side step costs 1 cell, a diagonal sqrt(2) (its true length)
NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
             (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))]


def blocked_cells(log_odds, resolution, inflate, occupied=0.5, free=-0.5):
    """
    Cells the robot's center must not enter (True = blocked)
    A cell is blocked if it is not known to be free (log-odds above free, so cells never seen
    count as blocked), or if it is within inflate meters of an occupied cell (log-odds above
    occupied): growing the obstacles by the robot's radius lets A* treat the robot as a point
    """
    obstacle = log_odds > occupied
    # distance, in cells, from every cell to the nearest obstacle cell (0 on obstacles)
    dist = cv2.distanceTransform((~obstacle).astype(np.uint8), cv2.DIST_L2,
                                 cv2.DIST_MASK_PRECISE)
    return (log_odds > free) | (dist * resolution <= inflate)


def astar(blocked, start, goal):
    """
    Shortest path of unblocked cells from start to goal (row, col), 8-connected, with A*:
    Dijkstra's search, but taking cells off the open list in order of f = g + h, where g is
    the cost from the start and h the straight-line distance to the goal
    h never overestimates the remaining cost, so the path found is a shortest one
    A diagonal step may not cut the corner of a blocked cell
    Returns (cells from start to goal, number of cells expanded), or (None, expanded) if the
    goal cannot be reached
    """
    rows, cols = blocked.shape
    start, goal = tuple(map(int, start)), tuple(map(int, goal))
    g = np.full(blocked.shape, np.inf)
    closed = np.zeros(blocked.shape, dtype=bool)
    parent = {}

    def h(cell):
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

    g[start] = 0.0
    open_list = [(h(start), 0, start)]      # (f, insertion order to break ties, cell)
    pushed = 1
    expanded = 0
    while open_list:
        _, _, cell = heapq.heappop(open_list)
        if closed[cell]:        # already expanded through a cheaper entry
            continue
        closed[cell] = True
        expanded += 1
        if cell == goal:
            path = [cell]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1], expanded
        r, c = cell
        for dr, dc, cost in NEIGHBORS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or blocked[nr, nc] or closed[nr, nc]:
                continue
            if dr and dc and (blocked[r + dr, c] or blocked[r, c + dc]):
                continue
            ng = g[r, c] + cost
            if ng < g[nr, nc]:
                g[nr, nc] = ng
                parent[(nr, nc)] = cell
                heapq.heappush(open_list, (ng + h((nr, nc)), pushed, (nr, nc)))
                pushed += 1
    return None, expanded


def line_of_sight(blocked, a, b):
    """
    Check that the straight line of cells from a to b (row, col), b included, is unblocked
    """
    return not blocked[b] and not any(blocked[cell] for cell in bresenham(*a, *b))


def shorten(blocked, path):
    """
    Waypoints along path (cells): from each waypoint, go straight to the farthest later cell
    of the path that can be reached without crossing a blocked cell
    The first and last cells of the path are always waypoints
    """
    waypoints = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_of_sight(blocked, path[i], path[j]):
            j -= 1
        waypoints.append(path[j])
        i = j
    return waypoints


def path_length(points):
    """
    Length in meters of the polyline through points (N x 2)
    """
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def drive_commands(points, heading=0.0):
    """
    (turn in degrees, distance in meters) for each straight leg of the polyline through
    points, starting at heading (radians): turn in place, then drive straight
    Turns are in [-180, 180), + = left (CCW), as day3pkg's Executive takes them (t, m)
    """
    commands = []
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        leg = math.atan2(y1 - y0, x1 - x0)
        turn = (leg - heading + math.pi) % (2.0 * math.pi) - math.pi
        commands.append((math.degrees(turn), math.hypot(x1 - x0, y1 - y0)))
        heading = leg
    return commands


@dataclass
class Plan:
    """
    Result of plan_path(); points are world (x, y) in meters, cells are (row, col)
    """
    start: tuple
    goal: tuple
    blocked: np.ndarray                             # cells the robot's center must avoid
    grown: np.ndarray                               # free cells blocked for being too close
    cells: list = field(default_factory=list)       # A* path, cell by cell
    waypoints: list = field(default_factory=list)   # shortened path, cells
    points: list = field(default_factory=list)      # shortened path, world points
    expanded: int = 0                               # cells A* took off the open list
    direct_blocked: bool = False                    # a straight line start -> goal hits something


def plan_path(grid, start, goal, inflate, occupied=0.5, free=-0.5):
    """
    Plan a path on grid (day6pkg OccupancyGrid) from world point start to world point goal
    for a robot whose center must stay inflate meters from every occupied cell
    The waypoints are the centers of the shortened path's cells, except that the first and
    last are start and goal themselves
    Raises ValueError if start or goal is outside the grid or blocked, or if there is no path
    """
    blocked = blocked_cells(grid.log_odds, grid.resolution, inflate, occupied, free)
    cells = []
    for name, (x, y) in (('start', start), ('goal', goal)):
        cell = grid.world_to_cell(x, y)
        if not grid.in_bounds(*cell):
            raise ValueError(f'The {name} ({x:.2f}, {y:.2f}) m is outside the grid')
        if blocked[cell]:
            raise ValueError(
                f'The {name} ({x:.2f}, {y:.2f}) m is blocked: it is unknown, occupied, or '
                f'within {inflate:.2f} m of an obstacle')
        cells.append(cell)
    # a free cell can only be blocked because it is too close to an obstacle
    plan = Plan(tuple(start), tuple(goal), blocked, blocked & (grid.log_odds <= free))
    plan.direct_blocked = not line_of_sight(blocked, cells[0], cells[1])
    plan.cells, plan.expanded = astar(blocked, cells[0], cells[1])
    if plan.cells is None:
        raise ValueError(f'No path: A* expanded all {plan.expanded} reachable cells')
    plan.waypoints = shorten(blocked, plan.cells)
    middle = [tuple(float(v) for v in grid.cell_to_world(*c)) for c in plan.waypoints[1:-1]]
    plan.points = [tuple(start)] + middle + [tuple(goal)]
    return plan
