"""
Synthetic occupancy grids for the tests, made without ROS or a robot: a room with two boxes,
scanned by a simulated LiDAR every 0.1 m along a straight 3.2 m drive (Task 1's path), once
there (grid A) and once back after turning 180 degrees (grid B, in its own frame)
Run B's frame is placed where odometry says plus a small drift, as on the real robot
"""

import functools
import math
import os

import numpy as np

from day6pkg.grid import OccupancyGrid
from day7pkg.landmarks import invert, transform_points


def box(x0, y0, x1, y1):
    """
    The four sides of an axis-aligned rectangle, as ((x, y), (x, y)) segments
    """
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


# the world, in grid A's frame (m): the start is (0, 0) and the robot drives along +x
ROOM = box(-1.0, -1.4, 4.4, 1.6)
BOX_LEFT = box(1.2, 0.5, 1.7, 1.0)          # left of the path
BOX_RIGHT = box(2.3, -0.9, 2.8, -0.4)       # right of the path
WORLD = ROOM + BOX_LEFT + BOX_RIGHT
CORNERS = np.array(sorted({p for seg in WORLD for p in seg}))
DISTANCE = 3.2
# odometry's estimate of B's frame in A's frame: drive 3.2 m, then turn 180 degrees about the
# robot's center, which moves the LiDAR (4 cm behind the center) 8 cm further along +x
ODOM_B_IN_A = (DISTANCE + 0.08, 0.0, math.pi)
DRIFT = (0.04, -0.03, math.radians(2.0))


def transform_segments(t, segments):
    """
    Segments moved by transform t = (x, y, theta)
    """
    return [tuple(map(tuple, transform_points(t, seg))) for seg in segments]


def scan(segments, x, y, yaw, n_rays=360, max_range=12.0, noise=0.0, rng=None):
    """
    Ranges (m) of a simulated LiDAR at (x, y, yaw), rays at yaw - pi + k * 2 pi / n_rays,
    to the nearest segment; inf where a ray hits nothing within max_range
    noise: standard deviation (m) of Gaussian noise added to each range
    """
    a = yaw - math.pi + np.arange(n_rays) * 2.0 * math.pi / n_rays
    d = np.column_stack([np.cos(a), np.sin(a)])
    ranges = np.full(n_rays, np.inf)
    for p, q in segments:
        p, q = np.asarray(p, dtype=float), np.asarray(q, dtype=float)
        e = q - p
        w = p - np.array([x, y])
        denom = d[:, 0] * e[1] - d[:, 1] * e[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[0] * e[1] - w[1] * e[0]) / denom      # distance along the ray
            u = (w[0] * d[:, 1] - w[1] * d[:, 0]) / denom  # where on the segment, 0 to 1
        hit = (np.abs(denom) > 1e-12) & (t > 1e-9) & (u >= 0.0) & (u <= 1.0)
        ranges = np.where(hit & (t < ranges), t, ranges)
    if noise and rng is not None:
        ranges = ranges + rng.normal(0.0, noise, n_rays)
    return np.where(ranges <= max_range, ranges, np.inf)


def drive(segments, distance=DISTANCE, step=0.1, size=200, resolution=0.05, n_rays=360,
          noise=0.01, seed=0):
    """
    Grid from a straight drive from (0, 0) along +x of the segments' frame, adding a scan at
    the start and after every step, like the Grid Node
    Returns (grid, poses)
    """
    rng = np.random.default_rng(seed)
    grid = OccupancyGrid(size, resolution)
    poses = []
    for k in range(round(distance / step) + 1):
        pose = (k * step, 0.0, 0.0)
        ranges = scan(segments, *pose, n_rays, noise=noise, rng=rng)
        grid.update(pose, ranges, -math.pi, 2.0 * math.pi / n_rays, 0.15, 12.0)
        poses.append(pose)
    return grid, poses


@functools.lru_cache(maxsize=None)
def run_pair(drift=DRIFT):
    """
    Grid A (there) and grid B (back, in its own frame), with their poses, and B's true frame
    in A's frame: odometry's estimate plus drift
    Cached, since the tests only read the grids
    """
    true_b = tuple(o + d for o, d in zip(ODOM_B_IN_A, drift))
    grid_a, poses_a = drive(WORLD, seed=1)
    grid_b, poses_b = drive(transform_segments(invert(true_b), WORLD), seed=2)
    return grid_a, poses_a, grid_b, poses_b, true_b


def matrix(t):
    """
    3x3 homogeneous matrix of transform t = (x, y, theta)
    """
    c, s = math.cos(t[2]), math.sin(t[2])
    return np.array([[c, -s, t[0]], [s, c, t[1]], [0.0, 0.0, 1.0]])


def save_pair(directory):
    """
    Write run_a/grid.npz and run_b/grid.npz under directory as the Grid Node would, with
    ^world T_odom such that odometry puts B's frame at ODOM_B_IN_A in A's frame
    Returns the two paths
    """
    grid_a, poses_a, grid_b, poses_b, _ = run_pair()
    paths = []
    for name, grid, poses, world_from_odom in (
            ('run_a', grid_a, poses_a, np.eye(3)),
            ('run_b', grid_b, poses_b, np.linalg.inv(matrix(ODOM_B_IN_A)))):
        os.makedirs(os.path.join(directory, name), exist_ok=True)
        path = os.path.join(directory, name, 'grid.npz')
        grid.save(path, poses=np.array(poses), world_from_odom=world_from_odom)
        paths.append(path)
    return paths
