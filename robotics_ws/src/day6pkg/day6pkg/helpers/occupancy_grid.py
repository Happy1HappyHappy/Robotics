"""
Occupancy grid built from LiDAR scans: log-odds cells, Bresenham ray casting
Pure NumPy (no ROS), so it can be tested and used without the robot
"""

import math

import numpy as np


def log_odds(p):
    """
    Base-10 log-odds of probability p, as in the slides: L = log10(p / (1 - p))
    """
    return math.log10(p / (1.0 - p))


def bresenham(r0, c0, r1, c1):
    """
    Grid cells (row, col) on the line from (r0, c0) up to but not including (r1, c1)
    Integer-only ray casting; works in every direction
    """
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
    err = dr - dc
    r, c = r0, c0
    while (r, c) != (r1, c1):
        cells.append((r, c))
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


class OccupancyGrid:
    """
    Square occupancy grid centered on the world origin, one log-odds value per cell
    World frame: x forward and y left as the robot faced at the start (REP 103)
    The array is laid out like the image it is drawn as: row 0 is the far +x edge (top),
    col 0 is the far +y edge (left), so the robot's start heading points up
    L = 0 is unknown (p = 0.5), L > 0 leans occupied, L < 0 leans free
    """
    def __init__(self, size=200, resolution=0.05, p_free=0.1, p_occupied=0.9, clamp=3.0):
        """
        size: cells per side (200 x 5 cm = 10 m x 10 m)
        resolution: cell side in meters
        p_free / p_occupied: probability a cell is occupied, given a ray passed through it /
        ended in it; added to the cell as log-odds
        clamp: |L| limit, so a cell can change state again later
        """
        self.size = size
        self.resolution = resolution
        self.l_free = log_odds(p_free)
        self.l_occupied = log_odds(p_occupied)
        self.clamp = clamp
        self.log_odds = np.zeros((size, size))

    def world_to_cell(self, x, y):
        """
        Cell (row, col) that holds world point (x, y), in meters
        The world origin is the corner shared by the four center cells
        The result may lie outside the grid; check it with in_bounds()
        """
        half = self.size // 2
        return (half - 1 - math.floor(x / self.resolution),
                half - 1 - math.floor(y / self.resolution))

    def in_bounds(self, row, col):
        """
        Check that (row, col) is a cell of the grid
        """
        return 0 <= row < self.size and 0 <= col < self.size

    def update(self, pose, ranges, angle_min, angle_increment, range_min, range_max):
        """
        Add one LiDAR scan taken from sensor pose (x, y, yaw) in the world frame (m, rad)
        Ray k points at angle_min + k * angle_increment in the sensor frame (CCW, 0 = sensor x)
        Each valid ray marks the cells it passes through as free and its end cell as occupied
        Every cell is updated at most once per scan; if any ray ends in a cell, occupied wins
        Ranges outside [range_min, range_max], inf and nan are skipped
        Returns the number of rays used
        """
        x, y, yaw = pose
        r0, c0 = self.world_to_cell(x, y)
        free = np.zeros(self.log_odds.shape, dtype=bool)
        occupied = np.zeros(self.log_odds.shape, dtype=bool)
        used = 0    # number of rays that were valid and used to update the grid
        for k, r in enumerate(ranges):
            if not range_min <= r <= range_max:     # also False for nan
                continue
            a = yaw + angle_min + k * angle_increment  # angle of ray k in the world frame
            # Calculate the end point of the ray in the world frame
            r1, c1 = self.world_to_cell(x + r * math.cos(a), y + r * math.sin(a))
            # Mark the cells along the ray as free, and the end cell as occupied
            for row, col in bresenham(r0, c0, r1, c1):
                if self.in_bounds(row, col):
                    free[row, col] = True
            if self.in_bounds(r1, c1):
                occupied[r1, c1] = True
            used += 1
        free &= ~occupied   # only mark free if free and not occupied
        # numpy boolean indexing: add to all cells where free is True
        self.log_odds[free] += self.l_free
        self.log_odds[occupied] += self.l_occupied
        # clamp the log-odds to avoid runaway certainty
        np.clip(self.log_odds, -self.clamp, self.clamp, out=self.log_odds)
        return used

    def probability(self):
        """
        Occupancy probability of every cell: p = 1 - 1 / (1 + 10^L)
        """
        return 1.0 - 1.0 / (1.0 + np.power(10.0, self.log_odds))

    def ros_layout(self):
        """
        The grid laid out as nav_msgs/OccupancyGrid wants it: (values, origin)
        values: size x size int8, occupancy probability in percent (-1 = unknown, L = 0);
        row j counts from the -y edge and column i from the -x edge, so data[j * size + i]
        origin: world (x, y) in meters of the outer corner of cell (i, j) = (0, 0)
        """
        values = np.round(self.probability() * 100.0).astype(np.int8)
        values[self.log_odds == 0.0] = -1
        corner = (self.size // 2 - self.size) * self.resolution
        return np.ascontiguousarray(values[::-1, ::-1].T), (corner, corner)

    @classmethod
    def from_ros_layout(cls, values, resolution):
        """
        Grid from a nav_msgs/OccupancyGrid laid out by ros_layout() (values: size * size)
        Percentages become log-odds again, so thresholds on log-odds still apply
        """
        size = math.isqrt(len(values))
        percent = np.asarray(values, dtype=float).reshape(size, size).T[::-1, ::-1]
        p = np.where(percent < 0.0, 50.0, percent) / 100.0   # -1 = unknown: p = 0.5, L = 0
        grid = cls(size, resolution)
        with np.errstate(divide='ignore'):                   # p = 0 or 1: L = -inf or inf
            grid.log_odds = np.clip(np.log10(p / (1.0 - p)), -grid.clamp, grid.clamp)
        return grid

    def save(self, path, **extra):
        """
        Write the grid to path as a NumPy .npz file: log-odds, resolution, and any extra
        arrays passed by name (e.g. poses=...)
        """
        np.savez(path, log_odds=self.log_odds, resolution=self.resolution, **extra)

    @classmethod
    def load(cls, path):
        """
        Read a grid written by save(); returns (grid, dict of the extra arrays)
        """
        with np.load(path) as data:
            grid = cls(data['log_odds'].shape[0], float(data['resolution']))
            grid.log_odds = data['log_odds'].copy()
            extra = {k: data[k] for k in data.files if k not in ('log_odds', 'resolution')}
        return grid, extra
