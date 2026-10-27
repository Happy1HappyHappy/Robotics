"""
Tests of the occupancy grid (no ROS, no robot): python3 -m pytest test/test_grid.py
"""

import math

import numpy as np
import pytest

from day6pkg.helpers.occupancy_grid import bresenham, log_odds, OccupancyGrid


@pytest.mark.parametrize('end', [(5, 2), (2, 5), (-5, 2), (2, -5), (-3, -3), (0, 4), (4, 0)])
def test_bresenham_is_a_connected_line(end):
    cells = bresenham(0, 0, *end)
    assert cells[0] == (0, 0)
    assert end not in cells
    assert len(cells) == max(abs(end[0]), abs(end[1]))
    for (r0, c0), (r1, c1) in zip(cells, cells[1:] + [end]):
        assert max(abs(r1 - r0), abs(c1 - c0)) == 1     # each step goes to a neighbor


def test_bresenham_same_cell_is_empty():
    assert bresenham(3, 3, 3, 3) == []


def test_world_to_cell_puts_forward_up_and_left_left():
    g = OccupancyGrid(200, 0.05)
    assert g.world_to_cell(0.0, 0.0) == (99, 99)
    assert g.world_to_cell(0.06, 0.0) == (98, 99)     # +x (forward) = up = smaller row
    assert g.world_to_cell(0.0, 0.06) == (99, 98)     # +y (left) = smaller col
    assert g.world_to_cell(-0.01, -0.01) == (100, 100)
    assert g.world_to_cell(4.99, 4.99) == (0, 0)
    assert g.in_bounds(*g.world_to_cell(-4.99, -4.99))
    assert not g.in_bounds(*g.world_to_cell(5.0, 0.0))


def test_one_ray_marks_free_cells_then_one_occupied_cell():
    g = OccupancyGrid(200, 0.05)
    used = g.update((0.0, 0.0, 0.0), [1.02], 0.0, 0.0, 0.1, 10.0)
    assert used == 1
    hit = g.world_to_cell(1.02, 0.0)
    assert g.log_odds[hit] == pytest.approx(log_odds(0.9))
    for row in range(hit[0] + 1, 100):
        assert g.log_odds[row, 99] == pytest.approx(log_odds(0.1))
    assert (g.log_odds != 0).sum() == 100 - hit[0]      # nothing else changed


def test_pose_rotates_and_moves_the_rays():
    g = OccupancyGrid(200, 0.05)
    # sensor 1 m forward, facing left: a ray at angle 0 hits 0.52 m to the left
    g.update((1.0, 0.0, math.pi / 2), [0.52], 0.0, 0.0, 0.1, 10.0)
    assert g.log_odds[g.world_to_cell(1.0, 0.52)] > 0
    assert g.log_odds[g.world_to_cell(1.0, 0.2)] < 0


def test_invalid_ranges_are_skipped():
    g = OccupancyGrid(200, 0.05)
    used = g.update((0.0, 0.0, 0.0), [float('inf'), float('nan'), 0.05, 20.0, 1.0],
                    0.0, 0.1, 0.15, 12.0)
    assert used == 1


def test_occupied_wins_and_each_cell_counts_once_per_scan():
    g = OccupancyGrid(200, 0.05)
    # two rays along +x: one ends at 0.52 m, the other passes it and ends at 1.02 m;
    # three more identical rays must not add more than one update per cell
    g.update((0.0, 0.0, 0.0), [0.52, 1.02, 1.02, 1.02, 1.02], 0.0, 0.0, 0.1, 10.0)
    assert g.log_odds[g.world_to_cell(0.52, 0.0)] == pytest.approx(log_odds(0.9))
    assert g.log_odds[g.world_to_cell(1.02, 0.0)] == pytest.approx(log_odds(0.9))
    assert g.log_odds[g.world_to_cell(0.3, 0.0)] == pytest.approx(log_odds(0.1))


def test_log_odds_are_clamped():
    g = OccupancyGrid(200, 0.05, clamp=2.0)
    for _ in range(10):
        g.update((0.0, 0.0, 0.0), [1.0], 0.0, 0.0, 0.1, 10.0)
    assert g.log_odds.max() == pytest.approx(2.0)
    assert g.log_odds.min() == pytest.approx(-2.0)
    assert g.probability().max() == pytest.approx(1.0 - 1.0 / 101.0)


def test_save_and_load_keep_the_grid_and_the_extra_arrays(tmp_path):
    g = OccupancyGrid(20, 0.05)
    g.log_odds[3, 4] = 2.5
    poses = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])
    g.save(tmp_path / 'grid.npz', poses=poses, world_from_odom=np.eye(3))
    loaded, extra = OccupancyGrid.load(tmp_path / 'grid.npz')
    assert loaded.size == 20 and loaded.resolution == 0.05
    assert np.array_equal(loaded.log_odds, g.log_odds)
    assert np.array_equal(extra['poses'], poses)
    assert np.array_equal(extra['world_from_odom'], np.eye(3))


@pytest.mark.parametrize('size', [300, 5])
def test_ros_layout_puts_each_cell_at_its_world_position(size):
    g = OccupancyGrid(size, 0.05)
    g.log_odds[g.world_to_cell(0.06, -0.03)] = 3.0
    values, (x0, y0) = g.ros_layout()
    assert values.dtype == np.int8 and values.shape == (size, size)
    assert (values == -1).sum() == size * size - 1      # everything else unknown
    j, i = np.argwhere(values == 100)[0]                # ROS: data[j * width + i]
    assert x0 + (i + 0.5) * 0.05 == pytest.approx(0.075)    # center of the cell holding x
    assert y0 + (j + 0.5) * 0.05 == pytest.approx(-0.025)


def test_ros_layout_round_trip_keeps_the_occupied_threshold():
    g = OccupancyGrid(10, 0.05)
    g.log_odds[2, 3], g.log_odds[4, 5], g.log_odds[6, 7] = 0.954, 1.908, -3.0
    values, _ = g.ros_layout()
    back = OccupancyGrid.from_ros_layout(values.ravel().tolist(), 0.05)
    assert back.size == 10 and back.log_odds[0, 0] == 0.0      # unknown stays 0
    assert back.log_odds[2, 3] < 1.0 < back.log_odds[4, 5]     # one hit vs two hits
    assert back.log_odds[6, 7] == pytest.approx(-3.0)
