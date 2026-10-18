"""
Offline tests of the occupancy grid (no ROS, no robot): python3 -m pytest test/test_grid.py
"""

import math

import pytest

from day6pkg.grid import bresenham, log_odds, OccupancyGrid
from day6pkg.offline import load_scan


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


def test_load_scan_reads_echo_output(tmp_path):
    f = tmp_path / 'scan.yaml'
    f.write_text('header:\n  stamp:\n    sec: 1\n    nanosec: 2\n  frame_id: rplidar_link\n'
                 'angle_min: -3.14\nangle_max: 3.14\nangle_increment: 0.01\n'
                 'time_increment: 0.0\nscan_time: 0.1\nrange_min: 0.15\nrange_max: 12.0\n'
                 'ranges:\n- 1.5\n- .inf\n- .nan\nintensities:\n- 47.0\n- 0.0\n- 0.0\n---\n')
    scan = load_scan(str(f))
    assert scan['ranges'][0] == 1.5
    assert math.isinf(scan['ranges'][1]) and math.isnan(scan['ranges'][2])
    assert scan['header']['frame_id'] == 'rplidar_link'


def test_load_scan_rejects_truncated_ranges(tmp_path):
    f = tmp_path / 'scan.yaml'
    f.write_text("angle_min: 0.0\nranges:\n- 1.5\n- '...'\n---\n")
    with pytest.raises(ValueError, match='full-length'):
        load_scan(str(f))
