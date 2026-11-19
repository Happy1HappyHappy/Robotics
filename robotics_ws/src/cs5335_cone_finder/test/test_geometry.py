# Copyright 2026 Claire Liu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests of the geometry module (no ROS needed)."""

import math
from types import SimpleNamespace

from cs5335_cone_finder import geometry as geo
import numpy as np
import pytest

LIMITS = geo.MotionLimits()


def test_motion_limits_defaults_are_slow():
    assert LIMITS.max_linear <= 0.31
    assert LIMITS.max_angular <= 1.9
    assert 0.0 < LIMITS.min_linear < LIMITS.max_linear


@pytest.mark.parametrize('angle, wrapped', [
    (0.0, 0.0), (math.pi, -math.pi), (-math.pi, -math.pi),
    (3.0 * math.pi / 2.0, -math.pi / 2.0),
    (-3.0 * math.pi / 2.0, math.pi / 2.0), (4.0 * math.pi + 0.1, 0.1)])
def test_wrap_angle(angle, wrapped):
    assert geo.wrap_angle(angle) == pytest.approx(wrapped)


def test_yaw_from_quaternion():
    yaw = 0.7
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
    assert geo.yaw_from_quaternion(q) == pytest.approx(yaw)


def test_transform_and_apply_round_trip():
    t = geo.transform(1.0, 2.0, math.pi / 2.0)
    moved = geo.apply(t, [[1.0, 0.0]])
    assert moved[0] == pytest.approx([1.0, 3.0])
    back = geo.apply(np.linalg.inv(t), moved)
    assert back[0] == pytest.approx([1.0, 0.0])


def test_scan_points_drops_invalid_returns():
    ranges = [1.0, math.inf, math.nan, 0.01, 20.0, 2.0]
    points = geo.scan_points(ranges, 0.0, math.pi / 2.0, 0.15, 12.0)
    assert points.shape == (2, 2)
    assert points[0] == pytest.approx([1.0, 0.0])
    angle = 5.0 * math.pi / 2.0
    assert points[1] == pytest.approx([2.0 * math.cos(angle), 2.0 * math.sin(angle)])


def test_nearest_cluster_keeps_the_cone_not_the_wall_behind():
    cone = [[1.0, -0.03], [1.0, 0.0], [1.02, 0.03]]
    wall = [[2.0, y] for y in np.linspace(-0.3, 0.3, 7)]
    elsewhere = [[0.5, 0.5]]   # nearer, but in another direction
    cluster = geo.nearest_cluster(cone + wall + elsewhere, 0.0, 0.1, 0.15)
    assert len(cluster) == 3
    assert np.all(cluster[:, 0] < 1.1)


def test_nearest_cluster_empty_when_nothing_that_way():
    assert geo.nearest_cluster([[1.0, 0.0]], math.pi, 0.1, 0.15).shape == (0, 2)


def test_nearest_cluster_across_the_wrap():
    cluster = geo.nearest_cluster([[-1.0, 0.01], [-1.0, -0.01]], math.pi, 0.05, 0.1)
    assert len(cluster) == 2


def test_gap_to_uses_the_nearest_point():
    assert geo.gap_to([[1.0, 0.0], [0.0, 0.5]], 0.171) == pytest.approx(0.5 - 0.171)


def test_turn_command_is_proportional_and_clamped():
    assert geo.turn_command(0.2, LIMITS) == pytest.approx(0.3)
    assert geo.turn_command(3.0, LIMITS) == pytest.approx(LIMITS.max_angular)
    assert geo.turn_command(-3.0, LIMITS) == pytest.approx(-LIMITS.max_angular)
    assert geo.turn_command(0.01, LIMITS) == pytest.approx(LIMITS.min_angular)
    assert geo.turn_command(-0.01, LIMITS) == pytest.approx(-LIMITS.min_angular)


def test_approach_command_stops_at_the_gap():
    assert geo.approach_command(0.0, 0.05, 0.085, LIMITS) == (0.0, 0.0)


def test_approach_command_turns_first_when_far_off():
    v, w = geo.approach_command(1.0, 1.0, 0.085, LIMITS)
    assert v == 0.0
    assert w > 0.0


def test_approach_command_slows_near_the_target():
    far_v, _ = geo.approach_command(0.0, 2.0, 0.085, LIMITS)
    near_v, _ = geo.approach_command(0.0, 0.1, 0.085, LIMITS)
    assert far_v == pytest.approx(LIMITS.max_linear)
    assert near_v == pytest.approx(LIMITS.min_linear)


def test_approach_command_steers_toward_the_target():
    v, w = geo.approach_command(-0.2, 1.0, 0.085, LIMITS)
    assert v > 0.0
    assert w < 0.0


def test_path_clear():
    assert geo.path_clear([[0.3, 0.0]], 0.4, 0.17) is False
    assert geo.path_clear([[0.5, 0.0]], 0.4, 0.17) is True    # beyond reach
    assert geo.path_clear([[0.3, 0.3]], 0.4, 0.17) is True    # beside the strip
    assert geo.path_clear([[-0.3, 0.0]], 0.4, 0.17) is True   # behind
    assert geo.path_clear(np.zeros((0, 2)), 0.4, 0.17) is True


def test_simulated_drive_reaches_a_point():
    """Drive a unicycle model with approach_command and check it arrives within limits."""
    x, y, yaw, dt = 0.0, 0.0, 0.0, 0.05
    goal = (1.0, 0.6)
    for _ in range(2000):
        t_base_odom = np.linalg.inv(geo.transform(x, y, yaw))
        tx, ty = geo.apply(t_base_odom, [goal])[0]
        v, w = geo.approach_command(math.atan2(ty, tx), math.hypot(tx, ty), 0.05, LIMITS)
        assert 0.0 <= v <= LIMITS.max_linear
        assert abs(w) <= LIMITS.max_angular
        if v == 0.0 and w == 0.0:
            break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
    assert math.hypot(goal[0] - x, goal[1] - y) <= 0.05
