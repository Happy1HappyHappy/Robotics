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

"""Tests of the control module (no ROS needed)."""

import math
from types import SimpleNamespace

from cs5335_cone_navigation import control as ctl
import numpy as np
import pytest

LIMITS = ctl.MotionLimits()


@pytest.mark.parametrize('angle, wrapped', [
    (0.0, 0.0), (math.pi, -math.pi), (3.0 * math.pi / 2.0, -math.pi / 2.0),
    (-3.0 * math.pi / 2.0, math.pi / 2.0), (4.0 * math.pi + 0.1, 0.1)])
def test_wrap_angle(angle, wrapped):
    assert ctl.wrap_angle(angle) == pytest.approx(wrapped)


def test_yaw_from_quaternion():
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.35), w=math.cos(0.35))
    assert ctl.yaw_from_quaternion(q) == pytest.approx(0.7)


def test_transform_and_apply():
    t = ctl.transform(1.0, 2.0, math.pi / 2.0)
    assert ctl.apply(t, [[1.0, 0.0]])[0] == pytest.approx([1.0, 3.0])
    assert ctl.apply(np.linalg.inv(t), [[1.0, 3.0]])[0] == pytest.approx([1.0, 0.0])


def test_scan_points_drops_invalid_returns():
    points = ctl.scan_points([1.0, math.inf, math.nan, 0.01, 20.0], 0.0, 0.1, 0.15, 12.0)
    assert points.tolist() == [[1.0, 0.0]]


def test_gap_to():
    assert ctl.gap_to([[1.0, 0.0], [0.0, 0.5]], 0.171) == pytest.approx(0.329)


def test_turn_command():
    assert ctl.turn_command(0.2, LIMITS) == pytest.approx(0.2)
    assert ctl.turn_command(2.0, LIMITS) == pytest.approx(LIMITS.max_angular)
    assert ctl.turn_command(-0.01, LIMITS) == pytest.approx(-LIMITS.min_angular)


def test_approach_command():
    assert ctl.approach_command(0.0, 0.05, 0.08, LIMITS) == (0.0, 0.0)
    v, w = ctl.approach_command(1.0, 1.0, 0.08, LIMITS)
    assert v == 0.0 and w > 0.0                     # turn in place first
    assert ctl.approach_command(0.0, 2.0, 0.08, LIMITS)[0] == pytest.approx(LIMITS.max_linear)
    assert ctl.approach_command(0.0, 0.1, 0.08, LIMITS)[0] == pytest.approx(LIMITS.min_linear)
    assert ctl.approach_command(-0.2, 1.0, 0.08, LIMITS)[1] < 0.0


def test_path_clear():
    assert ctl.path_clear([[0.3, 0.0]], 0.4, 0.17) is False
    assert ctl.path_clear([[0.5, 0.0], [0.3, 0.3], [-0.3, 0.0]], 0.4, 0.17) is True


def test_simulated_approach_stops_at_the_gap():
    """Drive a unicycle model toward a point and check the stop gap and the limits."""
    x, y, yaw, dt = 0.0, 0.0, 0.0, 0.05
    cone, radius, stop = np.array([1.2, 0.4]), 0.171, 0.0754
    for _ in range(3000):
        rel = ctl.apply(np.linalg.inv(ctl.transform(x, y, yaw)), cone)[0]
        gap = float(np.hypot(*rel)) - radius
        v, w = ctl.approach_command(math.atan2(rel[1], rel[0]), gap, stop, LIMITS)
        assert 0.0 <= v <= LIMITS.max_linear and abs(w) <= LIMITS.max_angular
        if v == 0.0 and w == 0.0:
            break
        x, y, yaw = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt
    assert stop - LIMITS.max_linear * dt <= gap <= stop
