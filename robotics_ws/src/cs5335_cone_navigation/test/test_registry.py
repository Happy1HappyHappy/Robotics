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

"""Tests of the registry module (no ROS needed)."""

import math

from cs5335_cone_navigation.registry import ConeRegistry, Observation
import numpy as np
import pytest

ROBOT = np.array([0.0, 0.0])


def registry(min_observations=1):
    """Return a registry with the node's default thresholds."""
    return ConeRegistry(association_distance=0.25, association_angle=0.2,
                        min_observations=min_observations)


def seen(color, x=None, y=None, bearing=None, confidence=0.9):
    """Return an observation, with a LiDAR position if x and y are given."""
    position = None if x is None else np.array([x, y])
    if bearing is None:
        bearing = math.atan2(y, x)
    return Observation(color, bearing, position, ROBOT, confidence)


def test_the_same_cone_seen_again_keeps_one_id():
    r = registry()
    for dx in (0.0, 0.05, -0.04):
        r.add(seen('red', 1.0 + dx, 0.5))
    assert [c.cone_id for c in r.confirmed()] == [1]


def test_two_cones_of_the_same_color_get_two_ids():
    r = registry()
    r.add(seen('red', 1.0, 0.5))
    r.add(seen('red', -1.0, 0.8))
    cones = r.confirmed()
    assert [c.cone_id for c in cones] == [1, 2]
    assert all(c.color == 'red' for c in cones)


def test_cones_of_different_colors_never_merge():
    r = registry()
    r.add(seen('red', 1.0, 0.5))
    r.add(seen('blue', 1.0, 0.5))
    assert len(r.confirmed()) == 2


def test_an_id_needs_min_observations():
    r = registry(min_observations=3)
    r.add(seen('green', 1.0, 0.0))
    r.add(seen('green', 1.0, 0.0))
    assert r.confirmed() == []
    r.add(seen('green', 1.0, 0.0))
    assert r.confirmed()[0].cone_id == 1


def test_position_is_the_confidence_weighted_mean():
    r = registry()
    r.add(seen('red', 1.0, 0.0, confidence=0.75))
    cone = r.add(seen('red', 1.2, 0.0, confidence=0.25))
    assert cone.position == pytest.approx([1.05, 0.0])
    assert cone.confidence == pytest.approx(0.75)


def test_observations_without_lidar_match_by_direction():
    r = registry()
    r.add(seen('blue', bearing=0.5))
    cone = r.add(seen('blue', bearing=0.55))
    assert len(r.confirmed()) == 1
    assert not cone.distance_valid()
    r.add(seen('blue', bearing=1.5))            # another direction: another cone
    assert len(r.confirmed()) == 2


def test_a_lidar_position_fills_in_a_cone_seen_without_one():
    r = registry()
    r.add(seen('red', bearing=math.atan2(0.5, 1.0)))
    cone = r.add(seen('red', 1.0, 0.5))
    assert len(r.confirmed()) == 1
    assert cone.distance_valid()


def test_get():
    r = registry()
    r.add(seen('red', 1.0, 0.0))
    assert r.get(1).color == 'red'
    assert r.get(2) is None
