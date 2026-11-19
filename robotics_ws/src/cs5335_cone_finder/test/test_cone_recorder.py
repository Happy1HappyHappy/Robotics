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

"""Tests of the cone_recorder module (no ROS needed)."""

import math

from cs5335_cone_finder.cone_recorder import Candidate, ConeRecorder, Sighting


def recorder(max_cones=3, min_sightings=2):
    """Return a recorder with the node's default settings."""
    return ConeRecorder(max_cones=max_cones, same_cone_angle=0.35, band=0.3,
                        center_tolerance=0.09, min_sightings=min_sightings)


def sight(bearing, offset=0.0, color='orange'):
    """Return a sighting of a cone 0.05 rad wide."""
    return Sighting(bearing, offset, 0.05, color)


def test_sightings_outside_the_band_do_not_count():
    r = recorder()
    assert r.add(sight(1.0, offset=0.5)) is None
    assert r.cones() == []


def test_sightings_of_one_cone_are_grouped():
    r = recorder()
    a = r.add(sight(1.0, offset=0.2))
    b = r.add(sight(1.05, offset=0.1))
    assert a is b
    assert a.sightings() == 2
    assert a.best.offset == 0.1     # the more centered sighting


def test_cones_need_min_sightings():
    r = recorder(min_sightings=2)
    r.add(sight(1.0))
    assert r.cones() == []
    r.add(sight(1.0))
    assert len(r.cones()) == 1


def test_first_cones_seen_are_kept_in_order():
    r = recorder(max_cones=2, min_sightings=1)
    for bearing in (0.5, 2.0, -2.0):
        r.add(sight(bearing))
    assert [c.best.bearing for c in r.cones()] == [0.5, 2.0]


def test_one_off_false_detection_does_not_take_a_slot():
    r = recorder(max_cones=2, min_sightings=2)
    r.add(sight(0.5))               # seen once only
    for bearing in (2.0, 2.0, -2.0, -2.0):
        r.add(sight(bearing))
    assert [c.best.bearing for c in r.cones()] == [2.0, -2.0]


def test_complete_needs_centered_sightings():
    r = recorder(max_cones=1, min_sightings=1)
    r.add(sight(1.0, offset=0.2))
    assert not r.complete()
    r.add(sight(1.0, offset=0.05))
    assert r.complete()


def test_bearings_match_across_pi():
    r = recorder(min_sightings=1)
    a = r.add(sight(math.pi - 0.05))
    b = r.add(sight(-math.pi + 0.05))
    assert a is b


def test_color_is_the_majority_vote():
    r = recorder(min_sightings=1)
    for color in ('orange', 'red', 'orange'):
        cone = r.add(sight(1.0, color=color))
    assert cone.color() == 'orange'


def test_color_tie_goes_to_the_best_sighting():
    c = Candidate(0, sight(1.0, offset=0.01, color='red'))
    c.votes.update(['orange', 'red'])
    assert c.color() == 'red'
