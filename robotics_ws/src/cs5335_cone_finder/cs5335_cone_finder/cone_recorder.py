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

"""
Record the first cones seen while the robot turns in place.

Each image can show a cone, and the same cone shows up in many images as the robot turns.
Sightings whose directions (in the odom frame) are close together belong to one cone. A
cone counts once it has been seen min_sightings times near the middle of the image, which
filters out one-off false detections, and the cones are numbered in the order they were
first seen. No ROS imports.
"""

from collections import Counter
from dataclasses import dataclass, field

from cs5335_cone_finder.geometry import wrap_angle


@dataclass
class Sighting:
    """
    One detection of a cone in one image.

    bearing is the direction of the cone's center in the odom frame and offset its
    direction from the camera's optical axis (+ = left); half_width is half the angle the
    cone covers in the image. All are in radians. color is the color name.
    """

    bearing: float
    offset: float
    half_width: float
    color: str


@dataclass
class Candidate:
    """
    A cone seen during the scan.

    order says when it was first seen (0 = first); best is its sighting closest to the
    middle of the image, where its direction is most accurate; votes counts the colors of
    its sightings.
    """

    order: int
    best: Sighting
    votes: Counter = field(default_factory=Counter)

    def color(self):
        """Return the color most of its sightings have (on a tie, the best sighting's)."""
        top = max(self.votes.values())
        winners = [c for c, n in self.votes.items() if n == top]
        return self.best.color if self.best.color in winners else winners[0]

    def sightings(self):
        """Return how many times it was seen near the middle of the image."""
        return sum(self.votes.values())


class ConeRecorder:
    """
    Group sightings into cones and keep the first max_cones confirmed ones.

    A sighting within same_cone_angle rad of a candidate's best direction is that
    candidate; otherwise it starts a new one. Only sightings within band rad of the image
    center count. A candidate is confirmed once it has min_sightings sightings. The
    recording is complete when max_cones candidates are confirmed and each was seen within
    center_tolerance rad of the image center.
    """

    def __init__(self, max_cones, same_cone_angle, band, center_tolerance, min_sightings):
        self._max_cones = max_cones
        self._same_cone_angle = same_cone_angle
        self._band = band
        self._center_tolerance = center_tolerance
        self._min_sightings = min_sightings
        self._candidates = []

    def add(self, sighting):
        """
        Add a sighting and return the candidate it was counted for.

        The result is None if the sighting is outside the band around the image center.
        """
        if abs(sighting.offset) > self._band:
            return None
        match, closest = None, self._same_cone_angle
        for candidate in self._candidates:
            off = abs(wrap_angle(sighting.bearing - candidate.best.bearing))
            if off <= closest:
                match, closest = candidate, off
        if match is None:
            match = Candidate(len(self._candidates), sighting)
            self._candidates.append(match)
        elif abs(sighting.offset) < abs(match.best.offset):
            match.best = sighting
        match.votes[sighting.color] += 1
        return match

    def cones(self):
        """Return the first max_cones confirmed candidates, in the order first seen."""
        confirmed = [c for c in self._candidates if c.sightings() >= self._min_sightings]
        return confirmed[:self._max_cones]

    def complete(self):
        """Return whether max_cones cones are confirmed and each was seen near the center."""
        cones = self.cones()
        return (len(cones) == self._max_cones
                and all(abs(c.best.offset) <= self._center_tolerance for c in cones))
