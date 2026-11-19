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
The registry of cones seen during exploration, with unique IDs.

The same cone shows up in many camera images while the robot turns. Each observation is
matched to a known cone of the same color: by position when both have a LiDAR position,
otherwise by direction from the robot. Two cones of the same color stay separate unless
they are within the association threshold. A cone gets an ID once it has been seen
min_observations times, so one-off false detections get none. No ROS imports.
"""

from dataclasses import dataclass
import math
from typing import Optional

from cs5335_cone_navigation.control import wrap_angle
import numpy as np


@dataclass
class Observation:
    """
    One sighting of a cone, in the odom frame.

    bearing is the direction (rad) from the robot to the cone; position (x, y in m) comes
    from the LiDAR, or is None if no LiDAR return matched; robot is the robot's position
    (x, y in m); confidence is the detector's score (0-1).
    """

    color: str
    bearing: float
    position: Optional[np.ndarray]
    robot: np.ndarray
    confidence: float


@dataclass
class TrackedCone:
    """
    A cone in the registry, in the odom frame.

    position is the confidence-weighted mean of its LiDAR positions (None if the LiDAR
    never saw it, then its distance is unavailable); bearing is its last direction from the
    robot; confidence is its best score; cone_id is None until it is confirmed.
    """

    color: str
    bearing: float
    position: Optional[np.ndarray]
    confidence: float
    observations: int = 1
    weight: float = 0.0
    cone_id: Optional[int] = None

    def distance_valid(self):
        """Return whether the LiDAR measured this cone."""
        return self.position is not None


class ConeRegistry:
    """
    Cones seen so far, matched by color and position (or direction).

    association_distance (m) and association_angle (rad) are how close an observation must
    be to a cone of the same color to count as that cone.
    """

    def __init__(self, association_distance, association_angle, min_observations):
        self._distance = association_distance
        self._angle = association_angle
        self._min_observations = min_observations
        self._cones = []
        self._next_id = 1

    def _match(self, obs):
        """Return the known cone an observation belongs to, or None."""
        best, best_score = None, 1.0
        for cone in self._cones:
            if cone.color != obs.color:
                continue
            if obs.position is not None and cone.position is not None:
                score = float(np.linalg.norm(obs.position - cone.position)) / self._distance
            else:
                if cone.position is not None:
                    toward = cone.position - obs.robot
                    direction = math.atan2(toward[1], toward[0])
                else:
                    direction = cone.bearing
                score = abs(wrap_angle(obs.bearing - direction)) / self._angle
            if score <= best_score:
                best, best_score = cone, score
        return best

    def add(self, obs):
        """Add an observation and return the cone it was counted for."""
        cone = self._match(obs)
        if cone is None:
            cone = TrackedCone(obs.color, obs.bearing, None, obs.confidence, observations=0)
            self._cones.append(cone)
        cone.observations += 1
        cone.bearing = obs.bearing
        cone.confidence = max(cone.confidence, obs.confidence)
        if obs.position is not None:
            total = cone.weight + obs.confidence
            if cone.position is None or total <= 0.0:
                cone.position = np.asarray(obs.position, dtype=float)
            else:
                cone.position = (cone.position * cone.weight
                                 + np.asarray(obs.position) * obs.confidence) / total
            cone.weight = total
        if cone.cone_id is None and cone.observations >= self._min_observations:
            cone.cone_id = self._next_id
            self._next_id += 1
        return cone

    def confirmed(self):
        """Return the cones with an ID, in ID order."""
        return sorted((c for c in self._cones if c.cone_id is not None),
                      key=lambda c: c.cone_id)

    def get(self, cone_id):
        """Return the confirmed cone with an ID, or None."""
        return next((c for c in self._cones if c.cone_id == cone_id), None)
