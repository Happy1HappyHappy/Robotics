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
Where the robot was at a given time, from its recent odometry poses.

A camera image or LiDAR scan was taken some time before it is used, and YOLO takes a
noticeable part of a second per image. While the robot turns, the direction of a cone must
be added to the robot's heading at the moment the image was taken, not when the result
arrived. No ROS imports.
"""

import bisect
from dataclasses import dataclass

from cs5335_cone_finder.geometry import wrap_angle


@dataclass
class PlanarPose:
    """A robot pose in the odom frame: x and y in m, yaw in rad."""

    x: float
    y: float
    yaw: float


class PoseHistory:
    """
    Recent robot poses with their timestamps (s), oldest first.

    Poses older than max_age s before the newest one are dropped. Stamps must increase;
    a pose whose stamp is not newer than the last one is ignored.
    """

    def __init__(self, max_age=5.0):
        self._max_age = max_age
        self._stamps = []
        self._poses = []

    def add(self, stamp, pose):
        """Add a pose (PlanarPose) with its timestamp (s)."""
        if self._stamps and stamp <= self._stamps[-1]:
            return
        self._stamps.append(stamp)
        self._poses.append(pose)
        old = bisect.bisect_left(self._stamps, stamp - self._max_age)
        if old > 0:
            del self._stamps[:old]
            del self._poses[:old]

    def latest(self):
        """Return the newest pose, or None if there is none."""
        return self._poses[-1] if self._poses else None

    def pose_at(self, stamp, tolerance):
        """
        Return the pose at a time (s), interpolated between the two poses around it.

        A time up to tolerance s before the oldest or after the newest pose gets that pose.
        The result is None if the time is farther outside the history, or it is empty.
        """
        if not self._stamps:
            return None
        if stamp < self._stamps[0] - tolerance or stamp > self._stamps[-1] + tolerance:
            return None
        if stamp <= self._stamps[0]:
            return self._poses[0]
        if stamp >= self._stamps[-1]:
            return self._poses[-1]
        i = bisect.bisect_right(self._stamps, stamp)
        t0, t1 = self._stamps[i - 1], self._stamps[i]
        p0, p1 = self._poses[i - 1], self._poses[i]
        f = (stamp - t0) / (t1 - t0)
        return PlanarPose(p0.x + f * (p1.x - p0.x), p0.y + f * (p1.y - p0.y),
                          wrap_angle(p0.yaw + f * wrap_angle(p1.yaw - p0.yaw)))
