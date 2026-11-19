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

"""Tests of the pose_history module (no ROS needed)."""

import math

from cs5335_cone_finder.pose_history import PlanarPose, PoseHistory
import pytest


def test_empty_history():
    history = PoseHistory()
    assert history.latest() is None
    assert history.pose_at(1.0, 0.1) is None


def test_latest_and_interpolation():
    history = PoseHistory()
    history.add(10.0, PlanarPose(0.0, 0.0, 0.0))
    history.add(11.0, PlanarPose(1.0, 2.0, 0.4))
    assert history.latest() == PlanarPose(1.0, 2.0, 0.4)
    mid = history.pose_at(10.25, 0.1)
    assert (mid.x, mid.y, mid.yaw) == pytest.approx((0.25, 0.5, 0.1))


def test_yaw_interpolates_the_short_way_across_pi():
    history = PoseHistory()
    history.add(0.0, PlanarPose(0.0, 0.0, math.pi - 0.1))
    history.add(1.0, PlanarPose(0.0, 0.0, -math.pi + 0.1))
    assert abs(history.pose_at(0.5, 0.1).yaw) == pytest.approx(math.pi)


def test_stamps_just_outside_get_the_end_pose_and_far_outside_none():
    history = PoseHistory()
    history.add(10.0, PlanarPose(0.0, 0.0, 0.0))
    history.add(11.0, PlanarPose(1.0, 0.0, 0.0))
    assert history.pose_at(11.05, 0.1).x == pytest.approx(1.0)
    assert history.pose_at(9.95, 0.1).x == pytest.approx(0.0)
    assert history.pose_at(11.5, 0.1) is None
    assert history.pose_at(9.0, 0.1) is None


def test_old_poses_are_dropped():
    history = PoseHistory(max_age=1.0)
    for i in range(30):
        history.add(i * 0.1, PlanarPose(float(i), 0.0, 0.0))
    assert history.pose_at(0.0, 0.1) is None
    assert history.pose_at(2.5, 0.1).x == pytest.approx(25.0)


def test_stamps_that_go_back_are_ignored():
    history = PoseHistory()
    history.add(10.0, PlanarPose(0.0, 0.0, 0.0))
    history.add(9.0, PlanarPose(5.0, 0.0, 0.0))
    history.add(10.0, PlanarPose(6.0, 0.0, 0.0))
    assert history.latest() == PlanarPose(0.0, 0.0, 0.0)
