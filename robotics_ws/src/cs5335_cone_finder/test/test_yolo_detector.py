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

"""Tests of the yolo_detector module with a stand-in model (no ROS or ultralytics needed)."""

from types import SimpleNamespace

from cs5335_cone_finder.yolo_detector import YoloConeDetector
import cv2
import numpy as np
import pytest


class FakeTensor:
    """Stand-in for a torch tensor: has cpu() and numpy()."""

    def __init__(self, values):
        self._values = np.asarray(values, dtype=float)

    def cpu(self):
        """Return itself, as a CPU tensor would."""
        return self

    def numpy(self):
        """Return the values."""
        return self._values


class FakeModel:
    """Stand-in for an ultralytics model that returns fixed boxes."""

    def __init__(self, boxes, names, world=False):
        self._boxes = boxes
        self.names = names
        self.prompts = None
        self.options = None
        if world:
            self.set_classes = self._set_classes

    def _set_classes(self, classes):
        self.prompts = classes

    def predict(self, image, **options):
        """Return one result holding the fixed boxes."""
        self.options = options
        xyxy = [b[:4] for b in self._boxes]
        result = SimpleNamespace(
            names=self.names,
            boxes=SimpleNamespace(xyxy=FakeTensor(xyxy),
                                  conf=FakeTensor([b[4] for b in self._boxes]),
                                  cls=FakeTensor([b[5] for b in self._boxes])))
        return [result]


def image_with_orange_cone():
    """Return a gray 100 x 100 image with an orange block at x 40-60, y 30-80."""
    hsv = np.full((100, 100, 3), (0, 0, 120), np.uint8)
    hsv[30:80, 40:60] = (12, 220, 200)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def test_world_model_gets_the_prompts_and_options():
    model = FakeModel([], {0: 'traffic cone'}, world=True)
    detector = YoloConeDetector(model, ['traffic cone'], confidence=0.3, image_size=256,
                                device='cpu')
    assert detector.detect(image_with_orange_cone()) == []
    assert model.prompts == ['traffic cone']
    assert model.options == {'imgsz': 256, 'conf': 0.3, 'verbose': False, 'device': 'cpu'}


def test_cone_gets_its_box_and_color():
    model = FakeModel([(40.0, 30.0, 60.0, 80.0, 0.8, 0)], {0: 'traffic cone'})
    cones = YoloConeDetector(model, ['traffic cone']).detect(image_with_orange_cone())
    assert len(cones) == 1
    cone = cones[0]
    assert (cone.x1, cone.y1, cone.x2, cone.y2) == (40.0, 30.0, 60.0, 80.0)
    assert cone.confidence == pytest.approx(0.8)
    assert cone.color == 'orange'
    assert 'device' not in model.options


def test_other_classes_are_left_out():
    model = FakeModel([(40.0, 30.0, 60.0, 80.0, 0.9, 1)], {0: 'traffic cone', 1: 'person'})
    assert YoloConeDetector(model, ['traffic cone']).detect(image_with_orange_cone()) == []


def test_box_on_gray_has_no_color():
    model = FakeModel([(0.0, 0.0, 30.0, 30.0, 0.5, 0)], {0: 'traffic cone'})
    cones = YoloConeDetector(model, ['traffic cone']).detect(image_with_orange_cone())
    assert cones[0].color is None
