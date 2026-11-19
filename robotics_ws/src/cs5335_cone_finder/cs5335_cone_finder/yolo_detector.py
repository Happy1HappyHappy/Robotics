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
Find traffic cones in images with an ultralytics YOLO model and name their colors.

The standard (COCO) YOLO classes have no traffic cone, so the default model is YOLO-World,
which finds objects from a text prompt ("traffic cone") without training. A model trained
on cones works too. No ROS imports; ultralytics is imported only when a model is loaded by
name, so the rest of the package (and its tests) work without it.
"""

from dataclasses import dataclass
from typing import Optional

from cs5335_cone_finder.color import classify_color, ColorSettings, inner_region
import numpy as np


@dataclass
class ConeBox:
    """
    One cone found in an image.

    x1, y1 is the box's top-left corner and x2, y2 its bottom-right one, in pixels with the
    pixel edges at whole numbers (as ultralytics reports them). confidence is the
    detector's score (0-1), color the color name or None if the pixels show no clear color,
    and color_fraction the fraction of the pixels looked at that have that color.
    """

    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    color: Optional[str]
    color_fraction: float


def _to_numpy(values):
    """Return a torch tensor or array-like as a NumPy array."""
    if hasattr(values, 'cpu'):
        values = values.cpu().numpy()
    return np.asarray(values)


class YoloConeDetector:
    """
    Find traffic cones in BGR images and name their colors.

    model is an ultralytics model, or the name or path of one to load. classes are the
    class names that count as cones: a YOLO-World model gets them as its text prompts; for
    any other model they pick which of its classes are cones. confidence is the minimum
    score (0-1), image_size the size (px) images are scaled to, and device where the model
    runs ('cpu', '0' for the first GPU, or '' to let ultralytics choose).
    """

    def __init__(self, model, classes, confidence=0.2, image_size=320, device='',
                 colors=None):
        if isinstance(model, str):
            from ultralytics import YOLO
            model = YOLO(model)
        if hasattr(model, 'set_classes'):   # YOLO-World (and YOLOE): classes are prompts
            model.set_classes(list(classes))
        self._model = model
        self._classes = set(classes)
        self._options = {'imgsz': image_size, 'conf': confidence, 'verbose': False}
        if device:
            self._options['device'] = device
        self._colors = colors if colors is not None else ColorSettings()

    def detect(self, bgr):
        """Return the cones (a list of ConeBox) in a BGR image."""
        result = self._model.predict(bgr, **self._options)[0]
        names = result.names
        xyxy = _to_numpy(result.boxes.xyxy).reshape(-1, 4)
        scores = _to_numpy(result.boxes.conf).reshape(-1)
        labels = _to_numpy(result.boxes.cls).reshape(-1).astype(int)
        height, width = bgr.shape[:2]
        cones = []
        for (x1, y1, x2, y2), score, label in zip(xyxy, scores, labels):
            if names[label] not in self._classes:
                continue
            left, top, right, bottom = inner_region((x1, y1, x2, y2), self._colors.region,
                                                    width, height)
            color, fraction = classify_color(bgr[top:bottom, left:right], self._colors)
            cones.append(ConeBox(float(x1), float(y1), float(x2), float(y2), float(score),
                                 color, fraction))
        return cones
