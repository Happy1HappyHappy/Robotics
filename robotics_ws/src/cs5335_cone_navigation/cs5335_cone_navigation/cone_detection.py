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
Find traffic cones of the target colors in a camera image with classical computer vision.

Pipeline: an HSV mask per target color, opening and closing, every external contour, then
a color check and shape checks (aspect ratio, tapering, symmetry, base and contour
geometry) combined into a weighted confidence. A candidate is accepted only if it passes
every check and its confidence reaches min_confidence. Colors outside the target set get no
mask, so cones of those colors never become candidates. No ROS imports.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

# Starting HSV ranges on OpenCV's scale (H 0-179, S and V 0-255), six numbers per range:
# h_lo, s_lo, v_lo, h_hi, s_hi, v_hi. Red needs two ranges because its hue wraps through 0.
# Tune them under the lab's lights with the debug image and mask.
DEFAULT_HSV = {
    'red': [0, 100, 70, 5, 255, 255, 165, 100, 70, 179, 255, 255],
    'orange': [6, 120, 100, 20, 255, 255],
    'yellow': [21, 100, 100, 35, 255, 255],
    'green': [40, 70, 50, 85, 255, 255],
    'blue': [95, 120, 50, 130, 255, 255],
    'purple': [131, 60, 50, 160, 255, 255],
}

FEATURES = ('color', 'aspect', 'taper', 'symmetry', 'geometry')
DEFAULT_WEIGHTS = {'color': 0.25, 'aspect': 0.15, 'taper': 0.3, 'symmetry': 0.2,
                   'geometry': 0.1}
BASE_ROWS = 0.15        # the bottom 15% of a cone's rows are its base

# BGR colors to draw accepted cones with; rejected candidates are gray
DRAW_COLORS = {'red': (0, 0, 255), 'orange': (0, 140, 255), 'yellow': (0, 220, 255),
               'green': (0, 190, 0), 'blue': (255, 80, 0), 'purple': (200, 0, 200)}
REJECTED_COLOR = (150, 150, 150)


@dataclass
class DetectionSettings:
    """
    Thresholds and weights of the cone checks.

    open_kernel and close_kernel are the sizes (px) of the opening and closing; contours
    smaller than min_area (px) are noise. A cone's aspect ratio (height / width) is within
    [min_aspect, max_aspect]; its taper, symmetry and solidity reach min_taper,
    min_symmetry and min_solidity; at least min_color_fraction of the strongly colored
    pixels around it are of its color, looking color_context of its size beyond each side
    of its box. Its outline, simplified with approxPolyDP (approx_epsilon times its
    perimeter), should have 3 to max_vertices corners. A box within edge_margin px of the
    image edge may be cut off and is rejected. weights combine the feature scores into the
    confidence, which must reach min_confidence.
    """

    open_kernel: int = 3
    close_kernel: int = 5
    min_area: int = 300
    min_aspect: float = 0.4
    max_aspect: float = 2.5
    min_taper: float = 0.2
    min_symmetry: float = 0.6
    min_solidity: float = 0.75
    max_vertices: int = 10
    approx_epsilon: float = 0.03
    min_color_fraction: float = 0.6
    color_context: float = 0.25
    edge_margin: int = 2
    weights: dict = field(default_factory=lambda: dict(DEFAULT_WEIGHTS))
    min_confidence: float = 0.7


@dataclass
class Candidate:
    """
    One contour of a target color, accepted as a cone or rejected.

    left, top, right and bottom are its bounding box in pixels (right and bottom one past
    the last pixel); u and v its centroid, with pixel (i, j) centered at (i, j). scores
    holds each feature's score (0-1) and confidence their weighted mean, a heuristic score
    and not a probability. reason says why it was rejected ('' if it was accepted).
    """

    color: str
    left: int
    top: int
    right: int
    bottom: int
    u: float
    v: float
    scores: dict
    confidence: float
    reason: str

    def accepted(self):
        """Return whether the candidate was accepted as a cone."""
        return not self.reason


def parse_ranges(values):
    """
    Return the HSV ranges [(low, high), ...] in a flat list of six numbers per range.

    Raise ValueError if the list is empty, its length is not a multiple of six, or a range
    has h_lo > h_hi (give red two ranges instead).
    """
    values = [int(v) for v in values]
    if not values or len(values) % 6:
        raise ValueError('HSV ranges need six numbers each: h_lo s_lo v_lo h_hi s_hi v_hi')
    ranges = []
    for i in range(0, len(values), 6):
        h_lo, s_lo, v_lo, h_hi, s_hi, v_hi = values[i:i + 6]
        if h_lo > h_hi:
            raise ValueError('each HSV range needs h_lo <= h_hi; give red two ranges')
        ranges.append((np.array([h_lo, s_lo, v_lo], np.uint8),
                       np.array([h_hi, s_hi, v_hi], np.uint8)))
    return ranges


def color_mask(hsv, ranges):
    """Return the mask (uint8, 255 = in a range) of an HSV image's pixels in any range."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in ranges:
        mask |= cv2.inRange(hsv, low, high)
    return mask


def clean_mask(mask, settings):
    """Return a mask after an opening (removes specks) and a closing (fills small holes)."""
    if settings.open_kernel > 1:
        kernel = np.ones((settings.open_kernel, settings.open_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    if settings.close_kernel > 1:
        kernel = np.ones((settings.close_kernel, settings.close_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def shape_features(contour, settings):
    """
    Return the shape features of a contour as a dict.

    aspect is the bounding box's height / width. taper is 1 - (mean width of the top third)
    / (mean width of the bottom third), large for a cone, which is narrower at the top.
    symmetry is 1 - (mean difference between each row's left and right extent from the
    central axis) / (mean width), 1 for a perfectly symmetric shape. base is the mean width
    of the bottom 15% of the rows / the largest width. solidity is the contour's area / its
    convex hull's area. vertices is the number of corners of the outline simplified with
    approxPolyDP.
    """
    x, y, w, h = cv2.boundingRect(contour)
    filled = np.zeros((h, w), np.uint8)
    cv2.drawContours(filled, [contour - np.array([x, y])], -1, 255, thickness=cv2.FILLED)
    rows = filled > 0
    present = rows.any(axis=1)
    left = np.argmax(rows, axis=1)
    right = w - 1 - np.argmax(rows[:, ::-1], axis=1)
    widths = np.where(present, right - left + 1, 0).astype(float)

    def mean_width(part):
        part = part[part > 0]
        return float(part.mean()) if len(part) else 0.0

    third = max(1, h // 3)
    top, bottom = mean_width(widths[:third]), mean_width(widths[-third:])
    taper = 1.0 - top / bottom if bottom > 0.0 else 0.0

    moments = cv2.moments(filled, binaryImage=True)
    axis = moments['m10'] / moments['m00'] if moments['m00'] > 0 else (w - 1) / 2.0
    left_extent = axis - left[present] + 0.5
    right_extent = right[present] - axis + 0.5
    spread = float(np.mean(left_extent + right_extent))
    symmetry = 1.0 - float(np.mean(np.abs(left_extent - right_extent))) / spread

    base_rows = max(1, int(round(h * BASE_ROWS)))
    base = mean_width(widths[-base_rows:]) / widths.max()

    area = cv2.contourArea(contour)
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    solidity = area / hull_area if hull_area > 0.0 else 0.0
    perimeter = cv2.arcLength(contour, True)
    vertices = len(cv2.approxPolyDP(contour, settings.approx_epsilon * perimeter, True))
    return {'aspect': h / w, 'taper': taper, 'symmetry': symmetry, 'base': base,
            'solidity': solidity, 'vertices': vertices}


def color_fraction(hsv, ranges, box, context):
    """
    Return the fraction of the strongly colored pixels around a box that are in the ranges.

    box is (left, top, right, bottom) in pixels; it is widened by context times its width
    and height on each side. A pixel is strongly colored if its saturation and value reach
    the lowest s_lo and v_lo of the ranges. An object of a nearby color, such as an orange
    cone partly inside the red range, has many strongly colored pixels outside the ranges,
    so it gets a low fraction.
    """
    left, top, right, bottom = box
    dx, dy = int((right - left) * context), int((bottom - top) * context)
    region = hsv[max(0, top - dy):bottom + dy, max(0, left - dx):right + dx]
    s_min = min(int(low[1]) for low, _ in ranges)
    v_min = min(int(low[2]) for low, _ in ranges)
    strong = (region[..., 1] >= s_min) & (region[..., 2] >= v_min)
    count = int(np.count_nonzero(strong))
    if count == 0:
        return 0.0
    inside = color_mask(region, ranges) > 0
    return int(np.count_nonzero(inside & strong)) / count


def score_candidate(features, color_frac, settings):
    """
    Return the feature scores (0-1), the weighted confidence and why it fails ('' if not).

    The aspect score is 1 in the middle of [min_aspect, max_aspect], 0.5 at its ends and 0
    outside. The taper score reaches 1 at twice min_taper. The geometry score averages the
    solidity, the base (full at 90% of the largest width) and the number of corners.
    """
    aspect = features['aspect']
    middle = (settings.min_aspect + settings.max_aspect) / 2.0
    half = max(1e-6, (settings.max_aspect - settings.min_aspect) / 2.0)
    in_range = settings.min_aspect <= aspect <= settings.max_aspect
    corners_ok = 3 <= features['vertices'] <= settings.max_vertices
    scores = {
        'color': color_frac,
        'aspect': 1.0 - 0.5 * abs(aspect - middle) / half if in_range else 0.0,
        'taper': min(1.0, max(0.0, features['taper']) / max(1e-6, 2.0 * settings.min_taper)),
        'symmetry': max(0.0, features['symmetry']),
        'geometry': (features['solidity'] + min(1.0, features['base'] / 0.9)
                     + (1.0 if corners_ok else 0.5)) / 3.0,
    }
    total = sum(settings.weights.get(name, 0.0) for name in FEATURES)
    confidence = sum(settings.weights.get(name, 0.0) * scores[name]
                     for name in FEATURES) / total
    checks = (
        (color_frac >= settings.min_color_fraction,
         f'color {color_frac:.2f} < {settings.min_color_fraction:.2f}'),
        (in_range, f'aspect {aspect:.2f} outside [{settings.min_aspect}, '
                   f'{settings.max_aspect}]'),
        (features['taper'] >= settings.min_taper,
         f'taper {features["taper"]:.2f} < {settings.min_taper:.2f}'),
        (features['symmetry'] >= settings.min_symmetry,
         f'symmetry {features["symmetry"]:.2f} < {settings.min_symmetry:.2f}'),
        (features['solidity'] >= settings.min_solidity,
         f'solidity {features["solidity"]:.2f} < {settings.min_solidity:.2f}'),
        (confidence >= settings.min_confidence,
         f'confidence {confidence:.2f} < {settings.min_confidence:.2f}'),
    )
    reason = next((message for ok, message in checks if not ok), '')
    return scores, confidence, reason


def touches_edge(box, width, height, margin):
    """Return whether a box (left, top, right, bottom) is within margin px of the edge."""
    left, top, right, bottom = box
    return left <= margin or top <= margin or right >= width - margin or \
        bottom >= height - margin


def detect_cones(bgr, targets, settings):
    """
    Return every candidate of the target colors in a BGR image, accepted or not.

    targets maps each target color name to its HSV ranges (from parse_ranges). Every
    external contour of each color's mask is a candidate, so several cones, also of the
    same color, are found in one image; contours smaller than min_area are left out.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    height, width = bgr.shape[:2]
    candidates = []
    for color, ranges in targets.items():
        mask = clean_mask(color_mask(hsv, ranges), settings)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < settings.min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            box = (x, y, x + w, y + h)
            moments = cv2.moments(contour)
            u = moments['m10'] / moments['m00']
            v = moments['m01'] / moments['m00']
            features = shape_features(contour, settings)
            fraction = color_fraction(hsv, ranges, box, settings.color_context)
            scores, confidence, reason = score_candidate(features, fraction, settings)
            if not reason and touches_edge(box, width, height, settings.edge_margin):
                reason = 'touches the image edge'
            candidates.append(Candidate(color, *box, u, v, scores, confidence, reason))
    return candidates


def target_mask(bgr, targets, settings):
    """Return the cleaned masks of all target colors combined, for debugging."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(bgr.shape[:2], np.uint8)
    for ranges in targets.values():
        mask |= clean_mask(color_mask(hsv, ranges), settings)
    return mask


def draw_candidates(bgr, candidates):
    """
    Return a copy of a BGR image with every candidate boxed and labeled.

    Accepted cones get their color and confidence; rejected candidates are gray and
    labeled UNKNOWN with the check they failed. A yellow line marks the center column.
    """
    out = bgr.copy()
    height, width = out.shape[:2]
    cv2.line(out, (width // 2, 0), (width // 2, height - 1), (0, 255, 255), 1)
    for c in candidates:
        if c.accepted():
            color, label = DRAW_COLORS.get(c.color, (255, 255, 255)), \
                f'{c.color} {c.confidence:.2f}'
        else:
            color, label = REJECTED_COLOR, f'UNKNOWN ({c.reason.split()[0]})'
        cv2.rectangle(out, (c.left, c.top), (c.right - 1, c.bottom - 1), color, 2)
        org = (c.left, max(10, c.top - 3))
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3,
                    cv2.LINE_AA)
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    return out
