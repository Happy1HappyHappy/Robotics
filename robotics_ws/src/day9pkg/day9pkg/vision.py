"""
Colored cones in camera images (Project 9): a mask of the pixels of each color (in HSV),
the largest region (blob) of that color, and which way a pixel column is from the camera
Pure OpenCV/NumPy (no ROS), so it can be tested and used without the robot
Bearings follow REP 103: radians, 0 = straight ahead, positive = to the left
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

# Starting HSV ranges for common cone colors, on OpenCV's scale: H 0-179 (degrees / 2),
# S and V 0-255. Each is [h_lo, s_lo, v_lo, h_hi, s_hi, v_hi]; h_lo > h_hi means the hue
# range wraps through 0, which red needs. Tune them under the lab's lights: click on a cone
# in the Cone Node's window to print its HSV
DEFAULT_HSV = {
    'red': [170, 120, 70, 4, 255, 255],
    'orange': [5, 120, 100, 18, 255, 255],
    'yellow': [20, 100, 100, 35, 255, 255],
    'green': [40, 80, 50, 85, 255, 255],
    'blue': [95, 120, 50, 130, 255, 255],
    'purple': [131, 60, 50, 155, 255, 255],
    'pink': [156, 60, 120, 169, 255, 255],
}

# opening removes specks of noise; closing fills small gaps inside a cone
OPEN_KERNEL = np.ones((3, 3), np.uint8)
CLOSE_KERNEL = np.ones((5, 5), np.uint8)


@dataclass
class Blob:
    """
    One region of a color in an image, in pixels: its center (u = column, v = row), its
    bounding box (right and bottom are one past the last pixel) and its area
    """
    u: float
    v: float
    left: int
    top: int
    right: int
    bottom: int
    area: int


@dataclass
class ShapeLimits:
    """
    What a region must look like to count as a cone.

    Its height / width is within [min_aspect, max_aspect]; its taper (narrower at the top),
    left-right symmetry and solidity reach min_taper, min_symmetry and min_solidity. See
    shape_features for what each measures.
    """

    min_aspect: float = 0.4
    max_aspect: float = 3.0
    min_taper: float = 0.2
    min_symmetry: float = 0.6
    min_solidity: float = 0.7


def shape_features(region):
    """
    Return the shape features of a region (a 2D boolean array, True = in the region).

    aspect is its height / width. taper is 1 - (mean width of the top third of its rows) /
    (mean width of the bottom third), large for a cone, about 0 for a box or a ball.
    symmetry is 1 - (mean difference between each row's left and right extent from the
    central axis) / (mean width), 1 for a perfectly symmetric shape. solidity is its pixel
    count / its convex hull's area; a white stripe across a cone lowers it a little.
    """
    region = np.asarray(region, dtype=bool)
    height, width = region.shape
    present = region.any(axis=1)
    if not present.any():
        return {'aspect': 0.0, 'taper': 0.0, 'symmetry': 0.0, 'solidity': 0.0}
    left = np.argmax(region, axis=1)
    right = width - 1 - np.argmax(region[:, ::-1], axis=1)
    widths = np.where(present, right - left + 1, 0).astype(float)

    def mean_width(part):
        part = part[part > 0]
        return float(part.mean()) if len(part) else 0.0

    third = max(1, height // 3)
    bottom = mean_width(widths[-third:])
    taper = 1.0 - mean_width(widths[:third]) / bottom if bottom > 0.0 else 0.0
    axis = float(np.nonzero(region)[1].mean())
    left_extent = axis - left[present] + 0.5
    right_extent = right[present] - axis + 0.5
    symmetry = 1.0 - (float(np.mean(np.abs(left_extent - right_extent)))
                      / float(np.mean(left_extent + right_extent)))
    points = np.argwhere(region)[:, ::-1].astype(np.int32)
    hull_area = cv2.contourArea(cv2.convexHull(points))
    solidity = min(1.0, region.sum() / hull_area) if hull_area > 0.0 else 0.0
    return {'aspect': height / width, 'taper': taper, 'symmetry': symmetry,
            'solidity': solidity}


def cone_shaped(region, limits):
    """Return whether a region (2D boolean array) passes the cone shape checks."""
    f = shape_features(region)
    return (limits.min_aspect <= f['aspect'] <= limits.max_aspect
            and f['taper'] >= limits.min_taper and f['symmetry'] >= limits.min_symmetry
            and f['solidity'] >= limits.min_solidity)


def color_mask(hsv, hsv_range):
    """
    Mask (uint8, 255 = in range) of the pixels of an HSV image that are within hsv_range
    hsv_range: [h_lo, s_lo, v_lo, h_hi, s_hi, v_hi]; h_lo > h_hi wraps through hue 0
    """
    h_lo, s_lo, v_lo, h_hi, s_hi, v_hi = (int(x) for x in hsv_range)
    if h_lo <= h_hi:
        return cv2.inRange(hsv, np.array([h_lo, s_lo, v_lo], np.uint8),
                           np.array([h_hi, s_hi, v_hi], np.uint8))
    # wraps: hues h_lo..179 and 0..h_hi
    top = cv2.inRange(hsv, np.array([h_lo, s_lo, v_lo], np.uint8),
                      np.array([179, s_hi, v_hi], np.uint8))
    bottom = cv2.inRange(hsv, np.array([0, s_lo, v_lo], np.uint8),
                         np.array([h_hi, s_hi, v_hi], np.uint8))
    return cv2.bitwise_or(top, bottom)


def largest_blob(mask, min_area, shape=None):
    """
    Return the largest cone-like region of a mask after cleaning it up, or None.

    A region is a connected piece together with the other pieces above or below it (a
    cone's white stripe splits it into pieces stacked in the same columns). Regions are
    tried from the largest down; with shape (ShapeLimits), the first one shaped like a cone
    is returned, so a larger box or ball of the same color is skipped. Without shape, the
    largest is returned. Pieces smaller than min_area pixels are not tried.
    """
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:                  # label 0 is the background
        return None
    areas = stats[:, cv2.CC_STAT_AREA]
    lefts = stats[:, cv2.CC_STAT_LEFT]
    rights = lefts + stats[:, cv2.CC_STAT_WIDTH]
    tops = stats[:, cv2.CC_STAT_TOP]
    bottoms = tops + stats[:, cv2.CC_STAT_HEIGHT]
    for i in 1 + np.argsort(-areas[1:], kind='stable'):
        if areas[i] < min_area:
            return None
        # pieces whose columns overlap this one's, and not just specks
        parts = [j for j in range(1, n) if lefts[j] < rights[i] and rights[j] > lefts[i]
                 and areas[j] >= min_area / 4]
        left, top = int(lefts[parts].min()), int(tops[parts].min())
        right, bottom = int(rights[parts].max()), int(bottoms[parts].max())
        if shape is not None and not cone_shaped(
                np.isin(labels[top:bottom, left:right], parts), shape):
            continue
        weights = areas[parts].astype(float)
        return Blob(u=float(np.average(centroids[parts, 0], weights=weights)),
                    v=float(np.average(centroids[parts, 1], weights=weights)),
                    left=left, top=top, right=right, bottom=bottom, area=int(weights.sum()))
    return None


def find_blobs(bgr, hsv_ranges, min_area, shape=None):
    """
    Return the largest cone of each color in a BGR image, as {color name: Blob}.

    hsv_ranges is {color name: [h_lo, s_lo, v_lo, h_hi, s_hi, v_hi]}. With shape
    (ShapeLimits), only regions shaped like a cone count. Colors with no such region of at
    least min_area pixels are left out.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blobs = {}
    for name, hsv_range in hsv_ranges.items():
        blob = largest_blob(color_mask(hsv, hsv_range), min_area, shape)
        if blob is not None:
            blobs[name] = blob
    return blobs


def pixel_bearing(u, fx, cx):
    """
    Bearing (rad, + = left) of the ray through pixel column u, for a camera looking straight
    ahead along the robot's x axis
    Pinhole model: a point x to the right of the optical axis at depth z lands on column
    u = cx + fx x / z, so the ray is atan((cx - u) / fx) to the left of straight ahead
    fx: focal length in pixels; cx: column of the optical axis
    """
    return math.atan2(cx - u, fx)


def blob_bearings(blob, fx, cx):
    """
    Bearings (rad, + = left) of a blob's center, left edge and right edge
    """
    return (pixel_bearing(blob.u, fx, cx),
            pixel_bearing(blob.left - 0.5, fx, cx),
            pixel_bearing(blob.right - 0.5, fx, cx))


def focal_from_fov(width, hfov):
    """
    Focal length in pixels of a camera whose image is width pixels across hfov radians
    """
    return (width / 2.0) / math.tan(hfov / 2.0)


def draw_blobs(bgr, blobs, highlight=None):
    """
    Copy of a BGR image with each blob's bounding box and color name drawn on it
    The blob named highlight (the cone being driven to) gets a thicker box
    """
    out = bgr.copy()
    for name, b in blobs.items():
        thick = 2 if name == highlight else 1
        cv2.rectangle(out, (b.left, b.top), (b.right - 1, b.bottom - 1), (0, 0, 0), thick + 2)
        cv2.rectangle(out, (b.left, b.top), (b.right - 1, b.bottom - 1),
                      (255, 255, 255), thick)
        org = (b.left, max(10, b.top - 3))
        cv2.putText(out, name, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, name, org, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1,
                    cv2.LINE_AA)
    return out
