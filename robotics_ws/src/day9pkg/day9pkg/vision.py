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


def largest_blob(mask, min_area):
    """
    Largest connected region of a mask after cleaning it up, together with the other pieces
    above or below it (a cone's white stripe splits it into pieces stacked in the same
    columns); None if the largest has fewer than min_area pixels
    """
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    n, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:                  # label 0 is the background
        return None
    areas = stats[:, cv2.CC_STAT_AREA]
    i = 1 + int(np.argmax(areas[1:]))
    if areas[i] < min_area:
        return None
    lefts = stats[:, cv2.CC_STAT_LEFT]
    rights = lefts + stats[:, cv2.CC_STAT_WIDTH]
    tops = stats[:, cv2.CC_STAT_TOP]
    bottoms = tops + stats[:, cv2.CC_STAT_HEIGHT]
    # pieces whose columns overlap the largest one's, and not just specks
    parts = [j for j in range(1, n) if lefts[j] < rights[i] and rights[j] > lefts[i]
             and areas[j] >= min_area / 4]
    weights = areas[parts].astype(float)
    return Blob(u=float(np.average(centroids[parts, 0], weights=weights)),
                v=float(np.average(centroids[parts, 1], weights=weights)),
                left=int(lefts[parts].min()), top=int(tops[parts].min()),
                right=int(rights[parts].max()), bottom=int(bottoms[parts].max()),
                area=int(weights.sum()))


def find_blobs(bgr, hsv_ranges, min_area):
    """
    The largest blob of each color in a BGR image, as {color name: Blob}; colors with no
    blob of at least min_area pixels are left out
    hsv_ranges: {color name: [h_lo, s_lo, v_lo, h_hi, s_hi, v_hi]}
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blobs = {}
    for name, hsv_range in hsv_ranges.items():
        blob = largest_blob(color_mask(hsv, hsv_range), min_area)
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
