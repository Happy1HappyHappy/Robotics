"""
Draw an occupancy grid as an image with OpenCV
"""

import math

import cv2
import numpy as np

START_COLOR = (0, 160, 0)       # BGR: green
PATH_COLOR = (255, 0, 0)        # blue
SENSOR_COLOR = (0, 0, 255)      # red
TEXT_COLOR = (0, 0, 0)          # black
SEGMENT_COLOR = (0, 140, 255)   # orange


def world_to_pixel(grid, x, y, scale):
    """
    Pixel (u = column, v = row) of world point (x, y) in an image drawn at scale px per cell
    """
    half = grid.size // 2
    return (int(round((half - y / grid.resolution) * scale)),
            int(round((half - x / grid.resolution) * scale)))


def grid_image(grid, scale=3, poses=(), label=''):
    """
    Draw the grid as a BGR image: occupied = black, free = white, unknown = gray
    Up is +x (the robot's start heading), left is +y
    scale: pixels per cell side
    poses: (x, y, heading) in the world frame (m, rad), oldest first, to mark the sensor's
    path: all as dots, the first as the start (green), the last with an arrow along the
    heading (red); pass the robot's heading, not the LiDAR's, so the arrow points forward
    label: text drawn in the top-left corner
    """
    gray = np.round((1.0 - grid.probability()) * 255.0).astype(np.uint8)
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

    for x, y, _ in poses:
        cv2.circle(img, world_to_pixel(grid, x, y, scale), 2, PATH_COLOR, -1)
    if poses:
        x, y, _ = poses[0]
        cv2.circle(img, world_to_pixel(grid, x, y, scale), 5, START_COLOR, 2)
        x, y, yaw = poses[-1]
        tip = (x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw))  # 0.3 m heading arrow
        cv2.arrowedLine(img, world_to_pixel(grid, x, y, scale),
                        world_to_pixel(grid, *tip, scale), SENSOR_COLOR, 2, tipLength=0.3)

    # 1 m scale bar in the bottom-left corner
    bar = int(round(1.0 / grid.resolution * scale))
    v = img.shape[0] - 12
    cv2.line(img, (10, v), (10 + bar, v), TEXT_COLOR, 2)
    cv2.putText(img, '1 m', (10, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_COLOR, 1,
                cv2.LINE_AA)
    if label:
        cv2.putText(img, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOR, 1,
                    cv2.LINE_AA)
    return img


def map_image(grid, scale=3, poses=(), segments=(), corners=(), label=''):
    """
    The grid image (see grid_image) with wall segments (orange) and corner landmarks (red
    circles, L1, L2, ...) on top
    segments: [(start, end), ...] and corners: K x 2, world (x, y) in meters
    """
    img = grid_image(grid, scale, poses, label)
    for start, end in segments:
        cv2.line(img, world_to_pixel(grid, *start, scale), world_to_pixel(grid, *end, scale),
                 SEGMENT_COLOR, 2)
    for k, (x, y) in enumerate(corners, 1):
        u, v = world_to_pixel(grid, x, y, scale)
        cv2.circle(img, (u, v), 6, SENSOR_COLOR, 2)
        cv2.putText(img, f'L{k}', (u + 7, v - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, SENSOR_COLOR,
                    1, cv2.LINE_AA)
    return img


def summary_image(images, gap=6):
    """
    Images of the same height side by side, separated by white gaps of gap px
    """
    spacer = np.full((images[0].shape[0], gap, 3), 255, dtype=np.uint8)
    parts = []
    for img in images:
        parts += [img, spacer]
    return np.hstack(parts[:-1])
