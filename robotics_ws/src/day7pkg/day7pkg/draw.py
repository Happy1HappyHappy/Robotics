"""
Images for Project 7: landmarks on a grid, two grids overlaid, and a planned path
Built on day6pkg's grid_image (occupied = black, free = white, unknown = gray, +x up)
"""

import cv2
import numpy as np

from day6pkg.render import grid_image, world_to_pixel
from day7pkg.landmarks import transform_points

SEGMENT_COLOR = (0, 140, 255)       # BGR: orange
CORNER_A_COLOR = (0, 0, 255)        # red
CORNER_B_COLOR = (255, 0, 0)        # blue
MATCH_COLOR = (0, 160, 0)           # green
OVERLAY_B_COLOR = (230, 160, 60)    # light blue: cells occupied in B only
BOTH_COLOR = (120, 0, 120)          # purple: cells occupied in both grids
BLOCKED_COLOR = (200, 200, 255)     # light red: cells the robot's center must avoid
ASTAR_COLOR = (255, 170, 0)         # light blue: A* cells
PATH_COLOR = (0, 160, 0)            # green: shortened path
GOAL_COLOR = (0, 0, 255)            # red
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _label(img, text, uv, color):
    """
    Small text just above and right of pixel uv
    """
    cv2.putText(img, text, (uv[0] + 6, uv[1] - 6), FONT, 0.4, color, 1, cv2.LINE_AA)


def _cell_pixels(mask, scale):
    """
    Image mask (scaled up by scale px per cell side) of the cells where mask is True
    """
    return cv2.resize(mask.astype(np.uint8), None, fx=scale, fy=scale,
                      interpolation=cv2.INTER_NEAREST).astype(bool)


def landmark_image(grid, segments, corners, prefix, scale=3, poses=(), label=''):
    """
    The grid with its RANSAC segments (orange) and corner landmarks (red circles, labeled
    prefix + number from 1, in the order of corners)
    """
    img = grid_image(grid, scale, poses, label)
    for seg in segments:
        cv2.line(img, world_to_pixel(grid, *seg.start, scale),
                 world_to_pixel(grid, *seg.end, scale), SEGMENT_COLOR, 2)
    for k, (x, y) in enumerate(corners, 1):
        uv = world_to_pixel(grid, x, y, scale)
        cv2.circle(img, uv, 7, CORNER_A_COLOR, 2)
        _label(img, f'{prefix}{k}', uv, CORNER_A_COLOR)
    return img


def overlay_image(grid_a, grid_b, transform, corners_a, corners_b, matches, scale=3,
                  occupied=1.0, label=''):
    """
    Grid A with grid B laid over it in A's frame using transform (B -> A): cells occupied in
    B only are light blue, in both purple; A's landmarks are red circles (A1, ...), B's
    landmarks moved into A's frame are blue squares (B1, ...), and matched pairs are joined
    by a green line
    """
    img = grid_image(grid_a, scale, (), label)
    # for every cell of A, the cell of B at the same place: B's frame point = T^-1 (A point)
    rows, cols = np.indices(grid_a.log_odds.shape)
    xa, ya = grid_a.cell_to_world(rows, cols)
    x, y, theta = transform
    c, s = np.cos(theta), np.sin(theta)
    xb = c * (xa - x) + s * (ya - y)
    yb = -s * (xa - x) + c * (ya - y)
    half = grid_b.size // 2
    map_row = (half - 0.5 - xb / grid_b.resolution).astype(np.float32)
    map_col = (half - 0.5 - yb / grid_b.resolution).astype(np.float32)
    warped = cv2.remap(grid_b.log_odds.astype(np.float32), map_col, map_row, cv2.INTER_NEAREST,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)
    in_a = _cell_pixels(grid_a.log_odds > occupied, scale)
    in_b = _cell_pixels(warped > occupied, scale)
    img[in_b & ~in_a] = OVERLAY_B_COLOR
    img[in_b & in_a] = BOTH_COLOR

    moved = transform_points(transform, corners_b)
    for i, j, _ in matches:
        cv2.line(img, world_to_pixel(grid_a, *corners_a[i], scale),
                 world_to_pixel(grid_a, *moved[j], scale), MATCH_COLOR, 2)
    for k, (xc, yc) in enumerate(corners_a, 1):
        uv = world_to_pixel(grid_a, xc, yc, scale)
        cv2.circle(img, uv, 7, CORNER_A_COLOR, 2)
        _label(img, f'A{k}', uv, CORNER_A_COLOR)
    for k, (xc, yc) in enumerate(moved, 1):
        u, v = world_to_pixel(grid_a, xc, yc, scale)
        cv2.rectangle(img, (u - 5, v - 5), (u + 5, v + 5), CORNER_B_COLOR, 2)
        cv2.putText(img, f'B{k}', (u + 6, v + 14), FONT, 0.4, CORNER_B_COLOR, 1, cv2.LINE_AA)
    return img


def plan_image(grid, plan, scale=3, poses=(), label=''):
    """
    The grid with the free cells the robot's center must still avoid because they are too
    close to an obstacle (light red), the A* cells (light blue), the shortened path (green),
    the start (green circle) and the goal (red cross)
    """
    img = grid_image(grid, scale, poses, label)
    img[_cell_pixels(plan.grown, scale)] = BLOCKED_COLOR
    for row, col in plan.cells:
        x, y = grid.cell_to_world(row, col)
        cv2.circle(img, world_to_pixel(grid, x, y, scale), 1, ASTAR_COLOR, -1)
    pts = [world_to_pixel(grid, x, y, scale) for x, y in plan.points]
    for p, q in zip(pts[:-1], pts[1:]):
        cv2.line(img, p, q, PATH_COLOR, 2)
    for p in pts[1:-1]:
        cv2.circle(img, p, 4, PATH_COLOR, -1)
    cv2.circle(img, pts[0], 7, PATH_COLOR, 2)
    u, v = pts[-1]
    cv2.line(img, (u - 6, v - 6), (u + 6, v + 6), GOAL_COLOR, 2)
    cv2.line(img, (u - 6, v + 6), (u + 6, v - 6), GOAL_COLOR, 2)
    return img
