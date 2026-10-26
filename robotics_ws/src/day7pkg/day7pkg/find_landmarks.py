"""
Project 7, Task 2: find corner landmarks in the two occupancy grids from Task 1, line the
second grid's landmarks up with the first's, and compare the two lists
    ros2 run day7pkg landmarks run_a/grid.npz run_b/grid.npz --output-dir results
Writes landmarks_a.png, landmarks_b.png, landmarks_overlay.png and landmarks.txt to the
output directory, and prints the report
Not a ROS node: it works on the grid.npz files that day6pkg's Grid Node saved
"""

import argparse
import math
import os

import cv2
import numpy as np

from day6pkg.grid import OccupancyGrid
from day7pkg import draw
from day7pkg.landmarks import (align, find_corners, match, occupied_points, ransac_segments,
                               transform_points, wrap_angle)


def parse_args(argv=None):
    """
    Command-line options; defaults suit a 5 cm grid
    """
    p = argparse.ArgumentParser(
        description='Find corner landmarks in two occupancy grids and compare them')
    p.add_argument('grid_a', help='grid.npz of the first run; its frame is the reference')
    p.add_argument('grid_b', help='grid.npz of the run back along the same path')
    p.add_argument('--output-dir', default='.', help='where to write the images and report')
    p.add_argument('--occupied', type=float, default=1.0,
                   help='log-odds above which a cell is a point for RANSAC (default 1.0)')
    p.add_argument('--sample-radius', type=float, default=0.5,
                   help='m, how far apart the two random points may be')
    p.add_argument('--inlier-distance', type=float, default=0.05,
                   help='m, how close to the line a point must be to be on it')
    p.add_argument('--min-points', type=int, default=8,
                   help='points a segment needs to be accepted')
    p.add_argument('--min-length', type=float, default=0.3, help='m, shortest segment')
    p.add_argument('--max-gap', type=float, default=0.15,
                   help='m, a gap this long along a line splits it into two segments')
    p.add_argument('--max-failures', type=int, default=300,
                   help='RANSAC stops after this many tries in a row find no segment')
    p.add_argument('--seed', type=int, default=0, help='random seed, for repeatable results')
    p.add_argument('--corner-angle', type=float, default=60.0,
                   help='deg, smallest angle between two segments that make a corner')
    p.add_argument('--corner-distance', type=float, default=0.15,
                   help='m, how far from the corner each segment may end')
    p.add_argument('--match-distance', type=float, default=0.3,
                   help='m, landmarks this close after lining up are the same landmark')
    p.add_argument('--no-prior', action='store_true',
                   help="search every rotation instead of only those near odometry's estimate")
    p.add_argument('--image-scale', type=int, default=3, help='image pixels per cell side')
    return p.parse_args(argv)


def load(path):
    """
    Grid, poses (list of (x, y, heading)) and ^world T_odom (3x3, or None) from a grid.npz
    """
    grid, extra = OccupancyGrid.load(path)
    poses = [tuple(p) for p in extra.get('poses', np.zeros((0, 3)))]
    return grid, poses, extra.get('world_from_odom')


def landmarks(grid, args):
    """
    RANSAC segments and corner landmarks (N x 2, m) of a grid, with the options in args
    """
    points = occupied_points(grid, args.occupied)
    segments = ransac_segments(points, args.sample_radius, args.inlier_distance,
                               args.min_points, args.min_length, args.max_gap,
                               args.max_failures, args.seed)
    corners = find_corners(segments, math.radians(args.corner_angle), args.corner_distance)
    return points, segments, corners


def odometry_transform(world_from_odom_a, world_from_odom_b):
    """
    Odometry's estimate of B's frame in A's frame, (x, y, theta):
    ^A T_B = ^A T_odom . (^B T_odom)^-1, since both runs share the odom frame
    """
    if world_from_odom_a is None or world_from_odom_b is None:
        return None
    T = np.asarray(world_from_odom_a) @ np.linalg.inv(np.asarray(world_from_odom_b))
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


def fmt_transform(t):
    """
    (x, y, theta) as text, theta in degrees from 0 to 360, so a run that turned around reads
    as about 180 whether it turned a little more or a little less
    """
    return f'x={t[0]:.3f} m, y={t[1]:.3f} m, theta={math.degrees(t[2]) % 360.0:.1f} deg'


def report(args, a, b, transform, odom, matches):
    """
    Lines of the comparison report
    a, b: (grid, poses, points, segments, corners) of each run
    """
    lines = []
    for name, path, (grid, poses, points, segments, corners) in (
            ('A', args.grid_a, a), ('B', args.grid_b, b)):
        lines.append(f'Grid {name}: {path} ({len(poses)} scans, {len(points)} occupied cells, '
                     f'{len(segments)} segments, {len(corners)} landmarks)')
    lines.append('')
    if odom is not None:
        lines.append(f"Odometry:  B's frame in A's frame is at {fmt_transform(odom)}")
    if transform is not None:
        lines.append(f"Landmarks: B's frame in A's frame is at {fmt_transform(transform)} "
                     f'({len(matches)} landmarks matched)')
    else:
        lines.append('Landmarks: fewer than 2 landmarks matched, so they cannot line up the '
                     'grids' + ('; comparing with odometry instead' if odom is not None else ''))
    if transform is not None and odom is not None:
        lines.append(f'Difference (odometry error over the two runs): '
                     f'{math.hypot(transform[0] - odom[0], transform[1] - odom[1]):.3f} m, '
                     f'{math.degrees(wrap_angle(transform[2] - odom[2])):.1f} deg')
    lines.append('')

    corners_a, corners_b = a[4], b[4]
    used = transform if transform is not None else odom
    lines.append("Landmarks in A (A's frame, m):")
    lines += [f'  A{k:<3d} ({x:6.2f}, {y:6.2f})' for k, (x, y) in enumerate(corners_a, 1)]
    if used is not None:
        lines.append("Landmarks in B (B's frame -> A's frame, m):")
        moved = transform_points(used, corners_b)
        lines += [f'  B{k:<3d} ({x:6.2f}, {y:6.2f}) -> ({mx:6.2f}, {my:6.2f})'
                  for k, ((x, y), (mx, my)) in enumerate(zip(corners_b, moved), 1)]
    else:
        lines.append("Landmarks in B (B's frame, m):")
        lines += [f'  B{k:<3d} ({x:6.2f}, {y:6.2f})' for k, (x, y) in enumerate(corners_b, 1)]
        lines.append('Without a transform the two lists cannot be compared.')
        return lines
    lines.append('')

    lines.append("Found in both (distance between the two positions in A's frame):")
    odom_moved = transform_points(odom, corners_b) if odom is not None else None
    header = '            lined up' + ('   odometry only' if odom_moved is not None else '')
    lines.append(header)
    odom_d = []
    for i, j, d in matches:
        row = f'  A{i + 1:<3d}= B{j + 1:<3d} {d:6.3f} m'
        if odom_moved is not None:
            odom_d.append(float(np.linalg.norm(corners_a[i] - odom_moved[j])))
            row += f'      {odom_d[-1]:6.3f} m'
        lines.append(row)
    if matches:
        mean = np.mean([d for _, _, d in matches])
        text = f'Mean difference in position: {mean:.3f} m'
        if transform is not None:
            text += ' after lining up the landmarks'
        if odom_d and transform is not None:
            text += f' ({np.mean(odom_d):.3f} m with odometry only)'
        lines.append(text)
        lines.append(f'Largest difference: {max(d for _, _, d in matches):.3f} m')
    only_a = [f'A{i + 1}' for i in range(len(corners_a)) if i not in {m[0] for m in matches}]
    only_b = [f'B{j + 1}' for j in range(len(corners_b)) if j not in {m[1] for m in matches}]
    lines.append(f"Only in A: {', '.join(only_a) or 'none'}")
    lines.append(f"Only in B: {', '.join(only_b) or 'none'}")
    return lines


def main(argv=None):
    """
    Entry point: find, line up and compare the landmarks, then write the images and report
    """
    args = parse_args(argv)
    runs = []
    for path in (args.grid_a, args.grid_b):
        grid, poses, world_from_odom = load(path)
        points, segments, corners = landmarks(grid, args)
        runs.append((grid, poses, points, segments, corners, world_from_odom))
    a, b = runs[0][:5], runs[1][:5]

    odom = odometry_transform(runs[0][5], runs[1][5])
    transform, matches = align(a[4], b[4], args.match_distance,
                               prior=None if args.no_prior else odom)
    if transform is None and odom is not None:
        matches = match(a[4], transform_points(odom, b[4]), args.match_distance)

    lines = report(args, a, b, transform, odom, matches)
    print('\n'.join(lines))

    os.makedirs(args.output_dir, exist_ok=True)
    s = args.image_scale
    images = {
        'landmarks_a.png': draw.landmark_image(a[0], a[3], a[4], 'A', s, a[1],
                                               f'Grid A: {len(a[4])} landmarks'),
        'landmarks_b.png': draw.landmark_image(b[0], b[3], b[4], 'B', s, b[1],
                                               f'Grid B: {len(b[4])} landmarks'),
    }
    used = transform if transform is not None else odom
    if used is not None:
        images['landmarks_overlay.png'] = draw.overlay_image(
            a[0], b[0], used, a[4], b[4], matches, s, args.occupied,
            "Grid B in A's frame" + (' (odometry)' if transform is None else ''))
    for name, img in images.items():
        path = os.path.join(args.output_dir, name)
        if not cv2.imwrite(path, img):
            print(f'Could not save {path}')
            return 1
        print(f'Saved {path}')
    path = os.path.join(args.output_dir, 'landmarks.txt')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'Saved {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
