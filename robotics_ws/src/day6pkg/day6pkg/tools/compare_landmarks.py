"""
Compare the landmarks of two Project 7 runs, e.g. the way there and the way back:
ros2 run day6pkg compare_landmarks <run A> <run B>
from the directory the runs were started in (robotics_ws), e.g.
ros2 run day6pkg compare_landmarks 2026-10-27_16-40-12 2026-10-27_16-45-03
Reads each run's grid and landmarks from logs/ (_final, else _latest), puts run B's
landmarks in run A's frame through the odometry frame both share, pairs the landmarks found
in both, and prints each pair and the mean difference in their position; saves run A's map
with both runs' landmarks as result/grid/<A>_vs_<B>_landmarks.png
Needs only the files: no ROS nodes, no robot
"""

import argparse
import os
import sys

import numpy as np

from day6pkg.helpers.compare import match_landmarks, transform_points, world_from_world
from day6pkg.helpers.occupancy_grid import OccupancyGrid
from day6pkg.helpers.render import comparison_image, save_png


def run_file(log_dir, run, kind):
    """
    Path of the run's kind ('grid' or 'landmarks') file in log_dir: <run>_<kind>_final.npz,
    else <run>_<kind>_latest.npz (the run did not end with Ctrl-C), else None
    """
    for end in ('final', 'latest'):
        path = os.path.join(log_dir, f'{run}_{kind}_{end}.npz')
        if os.path.exists(path):
            return path
    return None


def load_run(log_dir, run):
    """
    The run's grid, poses, ^world T_odom, corners (K x 2, m) and segments, all in the run's
    world frame, from its files in log_dir; exits with a message if a file is missing
    """
    paths = {kind: run_file(log_dir, run, kind) for kind in ('grid', 'landmarks')}
    for kind, path in paths.items():
        if path is None:
            sys.exit(f'No {kind} file of run {run} in {log_dir}/ ({run}_{kind}_final.npz)')
        print(f'Read {path}')
    grid, extra = OccupancyGrid.load(paths['grid'])
    with np.load(paths['landmarks']) as data:
        corners = data['corners'].reshape(-1, 2)
        segments = data['segments'].reshape(-1, 2, 2)
    return grid, extra['poses'], extra['world_from_odom'], corners, segments


def main(args=None):
    """
    Entry point: compare the two runs named on the command line
    """
    parser = argparse.ArgumentParser(description='Compare the landmarks of two Project 7 runs')
    parser.add_argument('run_a', help='the run compared with, e.g. the way there')
    parser.add_argument('run_b', help='the run compared, e.g. the way back')
    parser.add_argument('--max-distance', type=float, default=0.3,
                        help='m, two landmarks farther apart are not the same (default 0.3)')
    parser.add_argument('--log-dir', default='logs', help='where the runs saved their .npz')
    parser.add_argument('--result-dir', default='result', help='the image goes to <this>/grid')
    parser.add_argument('--scale', type=int, default=3, help='image pixels per cell side')
    opts = parser.parse_args(args)

    grid, poses, a_world_from_odom, corners, segments = load_run(opts.log_dir, opts.run_a)
    _, _, b_world_from_odom, b_corners, _ = load_run(opts.log_dir, opts.run_b)
    # run B's landmarks in run A's world frame
    other = transform_points(world_from_world(a_world_from_odom, b_world_from_odom), b_corners)
    pairs = match_landmarks(corners, other, opts.max_distance)

    print(f'\nRun A {opts.run_a}: {len(corners)} landmarks, run B {opts.run_b}: '
          f"{len(other)}, in run A's frame (m)")
    for i, j, d in pairs:
        (ax, ay), (bx, by) = corners[i], other[j]
        print(f'  A L{i + 1} ({ax:+.3f}, {ay:+.3f})  B L{j + 1} ({bx:+.3f}, {by:+.3f})  '
              f'{d:.3f} m apart')
    if pairs:
        mean = np.mean([d for _, _, d in pairs])
        dx, dy = np.mean([other[j] - corners[i] for i, j, _ in pairs], axis=0)
        print(f'{len(pairs)} found in both (within {opts.max_distance} m): mean difference '
              f'{mean:.3f} m, mean offset B - A ({dx:+.3f}, {dy:+.3f}) m')
        label = f'{len(pairs)} in both, mean difference {mean:.3f} m (red: A, teal: B)'
    else:
        print(f'No landmark found in both (within {opts.max_distance} m); if the robot was '
              'restarted between the runs, its odometry was reset and they do not line up')
        label = 'no landmark in both (red: A, teal: B)'

    img = comparison_image(grid, opts.scale, [tuple(p) for p in poses], segments, corners,
                           other, pairs, label)
    out_dir = os.path.join(opts.result_dir, 'grid')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'{opts.run_a}_vs_{opts.run_b}_landmarks.png')
    print(f'Saved {path}' if save_png(path, img) else f'Could not save {path}')


if __name__ == '__main__':
    main()
