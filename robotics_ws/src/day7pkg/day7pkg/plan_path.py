"""
Project 7, Task 3: plan a path with A* on an occupancy grid from Task 1, from the robot's
start position to a goal, keeping the robot's whole body clear of obstacles
    ros2 run day7pkg plan run_a/grid.npz --goal 2.2 0.7 --output-dir results
Writes path.png and path.txt to the output directory and prints the plan, with the turns
and drives that would follow it (in day3pkg's Executive: t <deg>, m <meters>)
Not a ROS node: it works on the grid.npz file that day6pkg's Grid Node saved
"""

import argparse
import math
import os

import cv2

from day6pkg.grid import OccupancyGrid
from day7pkg import draw
from day7pkg.planner import drive_commands, path_length, plan_path


def parse_args(argv=None):
    """
    Command-line options; points are (x, y) in meters in the grid's frame (x = the robot's
    heading at the start of the run, y = its left)
    """
    p = argparse.ArgumentParser(description='Plan a path with A* on an occupancy grid')
    p.add_argument('grid', help='grid.npz saved by the Grid Node')
    p.add_argument('--goal', type=float, nargs=2, required=True, metavar=('X', 'Y'),
                   help='m, where to go')
    p.add_argument('--start', type=float, nargs=2, metavar=('X', 'Y'),
                   help="m, where to start (default: the run's first sensor position, the "
                        "grid's origin)")
    p.add_argument('--robot-radius', type=float, default=0.17,
                   help='m, half the TurtleBot 4 width (about 0.34 m across)')
    p.add_argument('--margin', type=float, default=0.05,
                   help='m, extra clearance on top of the robot radius')
    p.add_argument('--occupied', type=float, default=0.5,
                   help='log-odds above which a cell is an obstacle')
    p.add_argument('--free', type=float, default=-0.5,
                   help='log-odds below which a cell is free (between: unknown, avoided)')
    p.add_argument('--min-distance', type=float, default=2.0,
                   help='m, warn if the goal is closer than this to the start')
    p.add_argument('--output-dir', default='.', help='where to write the image and plan')
    p.add_argument('--image-scale', type=int, default=3, help='image pixels per cell side')
    return p.parse_args(argv)


def bends(plan):
    """
    'N bend(s)' for the shortened path
    """
    n = max(len(plan.points) - 2, 0)
    return f'{n} bend' + ('' if n == 1 else 's')


def describe(args, grid, plan, inflate):
    """
    Lines of the plan report
    """
    (sx, sy), (gx, gy) = plan.start, plan.goal
    distance = math.hypot(gx - sx, gy - sy)
    points = plan.points
    astar_length = path_length([grid.cell_to_world(*c) for c in plan.cells])
    lines = [
        f'Grid: {args.grid} ({grid.size} x {grid.size} cells of {grid.resolution:.3f} m)',
        f'Start ({sx:.2f}, {sy:.2f}) m -> goal ({gx:.2f}, {gy:.2f}) m: {distance:.2f} m apart',
        f'Obstacles grown by {inflate:.2f} m (robot radius {args.robot_radius:.2f} m + margin '
        f'{args.margin:.2f} m); unknown cells are avoided too',
        'Straight line from start to goal: ' +
        ('blocked, so the path has to go around' if plan.direct_blocked else 'clear'),
        f'A*: {len(plan.cells)} cells, {astar_length:.2f} m, {plan.expanded} cells expanded',
        f'Shortened path: {len(points) - 1} straight legs, {path_length(points):.2f} m, '
        f'{bends(plan)}',
    ]
    if distance < args.min_distance:
        lines.append(f'Note: the goal is less than {args.min_distance:.1f} m from the start')
    if not plan.direct_blocked:
        lines.append('Note: nothing is in the way; pick a goal behind an obstacle so that '
                     'the path has to bend')
    lines.append("Waypoints (m, the grid's frame):")
    for k, (x, y) in enumerate(points):
        name = 'start' if k == 0 else 'goal' if k == len(points) - 1 else str(k)
        lines.append(f'  {name:<6s}({x:6.2f}, {y:6.2f})')
    lines.append("Drive commands for day3pkg's Executive, robot facing +x at the start:")
    for turn, dist in drive_commands(points):
        lines.append(f'  t {turn:.1f}')
        lines.append(f'  m {dist:.2f}')
    return lines


def main(argv=None):
    """
    Entry point: plan the path, then print it and write the image and the plan
    """
    args = parse_args(argv)
    grid, extra = OccupancyGrid.load(args.grid)
    poses = [tuple(p) for p in extra.get('poses', [])]
    start = tuple(args.start) if args.start else (poses[0][:2] if poses else (0.0, 0.0))
    inflate = args.robot_radius + args.margin
    try:
        plan = plan_path(grid, start, tuple(args.goal), inflate, args.occupied, args.free)
    except ValueError as e:
        print(e)
        return 1

    lines = describe(args, grid, plan, inflate)
    print('\n'.join(lines))
    os.makedirs(args.output_dir, exist_ok=True)
    path = os.path.join(args.output_dir, 'path.png')
    img = draw.plan_image(grid, plan, args.image_scale, poses,
                          f'A* path: {path_length(plan.points):.2f} m, {bends(plan)}')
    if not cv2.imwrite(path, img):
        print(f'Could not save {path}')
        return 1
    print(f'Saved {path}')
    path = os.path.join(args.output_dir, 'path.txt')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'Saved {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
