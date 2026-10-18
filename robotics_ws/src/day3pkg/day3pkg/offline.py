"""
Task 1 offline: occupancy grid from one saved /scan message, without the robot

Save one message while connected to the robot (--full-length keeps all the ranges;
without it, 'ros2 topic echo' cuts arrays off after 128 elements):
    ros2 topic echo /scan --once --full-length > scan.yaml
Then, offline:
    ros2 run day3pkg offline_grid scan.yaml
"""

import argparse
import math
import os

import cv2
import yaml

from day3pkg.grid import OccupancyGrid
from day3pkg.render import grid_image


def load_scan(path):
    """
    Read the first sensor_msgs/LaserScan in a file written by 'ros2 topic echo'
    Returns the message as a dict of its fields; ranges become floats (.inf / .nan included)
    Raises ValueError if the file holds no scan, or its ranges were truncated
    """
    try:
        with open(path) as f:
            docs = list(yaml.safe_load_all(f))
    except yaml.YAMLError as e:
        raise ValueError(f'{path}: not the YAML that ros2 topic echo writes ({e})')
    scans = [d for d in docs if isinstance(d, dict) and 'ranges' in d]
    if not scans:
        raise ValueError(f'{path}: no LaserScan message found')
    scan = scans[0]
    if any(isinstance(r, str) for r in scan['ranges']):
        raise ValueError(f"{path}: the ranges were cut off ('...'); "
                         'capture the scan again with --full-length')
    scan['ranges'] = [float(r) for r in scan['ranges']]
    return scan


def main(argv=None):
    """
    Build the grid from one scan with the sensor at the origin, then save and show it
    """
    parser = argparse.ArgumentParser(description='Occupancy grid from one saved LaserScan')
    parser.add_argument('scan_file',
                        help="output of 'ros2 topic echo /scan --once --full-length'")
    parser.add_argument('--mount-yaw', type=float, default=90.0,
                        help='LiDAR yaw relative to the robot in degrees, so the robot '
                             'faces up in the image (TurtleBot 4 URDF: 90; 0 = draw the '
                             'scan in the LiDAR frame)')
    parser.add_argument('--size', type=int, default=200, help='cells per side')
    parser.add_argument('--resolution', type=float, default=0.05, help='cell side, m')
    parser.add_argument('--scale', type=int, default=3, help='image pixels per cell side')
    parser.add_argument('--output', default='grid_offline.png',
                        help="image file to save ('' = do not save)")
    parser.add_argument('--no-show', action='store_true',
                        help='only save the image, do not open a window')
    args = parser.parse_args(argv)

    try:
        scan = load_scan(args.scan_file)
    except (OSError, ValueError) as e:
        parser.exit(1, f'Error: {e}\n')

    grid = OccupancyGrid(args.size, args.resolution)
    pose = (0.0, 0.0, math.radians(args.mount_yaw))    # Task 1: sensor at the origin
    used = grid.update(pose, scan['ranges'], scan['angle_min'], scan['angle_increment'],
                       scan['range_min'], scan['range_max'])
    print(f'{used} of {len(scan["ranges"])} rays used '
          f'(range {scan["range_min"]:g} to {scan["range_max"]:g} m)')

    img = grid_image(grid, args.scale, [(0.0, 0.0, 0.0)],     # robot faces +x (up)
                     os.path.basename(args.scan_file))
    if args.output:
        cv2.imwrite(args.output, img)
        print(f'Saved {args.output}')
    if not args.no_show:
        cv2.imshow('Occupancy grid', img)
        print('Press any key in the image window to close it')
        cv2.waitKey(0)                  # required, or the window never draws
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
