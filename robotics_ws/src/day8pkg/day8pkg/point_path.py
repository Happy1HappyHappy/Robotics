"""
Point Path for the day8pkg package (Project 8: TurtleBot 4 Navigator)
Collects waypoints with RViz "Publish Point" (/clicked_point), computes each heading toward
the next point, then drives through them with Nav2
Unlike create_path, the clicks do NOT go to AMCL, so localization is left alone: set the
robot's pose ONCE with "2D Pose Estimate" before running
Each click logs its distance from the previous point, which helps to make 1 m segments
"""

import math
import threading

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator


def make_pose(navigator, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def points_to_poses(navigator, points):
    """Each pose faces the next point; the last keeps the previous heading."""
    poses = []
    yaw = 0.0
    for i, (x, y) in enumerate(points):
        if i + 1 < len(points):
            nx, ny = points[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        poses.append(make_pose(navigator, x, y, yaw))
    return poses


def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()
    points = []

    def on_point(msg):
        x, y = msg.point.x, msg.point.y
        if points:
            px, py = points[-1]
            dist = math.hypot(x - px, y - py)
            navigator.info(f'Point {len(points) + 1} added: ({x:.2f}, {y:.2f}), '
                           f'{dist:.2f} m from previous')
        else:
            navigator.info(f'Point 1 added: ({x:.2f}, {y:.2f})')
        points.append((x, y))

    sub = navigator.create_subscription(PointStamped, 'clicked_point', on_point, 10)

    done = threading.Event()

    def wait_for_enter():
        input()
        done.set()

    threading.Thread(target=wait_for_enter, daemon=True).start()
    navigator.info('Click points with "Publish Point" in RViz, then press Enter here.')

    while not done.is_set():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    navigator.destroy_subscription(sub)

    if len(points) == 0:
        navigator.error('No points were given, exiting.')
        rclpy.shutdown()
        return

    poses = points_to_poses(navigator, points)
    navigator.info(f'Path created with {len(poses)} poses.')

    # Localization is NOT reset here: it must already be set with 2D Pose Estimate.
    if navigator.getDockedStatus():
        navigator.info('Undocking...')
        navigator.undock()

    navigator.info('Following waypoints...')
    navigator.startFollowWaypoints(poses)

    navigator.info('Done. Docking...')
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
