"""
Create Path for the day8pkg package (Project 8: TurtleBot 4 Navigator)
Same flow as the tutorial's create_path (turtlebot4_python_tutorials), but collects the RViz
"2D Pose Estimate" clicks with its own subscription instead of
TurtleBot4Navigator.createPath()
1. Click the path's poses in RViz with "2D Pose Estimate", then press Enter here
2. Dock if needed, reset the pose to the dock at (0, 0) and clear the costmaps
3. Undock, wait for AMCL, follow the clicked poses as waypoints with Nav2, and dock again
The clicks also move AMCL's estimate, which is why step 2 resets it
"""

import threading
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions, TurtleBot4Navigator)


def clear_costmaps_with_timeout(navigator, timeout_sec=3.0):
    """Like clearAllCostmaps(), but gives up instead of waiting forever."""
    for name in ['local_costmap/clear_entirely_local_costmap',
                 'global_costmap/clear_entirely_global_costmap']:
        client = navigator.create_client(ClearEntireCostmap, name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            navigator.warn(f'{name} not available, skipping')
            navigator.destroy_client(client)
            continue
        future = client.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=timeout_sec)
        if future.done():
            navigator.info(f'{name}: cleared')
        else:
            navigator.warn(f'{name}: no response within {timeout_sec}s, skipping')
        navigator.destroy_client(client)


def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()
    poses = []

    def on_pose(msg):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = navigator.get_clock().now().to_msg()
        p.pose = msg.pose.pose
        poses.append(p)
        navigator.info(f'Pose {len(poses)} added: '
                       f'({p.pose.position.x:.2f}, {p.pose.position.y:.2f})')

    sub = navigator.create_subscription(
        PoseWithCovarianceStamped, 'initialpose', on_pose, 10)

    done = threading.Event()

    def wait_for_enter():
        input()
        done.set()

    threading.Thread(target=wait_for_enter, daemon=True).start()
    navigator.info('Click poses with "2D Pose Estimate" in RViz, then press Enter here.')

    while not done.is_set():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    navigator.destroy_subscription(sub)
    navigator.info(f'Path created with {len(poses)} poses.')

    if len(poses) == 0:
        navigator.error('No poses were given, exiting.')
        rclpy.shutdown()
        return

    # Start on dock
    navigator.info('Checking dock status...')
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Reset localization (the clicks above also moved AMCL's estimate)
    navigator.info('Clearing costmaps and setting initial pose...')
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    clear_costmaps_with_timeout(navigator)
    navigator.setInitialPose(initial_pose)   # also resets initial_pose_received

    # Power saver turns the lidar off while docked, so AMCL cannot answer yet.
    # Undock first; AMCL keeps the (0, 0) pose and follows the undock via odom.
    # (waitUntilNav2Active() would re-publish (0, 0) forever while docked.)
    navigator.info('Undocking (lidar starts after undock)...')
    navigator.undock()

    navigator.info('Waiting for AMCL to report a pose...')
    start = time.time()
    while not navigator.initial_pose_received:
        rclpy.spin_once(navigator, timeout_sec=0.5)
        if time.time() - start > 30.0:
            navigator.error('No amcl_pose after 30 s: check /scan and AMCL.')
            rclpy.shutdown()
            return
    navigator.info('AMCL pose received.')

    navigator.info('Following waypoints...')
    navigator.startFollowWaypoints(poses)

    navigator.info('Docking...')
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
