"""
Create Path for the day8pkg package (Project 8: TurtleBot 4 Navigator)
Same flow as the tutorial's create_path (turtlebot4_python_tutorials), but collects the RViz
"2D Pose Estimate" clicks with its own subscription instead of
TurtleBot4Navigator.createPath()
1. Click the path's poses in RViz with "2D Pose Estimate", then press Enter here
2. Dock if needed, reset the pose to the dock and clear the costmaps
3. Undock, wait for AMCL and Nav2, follow the clicked poses as waypoints, and dock again
The clicks also move AMCL's estimate, which is why step 2 resets it
The dock's pose on the map comes from the dock_x, dock_y and dock_yaw_deg parameters; the
defaults (0, 0, facing +x) fit a map that was started on the dock, like the simulator's
"""

import threading
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator


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


def wait_until_active(navigator, clients, timeout_sec=5.0):
    """Like waitUntilNav2Active(), but asks again when an answer gets lost."""
    for name, client in clients.items():
        while rclpy.ok():
            state = 'not available'
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(navigator, future, timeout_sec=timeout_sec)
                state = future.result().current_state.label if future.done() else 'no answer'
            if state == 'active':
                navigator.info(f'{name} is active')
                break
            navigator.info(f'{name} is {state}, waiting...')
            time.sleep(1.0)


def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()
    # The dock's pose on the map; write values with a decimal point (1.0, not 1)
    dock_x = navigator.declare_parameter('dock_x', 0.0).value              # m
    dock_y = navigator.declare_parameter('dock_y', 0.0).value              # m
    dock_yaw_deg = navigator.declare_parameter('dock_yaw_deg', 0.0).value  # deg, 0 = facing +x
    # Created now so that Nav2 knows these clients by the time they ask: the first answer to a
    # brand-new client often gets lost over the robot's discovery server
    nav2_states = {name: navigator.create_client(GetState, f'{name}/get_state')
                   for name in ('bt_navigator', 'waypoint_follower')}
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

    # Reset localization to the dock (the clicks above also moved AMCL's estimate)
    navigator.info(f'Clearing costmaps and setting initial pose to the dock at '
                   f'({dock_x:.2f}, {dock_y:.2f}), {dock_yaw_deg:.0f} deg...')
    initial_pose = navigator.getPoseStamped([dock_x, dock_y], dock_yaw_deg)
    clear_costmaps_with_timeout(navigator)
    navigator.setInitialPose(initial_pose)   # also resets initial_pose_received

    # Power saver turns the lidar off while docked, so AMCL cannot answer yet.
    # Undock first; AMCL keeps the dock pose and follows the undock via odom.
    # (waitUntilNav2Active() would re-publish the dock pose forever while docked.)
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

    # Nav2 rejects goals until its nodes are active
    navigator.info('Waiting for Nav2 (bt_navigator, waypoint_follower) to be active...')
    wait_until_active(navigator, nav2_states)

    navigator.info('Following waypoints...')
    navigator.startFollowWaypoints(poses)

    navigator.info('Docking...')
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
