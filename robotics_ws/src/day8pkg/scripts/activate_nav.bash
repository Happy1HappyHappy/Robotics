#!/bin/bash
# Brings localization and Nav2 to active, node by node, asking again whenever an answer gets
# lost (which leaves a node half-started). Run it after starting localization and Nav2, with
# the robot off the dock and its pose set. It returns at once if everything is already active.
nodes="map_server amcl controller_server smoother_server planner_server behavior_server
velocity_smoother collision_monitor bt_navigator waypoint_follower docking_server"
for n in $nodes; do
    until s=$(timeout -k 3 10 ros2 lifecycle get /$n 2>/dev/null); [[ "$s" == active* ]]; do
        echo "== $n: ${s:-no answer}"
        if [[ "$s" == unconfigured* ]]; then
            timeout -k 3 15 ros2 lifecycle set /$n configure
        else
            timeout -k 3 15 ros2 lifecycle set /$n activate
        fi
    done
    echo "$n: active"
done
echo "All nodes active"
