#!/bin/bash
# Stops everything an earlier simulator run may have left behind, so that the next sim.bash
# starts clean. A second run that collides with leftovers from the first one fails in odd ways
# (nodes never found, no map, a slow Gazebo).
# It stops EVERY ROS 2 program on this computer (launches, nodes, RViz, Gazebo) and our nodes.
pkill -INT -f "ros2 launch"           # like Ctrl-C on each launch
sleep 3
pkill -9 -f "gz sim"                  # Gazebo server and window
pkill -9 -f "/opt/ros/jazzy/lib/"     # every ROS 2 node from /opt/ros (Nav2, RViz, bridges, ...)
pkill -9 -f "lib/day8pkg/"            # create_path, point_path
sleep 1
fastdds shm clean > /dev/null 2>&1    # shared-memory files the killed nodes left
ros2 daemon stop > /dev/null 2>&1
ros2 daemon start > /dev/null 2>&1
left=$(ps aux | grep -E "gz sim|/opt/ros/jazzy/lib/|lib/day8pkg/" | grep -v grep)
if [ -n "$left" ]; then
    echo "Still running (stop with kill -9 <number in the second column>):"
    echo "$left"
else
    echo "All clean"
fi
