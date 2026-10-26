# Robotics

Coursework for CS 5335 Robotic Science and Systems (Northeastern, Fall 2026),
built on ROS 2 Jazzy and a TurtleBot 4.

## Layout

| Path                                       | Contents                                                     |
| ------------------------------------------ | ------------------------------------------------------------ |
| [`robotics_ws/`](robotics_ws/README.md)    | colcon workspace with the ROS 2 packages for Projects 2-7    |
| `Week1/`                                   | One-line scripts that drive the robot from the command line  |
| `related_cmds.txt`                         | Setup and debugging commands                                 |

## Projects

| Project | Topic                         | Where                                                             |
| ------- | ----------------------------- | ----------------------------------------------------------------- |
| 2       | Timed driving with `cmd_vel`  | [`day2pkg`](robotics_ws/src/day2pkg/README.md)                     |
| 3       | ROS actions                   | [`day3pkg`](robotics_ws/src/day3pkg/README.md)                     |
| 4       | Turns and arcs (kinematics)   | [`day3pkg`](robotics_ws/src/day3pkg/README.md)                     |
| 5       | Odometry                      | [`day3pkg`](robotics_ws/src/day3pkg/README.md#odometry)           |
| 6       | Occupancy grid from LiDAR     | [`day6pkg`](robotics_ws/src/day6pkg/README.md)                     |
| 7       | Landmarks and A* path planning | [`day7pkg`](robotics_ws/src/day7pkg/README.md)                    |

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
```

Then see [`robotics_ws/README.md`](robotics_ws/README.md) to build and run the
packages.
