# Robotics

Coursework for CS 5335 Robotic Science and Systems (Northeastern, Fall 2026),
built on ROS 2 Jazzy and a TurtleBot 4.

## Layout

| Path                                       | Contents                                                     |
| ------------------------------------------ | ------------------------------------------------------------ |
| [`robotics_ws/`](robotics_ws/README.md)    | colcon workspace with the ROS 2 packages for Projects        |
| `Week1/`                                   | One-line scripts that drive the robot from the command line  |
| `related_cmds.txt`                         | Setup and debugging commands                                 |

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
```

Then see [`robotics_ws/README.md`](robotics_ws/README.md) to build and run the
packages.
