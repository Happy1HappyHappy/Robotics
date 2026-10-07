# Robotics

Coursework for CS 5335 Robotic Science and Systems (Northeastern, Fall 2026),
built on ROS 2 Jazzy and a TurtleBot 4.

## Layout

| Path                                       | Contents                                                     |
| ------------------------------------------ | ------------------------------------------------------------ |
| [`robotics_ws/`](robotics_ws/README.md)    | colcon workspace with the ROS 2 packages for Projects 2–4    |
| `Week1/`                                   | One-line scripts that drive the robot from the command line  |
| `related_cmds.txt`                         | Setup and debugging commands                                 |

## Week1 scripts

| Script            | What it does                                                    |
| ----------------- | --------------------------------------------------------------- |
| `teleop.bash`     | Keyboard teleop (`teleop_twist_keyboard`, stamped twists)       |
| `20cm.bash`       | Publish forward velocity 0.18 m/s, 12 messages at 11 Hz         |
| `ros_twist.bash`  | Same as `20cm.bash`                                             |
| `circle.bash`     | Spin in place at 0.2 rad/s, 12 messages at 11 Hz                |

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
```

Then see [`robotics_ws/README.md`](robotics_ws/README.md) to build and run the
packages.
