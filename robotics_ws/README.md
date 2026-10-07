# robotics_ws

ROS 2 (Jazzy) colcon workspace for CS 5335 Robotic Science and Systems. All
packages drive a TurtleBot 4 through `cmd_vel`.

## Packages

| Package                                   | Project | What it does                                                         |
| ----------------------------------------- | ------- | -------------------------------------------------------------------- |
| [`day2pkg`](src/day2pkg/README.md)         | 2       | Single node that drives a preset path (D shape) from timed segments  |
| [`day3pkg`](src/day3pkg/README.md)         | 3, 4    | Executive + Driving nodes; drive the robot with typed commands       |
| `day3_interfaces`                         | 3, 4    | Defines the `Drive.action` used by `day3pkg`                         |

See each package's README for details.

## Setup

Run in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
```

`ROS_DOMAIN_ID` must match the robot's.

## Build

From this directory (`robotics_ws`):

```bash
colcon build
source install/setup.bash
```

To build one package only: `colcon build --packages-select <pkg>`.

## Run

```bash
ros2 run day2pkg node1            # Project 2
ros2 launch day3pkg day3_launch.py   # Projects 3 and 4
```

## Troubleshooting

- Check that the robot is reachable: `ros2 topic echo /ip`
- Check the velocity commands being sent: `ros2 topic echo /cmd_vel`
- Topics not showing up: `ros2 daemon stop; ros2 daemon start`
