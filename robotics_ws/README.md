# robotics_ws

ROS 2 (Jazzy) colcon workspace for CS 5335 Robotic Science and Systems.
`day2pkg` and `day3pkg` drive a TurtleBot 4 through `cmd_vel` (`day3pkg` also reads
`/odom`); `day6pkg` only reads sensors and never moves the robot.

## Packages

| Package                                   | Project | What it does                                                         |
| ----------------------------------------- | ------- | -------------------------------------------------------------------- |
| [`day2pkg`](src/day2pkg/README.md)         | 2       | Single node that drives a preset path (D shape) from timed segments  |
| [`day3pkg`](src/day3pkg/README.md)         | 3, 4, 5 | Executive + Driving nodes; drive with typed commands, log odometry   |
| `day3_interfaces`                         | 3, 4, 5 | Defines the `Drive.action` used by `day3pkg`                         |
| [`day6pkg`](src/day6pkg/README.md)         | 6       | Occupancy grid from the LiDAR (`/scan` + `/odom`), saved as images   |

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
ros2 launch day3pkg day3_launch.py   # Projects 3, 4 and 5
ros2 run day6pkg mapper              # Project 6 (live)
ros2 run day6pkg offline_grid scan.yaml   # Project 6 (one saved scan)
```

For Project 5 (odometry), see the
[Odometry section of the day3pkg README](src/day3pkg/README.md#odometry).

## Troubleshooting

- Check that the robot is reachable: `ros2 topic echo /ip`
- Check the velocity commands being sent: `ros2 topic echo /cmd_vel`
- Read one odometry message: `ros2 topic echo /odom --once`
- Save one full LiDAR scan: `ros2 topic echo /scan --once --full-length > scan.yaml`
- Topics not showing up: `ros2 daemon stop; ros2 daemon start`
