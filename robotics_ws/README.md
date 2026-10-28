# robotics_ws

ROS 2 (Jazzy) colcon workspace for CS 5335 Robotic Science and Systems. `day2pkg`
drives a TurtleBot 4 through `cmd_vel`; `day3pkg` drives it with the Create 3's
built-in actions and reads `/odom`; `day6pkg` builds an occupancy grid from the LiDAR,
finds corner landmarks in it, and draws both.

## Packages

| Package                            | Project | What it does                                                        |
| ---------------------------------- | ------- | ------------------------------------------------------------------- |
| [`day2pkg`](src/day2pkg/README.md) | 2       | Single node that drives a preset path (D shape) from timed segments |
| [`day3pkg`](src/day3pkg/README.md) | 3, 4, 5 | Executive + Driving nodes: drive with typed commands, log odometry  |
| `day3_interfaces`                  | 3, 4, 5 | Defines the `Drive.action` used by `day3pkg`                        |
| [`day6pkg`](src/day6pkg/README.md) | 6, 7    | Grid, Landmark and Viewer nodes: occupancy grid, corners, drawing   |

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

Some packages need system packages first: `day3pkg` needs the Create 3's messages
(`irobot_create_msgs`) and `day6pkg` needs NumPy, OpenCV and scikit-image. To install everything the
packages declare, run once from this directory (`robotics_ws`):

```bash
rosdep install -i --from-path src --rosdistro jazzy -y
```

Then, from this directory:

```bash
colcon build
source install/setup.bash
```

To build one package only: `colcon build --packages-select <pkg>`.

## Run

```bash
ros2 run day2pkg node1            # Project 2
ros2 launch day3pkg day3_launch.py   # Projects 3, 4 and 5
ros2 launch day6pkg day6_launch.py   # Project 6: Grid + Viewer nodes with day3pkg's Driving + Executive
ros2 launch day6pkg project7_launch.py   # Project 7: Grid, Landmark and Viewer nodes
```

For Project 5 (odometry), see the
[Odometry section of the day3pkg README](src/day3pkg/README.md#odometry); for
Projects 6 and 7, the [day6pkg README](src/day6pkg/README.md).

## Troubleshooting

- Check that the robot is reachable: `ros2 topic echo /ip`
- Check the velocity commands being sent (`day2pkg`): `ros2 topic echo /cmd_vel`
- Check the Create 3's built-in actions (`day3pkg`): `ros2 action list -t`
- Read one odometry message: `ros2 topic echo /odom --once`
- Topics not showing up: `ros2 daemon stop; ros2 daemon start`
