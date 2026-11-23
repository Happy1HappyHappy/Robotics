# robotics_ws

ROS 2 (Jazzy) colcon workspace for CS 5335 Robotic Science and Systems. `day2pkg`
drives a TurtleBot 4 through `cmd_vel`; `day3pkg` drives it with the Create 3's
built-in actions and reads `/odom`; `day6pkg` builds an occupancy grid from the LiDAR;
`day7pkg` finds landmarks in two such grids and plans paths on them with A*; `day8pkg`
drives a path clicked in RViz with Nav2 on a map of the room; `day9pkg`
finds colored traffic cones with the camera and drives to them; `cs5335_cone_finder`
does the same with YOLO and visits each cone in turn, coming back to the start between;
`cs5335_cone_navigation` finds cones by color and shape with classical vision, keeps a
registry with unique IDs, and drives to the cone you pick.

## Packages

| Package                            | Project | What it does                                                        |
| ---------------------------------- | ------- | ------------------------------------------------------------------- |
| [`day2pkg`](src/day2pkg/README.md) | 2       | Single node that drives a preset path (D shape) from timed segments |
| [`day3pkg`](src/day3pkg/README.md) | 3, 4, 5 | Executive + Driving nodes: drive with typed commands, log odometry  |
| `day3_interfaces`                  | 3, 4, 5 | Defines the `Drive.action` used by `day3pkg`                        |
| [`day6pkg`](src/day6pkg/README.md) | 6, 7    | Grid Node: occupancy grid from the LiDAR, saved as image and data   |
| [`day7pkg`](src/day7pkg/README.md) | 7       | RANSAC corner landmarks, aligning two grids, A* path planning       |
| [`day8pkg`](src/day8pkg/README.md) | 8       | Create Path: drive a path clicked in RViz with Nav2, plus the course's simulator scripts |
| [`day9pkg`](src/day9pkg/README.md) | 9       | Cone Node: find colored cones by camera, drive to one with LiDAR    |
| [`cs5335_cone_finder`](src/cs5335_cone_finder/README.md) | 9 | Cone Detector (YOLO) + Cone Mission: record 3 cones, visit each and return |
| [`cs5335_cone_navigation`](src/cs5335_cone_navigation/README.md) | 9 | Cone Detector (color + shape) + Cone Navigator: registry with IDs, drive to a chosen cone |

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
(`irobot_create_msgs`), `day6pkg` needs NumPy and OpenCV, `day8pkg` needs the TurtleBot 4
navigation package (`turtlebot4_navigation`), and `day9pkg` also needs
`cv_bridge`. `cs5335_cone_finder` also needs `vision_msgs` and the pip package
`ultralytics` (see [its README](src/cs5335_cone_finder/README.md#install) for installing
it). To install everything the
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
ros2 run day6pkg mapper              # Project 6 (Grid Node only)
ros2 launch day6pkg day6_launch.py   # Project 6 with day3pkg's Driving + Executive
ros2 run day7pkg landmarks run_a/grid.npz run_b/grid.npz   # Project 7, Task 2
ros2 run day7pkg plan run_a/grid.npz --goal 2.2 0.7         # Project 7, Task 3
ros2 run day8pkg create_path         # Project 8, after localization and Nav2 (see its README)
ros2 run day9pkg cones --ros-args --params-file src/day9pkg/config/cones.yaml   # Project 9
ros2 launch cs5335_cone_finder cone_finder_launch.py params_file:=src/cs5335_cone_finder/config/cone_finder.yaml   # Project 9 with YOLO
ros2 launch cs5335_cone_navigation cone_navigation_launch.py params_file:=src/cs5335_cone_navigation/config/cone_navigation.yaml   # Project 9, classical vision
```

For Project 5 (odometry), see the
[Odometry section of the day3pkg README](src/day3pkg/README.md#odometry); for
Project 6, the [day6pkg README](src/day6pkg/README.md); for Project 7 (including how to
record the two grids), the [day7pkg README](src/day7pkg/README.md); for Project 8 (mapping
the room, starting localization and Nav2, and the demo), the [day8pkg README](src/day8pkg/README.md); for Project 9 (setting
up the cones and tuning the colors), the [day9pkg README](src/day9pkg/README.md).

## Troubleshooting

- Check that the robot is reachable: `ros2 topic echo /ip`
- Check the velocity commands being sent (`day2pkg`): `ros2 topic echo /cmd_vel`
- Check the Create 3's built-in actions (`day3pkg`): `ros2 action list -t`
- Read one odometry message: `ros2 topic echo /odom --once`
- Topics not showing up: `ros2 daemon stop; ros2 daemon start`
