# cs5335_cones: plan

Project 9: turn 360 degrees and name the 3 colored cones, then, in a loop, drive to the cone
the user names, stop about an inch from it without touching it, and return to the starting
position before asking for the next color.

## Idea

- **Camera**: which color, and whether it is in the middle of the image.
- **Odometry**: which way the robot was facing at that moment.
- **LiDAR**: how far away the thing straight ahead is.

## Task 1

1. Turn left at `turn_speed` until odometry says the robot has turned 360 degrees.
2. For each image and each color: HSV mask, then the largest blob, and its bearing
   `atan((cx - u) / fx)` from the focal length (`camera_info`, else `hfov`). If the bearing
   is smaller than in any earlier image, save the cone's yaw (robot yaw + bearing) and the
   LiDAR range straight ahead at that moment.
3. Stop, then print each color's direction and distance.

## Task 2 (loop until `q`)

1. Ask for a color.
2. Turn to the saved yaw (odometry).
3. Standing still, compute the blob's bearing and turn that far by odometry; repeat until
   it is within `center_tolerance` rad.
4. Drive slowly while keeping the cone centered. Stop when the LiDAR's front range is at or
   below `stop_range`.
5. Return to the odometry position recorded before Task 1, then ask for the next color.

## Files

| File | What |
| --- | --- |
| `cs5335_cones/sensing.py` | No ROS. `find_blob(bgr, hsv_range, min_area)` returns the blob's center column and area, or None. `front_range(ranges, angle_min, angle_increment, lidar_yaw, half_width)` returns the nearest valid range straight ahead |
| `cs5335_cones/cone_driver.py` | The node: Task 1, then Task 2 |
| `config/cones.yaml` | Colors, HSV ranges, speeds, `stop_range` |
| `test/` | The 3 ament lint tests, plus `test_sensing.py` |

Parameters: `colors`, `hsv.<color>`, `min_area`, `turn_speed`, `lidar_yaw`,
`front_half_width`, `data_timeout` (Task 1); Task 2 adds `stop_range` and
`center_tolerance`.

No launch file: `ros2 launch` does not pass keyboard input to a node, and Task 2 uses
`input()`. Run it with `ros2 run ... --ros-args --params-file config/cones.yaml`.

## Safety

- The robot stops on every exit: normal end, `q`, Ctrl-C, or an exception (`finally`).
- It stops if odometry or LiDAR data is older than 0.5 s.
- It stops on a bump or cliff (`hazard_detection`).
- Speed is at most 0.15 m/s and 0.8 rad/s.

## Left out (compared with day9pkg)

Shape check, tracking the cone in the odom frame, camera intrinsics, TF lookup, the OpenCV
window thread.

## Build order

1. Package skeleton, `sensing.py` and its test
2. A node that only watches and logs, with no motion: tune the HSV ranges on the robot
3. Task 1
4. Task 2
5. README

## Check on the robot

Before each run, from `robotics_ws`, check that every topic works:

```bash
bash src/cs5335_cones/scripts/check_topics.bash --move
```

It checks that the camera, LiDAR, odometry and hazard topics have a publisher and (except
hazards) that their messages arrive, that `cmd_vel` is `TwistStamped` with a subscriber, and,
with `--move`, that the robot turns about 17 degrees when told to. Without `--move` the robot
does not move.

| What | How |
| --- | --- |
| Every topic works | `scripts/check_topics.bash` (above) |
| Which way the LiDAR's 0 angle points (`lidar_yaw`) | `ros2 run tf2_ros tf2_echo base_link rplidar_link` |
| `stop_range` | Set the robot 1 inch from a cone's base and read the front range |
