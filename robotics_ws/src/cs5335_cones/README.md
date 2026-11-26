# cs5335_cones

## Data Flow

```text
main()
 ├─ Create ConeDriver
 ├─ Start the ROS executor in a background thread
 └─ node.run()                         Main thread
     ├─ _wait_for_data()               Wait for sensor data
     ├─ Record the starting odometry position
     ├─ find_cones()                    Turn once to find the cones
     │   └─ _look()                     Find color blobs; save direction and distance
     ├─ _report()                       Print the cones found
     └─ choose_and_approach()
         ├─ Wait for a color selection
         ├─ drive_to_cone()
         │   ├─ _turn_to()              Turn to the recorded direction
         │   ├─ Center the cone in the camera image
         │   ├─ _measure_cone_range()   Measure its LiDAR range once, standing still
         │   └─ Drive that far by odometry, then stop
         └─ return_to_origin()          Follow the recorded path back to the start
```

## Check Topics on the Robot

Run these commands from `robotics_ws` in a terminal configured for the robot's ROS network.

Check the topics without moving the robot:

```bash
bash src/cs5335_cones/scripts/check_topics.bash
```

Check the topics and verify that the robot can turn about 17 degrees:

```bash
bash src/cs5335_cones/scripts/check_topics.bash --move
```

The script checks that the camera, LiDAR, odometry, and hazard topics have publishers. It
also checks that camera, LiDAR, and odometry messages arrive, and that `cmd_vel` uses
`TwistStamped` and has a subscriber. The `--move` option additionally commands a brief turn;
without it, the robot does not move.

## Cone Search and Approach

During the 360-degree search, the robot uses odometry yaw to measure its rotation. For each
cone color, it keeps the sighting where the color blob is closest to the camera's optical
axis, with the LiDAR distance in meters at that time. The cone's direction is the robot's yaw
when that image arrived plus the blob's bearing from the pinhole model,
`atan((cx - u) / fx)`: `u` is the blob's center column, `fx` the focal length and `cx` the
optical center column, in pixels. `fx` and `cx` come from
`oakd/rgb/preview/camera_info`; until it arrives, `fx` is computed from the `hfov` parameter
and `cx` is the middle of the image.

For the approach, the robot turns to the direction recorded for the selected cone. Standing
still, it computes the cone's bearing in a new camera image and turns that far by odometry,
repeating until the bearing is within `center_tolerance`. Standing still, it then measures the cone's range
once with the LiDAR, and drives that range minus `stop_distance` by odometry, without
measuring again. Odometry positions recorded during the approach are followed in reverse to
return to the starting position.

The cones are shorter than the LiDAR. Its beam can still graze a cone's tip from far away
(more than about 1.5 m), but not up close, which is why the range is measured only once,
at the start. Before the first run, check that the range Task 1 prints for each cone
matches a tape measure. If the LiDAR does not see a cone from where the robot starts, the
approach stops with `the LiDAR saw a narrow object ahead in only ...` instead of driving.

A scan counts as seeing the cone when the nearest return within `front_half_width` of
straight ahead is narrow: most beams in that window pass beside it to something at least
0.15 m farther, or to nothing. A wall or a wide box ahead fills the window and does not
count. A narrow object such as a chair leg straight ahead does count, so keep the line from
the robot to the cone clear.

## Selecting a Cone

Run the node from an interactive terminal so it can receive keyboard input. After the search
finishes, enter one of the configured cone colors when prompted. The node approaches that
cone and returns to the starting position before prompting again. Enter `q` to leave the
selection loop.

## YAML Configuration

The settings are in `config/cones.yaml`. Run the node from `robotics_ws` to use this file:

```bash
ros2 run cs5335_cones cone_driver --ros-args --params-file src/cs5335_cones/config/cones.yaml
```

| Parameter | Meaning |
|---|---|
| `colors` | Cone colors to detect and offer for selection. |
| `hsv.<color>` | HSV color bounds for each color, in `[h_low, s_low, v_low, h_high, s_high, v_high]` order. OpenCV hue is 0–179; saturation and value are 0–255. If `h_low > h_high`, the hue range wraps through 0. |
| `min_area` | Minimum color blob area in pixels to count as a detection. |
| `turn_speed` | Angular speed in rad/s during the 360-degree search. |
| `lidar_yaw` | LiDAR mounting angle relative to the robot, in radians. |
| `stop_distance` | Distance in meters from the LiDAR to the cone's center at which the approach stops: from the LiDAR's center to the robot's front edge, plus the gap to leave (about 1 inch), plus half the cone's base. Measure it with a ruler; start large and lower it after test runs. |
| `center_tolerance` | Largest camera bearing to the cone, in radians, at which the approach stops aligning and drives. |
| `hfov` | Camera's horizontal field of view in radians, used for the focal length only while `camera_info` has not arrived. Check `timeout 5 ros2 topic echo /oakd/rgb/preview/camera_info --once`. |
