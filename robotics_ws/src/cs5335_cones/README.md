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
         │   └─ Align again after 1 m, then drive until stopping
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
cone color, it keeps the sighting where the color blob is closest to the center of the camera
image, along with the robot's yaw and the LiDAR distance in meters at that time.

For the approach, the robot turns to the yaw recorded for the selected cone. It drives 1 m
according to odometry, stops to realign the cone using the camera, and then continues toward
the cone until the LiDAR stopping condition is met. Odometry positions recorded during the
approach are followed in reverse to return to the starting position.

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
| `stop_range` | Front LiDAR distance in meters at which the approach stops. |
| `center_tolerance` | Allowed cone offset from the camera center, as a fraction of the image half-width. |

The 1 m odometry distance for the approach is not currently a YAML parameter.
