# cs5335_cone_navigation

CS 5335 Project 9 (Computer Vision and Autonomous Traffic Cone Navigation). The TurtleBot 4
turns 360 degrees, finds the traffic cones of the **target colors** (red, blue and green by
default) by **color and shape** with classical computer vision (OpenCV, no deep learning),
measures each one with the **2D LiDAR**, keeps them in a **registry with unique IDs**, and
then drives to the cone you pick, stopping about an inch from it. No depth camera is used.

## Architecture

Two nodes. The navigator's parts are separate modules, each with its own tests.

```text
camera image ──> cone_detector ── cone_detections (vision_msgs/Detection2DArray) ──┐
                  │  HSV masks of the target colors → opening/closing               │
                  │  → all contours → color + shape checks → confidence             │
                  └─ cone_detections/debug, cone_detections/mask (for viewing)      │
                                                                                    v
camera_info, scan, odom, hazard_detection, TF ──> cone_navigator ──> cmd_vel
      fusion.py    camera ray (camera_info + camera TF) ∩ LiDAR points (LiDAR TF)
      registry.py  unique IDs; same color AND close together = same cone
      state machine EXPLORE → SELECT_TARGET → ALIGN → APPROACH → STOP → COMPLETE ─┐
                                  ^                                                │
                                  └────────────────────────────────────────────────┘
```

| File | What it does | ROS |
| --- | --- | --- |
| `cone_detection.py` | Color segmentation, contours, shape recognition, confidence | no |
| `fusion.py` | Camera bearing from fx and cx; LiDAR points along the camera ray | no |
| `registry.py` | Cone registry and data association | no |
| `control.py` | Frames, LiDAR points, P controllers, obstacle strip | no |
| `cone_detector.py` | Node: image in, detections and debug images out | yes |
| `cone_navigator.py` | Node: exploration, registry, state machine, user prompt | yes |

**Why two nodes and not four.** Fusion, exploration and navigation all need the same
odometry pose, the same LiDAR scan and one control loop. As separate nodes they would need
a custom message, an interface package and time synchronization between nodes. They are
kept as separate modules in one node instead, as the project description allows.
Detections use the standard `vision_msgs/Detection2DArray`, so no custom message is needed.

## How it works

### Detection (`cone_detector`)

1. **HSV masks.** Each image is converted from BGR to HSV and a mask is made for each
   **target** color (`target_colors`, ranges in `hsv.<color>`). Red has two ranges because
   its hue wraps around 0. Colors outside the target set get no mask, so yellow or orange
   cones never become candidates.
2. **Opening and closing** remove specks and fill small holes.
3. **All contours** of each mask (`cv2.findContours`), not just the largest, so several
   cones, also of the same color, are found in one image. Contours under
   `min_contour_area` are noise.
4. **Shape checks** on each contour:
   - aspect ratio (height / width) within `[min_aspect, max_aspect]`;
   - **tapering**: 1 - (mean width of the top third) / (mean width of the bottom third) of
     at least `min_taper`. A rectangle or a disc has about 0, so it is rejected even in a
     target color;
   - **symmetry** of the left and right outline about the central axis;
   - **base**: how wide the bottom rows are compared with the widest row;
   - **solidity** (area / convex hull) and the corners of the `cv2.approxPolyDP` outline.
     No perfect triangle is required.
5. **Color check.** Of the strongly colored pixels around the contour (its box widened by
   `color_context`), at least `min_color_fraction` must be of the target color. An orange
   cone with a few reddish pixels fails this.
6. **Confidence** is the weighted mean of the color, aspect, taper, symmetry and geometry
   scores (`weights`). It is a heuristic score, not a calibrated probability. A candidate
   is accepted only if it passes every check and its confidence reaches `min_confidence`.
   A box touching the image edge is rejected, since part of the object may be cut off.

Only accepted cones are published. In each detection, `results[0].hypothesis.class_id` is
the color, `score` the confidence, `bbox.center` the contour's centroid and `bbox.size`
its box, in pixels.

### Fusion (`fusion.py`)

- **Camera bearing.** With the pinhole model, a centroid at column u is at
  atan2(u - cx, fx) to the right of the optical axis, using fx and cx from `camera_info`.
  The code uses atan2(cx - u, fx), the same angle with + = left as in REP 103.
- **Camera and LiDAR are not assumed aligned.** Both mountings come from TF: the camera
  frame (from `camera_info`) and the LiDAR frame (from the scan) relative to `base_link`.
  The camera ray starts at the camera's position, and the LiDAR points are compared with
  it as seen from there, in one frame.
- **LiDAR association.** Of the valid LiDAR points within the cone's angular half-width
  (plus `bearing_margin`) of the ray, the nearest one and those up to `cluster_depth`
  behind it are the cone. With several objects in the same direction, the nearest is the
  one the camera sees. If no point lies along the ray within `max_cone_range`, the
  distance is **unavailable**; no value is made up.

### Task 1: EXPLORE

- The initial heading is logged, and the robot turns at `scan_speed` until odometry says
  it has turned 360 degrees. Yaw changes are wrapped and summed; no timed sleep is used.
- Each accepted detection is fused with the newest LiDAR scan. Both are put in the odom
  frame using the robot's pose when each arrived, since the robot turns between them.
- **Registry and data association** (`registry.py`). An observation belongs to a known
  cone only if the colors match **and** the cone is within `association_distance` (by
  LiDAR position), or within `association_angle` in direction when one of them has no
  LiDAR position. Two cones of the same color stay separate. A cone gets an ID once it
  has been seen `min_observations` times, so one-off false detections get none. Its
  position is the confidence-weighted mean of its LiDAR positions.

### Task 2: SELECT_TARGET → ALIGN → APPROACH → STOP → COMPLETE

- **SELECT_TARGET**: lists the cones (ID, color, position, distance or "distance
  unavailable") and asks for an ID. Invalid IDs are rejected. `r` explores again, `q`
  quits.
- **ALIGN**: turns toward the cone by odometry, then centers it in the image with
  angular velocity = `kp_angular` × bearing error. Only detections of the target's color
  within `target_gate` of where it should be count, so it does not lock onto another
  cone. Each detection is compared with the robot's heading now, so the image's delay does
  not cause overshoot. It is done after 3 centered images in a row with the robot
  stopped. A still LiDAR scan must then show the cone; **without a LiDAR distance there is
  no approach**. If the cone is not in view, it waits `lost_time`, then searches by
  turning left, for at most one full turn.
- **APPROACH**: with each scan, the points within `track_gate` of the cone's last
  position are the cone. Linear velocity = `kd_linear` × (gap - stop distance), within
  `[min_linear_velocity, max_linear_velocity]`, so it slows down near the cone. It steers
  toward the cone and turns in place if the cone is more than `heading_limit` off. If the
  newest scan does not show the cone, it stops and waits. After `lost_time` the cone is
  lost, and it goes back to ALIGN once (`max_reacquire`). Anything other than the cone
  within `obstacle_clearance` ahead stops it.
- **STOP**: the robot stops when the LiDAR's gap reaches `stop_gap` + `cone_base_offset`,
  then measures the final gap in a still scan.
- **COMPLETE**: logs `Successfully reached Red Cone #1.` and `Select another target?`, and
  goes back to SELECT_TARGET. Targets can be visited in any order.

**The gap** is from the robot's edge to the nearest LiDAR point of the cone: the
TurtleBot 4 is round, so this is the distance from `base_link` minus `robot_radius`. The
LiDAR hits the cone at its own height, where the cone may be narrower than at the floor.
`cone_base_offset` is that difference and must be measured (see below). One-inch accuracy
is **not** claimed until measured on the robot.

## Before you start: check the hardware

Nothing here has been run on the robot yet. Run these first:

| What | How |
| --- | --- |
| **The LiDAR can see the cones.** Training cones are short; if they are lower than the LiDAR's scan plane, every distance is unavailable and Task 2 will not approach | `ros2 run tf2_ros tf2_echo base_link rplidar_link` (z is the LiDAR's height), then in RViz (LaserScan, `/scan`) check that each cone shows up as a small cluster of points |
| Camera topics and image size | `ros2 topic list \| grep oakd`, `timeout 5 ros2 topic echo /oakd/rgb/preview/camera_info --once` (frame_id, width, k) |
| Camera and LiDAR transforms exist | `ros2 run tf2_ros tf2_echo base_link oakd_rgb_camera_optical_frame` (use the camera_info frame_id) |
| `cmd_vel` type (sets `cmd_vel_stamped`) | `ros2 topic info -v /cmd_vel` |
| Odometry and scan rates | `ros2 topic hz /odom`, `ros2 topic hz /scan` |
| Hazards get through (bump and cliff stops) | `timeout 5 ros2 topic echo /hazard_detection --once` |
| `vision_msgs` and `cv_bridge` are installed | `ros2 interface show vision_msgs/msg/Detection2D`, `python3 -c "import cv_bridge"` |

## Build

From the workspace root (`robotics_ws`):

```bash
rosdep install -i --from-path src/cs5335_cone_navigation --rosdistro jazzy -y
```

```bash
colcon build --packages-select cs5335_cone_navigation
```

```bash
source install/setup.bash
```

## Run

**1. Tune the detection without moving the robot.** Start only the detector:

```bash
ros2 launch cs5335_cone_navigation cone_navigation_launch.py navigator:=false params_file:=src/cs5335_cone_navigation/config/cone_navigation.yaml
```

Watch the debug image and the masks (real-time debug visualization):

```bash
ros2 run rqt_image_view rqt_image_view /cone_detections/debug
```

Accepted cones are boxed in their color with their confidence. Rejected candidates are
gray and labeled `UNKNOWN (<check that failed>)`. `/cone_detections/mask` shows the target
colors' masks. Adjust `hsv.<color>` until each target cone is fully white in the mask and
nothing else is, then adjust the shape thresholds until each cone is accepted and other
objects are not.

**2. The whole task.** With the robot off the dock among the cones and someone ready to
lift it:

```bash
ros2 launch cs5335_cone_navigation cone_navigation_launch.py params_file:=src/cs5335_cone_navigation/config/cone_navigation.yaml
```

The navigator opens in its own xterm window, because it asks for target IDs. Once all data
has arrived, it **explores (turns 360 degrees) on its own**, lists the targets, and waits
for an ID. Ctrl-C in its window stops it at any time.

Over SSH without a desktop, start the detector with `navigator:=false` in one terminal and
the navigator in another:

```bash
ros2 run cs5335_cone_navigation cone_navigator --ros-args --params-file src/cs5335_cone_navigation/config/cone_navigation.yaml
```

Parameters are read once when a node starts; after editing the YAML file, restart.

## Calibration

### Camera and LiDAR alignment

1. Check that TF gives sensible mountings. At start-up the navigator logs `Camera (...) on
   the robot` and `LiDAR on the robot` with x, y and the direction each faces. The camera
   should face about 0 deg; the LiDAR's direction is whatever its frame is mounted at.
2. Put one cone about 1 m straight ahead and run the navigator. In the target list, the
   cone's position should be about (1, 0) m relative to the robot.
3. Turn the robot by hand (or with a small turn) so the cone is near the left and then the
   right edge of the image. If the listed positions swing sideways, the camera bearing and
   LiDAR disagree. Set `camera_yaw_offset` (rad, + = left) to the angle between where the
   camera and the LiDAR put the cone.
4. If TF is not available, set `use_tf: false` and measure `camera_mount` and
   `lidar_mount` (`[x m, y m, yaw rad]` on the robot). Do not leave them at zero.

### Stand-off distance

1. Drive to a cone, then measure with a ruler the gap between the robot's bumper and the
   cone at the floor.
2. The log of COMPLETE gives the LiDAR's gap. `cone_base_offset` = (LiDAR gap) - (ruler
   gap) + 0.0254 adjusts the stop to one inch at the floor. Repeat until the ruler reads
   about 2.5 cm.
3. Start with a larger `cone_base_offset` (the default 0.05 m errs on the far side).

## Testing

**Without the robot**: the modules without ROS have unit tests on synthetic images and
points. They cover two red cones in one image, rejected yellow and orange cones, a red
rectangle and a blue disc, an orange cone with a reddish patch, a cone cut off by the
edge, same-color cones kept apart in the registry, LiDAR association with an offset
camera, and a simulated approach that stops at the gap. From this package's directory:

```bash
python3 -m pytest test/test_cone_detection.py test/test_fusion.py test/test_registry.py test/test_control.py
```

`colcon test --packages-select cs5335_cone_navigation` also runs `test_parameters.py`
(needs rclpy) and the copyright, flake8 and pep257 checks.

**On the robot** (milestones 1-9 of the project): check each part before the next.

1. Camera and `cv_bridge`: the debug image shows up.
2. Colors: every target cone is white in the mask; other colors are not.
3. Shapes: target cones are accepted; boxes, balls and other-colored cones are not.
4. LiDAR: each cone shows up in RViz.
5. Fusion: listed distances match a tape measure.
6. Registry: two cones of the same color get two IDs; one cone does not get several.
7. Exploration: one full turn, then a correct list.
8. Navigation: ALIGN, APPROACH and STOP for each target, in different orders.
9. Failures: cover the camera, block the path, or pick up the robot; it must stop.

## Parameters

### Cone Detector (`cone_detector`)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `image_topic` | `oakd/rgb/preview/image_raw` | Camera image topic |
| `target_colors` | `['red', 'blue', 'green']` | Colors to accept; known: red, orange, yellow, green, blue, purple |
| `hsv.<color>` | see the YAML file | HSV ranges, six numbers each (`h_lo s_lo v_lo h_hi s_hi v_hi`); red needs two |
| `open_kernel`, `close_kernel` | 3, 5 | Opening and closing kernel sizes (px) |
| `min_contour_area` | 300 | Smaller contours (px) are noise |
| `min_aspect`, `max_aspect` | 0.4, 2.5 | Allowed height / width |
| `min_taper` | 0.2 | Smallest 1 - top width / bottom width |
| `min_symmetry` | 0.6 | Smallest left-right symmetry (0-1) |
| `min_solidity` | 0.75 | Smallest contour area / convex hull area |
| `max_vertices` | 10 | Most corners of the approxPolyDP outline |
| `approx_epsilon` | 0.03 | approxPolyDP tolerance (fraction of the perimeter) |
| `min_color_fraction` | 0.6 | Smallest fraction of strongly colored pixels around a cone that are its color |
| `color_context` | 0.25 | How far beyond its box the color check looks (fraction of its size) |
| `edge_margin` | 2 | Reject boxes this close (px) to the image edge |
| `weights.color`, `.aspect`, `.taper`, `.symmetry`, `.geometry` | 0.25, 0.15, 0.3, 0.2, 0.1 | Weights of the scores in the confidence |
| `min_confidence` | 0.7 | Smallest confidence of an accepted cone |

### Cone Navigator (`cone_navigator`)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `detections_topic` | `cone_detections` | The detector's output |
| `camera_info_topic` | `oakd/rgb/preview/camera_info` | Calibration of the detector's camera (same image size) |
| `scan_topic`, `odom_topic`, `cmd_vel_topic`, `hazard_topic` | `scan`, `odom`, `cmd_vel`, `hazard_detection` | Topics |
| `cmd_vel_stamped` | true | Publish `TwistStamped` (true) or `Twist` (false) |
| `use_tf` | true | Sensor mountings from TF |
| `camera_mount`, `lidar_mount` | `[0.0, 0.0, 0.0]` | `[x m, y m, yaw rad]` if `use_tf` is false; measure them |
| `camera_yaw_offset` | 0.0 rad | Calibration added to every camera bearing |
| `hfov` | 1.204 rad | Field of view, only if camera_info has no calibration |
| `scan_speed` | 0.25 rad/s | Turn speed while exploring or searching |
| `association_distance` | 0.25 m | Same color and this close = the same cone |
| `association_angle` | 0.2 rad | The same, by direction, without LiDAR positions |
| `min_observations` | 3 | Sightings before a cone gets an ID |
| `target_gate` | 0.35 rad | ALIGN accepts detections only this close to where the target should be |
| `bearing_margin` | 0.05 rad | Widens the camera's view of a cone when matching LiDAR points |
| `cluster_depth` | 0.15 m | LiDAR points this much behind the nearest are the cone too |
| `max_cone_range` | 3.0 m | Farther LiDAR returns are not matched |
| `max_linear_velocity`, `min_linear_velocity` | 0.15, 0.02 m/s | Forward speed limits |
| `max_angular_velocity`, `min_angular_velocity` | 0.3, 0.08 rad/s | Turn speed limits |
| `kp_angular` | 1.0 1/s | ALIGN: angular velocity = Kp × bearing error |
| `kd_linear` | 0.5 1/s | APPROACH: linear velocity = Kd × distance error |
| `angular_tolerance` | 0.05 rad | ALIGN is done within this error |
| `heading_limit` | 0.35 rad | APPROACH turns in place if the cone is farther off |
| `robot_radius` | 0.171 m | Radius of the robot |
| `stop_gap` | 0.0254 m | Gap to leave (one inch) |
| `cone_base_offset` | 0.05 m | Calibration: how much farther the cone reaches at the floor than where the LiDAR hits it |
| `track_gate` | 0.2 m | LiDAR points this close to the cone's last position are the cone |
| `lost_time` | 1.0 s | The cone is lost after this long unseen |
| `obstacle_clearance` | 0.2 m | APPROACH stops if anything else is this close ahead |
| `max_reacquire` | 1 | Times to go back to ALIGN after losing the cone |
| `rate_hz` | 20.0 Hz | Control loop rate |
| `settle_time` | 0.5 s | Wait after stopping before a still scan |
| `data_timeout` | 0.5 s | Stop if odometry, scans or detections are older |
| `move_timeout` | 60.0 s | Give up on one motion after this |
| `data_wait` | 30.0 s | Give up if the data has not all arrived by then |

## Safety

The robot stops (zero velocity, sent three times) at the end of every motion, on Ctrl-C or
a closed terminal, and on any of these:

| Event | What happens |
| --- | --- |
| Bump, cliff, wheel stall or wheel drop since the motion started | Stop, back to SELECT_TARGET |
| Odometry, scans or detections older than `data_timeout` | Stop, back to SELECT_TARGET |
| Anything but the cone within `obstacle_clearance` ahead | Stop, back to SELECT_TARGET |
| The cone missing from the newest scan | Wait; after `lost_time`, back to ALIGN once |
| The target's distance unavailable | No approach |
| A motion takes longer than `move_timeout` | Stop, back to SELECT_TARGET |
| An invalid target ID | Asked again |

Speeds stay within `max_linear_velocity` and `max_angular_velocity`, and a velocity that
is not a number is replaced by zero. The Create 3's own protections stay on. If no
`hazard_detection` message has arrived at the start, the navigator warns that it may not
see bumps or cliffs.

## Known limitations and assumptions

- **Not yet run on the robot or built with colcon.** Every threshold and calibration value
  is a starting point.
- **Short training cones may be invisible to the LiDAR.** If a cone is lower than the
  LiDAR's scan plane, its distance is unavailable and it cannot be approached (by design:
  no blind approach). Taller cones, or a camera-based distance estimate added later, would
  be needed.
- Cones that touch or overlap in the image form one contour and will usually fail the
  shape checks. Cones with white stripes may split into pieces that fail them too.
- Two cones of the same color closer together than `association_distance` merge into one.
- During exploration, the camera and LiDAR are put in the odom frame with the pose when
  each arrived; their transport delay causes a small bearing error while turning (slower
  `scan_speed` reduces it).
- Lighting changes the HSV values; re-tune `hsv.<color>` in a new room.
- The registry is in the odom frame, which drifts slowly; ALIGN re-finds each target with
  the camera and LiDAR.
