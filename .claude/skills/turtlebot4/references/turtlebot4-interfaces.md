# TurtleBot 4 / Create 3 interfaces

Sources: Day 1, 4, 5 and 6 slides, the Create 3 ROS 2 docs, `irobot_create_msgs`, and the
TurtleBot 4 user manual. Before relying on a field name, check it on the robot with
`ros2 interface show <type>`.

## Contents

- [Robot facts](#robot-facts)
- [Topics](#topics)
- [Odometry message](#odometry-message)
- [Create 3 actions](#create-3-actions)
- [Hazards and safety](#hazards-and-safety)
- [Coordinate frames](#coordinate-frames)
- [LiDAR](#lidar)
- [Camera](#camera)

## Robot facts

| Item | Value | Source |
| --- | --- | --- |
| Computers | Raspberry Pi 4 (Ubuntu + ROS 2), Create 3 controller | Day 1 |
| Wheel separation b | 0.233 m | Create 3 sim `control.yaml` |
| Wheel radius | 0.03575 m | Create 3 sim `control.yaml` |
| Encoder | 508.8 ticks per wheel revolution | Create 3 docs |
| Max linear speed | 0.306 m/s with safety on (0.46 m/s only with `safety_override full`, don't) | Create 3 docs |
| Max angular speed | 1.9 rad/s | TurtleBot 4 manual |
| Acceleration limit | 900 mm/s² (wheel ramping) | Create 3 docs |
| Sensors | Stereo camera (OAK-D Pro), 2D LiDAR (RPLIDAR A1), IMU, optical floor tracker (mouse), wheel encoders, IR proximity, cliff, bump, wheel drop, slip | Day 1, Day 5 |

## Topics

| Topic | Type | Rate | Notes |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | you publish 10–20 Hz | Only `linear.x`, `angular.z`. The robot runs the last command until a short timeout |
| `/odom` | `nav_msgs/msg/Odometry` | 20 Hz | Fused dead-reckoning pose and velocity |
| `/imu` | `sensor_msgs/msg/Imu` | 100 Hz | |
| `/mouse` | `irobot_create_msgs/msg/Mouse` | 62.5 Hz | Optical floor tracker, not affected by wheel slip |
| `/wheel_ticks` | `irobot_create_msgs/msg/WheelTicks` | 62.5 Hz | 508.8 ticks/rev |
| `/wheel_vels` | `irobot_create_msgs/msg/WheelVels` | 62.5 Hz | `velocity_left/right` in **rad/s**. Multiply by the wheel radius to get m/s |
| `/wheel_status` | `irobot_create_msgs/msg/WheelStatus` | 62.5 Hz | Wheels enabled, PWM, current |
| `/slip_status` | `irobot_create_msgs/msg/SlipStatus` | 20 Hz | `is_slipping` |
| `/stop_status` | `irobot_create_msgs/msg/StopStatus` | 1 Hz per Create 3 docs (slides say 100 Hz) | `is_stopped` |
| `/hazard_detection` | `irobot_create_msgs/msg/HazardDetectionVector` | periodic | See [Hazards and safety](#hazards-and-safety) |
| `/kidnap_status` | `irobot_create_msgs/msg/KidnapStatus` | | `is_kidnapped` (lifted and moved) |
| `/ir_intensity` | `irobot_create_msgs/msg/IrIntensityVector` | | Proximity IR readings |
| `/cliff_intensity` | `irobot_create_msgs/msg/IrIntensityVector` | | |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | | |
| `/dock_status` | `irobot_create_msgs/msg/DockStatus` | | `dock_visible`, `is_docked` |
| `/scan` | `sensor_msgs/msg/LaserScan` | ~5 Hz | See [LiDAR](#lidar) |
| `/oakd/...` | images, depth | | List with `ros2 topic list \| grep oakd` |
| `/ip` | | | Robot's IP. Handy as a "can I see the robot?" check |

Subscribe to sensor topics with `qos_profile_sensor_data`. If a topic shows up in
`ros2 topic list` but `echo` prints nothing, check QoS with `ros2 topic info -v <topic>`.

## Odometry message

`nav_msgs/msg/Odometry`:

```
header.frame_id         'odom'        (the parent frame)
child_frame_id          'base_link'
pose.pose.position      x, y, z       meters, in the odom frame
pose.pose.orientation   quaternion    -> yaw with yaw_from_quaternion()
pose.covariance         float64[36]   row-major 6x6 over [x, y, z, rot_x, rot_y, rot_z]
twist.twist.linear      x, y, z       m/s, in the child (robot) frame
twist.twist.angular     x, y, z       rad/s
twist.covariance        float64[36]
```

- The origin is wherever odometry was last reset (e.g. at start-up). It is not a fixed world
  point, so work with differences: snapshot the pose at the start, then compute the motion in
  the start frame (see `report_motion()` in `driver.py` and the math reference).
- For 2D motion use x, y and yaw. z, roll and pitch are about 0.
- Read one message by hand with `ros2 topic echo /odom --once`.

## Create 3 actions

These close the loop on odometry. They stop on a cliff or wheel stall, but **not** on a bump:
if something blocks the path, the robot keeps bumping until odometry slip says it arrived, so
stay ready to intervene. While a goal runs, `cmd_vel` is ignored. All angles are in
**radians**.

| Action | Type | Goal fields | Result / feedback |
| --- | --- | --- | --- |
| `/drive_distance` | `irobot_create_msgs/action/DriveDistance` | `distance` (m, negative = backward), `max_translation_speed` (m/s, default 0.3) | `pose` / `remaining_travel_distance` |
| `/rotate_angle` | `irobot_create_msgs/action/RotateAngle` | `angle` (rad, + = CCW), `max_rotation_speed` (rad/s, default 1.9) | `pose` / `remaining_angle_travel` |
| `/drive_arc` | `irobot_create_msgs/action/DriveArc` | `translate_direction` (1 forward, -1 backward), `angle` (rad), `radius` (m), `max_translation_speed` | `pose` / `remaining_angle_travel` |
| `/navigate_to_position` | `irobot_create_msgs/action/NavigateToPosition` | `goal_pose` (PoseStamped, odom frame), `achieve_goal_heading`, `max_translation_speed`, `max_rotation_speed` | `pose` / `navigate_state`, remaining distance/angle |
| `/dock` | `irobot_create_msgs/action/Dock` | (empty) | `is_docked` / `sees_dock` |
| `/undock` | `irobot_create_msgs/action/Undock` | (empty) | `is_docked` |
| `/wall_follow` | `irobot_create_msgs/action/WallFollow` | `follow_side` (1 left, -1 right), `max_runtime` (Duration) | `runtime` / `engaged` |

From the command line:

```bash
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5, max_translation_speed: 0.15}"
ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 1.57, max_rotation_speed: 0.5}"
ros2 action send_goal /drive_arc irobot_create_msgs/action/DriveArc "{angle: 1.57, radius: 0.3, translate_direction: 1, max_translation_speed: 0.3}"
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
```

From Python, use an `ActionClient` with these types (see the patterns reference) and add
`<depend>irobot_create_msgs</depend>` to `package.xml`. The slides' "Navigate To Pose" is the
Create 3's `/navigate_to_position` (`NavigateToPosition`). `NavigateToPose` is a different
action, from Nav2. `navigate_to_position` rotates, drives
straight, then optionally rotates. It is not a path planner, so use Nav2 for obstacle-aware
navigation.

## Hazards and safety

`/hazard_detection` publishes a list of the hazards detected right now. An empty list means
no hazards. Each element has a `header` (whose `frame_id` names the sensor, e.g.
`cliff_front_left`, `left_wheel`) and a `type`:

| Constant | Value | Meaning |
| --- | --- | --- |
| `BACKUP_LIMIT` | 0 | Will not back up further; front cliff sensors can't see behind |
| `BUMP` | 1 | Bumper contact |
| `CLIFF` | 2 | Cliff detected |
| `STALL` | 3 | Wheels stalled against something |
| `WHEEL_DROP` | 4 | Wheel dropped (lifted) |
| `OBJECT_PROXIMITY` | 5 | Obstacle very close (IR) |

```python
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector

def hazard_callback(self, msg: HazardDetectionVector):
    """
    Stop on any bump or cliff
    """
    if any(d.type in (HazardDetection.BUMP, HazardDetection.CLIFF) for d in msg.detections):
        self.publish_cmd(0.0, 0.0)
```

Under the default single-threaded executor, this callback cannot run while another callback
is busy. A behavior that loops (an action's execute callback) needs the multithreading setup
from the patterns reference.

- Backup limit: after the robot is lifted ("kidnapped"), or before it has driven forward, it
  refuses to reverse much. Drive forward a little first.
- E-stop: `ros2 service call /e_stop irobot_create_msgs/srv/EStop "{e_stop_on: true}"` cuts
  wheel power. Send `false` to re-enable.
- Never set `/motion_control safety_override`. It removes the cliff protection and speed cap.

## Coordinate frames

Standard ROS 2 frames (REP 105): `map` (your map) → `odom` (motion since start, continuous
but drifts) → `base_link` (robot center). `world` is optional.

The TurtleBot frames named in class (Day 4), all static and defined by the URDF:

| Frame | Meaning |
| --- | --- |
| `base_link` | Robot center |
| `base_footprint` | Point on the floor under `base_link` |
| `imu_link` | IMU |
| `lidar_link` | LiDAR (check the real name: `ros2 topic echo /scan --once \| grep frame_id`) |
| `camera_link` | Camera |
| `wheel_left_link`, `wheel_right_link` | Wheel centers |

Confirm the real frame names with `ros2 run tf2_tools view_frames` before hard-coding them.
Axes: x forward, y left, z up, and yaw is measured CCW from +x.

## LiDAR

`/scan` (`sensor_msgs/msg/LaserScan`), from the RPLIDAR A1. The class quoted about 5 Hz,
roughly 1° steps and up to 12 m range.

```
header.stamp      time of the first ray;  header.frame_id = LiDAR frame
angle_min, angle_max, angle_increment   rad, CCW about +z, 0 = forward along +x of the LiDAR frame
range_min, range_max                    m
ranges[]          m; discard values outside [range_min, range_max] and inf/nan
intensities[]     device units (may be empty)
```

Ray i is at angle `angle_min + i * angle_increment`, giving the point
`(r cos θ, r sin θ)` in the LiDAR frame. To transform it into `odom` or `map`, use TF2 at the
scan's timestamp (see the math reference for occupancy grids).

## Camera

OAK-D Pro (Luxonis): RGB, stereo depth and on-device spatial detections, under `/oakd/...`.
Full resolution is too much data for the class laptops, so prefer preview or low-rate streams.
Later classes cover this. Check what is actually published with `ros2 topic list | grep oakd`.
