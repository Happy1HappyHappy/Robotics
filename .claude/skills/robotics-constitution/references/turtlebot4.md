# TurtleBot 4: background knowledge

This file describes the robot as used in CS 5335 (Fall 2026, ROS 2 Jazzy). Sources: the Day 1,
4, 5 and 6 slides, the Create 3 docs, and the TurtleBot 4 manual. Names, message types, rates and
firmware behavior can change between releases, so confirm them on the robot before relying on
them (see [Looking things up](#looking-things-up)).

## What the robot is

- **iRobot Create 3 base**: the motors and wheels plus the low-level sensors (wheel encoders,
  IMU, optical floor sensor, bump, cliff, IR proximity, wheel drop). Its own controller runs the
  low-level ROS 2 interfaces.
- **Raspberry Pi 4** on top: Ubuntu + ROS 2 and the TurtleBot software. The 2D LiDAR and the
  stereo camera (OAK-D) are attached to it.
- **Your laptop** runs your nodes and talks to the robot over Wi-Fi through ROS 2 (DDS). The lab
  connection settings are in the repo's README and `related_cmds.txt`.
- **Differential drive**: two driven wheels. The robot can go forward or back and turn, but it
  cannot move sideways.

Specs (from the Create 3 / TurtleBot 4 docs; check them for your firmware):

| Quantity | Approx. value |
| --- | --- |
| Wheel separation b | 0.233 m |
| Wheel radius | 0.036 m |
| Max linear speed | 0.31 m/s with safety features on |
| Max angular speed | 1.9 rad/s |

## Three levels of control

| Level | What it is | Use it when | Things to know |
| --- | --- | --- | --- |
| **Velocity** | Publish linear-x / angular-z velocity commands on `cmd_vel`, continuously | You implement your own control loop | The robot keeps the last command for a short timeout, so always send an explicit stop. The message type has changed between ROS releases (`Twist` → `TwistStamped` on Jazzy); check it with `ros2 topic info -v /cmd_vel` |
| **Drive goals** | Create 3 actions: drive a distance, rotate an angle, drive an arc, go to an odometry position | Simple patterns, closed-loop on odometry | Velocity commands are ignored while a goal runs. Goals stop on a cliff or wheel stall, but not on a bump |
| **Behaviors** | Dock / undock, wall follow, Nav2 navigation | Autonomous tasks | Undock before driving |

The Day 5 slides say projects move from velocity commands to the drive goals from Project 5 on.
The slides' "Navigate To Pose" is the Create 3's navigate-to-position goal, which is not the
same as Nav2's `NavigateToPose`.

## Sensors

Classification from Day 5: proprioceptive (about the robot itself) or exteroceptive (about the
world); hard (raw) or soft (fused in software); passive or active; event-driven or sampling.

| Sensor | Kind | Typical use |
| --- | --- | --- |
| Wheel encoders / wheel velocities | proprioceptive, hard | Your own odometry, calibration |
| IMU (gyro + accelerometer) | proprioceptive, hard | Turn rate, tilt |
| Optical floor sensor | proprioceptive, hard | Motion that is not fooled by wheel slip |
| Odometry | proprioceptive, soft | Dead-reckoning pose and velocity with covariance. Drifts |
| Hazard detection | soft | Bump, cliff, wheel stall, wheel drop, close object, backup limit |
| LiDAR | exteroceptive, active | Ranges all around → obstacles, occupancy grids |
| Stereo camera | exteroceptive | Images, depth, object detections |
| Battery, dock status, kidnap (lifted) | status | Housekeeping and safety |

## Odometry

- Odometry is a fused dead-reckoning estimate. Its origin is wherever it was last reset, and its
  error grows with distance traveled.
- Work with differences from a start snapshot, expressed in the start frame (see
  `report_motion()` in `driver.py` and `robotics-math.md`).
- The message carries a pose (position + quaternion orientation, in the odom frame) and a twist
  (velocities in the robot frame), each with a 6×6 covariance.

## Safety features

- **Backup limit**: there are no cliff sensors at the back, so the robot refuses to reverse far
  until it has driven forward.
- **Speed cap**, plus an **e-stop** service that cuts wheel power.
- **Built-in goals stop on cliffs and stalls** but keep pushing against obstacles. Your own code
  must stop on bump and cliff hazards.
- Never disable the Create 3 safety overrides.

## Frames

- REP 105 chain: `map` → `odom` → `base_link`. `base_footprint` is the point on the floor under
  `base_link`.
- The sensor frames (IMU, LiDAR, camera, wheels) come from the robot's URDF and are static.
  Confirm their real names with `ros2 run tf2_tools view_frames` before hard-coding them.
- Axes: x forward, y left, z up. Yaw is counter-clockwise from +x.

## LiDAR data

A scan gives ranges at angles starting from `angle_min` in steps of `angle_increment`,
counter-clockwise, with 0 = forward along the LiDAR frame's x axis. Discard ranges outside
`[range_min, range_max]`, as well as inf and nan. Each range r at angle θ is the point
(r cos θ, r sin θ) in the LiDAR frame. Transform it to the world frame with TF2 at the scan's
timestamp (see `robotics-math.md` for occupancy grids).

## Looking things up

```bash
echo $ROS_DISTRO
ros2 topic list -t                    # every topic with its type
ros2 topic info -v /cmd_vel           # type + QoS of publishers and subscribers
ros2 topic echo /odom --once          # one real message
ros2 interface show <pkg>/msg/<Type>  # exact fields
ros2 action list -t                   # drive goals, dock, ...
ros2 run tf2_tools view_frames        # frame tree (PDF)
ros2 param list /motion_control       # robot parameters such as speed limits
```

Docs:

- Create 3 ROS 2 API: <https://iroboteducation.github.io/create3_docs/api/ros2/>
- TurtleBot 4 user manual: <https://turtlebot.github.io/turtlebot4-user-manual/>
- ROS 2 docs for the distribution in use: <https://docs.ros.org/>
