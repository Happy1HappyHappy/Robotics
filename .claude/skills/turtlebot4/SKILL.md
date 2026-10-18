---
name: turtlebot4
description: >
  Course-specific guide for programming the TurtleBot 4 (iRobot Create 3 base) with ROS 2 Jazzy
  and rclpy in Northeastern CS 5335 Robotic Science and Systems, and for this repository's
  packages (robotics_ws, day2pkg, day3pkg, day3_interfaces). Use whenever the task involves the
  TurtleBot or Create 3: driving with cmd_vel / TwistStamped, the Drive action, Create 3 actions
  (drive_distance, rotate_angle, drive_arc, navigate_to_position, dock, undock), odometry and
  /odom, hazard or bump detection, LiDAR /scan, TF2 frames, differential-drive kinematics,
  dead reckoning, Kalman filters, occupancy grids, or adding a node, launch file, parameter or
  interface to this workspace. Also use when debugging why the robot does not move, topics are
  missing, or a class Project does not behave as expected.
---

# TurtleBot 4 for CS 5335

The course: CS 5335 Robotic Science and Systems, Northeastern (Seattle), Fall 2026. Robot:
TurtleBot 4, which is a Raspberry Pi 4 on top of an iRobot Create 3 base. Stack: Ubuntu 24.04 +
ROS 2 **Jazzy**, Python (`rclpy`). Do not suggest Ubuntu 26.04 / ROS 2 Lyrical, because the
TurtleBot 4 does not support them.

This skill follows the course slides, and the slides are the standard. If another skill (such as
`ros2` or `robotics-software-principles`) says something different, follow this one.

Read the matching reference file before writing code in that area:

| File | Covers |
| --- | --- |
| [references/ros2-python-patterns.md](references/ros2-python-patterns.md) | Course code patterns: node, pub/sub, timer, service, action (incl. cancel), interfaces, parameters, launch, TF2, multithreading |
| [references/turtlebot4-interfaces.md](references/turtlebot4-interfaces.md) | Topics and rates, message fields, Create 3 actions, hazards and safety, frames, LiDAR, robot dimensions |
| [references/robotics-math.md](references/robotics-math.md) | Poses and transforms, differential drive, odometry closed loop, error models, Kalman filter, occupancy grids |

## Course AI policy (Day 1 slides)

Students may **not** use AI to complete assignments. AI is allowed as a debugging aid when
stuck, to gather information for class presentations, and freely on the group final project.

- **Weekly Projects**: act as a debugging aid and tutor. Explain the concept, point to the
  pattern in the references, find the bug and explain the minimal fix. Do not write the
  assignment's solution code for the student.
- **Final project**: implement freely, following this skill.
- If it is unclear which one the request is for, ask.

## Environment

Every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8                 # must match the robot
source ~/Robotics/robotics_ws/install/setup.bash   # after the first build
```

- The laptop must be on the same Wi-Fi as the robot and connected to the robot's Discovery
  Server (Computer Setup document, step 9).
- `ros2 topic echo /ip` shows whether the robot is reachable. When topics are missing, run
  `ros2 daemon stop; ros2 daemon start`.
- SSH: `ssh ubuntu@<robot-ip>`. Credentials are in the lab setup documents. Never write
  passwords, account names or IPs from the slides into code, docs or commits.
- This setup has no robot namespace, so topics are `/cmd_vel`, `/odom`, and so on. Inside
  nodes, use relative names (`'cmd_vel'`, `'odom'`) so a namespace can be added later.

## Rules for code that moves the robot

1. On Jazzy, `/cmd_vel` takes `geometry_msgs/msg/TwistStamped`, **not** `Twist`. Create 3 docs
   and older tutorials show `Twist`, which is wrong for this robot. Set
   `msg.header.stamp = self.get_clock().now().to_msg()` and fill only `twist.linear.x` and
   `twist.angular.z`. A differential drive cannot move sideways.
2. Directions follow REP 103: `+x` is forward and `+angular.z` is a left (CCW) turn. In the
   repo's `Drive` action, `turn_right=True` / a negative angle means CW.
3. Limits: linear speed ≤ 0.306 m/s (safe mode) and angular speed ≤ 1.9 rad/s. Clamp commands
   before publishing. On an arc ω = v / c, so lower v on tight radii. Never change the Create 3
   `safety_override` parameter.
4. Publish velocity commands continuously (10–20 Hz) while moving. Always send an explicit zero
   command when done, when canceled, and in `main()`'s `finally` block.
5. The Create 3 ignores `cmd_vel` while one of its actions (drive_distance, dock, …) is running,
   and it should not be driven while sitting on the dock (undock first).
6. ROS messages and Create 3 actions use **radians** and meters. Convert degrees only at the
   user-facing edge (`math.radians`), and wrap angle differences to [-π, π).
7. `float64` / `float32` message fields reject Python `int`, so wrap values with `float()`.
8. Under the default single-threaded executor, a long-running callback blocks every other
   callback. An action server that must accept a cancel or read `/odom` while executing needs a
   `MultiThreadedExecutor` and a `ReentrantCallbackGroup` for the action (see
   `robotics_ws/src/day3pkg/day3pkg/driver.py`).
9. Odometry drifts (dead reckoning). Compare poses relative to a start snapshot, not as
   absolute positions, and report errors in the robot's start frame.

## Architecture

The course uses a three-tier architecture (Day 2): **Planning** (what task, e.g. "go to room
216"), **Executive** (sequence steps, turn behaviors on and off), **Behavior/Control** (sensing
tied to motion, e.g. "drive 1 m"). In this repo, `executive.py` is the Executive and
`driver.py` is a Behavior. Put new code in the right tier, keep each node single-purpose, and
connect nodes with:

- **topic**: a continuous stream (sensor data, velocity commands)
- **service**: a quick request/response
- **action**: a long-running task with feedback, a result, and cancel

Driving interfaces, from low to high level. Use the highest level the assignment allows:

| Level | Interface | Notes |
| --- | --- | --- |
| Low | `/cmd_vel` (TwistStamped) | Your own control loop. Per the Day 5 slides, later projects move to the higher levels |
| Medium | `/drive_distance`, `/rotate_angle`, `/drive_arc` | Create 3 actions that close the loop on odometry |
| High | `/navigate_to_position`, `/dock`, `/undock`, `/wall_follow`, Nav2 | Goal fields are in the interfaces reference |

## Repo conventions

Workspace `robotics_ws/`, packages in `robotics_ws/src/`:

| Package | Project | Role |
| --- | --- | --- |
| `day2pkg` | 2 | Timed driving with `cmd_vel` |
| `day3pkg` | 3, 4, 5 | Executive + Driving nodes, `drive` action, odometry logging |
| `day3_interfaces` | 3, 4, 5 | `action/Drive.action` |

When adding or changing code, match these:

- **Packages**: Python nodes go in an `ament_python` package
  (`ros2 pkg create <pkg> --build-type ament_python --dependencies rclpy --license Apache-2.0`).
  Messages, services and actions go in a separate `ament_cmake` interfaces package, each file
  listed in `rosidl_generate_interfaces`. Interface names are UpperCamelCase.
- **File layout**: module docstring; imports grouped stdlib → `rclpy` → message packages → own
  interfaces; module-level helper functions; one `Node` subclass; `main()`;
  `if __name__ == '__main__': main()`.
- **Names**: the class is UpperCamelCase (`DrivingNode`) and the node name is snake_case
  (`driving_node`). Attributes that hold ROS handles end in `_`: `self.cmd_pub_`,
  `self.odom_sub_`, `self.timer_`, `self.client_`, `self.action_server_`.
- **Parameters**: every tunable (speeds, rates, paths) is a ROS parameter declared in
  `__init__` with a unit comment, e.g. `self.declare_parameter('linear_speed', 0.15)  # m/s`.
  Read it with `.value` and set defaults in the launch file. No magic numbers in logic.
- **Docstrings and comments**: triple-quoted docstrings on modules, classes, methods and helper
  functions that explain arguments and units. Use `# ---------- section ----------` separators
  in long classes, and comment the why (e.g. why a callback group is separate).
- **Logging**: `self.get_logger().info/warn/error` with units in the text, and
  `throttle_duration_sec` for repeated messages. Use `print()` only for the interactive CLI in
  the Executive.
- **`main()`**: `rclpy.init` → create node → spin (executor) → `except KeyboardInterrupt` →
  `finally`: stop the robot, `destroy_node()`, `rclpy.try_shutdown()`.
- **Packaging**: register executables in `setup.py` `console_scripts`; install `launch/` and
  `config/` through `data_files` with `glob`; add every message package used to
  `package.xml` as `<depend>`.
- **Lint**: the generated `test/` folders run `ament_flake8` (99-char lines, Google import
  order, single quotes) and `ament_pep257`. The existing code already fails some docstring
  (D205, D400) and import-order (I100) checks, so `colcon test` reports those failures. Keep
  new lines within 99 characters with single quotes. Don't reformat existing files unless asked,
  and don't claim the tests pass.
- **Docs**: each package has a README (what it does, build/run, commands, parameters). Update
  it, and the tables in the root and `robotics_ws` READMEs, when behavior or packages change.

## Build, run, verify

```bash
cd robotics_ws
colcon build --packages-select day3_interfaces day3pkg   # interfaces first when they change
source install/setup.bash
ros2 run day3pkg driver --ros-args -p linear_speed:=0.1
ros2 launch day3pkg day3_launch.py
colcon test --packages-select day3pkg && colcon test-result --verbose
```

Rebuild and re-source after **every** edit, including Python files and launch files.

Check with introspection instead of guessing:

- `ros2 node list` / `ros2 node info /driving_node`
- `ros2 topic list -t` / `ros2 topic echo /cmd_vel` / `ros2 topic hz /odom`
- `ros2 interface show day3_interfaces/action/Drive`
- `ros2 action list -t`
- `ros2 param dump /driving_node`

When reporting results, say exactly what was verified: built, linted, run without the robot,
or run on the robot. Do not claim robot behavior that was not observed.

## Before running on the robot

- Undock the robot, clear about 2 m of floor, and have someone ready to lift the robot or press
  Ctrl-C.
- Try new motion code first with short distances and low speeds.
- Anything that runs on its own should watch `/hazard_detection` and stop on BUMP or CLIFF.
- Dock the robot when finished so it charges.

## Official docs

- Create 3 ROS 2 API: <https://iroboteducation.github.io/create3_docs/api/ros2/>
- TurtleBot 4 user manual: <https://turtlebot.github.io/turtlebot4-user-manual/>
- ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/>
