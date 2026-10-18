---
name: robotics-constitution
description: >
  Ground rules for any ROS 2 or robotics work in this CS 5335 TurtleBot 4 repository: how to
  structure robot software, how to verify code against the installed system instead of trusting
  examples, when not to add complexity, and the safety and correctness checks for code that
  moves the robot or uses frames, units, time or QoS. Also the entry point to background
  knowledge on the TurtleBot 4 and robotics (kinematics, odometry, frames, sensors, Kalman
  filter, occupancy grids). Use when writing, reviewing, designing or debugging any node,
  launch file, interface or robot behavior here, and when explaining robotics concepts from
  the course.
---

# Robotics Constitution (CS 5335)

These are the few stable rules for robot code in this repository, kept short on purpose.
Concrete details (ROS distribution, topic names, message types, firmware behavior, lab network
settings) change between releases and semesters, so they are **not** rules here. Discover them
from the running system (Article II), and treat the reference files as background knowledge.

Order of authority: the user's explicit instruction → the course slides → this constitution →
reference files and examples.

## Article 0: Course policy

From the Day 1 slides: AI may not complete assignments. It may be used as a debugging aid, to
gather information for class presentations, and freely on the group final project.

- **Weekly projects**: explain, find the bug, propose the minimal fix, review. The student
  writes the solution.
- **Final project**: implement fully, following this constitution.
- If it is unclear which one a request is for, ask.

## Article I: Architecture

1. **Three tiers** (Day 2): Planning (what task), Executive (sequences steps, switches behaviors
   on and off), Behavior/Control (sensing tied to action). Put logic in its tier. Higher tiers
   sequence lower ones and never re-implement them.
2. **One node, one job.** If you can't describe a node without "and", split it.
3. **Nodes talk only through ROS interfaces**: topics for streams, services for quick
   request/response, actions for long-running tasks that need feedback or cancel. Never import
   another node's module to call it.
4. **Interfaces are contracts.** Document units, frames and sign conventions in the
   `.msg`/`.srv`/`.action` file. Keep custom interfaces in their own interfaces package.
5. **Configuration over constants.** Anything that may change (speeds, rates, tolerances, topic
   names, file paths) is a ROS parameter with a unit comment, with defaults set in a launch or
   YAML file.
6. **Callbacks never block.** A callback stores data or does brief work. Periodic work runs on
   timers. Slow work must never delay incoming data.
7. **Compose.** Build complex behaviors from small primitives that are already tested.
8. **Fit in.** Match the naming, layout, docstrings and idioms of the neighboring code.

## Article II: Verify against the real system

Examples, whether in this skill, the slides or on the web, are patterns, not facts. Plausible
code is not finished code.

1. **Establish the facts first**:
   - the ROS distribution: `echo $ROS_DISTRO`
   - the interfaces actually offered: `ros2 topic list -t`, `ros2 topic info -v <topic>`,
     `ros2 interface show <type>`, `ros2 action list -t`
   - the frames: `ros2 run tf2_tools view_frames`
   - the installed packages: `ros2 pkg list`
   - the lab setup: the repo's README and `related_cmds.txt`
2. **Use the API of that distribution.** When unsure about a function, field or package, check
   it (`ros2 interface show`, `python3 -c "import <pkg>"`, the docs for that distribution)
   instead of guessing.
3. **Build, run, test, fix, and repeat**: `colcon build --packages-select <pkgs>`,
   `source install/setup.bash`, run the node, and run `colcon test` where tests exist. Read the
   errors, fix them, and repeat until it builds and runs.
4. **Report exactly what was verified**: built, linted, ran without the robot, or ran on the
   robot. Name what still has to be checked on the robot and how. Never claim behavior that was
   not observed.

If a step can't be run here (no ROS, no robot), say so and give the user the command to run.

## Article III: Simplest correct design first

This is a practice course: code should be correct, readable and stable before it is clever.

1. Start single-threaded, with plain publishers, subscriptions and timers.
2. Add complexity only for a concrete need, and write the reason in a comment:
   - **multi-threaded executor + callback groups**: a long-running callback must not starve
     others (e.g. an action that must accept cancel or watch hazards while it executes)
   - **a custom interface**: no existing message fits
   - **a new node**: the work is a separate job or runs at a separate rate
3. Not used in this course unless the user asks: lifecycle nodes, components/composition, DDS or
   discovery tuning, C++.
4. Prefer the robot's built-in capabilities over re-implementing them, unless the assignment is
   to implement them.

## Article IV: Safety and correctness

Anything that moves the robot, or that uses poses, frames, time or sensor data, must pass these
checks before it is called done:

1. **Units**: SI inside code (m, s, rad, m/s, rad/s). Convert degrees only at the user-facing
   edge. Wrap angle differences to [-π, π).
2. **Frames**: every pose and vector has a named frame. Use REP 103 axes (x forward, y left,
   z up, positive yaw = counter-clockwise) and REP 105 frames (map → odom → base_link). Know
   which way each transform goes (^A T_B maps points from B into A).
3. **Time**: stamp outgoing messages with the node clock. Check how old the latest sensor data
   is before acting on it. Look transforms up at the data's timestamp.
4. **Limits**: clamp commands to the robot's limits (from its documentation or parameters), and
   default to slow.
5. **Stopping**: send an explicit stop when finished, when canceled, on error, and on shutdown
   (`finally`). Anything that runs by itself must stop on safety hazards (bump, cliff).
6. **Failure paths**: decide, and log, what happens if data never arrives, a topic disappears, a
   transform is unavailable, or a goal is rejected or aborted.
7. **QoS**: subscribers must be compatible with their publishers. A best-effort subscriber
   connects to either kind of publisher; a reliable one receives nothing from a best-effort
   publisher.
8. **First run**: short distance, low speed, robot off the dock, clear floor, someone ready to
   stop it.

## Knowledge (read when relevant)

| File | Contents |
| --- | --- |
| [references/turtlebot4.md](references/turtlebot4.md) | The robot: hardware, sensors, the three levels of control, odometry, safety features, frames, LiDAR data, and how to look up current details |
| [references/robotics-math.md](references/robotics-math.md) | Poses and transforms, differential drive, pose update, odometry closed loop, error models, Kalman filter / EKF, occupancy grids |
| [references/ros2-patterns.md](references/ros2-patterns.md) | Minimal skeletons: node, publisher/subscriber, timer, service, action (with cancel), interfaces, parameters, launch, logging, TF2, multithreading |

Some principles are adapted from
[arpitg1304/robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills)
(Apache-2.0).
