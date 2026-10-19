---
name: turtlebot4-ros2
description: >
  Guidance for ROS 2 and robotics work in this CS 5335 TurtleBot 4 repository: how to structure
  robot software, how to verify code against the installed ROS distribution and robot instead
  of trusting examples, how to develop and test when the robot is not connected, when not to
  add complexity, and the safety checks for code that moves the robot or uses frames, units,
  time or QoS. Includes background knowledge on the TurtleBot 4
  and robotics (kinematics, odometry, frames, sensors, Kalman filter, occupancy grids). Use when
  writing, reviewing, designing or debugging any node, launch file, interface or robot behavior
  here, and when explaining robotics concepts from the course. Also use before creating, naming
  or pushing a git branch in this repo.
---

# TurtleBot 4 ROS 2 Development

This skill covers how to build robot software for the CS 5335 TurtleBot 4: the architecture to
use, how to verify against the real system, how much complexity to allow, and the safety checks
to run. Concrete details (ROS distribution, topic names, message types, firmware behavior, lab
network settings) change between releases and semesters. Look them up on the running system;
the reference files hold background knowledge, not rules.

## When to use

- Writing or changing a ROS 2 node, launch file, parameter file or interface in this repo
- Reviewing or debugging robot code, or working out why the robot does not behave as expected
- Designing where new functionality should go
- Explaining robotics concepts from the course (kinematics, odometry, frames, filters, mapping)
- Creating, naming or pushing a git branch (see [Git branches](#git-branches))

## Step 1: Identify the context

Decide which context the request is in. It sets what takes priority:

| Context | Priority |
| --- | --- |
| **Course assignment** | Follow the course slides' architecture and constraints first. Use only what the task needs. Do not introduce techniques the course does not use (lifecycle nodes, components/composition, DDS or discovery tuning, C++, extra abstraction layers) unless the user asks for them. |
| **Personal project** | A fuller engineering design is fine. Explain why each added piece is needed. |
| **Running on the real robot** | Hardware safety, data validity and failure handling come first. Run the full [safety checklist](#safety-checklist). |
| **Learning a concept** | You may explain techniques beyond the course, but label them clearly as outside the course requirements. |

If the context is unclear, ask. In a course assignment the order of authority is: the user's
explicit instruction → the course slides → this skill → the reference files and examples.

## Step 2: Check the real system

Establish the facts instead of assuming them. Some checks need only the laptop. Others need the
robot to be connected.

**Works without the robot** (static: what is installed and built):

```bash
echo $ROS_DISTRO                      # ROS distribution
ros2 pkg list                         # installed packages
ros2 interface list                   # message/service/action types available
ros2 interface show <pkg>/msg/<Type>  # exact fields (if that package is installed)
ros2 pkg executables <pkg>            # what your packages provide
```

**Needs the robot connected** (dynamic: what is running right now):

```bash
ros2 topic list -t                    # topics actually offered, with types
ros2 topic info -v <topic>            # message type and QoS of each endpoint
timeout 5 ros2 topic echo <topic> --once
ros2 action list -t                   # available actions
ros2 run tf2_tools view_frames        # frame names and tree
```

Put a `timeout` in front of any command that waits for data (`topic echo`, `topic hz`,
`action send_goal`). With no publisher, it waits forever.

### If the robot is not connected

This is normal while developing, so don't stall on it.

- **Signs**: `ros2 topic list` shows only `/parameter_events` and `/rosout`, and `echo` prints
  nothing.
- An empty topic list means "not connected", not "this topic does not exist". Do not change code
  because a robot topic is missing.
- Ask whether the user is working offline. If they expect to be connected, point them to the
  connection steps in the README and `related_cmds.txt` (`ROS_DOMAIN_ID`, Discovery Server,
  `ros2 daemon stop; ros2 daemon start`).
- If `ros2 topic list` does not even show the topics of a node you just started on the laptop,
  the terminal is probably configured as a client of the robot's Discovery Server, and with no
  server reachable, local nodes can't find each other. Check with
  `printenv | grep -E 'ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT'`. For offline work, unset them in
  that terminal (see [references/ros2-patterns.md](references/ros2-patterns.md#testing-without-the-robot)).
- Continue with the static checks and the reference files. Mark every robot-specific fact you
  relied on (topic names, message types, QoS, frame names) as **not yet checked on the robot**.

If you cannot run commands here at all (no ROS installed), say so and give the user the
commands to run.

## Step 3: Design with the simplest correct architecture

- **Three tiers** (Day 2): Planning (what task), Executive (sequences steps, switches behaviors
  on and off), Behavior/Control (sensing tied to action). Put logic in its tier. Higher tiers
  sequence lower ones and never re-implement them.
- **One node, one job.** If you can't describe a node without "and", split it.
- **Nodes talk only through ROS interfaces**: topics for streams, services for quick
  request/response, actions for long-running tasks that need feedback or cancel. Never import
  another node's module to call it.
- **Interfaces are contracts.** Document units, frames and sign conventions in the
  `.msg`/`.srv`/`.action` file. Keep custom interfaces in their own interfaces package.
- **Configuration over constants.** Anything that may change (speeds, rates, tolerances, topic
  names, file paths) is a ROS parameter with a unit comment, with defaults set in a launch or
  YAML file.
- **Callbacks never block.** A callback stores data or does brief work. Periodic work runs on
  timers.
- **Compose** complex behaviors from small primitives that are already tested.
- **Fit in**: match the naming, layout, docstrings and idioms of the neighboring code.

Start single-threaded, with plain publishers, subscriptions and timers. Add complexity only for
a concrete need, and write the reason in a comment:

| Addition | Justified when |
| --- | --- |
| Multi-threaded executor + callback groups | A long-running callback must not starve others, e.g. an action that must accept cancel or watch hazards while it executes |
| Custom interface | No existing message fits |
| New node | The work is a separate job or runs at a separate rate |

Prefer the robot's built-in capabilities over re-implementing them, unless the assignment is to
implement them.

## Step 4: Implement from reference patterns, carefully

Before using a reference pattern, verify that its API, message types, topic names, QoS
settings, and frame conventions match the installed ROS distribution and robot. Do not claim
code is verified unless it has actually been built or tested.

When unsure about a function, field or package, check it (`ros2 interface show`,
`python3 -c "import <pkg>"`, the docs for that distribution) instead of guessing.

## Step 5: Build, run, test, fix

```bash
cd robotics_ws
colcon build --packages-select <pkgs>
source install/setup.bash
ros2 run <pkg> <executable>           # or ros2 launch <pkg> <launch file>
colcon test --packages-select <pkgs> && colcon test-result --verbose
```

Read the errors, fix them, and repeat until the code builds and runs. Do not stop at code that
only looks plausible.

For how the workspace is laid out and built (`src/`, `rosdep`, `colcon build`, sourcing the
underlay and overlay), follow the official ROS 2 Jazzy documentation in
[Official documentation](#official-documentation).

### Testing without the robot

Your nodes start without the robot; they just receive no robot data. They must start cleanly,
log that they are waiting (throttled), and neither move nor crash until data arrives. Then test
them with one of these (commands in
[references/ros2-patterns.md](references/ros2-patterns.md#testing-without-the-robot)):

| Method | Good for |
| --- | --- |
| Run the node and watch its outputs (`topic echo`, `node info`) | Does it publish the right commands, at the right rate, in the right type? |
| Feed fake inputs (`ros2 topic pub`, `ros2 action send_goal` to your own server) | Logic and edge cases: goal validation, cancel, stop paths |
| Replay a recording (`ros2 bag record` once on the robot, `ros2 bag play` later) | Sensor processing on real data: odometry, LiDAR, filters, mapping |
| Simulation (TurtleBot 4 Gazebo simulator, if installed) | Closed-loop motion without the robot. Close to the real robot, but not identical |
| Plain Python tests of pure functions | Kinematics, transforms, Kalman filter, grid updates |

Passing offline tests does not prove the code behaves correctly on the robot.

## Safety checklist

Anything that moves the robot, or uses poses, frames, time or sensor data, must pass every item
before it is called done:

- [ ] **Units**: SI inside code (m, s, rad, m/s, rad/s). Convert degrees only at the user-facing
  edge. Wrap angle differences to [-π, π).
- [ ] **Frames**: every pose and vector has a named frame. Use REP 103 axes (x forward, y left,
  z up, positive yaw = counter-clockwise) and REP 105 frames (map → odom → base_link). Know
  which way each transform goes (^A T_B maps points from B into A).
- [ ] **Time**: stamp outgoing messages with the node clock. Check how old sensor data is before
  acting on it. Look transforms up at the data's timestamp.
- [ ] **Limits**: clamp commands to the robot's limits (from its documentation or parameters),
  and default to slow.
- [ ] **Stop on every exit path**, not just the normal one:
  - normal completion
  - cancel
  - exception: wrap motion loops in `try` / `finally`, with the stop in `finally`
  - timeout: data goes stale, a goal runs too long, or no new command arrives. Use a watchdog
    timer
  - node shutdown: Ctrl-C, `destroy_node`, launch shutdown
  - hazards: anything that runs by itself stops on bump or cliff
- [ ] **Failure paths**: decide and log what happens if data never arrives, a topic disappears, a
  transform is unavailable, or a goal is rejected or aborted.
- [ ] **QoS**: subscribers are compatible with their publishers. A best-effort subscriber
  connects to either kind of publisher; a reliable one receives nothing from a best-effort
  publisher.
- [ ] **Code is not the only safety layer.** Keep the robot's built-in protections enabled
  (cliff detection, backup limit, speed cap, command timeout, e-stop) and never disable them
  from code. Test in a controlled environment: short distance, low speed, robot off the dock,
  clear floor, and someone ready to lift the robot or trigger the e-stop.

## Step 6: Report what was verified

State exactly what was done: built, linted, tested offline (and how), or run on the robot. If
the robot was not connected, list the robot-specific facts and behaviors that still have to be
checked on it, each with the command to check it. Never claim behavior that was not observed.

## Git branches

- If the user has not told you which branch to use, **ask for the branch name** before the
  first commit or push. Do not make one up.
- A name the environment assigned on its own (for example a cloud session's
  `claude/<random-words>` default) does not count as the user's choice. Ask anyway.
- Use the name exactly as the user gives it (for example `project5`).

## Reference files

| File | Read when |
| --- | --- |
| [references/turtlebot4.md](references/turtlebot4.md) | You need to know what the robot is: hardware, sensors, the three levels of control, odometry, safety features, frames, LiDAR data, how to look up current details |
| [references/robotics-math.md](references/robotics-math.md) | The task involves transforms, differential drive, pose update, odometry closed loop, error models, Kalman filter / EKF, or occupancy grids |
| [references/ros2-patterns.md](references/ros2-patterns.md) | You need a skeleton: node, publisher/subscriber, timer, service, action (with cancel), safe stopping, interfaces, parameters, launch, logging, TF2, multithreading |

## Official documentation

| Document | Read when |
| --- | --- |
| [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/index.html) | Any ROS 2 question: creating or building the workspace (folder layout, `rosdep install`, `colcon build`, sourcing the underlay and overlay), packages, topics, services, actions, parameters, launch, tf2. Use it before memory or examples from other distributions |

Some principles are adapted from
[arpitg1304/robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills)
(Apache-2.0).
