---
name: robotics-software-principles
description: >
  Software design principles for robot code in the CS 5335 ROS 2 / TurtleBot 4 workspace:
  single-purpose nodes, the three-tier architecture, depending on message and action
  interfaces, separating rates with timers and callback groups, fail-safe defaults, ROS
  parameters instead of hard-coded values, logging, and composing behaviors from simple
  commands. Use when designing or structuring robot nodes, making architecture decisions,
  reviewing or refactoring robot code, or deciding where new functionality should go. The
  turtlebot4 skill (based on the course slides) takes precedence where the two differ.
---

# Robotics Software Design Principles

> Adapted from [arpitg1304/robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills)
> (commit f9bc546, Apache-2.0) for CS 5335. **Modified**: replaced the robot-arm, abstract-base-class
> and plugin examples with ROS 2 / TurtleBot examples. Removed advice that conflicts with the
> course slides: raw threads and hand-written rate loops (use timers, callback groups and
> executors), hand-loaded YAML config (use ROS parameters), `structlog` (use the ROS logger),
> and the "idempotent absolute moves" principle (the course's `Drive` action and the Create 3
> drive actions are relative by design). Follow the `turtlebot4` skill wherever the two differ.

## Why Robotics Software Is Different

1. **Physical consequences**: a bug doesn't just crash a process, it drives a robot into a wall
   or off a ledge.
2. **Timing**: sensors arrive at fixed rates (odometry 20 Hz, IMU 100 Hz, LiDAR about 5 Hz) and
   velocity commands must keep flowing while the robot moves.
3. **Sensor uncertainty**: every measurement is noisy, and odometry drifts (dead reckoning).
4. **Many programs at once**: a robot is a "flock" of nodes passing messages (Day 1).
5. **Hard to debug**: you often only find out what happened from the logs.

The course structures robot software with the **three-tier architecture** (Day 2):

```
Planning    "Go into room 216"                 task sequences
Executive   "Go to the second intersection"    turns behaviors on and off
Behavior    "Move forward", "Turn left"        sensing tied to action
```

---

## Principle 1: Single Responsibility — One Node, One Job

Each node has one job, and nodes cooperate through topics, services and actions. In this repo,
`executive.py` turns user commands into goals (Executive tier) and `driver.py` turns goals into
`cmd_vel` while reading `/odom` (Behavior tier).

**Test**: can you describe the node without using "and"? If not, split it or move the extra
work to the tier where it belongs.

---

## Principle 2: Depend on Interfaces, Not Implementations

In ROS 2, the interface between modules is the **message, service or action type**, not another
node's Python code. The Executive depends only on `day3_interfaces/action/Drive`. The Driving
Node depends only on `geometry_msgs/TwistStamped` on `cmd_vel` and `nav_msgs/Odometry` on
`odom`. As a result:

- the Executive works with any node that serves `drive`
- the Driving Node works with the real Create 3 or a Gazebo simulation, because both use the
  same topics
- never import one node's module into another to call its methods. Send a message instead.

---

## Principle 3: Open-Closed — Extend Without Rewriting

Add capabilities by adding code, not by rewriting working loops. In `executive.py`, new shapes
are new entries in the `SHAPES` table, built from the existing `m` / `t` / `a` commands, so the
command loop and the action client stay unchanged. New sensors or behaviors become new
subscriptions, nodes or actions.

---

## Principle 4: Interface Segregation — Small, Focused Interfaces

Keep each message, service or action about one thing, and only subscribe to what a node
actually uses. Document units and sign conventions in the interface file itself (as
`Drive.action` does), so every user of it reads the same contract.

---

## Principle 5: Substitution — Same Interface, Same Behavior

Anything that implements an interface must honor its documented contract: units (meters,
radians), sign conventions (+ angular.z = left/CCW), and what the result and status mean. A
replacement driver, or the simulator, must behave the same for the same goal. Otherwise the
Executive silently does the wrong thing.

---

## Principle 6: Separation of Rates — Never Block a Callback

Different parts of the system run at different rates:

```
Source / consumer        Rate
────────────────────────────────────────
/imu                     100 Hz
/odom                    20 Hz
cmd_vel you publish      10–20 Hz
/scan (LiDAR)            ~5 Hz
Executive / user input   when a command arrives
```

Use the ROS mechanisms from the Day 2–3 slides, not hand-made threads or `while` loops with
sleeps inside callbacks:

- **Timers** (`create_timer`) for periodic work.
- **Callbacks that only store** the latest message (e.g. `self.latest_odom_ = msg`), so slow
  work never delays incoming data. A topic is the buffer between a fast producer and a slow
  consumer.
- **Callback groups + `MultiThreadedExecutor`** when one callback must run for a long time,
  e.g. an action's `execute_callback`. Put it in a `ReentrantCallbackGroup` so odometry,
  hazard and cancel callbacks keep running (see `driver.py`).
- Shared data written by more than one thread: protect it with `with self.lock_:`
  (`threading.Lock`), and keep the critical section small.

---

## Principle 7: Fail-Safe Defaults — Safe Until Proven Otherwise

Every node should default to the safest behavior:

- Low default speeds, declared as parameters (e.g. `linear_speed` 0.15 m/s) and clamped to the
  robot's limits (0.306 m/s, 1.9 rad/s).
- Validate requests up front: reject a bad goal in `goal_callback` instead of failing halfway.
- Always command zero velocity when finished, on cancel, on error, and in `main()`'s `finally`.
- Stop on hazards (`/hazard_detection` BUMP or CLIFF) in any behavior that runs on its own.
- If an expected input is missing (no odometry yet, no robot subscribed to `cmd_vel`), log a
  warning and wait or degrade. Never drive blindly on stale data.

```python
def goal_callback(self, goal: Drive.Goal):
    """
    Reject unsafe or meaningless goals before any motion starts
    """
    if goal.distance < 0.0 or goal.radius < 0.0:
        self.get_logger().warn(f'Reject: distance={goal.distance} m, radius={goal.radius} m')
        return GoalResponse.REJECT
    return GoalResponse.ACCEPT
```

---

## Principle 8: Configuration Over Code — Use ROS Parameters

Anything that may change between runs, robots or experiments is a **ROS parameter** (Day 3),
not a constant buried in the logic:

```python
# in __init__: declare with a default and a unit comment
self.declare_parameter('linear_speed', 0.15)    # m/s
self.declare_parameter('rate_hz', 10.0)         # Hz

# where it is used
v = self.get_parameter('linear_speed').value
```

Set values without rebuilding, either from the command line
(`--ros-args -p linear_speed:=0.1`), from a YAML file in `<pkg>/config/`, or in the launch
file:

```yaml
/driving_node:
  ros__parameters:
    linear_speed: 0.15
    rate_hz: 10.0
```

**What goes in parameters**: speeds, rates, tolerances, file paths, topic choices. **What stays
in code**: algorithms, control logic, message definitions.

---

## Principle 9: Observe Everything — Log What Matters

When the robot does something unexpected, the logs are often all you have. Use the ROS logger
and its levels (Day 2), not `print()` (except an interactive CLI prompt) and not extra logging
libraries:

| Level | Use |
| --- | --- |
| `fatal` / `error` | Cannot continue |
| `warn` | Keeps running, but the operator should look |
| `info` | Progress: start-up, goal accepted, goal finished |
| `debug` | Detail for developers |

- Log the values behind a decision, with units:
  `self.get_logger().warn(f'Reject: distance={d} m')`.
- Use `throttle_duration_sec=1.0` for messages inside loops.
- Record results you will analyze, such as `driver.py`'s odometry table and CSV, and inspect live
  data with `ros2 topic echo` and `ros2 bag record`.

---

## Principle 10: Composability — Build Complex Behaviors from Simple Ones

Build complex behavior from small, tested primitives. The Executive composes a letter P or a
figure-8 from single `move`, `turn` and `arc` goals, and each primitive is one `Drive` goal the
Driving Node already handles. Higher tiers sequence lower-tier behaviors. They do not
reimplement them.

---

## Principle 11: Graceful Degradation — Work With What You Have

When an input fails, keep doing what is still safe and say so. For example, `driver.py` still
drives when no odometry has arrived, but warns that the motion won't be reported. A behavior
that loses a safety input (hazards) should stop instead of carrying on.

---

## Quick Reference: Principle Checklist

Use this during code reviews:

| # | Principle | Check |
|---|-----------|-------|
| 1 | Single Responsibility | Can you describe the node without "and"? Is it in the right tier? |
| 2 | Interfaces | Do nodes talk only through messages, services and actions? |
| 3 | Open-Closed | Can you add a behavior without rewriting the working loop? |
| 4 | Interface Segregation | Is each interface focused, with units and signs documented? |
| 5 | Substitution | Would a replacement node or the simulator behave the same for the same goal? |
| 6 | Separation of Rates | Does any callback block? Are timers and callback groups used? |
| 7 | Fail-Safe Defaults | Low default speeds, goal validation, explicit stop, hazard stop? |
| 8 | Configuration | Are tunables ROS parameters with units? |
| 9 | Observe Everything | Can you tell what happened from the logs alone? |
| 10 | Composability | Are complex motions built from existing primitives? |
| 11 | Graceful Degradation | What does the node do when an input is missing? |
