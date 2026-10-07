# day2pkg

ROS 2 package for CS 5335 Project 2: a single node, `node1`, that drives the
TurtleBot open-loop by publishing `TwistStamped` messages on `/cmd_vel`. Motions
are queued as timed **segments** (a velocity held for a computed duration), and a
20 Hz timer plays them back one after another.

```
 main() ──(queue segments)──> Node1 ──(TwistStamped @ 20 Hz)──> /cmd_vel ──> TurtleBot
                               node1.py
```

## Driving functions

| Function             | Meaning                                                                     | Example                 |
| -------------------- | --------------------------------------------------------------------------- | ----------------------- |
| `DriveStraight(d)`   | Drive straight `d` meters; negative drives backward                         | `DriveStraight(1.0)`    |
| `Turn(deg)`          | Turn in place; `+` = left (CCW), `-` = right (CW)                           | `Turn(-90)`             |
| `DriveArc(r, deg)`   | Drive along an arc of radius `r` m; sign of `deg` gives direction; `r ≈ 0` turns in place | `DriveArc(0.5, -180)` |
| `DriveCircle(r)`     | Full circle of radius `r` m (`DriveArc(r, 360)`)                            | `DriveCircle(0.5)`      |
| `Pause(sec=0.5)`     | Hold zero velocity                                                          | `Pause()`               |

Each call only adds a segment to the queue; nothing moves until the node is spun.
Between segments the node sends one zero-velocity message, and after the last
segment it sends zero velocity three times and finishes.

The node waits (logging `Waiting for robot..`) until something subscribes to
`/cmd_vel`, so the timing does not start before the robot is listening.

## Default path

`main()` drives a **D-shaped path** (Task 6):

1. Drive straight 1.0 m
2. Turn right 90°
3. Drive a right semicircle, radius 0.5 m
4. Turn right 90°

with a 0.5 s pause between steps. To try other motions, edit the calls in `main()`;
earlier task examples are left there as comments.

## Build and run

From the workspace root (`robotics_ws`):

```bash
colcon build --packages-select day2pkg
source install/setup.bash
ros2 run day2pkg node1
```

Ctrl+C sends a zero-velocity message before shutting down.

## Tuning

Constants at the top of `day2pkg/node1.py`:

| Constant    | Default | Meaning                                                      |
| ----------- | ------- | ------------------------------------------------------------ |
| `TOPIC`     | `/cmd_vel` | Velocity command topic                                    |
| `RATE_HZ`   | `20.0`  | Publishing rate (Hz)                                         |
| `LIN_SPEED` | `0.10`  | Linear speed (m/s)                                           |
| `ANG_SPEED` | `0.50`  | Turn-in-place angular speed (rad/s)                          |
| `PAUSE_SEC` | `0.5`   | Default `Pause()` length (s)                                 |
| `LIN_CAL`   | `1.0`   | Scales straight/arc durations; raise it if the robot undershoots distance |
| `ANG_CAL`   | `1.0`   | Scales turn durations; raise it if the robot undershoots angles |

Because the motion is open-loop (time × speed), the calibration factors are the
way to correct for wheel slip and real-world drift.
