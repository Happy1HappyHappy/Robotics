# day3pkg

ROS 2 package for CS 5335 Projects 3, 4 and 5: an **Executive Node** that takes
driving commands from the user and a **Driving Node** that drives the TurtleBot with
`cmd_vel`. The two nodes communicate through the `drive` action
(`day3_interfaces/action/Drive.action`). For Project 5, the Driving Node also reads
`/odom` and logs how far odometry says the robot moved, and its error, after each
command.

```
 user ──> Executive Node ──(drive action)──> Driving Node ──(cmd_vel)──> TurtleBot
          executive.py       Drive.action      driver.py      TwistStamped
                                                    ^
                                                    └──(odom)── Odometry
```

## Commands

| Command                         | Meaning                                                           | Example         |
| ------------------------------- | ----------------------------------------------------------------- | --------------- |
| `m <meters>`                    | Drive straight forward                                            | `m 1.0`         |
| `t <degrees>`                   | Turn in place; `+` = left (CCW), `-` = right (CW)                 | `t 90`, `t -90` |
| `a <l\|r> <radius m> <degrees>` | Drive along a circular arc to the left or right; degrees 0 to 360 | `a l 0.5 90`    |
| `p` / `8`                       | Squared letter P / squared figure-8                               | `p`             |
| `rp` / `r8`                     | Rounded letter P / rounded figure-8 (built from arcs)             | `r8`            |
| `q`                             | Quit (also stops the launch)                                      | `q`             |

The Executive waits for each command to finish before it accepts the next one.

## Build and run

The launch file opens the Executive in an `xterm` window, so install `xterm` first
(one time, on Ubuntu):

```bash
sudo apt install xterm
```

Without it, `ros2 launch` fails to start the Executive. `xterm` also needs a
desktop display, so over SSH without X forwarding, run the two nodes in separate
terminals instead:

```bash
ros2 run day3pkg driver
ros2 run day3pkg executive
```

From the workspace root (`robotics_ws`):

```bash
colcon build --packages-select day3_interfaces day3pkg
source install/setup.bash
ros2 launch day3pkg day3_launch.py
```

The launch file starts both nodes and opens the Executive in its own `xterm` window
for input. Quitting the Executive shuts down the whole launch.

Rebuild `day3_interfaces` whenever `Drive.action` changes, and rebuild `day3pkg`
whenever a Python file or the launch file changes.

To cancel a running goal from another terminal:

```bash
./cancel.sh
```

## The `drive` action

Goal fields (the Result and Feedback are empty for now):

| Field      | Type      | Meaning                                                                   |
| ---------- | --------- | ------------------------------------------------------------------------- |
| `distance` | `float64` | Meters to drive straight, `>= 0`                                          |
| `angle`    | `float64` | Turn: degrees, sign gives direction. Arc: degrees along the arc, 0 to 360 |
| `radius`   | `float64` | Arc radius in meters; `> 0` means this is an arc command                  |

## Odometry

The Driving Node subscribes to `/odom`. After each command finishes, it waits for the
robot to stop, then logs the motion odometry reports and its error from the command
(odometry minus target, in the robot's starting frame):

```
Odom: dist=1.0043 m, forward=1.0039 m, lateral=-0.0281 m, dyaw=-1.35 deg
Error (odom - target): forward=0.0039 m, lateral=-0.0281 m, position=0.0284 m, yaw=-1.35 deg
```
Everything is measured in the **robot's starting frame**: the forward axis is the
robot's heading at the start, left is +lateral, right is −lateral.
(Lateral offset is exaggerated in the drawing.)

```
                 ↑ forward axis (heading at start)
                 │        ↗ heading at end
                 │       ╱   angle from forward axis = dyaw
                 │ lateral   (−1.35°, turned right)
                 ├───────● end
                 │      ╱
                 │     ╱
     forward     │    ╱
   = 1.0039 m    │   ╱  dist = 1.0043 m
                 │  ╱
                 │ ╱
                 │╱
    left (+) ←───● start ───→ right (−)
```

| Field | Meaning | Formula |
| --- | --- | --- |
| `forward` | How far the robot moved along its starting heading | `cos θ0·dx + sin θ0·dy` |
| `lateral` | How far it drifted left (+) or right (−) of that line | `−sin θ0·dx + cos θ0·dy` |
| `dist` | Straight-line distance from start to end (the hypotenuse) | `√(dx² + dy²)` |
| `dyaw` | How much the heading changed (left/CCW +, right/CW −) | `θ1 − θ0` |

- `forward`, `lateral` and `dist` form a right triangle: `dist² = forward² + lateral²`
- `dyaw` is independent of position; it only describes the change in heading
The target for `m` is the distance straight ahead, for `t` the turn angle, and for
`a` the arc's end point and heading. For a full circle (`a l 0.5 360`), `position`
is how far odometry says the robot ended up from its start. Nothing is logged for a
canceled goal.

It then logs a table of the last 20 results (older runs are dropped), with the mean
and sample standard deviation of each error column:

```
Odometry results (last 20):
 run command         forward  lateral     dyaw  err_fwd  err_pos  err_yaw
   1 m 1              1.0100   0.0200     0.57   0.0100   0.0224     0.57
   2 m 1              1.0300   0.0000     0.00   0.0300   0.0300     0.00
mean                                             0.0200   0.0262     0.29
 std                                             0.0141   0.0054     0.41
```

The table covers every command type, so restart the Driving Node before a series of
runs (e.g. 10 × `m 1.0`) to get the mean and std for that series alone.

The same table is saved to `odom_results.csv` in the directory you started the node
from, and the file is overwritten after every command so it always matches the latest
table. To save it somewhere else, or turn it off with `''`:

```bash
ros2 run day3pkg driver --ros-args -p csv_path:=/home/robo/odom_results.csv
```

To read one Odometry message by hand:

```bash
ros2 topic echo /odom --once
```

To append it to a file instead (run it before and after each command, so every
run's start and end messages are kept in order):

```bash
ros2 topic echo /odom --once >> odom_log.txt
```

To drive with the robot's built-in `DriveDistance` action instead of this package:

```bash
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 1.0, max_translation_speed: 0.15}"
```
