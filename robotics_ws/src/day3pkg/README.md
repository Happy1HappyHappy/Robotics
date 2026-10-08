# day3pkg

ROS 2 package for CS 5335 Projects 3 and 4: an **Executive Node** that takes driving
commands from the user and a **Driving Node** that drives the TurtleBot with
`cmd_vel`. The two nodes communicate through the `drive` action
(`day3_interfaces/action/Drive.action`).

```
 user ──> Executive Node ──(drive action)──> Driving Node ──(cmd_vel)──> TurtleBot
          executive.py       Drive.action      driver.py      TwistStamped
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
