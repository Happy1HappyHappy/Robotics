# day3pkg

ROS 2 package for CS 5335 Projects 3 to 6: an **Executive Node** that takes
driving commands from the user and a **Driving Node** that drives the TurtleBot with
`cmd_vel`. The two nodes communicate through the `drive` action
(`day3_interfaces/action/Drive.action`). For Project 5, the Driving Node also reads
`/odom` and logs how far odometry says the robot moved, and its error, after each
command.
For Project 6, a separate **Grid Node** builds an occupancy grid from the LiDAR; see
[Occupancy grid](#occupancy-grid-project-6).

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

## Occupancy grid (Project 6)

The **Grid Node** (`mapper.py`) builds an occupancy grid from the TurtleBot's LiDAR. It
reads `/scan`, places each scan using `/odom` and the LiDAR's mounting on the robot
(from TF), and draws and saves the grid as an image. `offline_grid` (`offline.py`)
builds the same grid from one saved scan, without the robot. The grid itself is in
`grid.py` and the drawing in `render.py`.

```
 /scan       LaserScan ──┐
 /odom       Odometry  ──┼──> Grid Node ──> window + grid_*.png
 /tf_static  LiDAR pose ─┘     mapper.py
```

The Grid Node **never moves the robot**. Drive it with the Executive / Driving Node or
the robot's `drive_distance` action (see
[Task 2](#task-2-drive-1-m-and-update-every-01-m)).

### The grid

| | |
| --- | --- |
| Size | 200 × 200 cells of 5 cm = 10 m × 10 m |
| Origin | The LiDAR's position at the start, at the center of the grid |
| Axes | x = the robot's start heading (image **up**), y = its left (image **left**) |
| Cell value | Log-odds `L = log10(p / (1 − p))`, starting at 0 (p = 0.5, unknown) |
| Ray passes through a cell | `L += log10(0.1 / 0.9) ≈ −0.95` (free) |
| Ray ends in a cell | `L += log10(0.9 / 0.1) ≈ +0.95` (occupied) |
| Limits | `L` is clamped to ±3, so a cell can change state again later |

For each scan, every ray with a valid range is cast from the sensor's cell to the cell
where it ends, with Bresenham's line algorithm (`grid.py`). Ranges outside
`[range_min, range_max]`, `inf` and `nan` are skipped. Each cell is updated at most once
per scan, and if any ray ends in a cell, that cell counts as occupied for the scan.

In the image, black is occupied, white is free and gray is unknown. The green circle is
the start, the blue dots are the sensor's position at each update, and the red arrow is
the robot's current heading. The bar in the bottom-left corner is 1 m.

### The LiDAR is mounted rotated by 90°

In the TurtleBot 4's URDF, the RPLIDAR (`rplidar_link`) sits 4 cm behind the robot's
center and is **rotated +90° (yaw)**. Scan angle 0 is the LiDAR's x axis, which points to
the robot's **left**, not forward. If you ignore this, the map is rotated by 90°, and in
Task 2 the walls smear as the robot drives, because the motion (forward) and the scan
no longer agree.

The Grid Node reads the mounting from TF, and `offline_grid` assumes 90° by default.
Check it on your robot (needs the robot connected):

```bash
ros2 run tf2_ros tf2_echo base_link rplidar_link
```

If the Grid Node keeps logging `Waiting for transform base_link -> rplidar_link`, TF is
not reaching your laptop. Run it with `-p use_tf:=false` to use `mount_yaw` (90°)
instead.

### Dependencies

The Grid Node and `offline_grid` need NumPy, OpenCV and PyYAML (`python3-numpy`,
`python3-opencv`, `python3-yaml`), which come with a ROS 2 desktop install. Build as
in [Build and run](#build-and-run).

### Task 1: occupancy grid from one scan

#### Offline (without the robot)

While connected to the robot, save one scan to a file:

```bash
ros2 topic echo /scan --once --full-length > scan.yaml
```

- On Jazzy, `ros2 topic echo` has no `-n 1`; use `--once`.
- **`--full-length` is required.** Without it, `ros2 topic echo` cuts every array off
  after 128 entries and writes `'...'`, so most of the scan would be lost.
  `offline_grid` refuses a file like that and tells you to capture it again.

Then, anywhere (no robot needed):

```bash
ros2 run day3pkg offline_grid scan.yaml
```

It prints how many rays it used, saves `grid_offline.png`, and shows the grid in a
window (press any key in the window to close it). The sensor is at the origin and the
robot faces up.

| Option | Default | Meaning |
| --- | --- | --- |
| `--mount-yaw` | `90` | LiDAR yaw on the robot, degrees. `0` draws the scan in the LiDAR's own frame |
| `--output` | `grid_offline.png` | Image file to save (`''` = do not save) |
| `--no-show` | off | Only save the image, do not open a window |
| `--size`, `--resolution`, `--scale` | `200`, `0.05`, `3` | Cells per side, cell side in m, image pixels per cell |

#### Live

With the robot standing still, run the Grid Node (see Task 2). Its first update is
Task 1: one scan with the sensor at the origin, saved as `grid_0.00m.png`.

### Task 2: drive 1 m and update every 0.1 m

1. Put the robot on the floor with about 1.5 m of clear space in front of it. If it is
   docked, undock it first:

   ```bash
   ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
   ```

2. Terminal 1: start the Grid Node while the robot stands still, and wait for
   `Update 1` (the start grid):

   ```bash
   ros2 run day3pkg mapper
   ```

3. Terminal 2: drive 1 m forward, slowly. Either use the robot's built-in action
   (closed loop on odometry, stops on cliffs):

   ```bash
   ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 1.0, max_translation_speed: 0.1}"
   ```

   or this package's Executive: `ros2 launch day3pkg day3_launch.py`, then `m 1.0`.

4. After `All snapshots saved`, stop the Grid Node with Ctrl-C.

The Grid Node adds a scan to the grid at the start and then after every 0.1 m of
travel (11 updates for 1 m), and logs each one:

```
Update 1: traveled 0.000 m, sensor at x=0.000 m, y=0.000 m, robot heading 0.0 deg, 1021 of 1080 rays used
Update 2: traveled 0.101 m, sensor at x=0.101 m, y=0.000 m, robot heading 0.0 deg, 1023 of 1080 rays used
...
Update 11: traveled 1.002 m, sensor at x=1.002 m, y=0.001 m, robot heading 0.0 deg, 1041 of 1080 rays used
```

Images saved in the directory you started it from (or `output_dir`):

| File | Contents |
| --- | --- |
| `grid_0.00m.png` | The grid at the start (Task 1) |
| `grid_0.50m.png` | After 0.5 m |
| `grid_1.00m.png` | After the complete 1 m |
| `grid_summary.png` | The snapshots so far, side by side, to show the instructor |
| `grid_final.png` | The grid when the node was stopped (Ctrl-C) |

How travel is measured: the straight-line distance of the robot's odometry position from
where it was at the first update. For a straight drive this is the distance driven, and
it does not drift while the robot stands still. A scan is added as soon as the robot
reaches the next multiple of 0.1 m. Odometry rarely stops exactly on 1.0 m, so if the
robot stops up to `distance_tolerance` (3 cm) short of a mark, the mark counts as
reached.

Each scan is placed with the latest odometry pose. At 0.1 m/s the robot moves about
1 cm between the two, which is less than a cell.

#### Parameters

Set with `--ros-args -p name:=value`, e.g.
`ros2 run day3pkg mapper --ros-args -p output_dir:=/home/robo/grids -p display:=false`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `size` | `200` | Cells per side |
| `resolution` | `0.05` | Cell side, m |
| `update_distance` | `0.1` | Travel between grid updates, m |
| `snapshot_distances` | `[0.0, 0.5, 1.0]` | Travel at which to save an image, m |
| `distance_tolerance` | `0.03` | A stop this far short of a mark still reaches it, m |
| `stopped_speed` | `0.005` | Odometry speed below this counts as stopped, m/s |
| `max_odom_age` | `0.5` | Skip scans when the latest odometry is older than this, s |
| `use_tf` | `true` | Read the LiDAR mounting from TF |
| `mount_yaw` | `90.0` | LiDAR yaw on the robot when `use_tf` is false, degrees |
| `output_dir` | `.` | Where the images are saved (must exist) |
| `display` | `true` | Show the grid in a window (skipped without a display) |
| `image_scale` | `3` | Image pixels per cell side |

### Testing without the robot

- Unit tests of the grid and the scan file reader, from this package's directory
  (`robotics_ws/src/day3pkg`):

  ```bash
  python3 -m pytest test/test_grid.py
  ```

- Record the topics once on the robot and replay them later into the Grid Node:

  ```bash
  ros2 bag record /scan /odom /tf /tf_static     # on the robot's network, while driving
  ros2 bag play <bag folder>                     # later, offline, with the Grid Node running
  ```

  If the replay does not deliver the LiDAR transform, add `-p use_tf:=false`.

### Check on the robot first

These were not checked on a real robot yet:

| What | Command |
| --- | --- |
| The topics exist with these types | `ros2 topic list -t \| grep -E 'scan\|odom'` |
| The scan's frame name and range limits | `timeout 5 ros2 topic echo /scan --once --no-arr` |
| The LiDAR mounting (yaw 90°?) | `ros2 run tf2_ros tf2_echo base_link rplidar_link` |
| The scan rate | `timeout 10 ros2 topic hz /scan` |
