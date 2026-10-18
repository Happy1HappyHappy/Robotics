# day6pkg

ROS 2 package for CS 5335 Project 6: an **occupancy grid** built from the TurtleBot's
LiDAR. The **Grid Node** reads `/scan`, places each scan using `/odom` and the LiDAR's
mounting on the robot (from TF), and draws and saves the grid as an image.
`offline_grid` builds the same grid from one saved scan, without the robot.

```
 /scan       LaserScan ──┐
 /odom       Odometry  ──┼──> Grid Node ──> window + grid_*.png
 /tf_static  LiDAR pose ─┘     mapper.py
```

The Grid Node **never moves the robot**. Drive it with `day3pkg` or the robot's
`drive_distance` action (see [Task 2](#task-2-drive-1-m-and-update-every-01-m)).

## The grid

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

## The LiDAR is mounted rotated by 90°

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

## Build

From the workspace root (`robotics_ws`):

```bash
colcon build --packages-select day6pkg
source install/setup.bash
```

Needs NumPy, OpenCV and PyYAML (`python3-numpy`, `python3-opencv`, `python3-yaml`),
which come with a ROS 2 desktop install.

## Task 1: occupancy grid from one scan

### Offline (without the robot)

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
ros2 run day6pkg offline_grid scan.yaml
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

### Live

With the robot standing still, run the Grid Node (see Task 2). Its first update is
Task 1: one scan with the sensor at the origin, saved as `grid_0.00m.png`.

## Task 2: drive 1 m and update every 0.1 m

1. Put the robot on the floor with about 1.5 m of clear space in front of it. If it is
   docked, undock it first:

   ```bash
   ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
   ```

2. Terminal 1: start the Grid Node while the robot stands still, and wait for
   `Update 1` (the start grid):

   ```bash
   ros2 run day6pkg mapper
   ```

3. Terminal 2: drive 1 m forward, slowly. Either use the robot's built-in action
   (closed loop on odometry, stops on cliffs):

   ```bash
   ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 1.0, max_translation_speed: 0.1}"
   ```

   or `day3pkg`: `ros2 launch day3pkg day3_launch.py`, then `m 1.0`.

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

### Parameters

Set with `--ros-args -p name:=value`, e.g.
`ros2 run day6pkg mapper --ros-args -p output_dir:=/home/robo/grids -p display:=false`.

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

## Testing without the robot

- Unit tests of the grid and the scan file reader, from this package's directory:

  ```bash
  python3 -m pytest test/test_grid.py
  ```

- Record the topics once on the robot and replay them later into the Grid Node:

  ```bash
  ros2 bag record /scan /odom /tf /tf_static     # on the robot's network, while driving
  ros2 bag play <bag folder>                     # later, offline, with the Grid Node running
  ```

  If the replay does not deliver the LiDAR transform, add `-p use_tf:=false`.

## Check on the robot first

These were not checked on a real robot yet:

| What | Command |
| --- | --- |
| The topics exist with these types | `ros2 topic list -t \| grep -E 'scan\|odom'` |
| The scan's frame name and range limits | `timeout 5 ros2 topic echo /scan --once --no-arr` |
| The LiDAR mounting (yaw 90°?) | `ros2 run tf2_ros tf2_echo base_link rplidar_link` |
| The scan rate | `timeout 10 ros2 topic hz /scan` |
