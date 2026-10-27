# day6pkg

ROS 2 package for CS 5335 Projects 6 and 7. The **Grid Node** (`grid_node.py`) builds an
occupancy grid from the TurtleBot's LiDAR: it reads `/scan`, places each scan using `/odom`
and the LiDAR's mounting on the robot (from TF), and publishes the grid. The **Landmark
Node** (Project 7) finds corner landmarks in it. Both only compute and publish, for any
node that needs their data (later, to decide where to drive); the **Viewer Node** is the
only one that draws. The grid itself is in `occupancy_grid.py`, the landmark search in
`landmarks.py` and the drawing in `render.py`.

```
 /scan       LaserScan ──┐                  /grid ──────┬──────────────> Viewer Node ──> window
 /odom       Odometry  ──┼──> Grid Node ──> /grid_path ─┼──────────────>  viewer_node.py   + grid_*.png
 /tf_static  LiDAR pose ─┘   grid_node.py      │        └> Landmark Node ──> /landmarks ──┘
                                               ▼            landmark_node.py  /wall_segments
                                           grid.npz                              landmarks.npz
```

Without the Viewer Node the others run the same and nothing is drawn; without the
Landmark Node the Viewer Node draws the grid alone.

None of them **ever moves the robot**. Drive it with the robot's `drive_distance` action or
the Executive in [`day3pkg`](../day3pkg/README.md) (see
[Task 2](#task-2-drive-1-m-and-update-every-01-m)). `launch/day6_launch.py` (Project 6)
starts the Grid Node and the Viewer Node together with `day3pkg`'s Driving and Executive
nodes; `launch/project7_launch.py` starts the three nodes here (see
[Project 7](#project-7-landmarks-while-driving)).

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
where it ends, with Bresenham's line algorithm (`occupancy_grid.py`). Ranges outside
`[range_min, range_max]`, `inf` and `nan` are skipped. Each cell is updated at most once
per scan, and if any ray ends in a cell, that cell counts as occupied for the scan.

In the Viewer Node's image, black is occupied, white is free and gray is unknown. The green
circle is the start, the blue dots are the sensor's position at each update, and the red
arrow is the robot's current heading. With the Landmark Node running, the wall segments
are orange lines and the corner landmarks red circles (`L1`, `L2`, ...). The bar in the
bottom-left corner is 1 m.

Example image:
<img width="1812" height="600" alt="grid_summary" src="https://github.com/user-attachments/assets/2010fb33-4cf8-4db3-a9eb-5d0574209ab3" />


## The LiDAR is mounted rotated by 90°

In the TurtleBot 4's URDF, the RPLIDAR (`rplidar_link`) sits 4 cm behind the robot's
center and is **rotated +90° (yaw)**. Scan angle 0 is the LiDAR's x axis, which points to
the robot's **left**, not forward. If you ignore this, the map is rotated by 90°, and in
Task 2 the walls smear as the robot drives, because the motion (forward) and the scan
no longer agree.

The Grid Node reads the mounting from TF. Check it on your robot (needs the robot
connected):

```bash
ros2 run tf2_ros tf2_echo base_link rplidar_link
```

If the Grid Node keeps logging `Waiting for transform base_link -> rplidar_link`, TF is
not reaching your laptop. Run it with `-p use_tf:=false` to use `mount_yaw` (90°)
instead.

## Build

The nodes need NumPy and OpenCV (`python3-numpy`, `python3-opencv`), which come with a
ROS 2 desktop install, and the Landmark Node scikit-image (`python3-skimage`). To install
whatever is missing, run once from the workspace root:

```bash
rosdep install -i --from-path src --rosdistro jazzy -y
```

From the workspace root (`robotics_ws`):

```bash
colcon build --packages-select day6pkg
source install/setup.bash
```

Rebuild `day6pkg` whenever a Python file or the launch file changes. The launch file also
starts `day3pkg`'s nodes, so for it build all three packages:

```bash
colcon build --packages-select day3_interfaces day3pkg day6pkg
source install/setup.bash
```

`day3pkg` has its own needs (`xterm` for the Executive, `irobot_create_msgs` for the
Driving Node); see the [day3pkg README](../day3pkg/README.md#build-and-run).

## Task 1: occupancy grid from one scan

Start the Grid Node and the Viewer Node with the robot standing still (with the launch
file or in separate terminals; see Task 2). The Grid Node's first update is Task 1: one
scan with the sensor at the origin, which the Viewer Node shows in its window and saves as
`grid_0.00m.png`.

## Task 2: drive 1 m and update every 0.1 m

First put the robot on the floor with about 1.5 m of clear space in front of it. If it
is docked, undock it (the LiDAR may also be off while it is docked):

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

### With the launch file (one command)

```bash
ros2 launch day6pkg day6_launch.py
```

1. The Grid Node and the Viewer Node log in this terminal and the Viewer Node opens the
   grid window; `day3pkg`'s Executive opens in its own `xterm` window. Wait for
   `Update 1` (the start grid).
2. In the Executive window, type `m 1.0`.
3. After `All snapshots saved`, type `q` in the Executive. This stops the whole launch:
   the Grid Node saves `grid.npz` and the Viewer Node `grid_final.png` on the way out.

This needs a desktop (the `xterm` and the grid window). Over SSH without X forwarding,
use separate terminals instead.

### In separate terminals

1. Terminal 1: start the Viewer Node (it waits for the grid):

   ```bash
   ros2 run day6pkg viewer_node
   ```

2. Terminal 2: start the Grid Node while the robot stands still, and wait for
   `Update 1` (the start grid):

   ```bash
   ros2 run day6pkg grid_node
   ```

3. Terminal 3: drive 1 m forward, slowly. Either use the robot's built-in action
   (closed loop on odometry, stops on cliffs):

   ```bash
   ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 1.0, max_translation_speed: 0.1}"
   ```

   or the Executive in `day3pkg`: `ros2 launch day3pkg day3_launch.py`, then `m 1.0`.

4. After `All snapshots saved` (Viewer Node), stop both nodes with Ctrl-C.

The Grid Node adds a scan to the grid at the start and then after every 0.1 m of
travel (11 updates for 1 m), and logs each one:

```
Update 1: traveled 0.000 m, sensor at x=0.000 m, y=0.000 m, robot heading 0.0 deg, 1021 of 1080 rays used
Update 2: traveled 0.101 m, sensor at x=0.101 m, y=0.000 m, robot heading 0.0 deg, 1023 of 1080 rays used
...
Update 11: traveled 1.002 m, sensor at x=1.002 m, y=0.001 m, robot heading 0.0 deg, 1041 of 1080 rays used
```

After every update the Grid Node publishes the grid on `/grid` (`nav_msgs/OccupancyGrid`,
occupancy in percent, -1 unknown) and the pose of every update so far on `/grid_path`
(`nav_msgs/Path`). Both keep their latest message, so a node started later gets them at
once. Files saved in the directory each node was started from (or its `output_dir`):

| File | Saved by | Contents |
| --- | --- | --- |
| `grid_0.00m.png` | Viewer Node | The grid at the start (Task 1) |
| `grid_0.50m.png` | Viewer Node | After 0.5 m |
| `grid_1.00m.png` | Viewer Node | After the complete 1 m |
| `grid_summary.png` | Viewer Node | The snapshots so far, side by side, to show the instructor |
| `grid_final.png` | Viewer Node | The last image, when it was stopped (Ctrl-C) |
| `grid.npz` | Grid Node | The grid itself on Ctrl-C: log-odds, resolution, the pose of each update and ^world T_odom |

How travel is measured: the straight-line distance of the robot's odometry position from
where it was at the first update. For a straight drive this is the distance driven, and
it does not drift while the robot stands still. A scan is added as soon as the robot
reaches the next multiple of 0.1 m. Odometry rarely stops exactly on 1.0 m, so if the
robot stops up to `distance_tolerance` (3 cm) short of a mark, the mark counts as
reached.

Each scan is placed with the latest odometry pose. At 0.1 m/s the robot moves about
1 cm between the two, which is less than a cell.

## Parameters

Set with `--ros-args -p name:=value`, e.g.
`ros2 run day6pkg viewer_node --ros-args -p output_dir:=/home/robo/grids -p display:=false`,
or in a YAML file (as `config/project7.yaml`).

Grid Node (`grid_node`):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `size` | `200` | Cells per side |
| `resolution` | `0.05` | Cell side, m |
| `update_distance` | `0.1` | Travel between grid updates, m |
| `distance_tolerance` | `0.03` | A stop this far short of a mark still reaches it, m |
| `stopped_speed` | `0.005` | Odometry speed below this counts as stopped, m/s |
| `max_odom_age` | `0.5` | Skip scans when the latest odometry is older than this, s |
| `use_tf` | `true` | Read the LiDAR mounting from TF |
| `mount_yaw` | `90.0` | LiDAR yaw on the robot when `use_tf` is false, degrees |
| `grid_frame` | `world` | `frame_id` of `/grid` and `/grid_path` |
| `save_grid` | `true` | Save the grid as `grid.npz` on Ctrl-C |
| `output_dir` | `.` | Where `grid.npz` is saved (created if missing) |

Viewer Node (`viewer_node`):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `snapshot_distances` | `[0.0, 0.5, 1.0]` | Travel at which to save an image, m |
| `distance_tolerance` | `0.03` | This far short of a snapshot distance still reaches it, m |
| `image_scale` | `3` | Image pixels per cell side |
| `display` | `true` | Show the image in a window (skipped without a display) |
| `save_images` | `true` | Save the snapshots and `grid_final.png` |
| `output_dir` | `.` | Where the images are saved (created if missing) |

## Project 7: landmarks while driving

`launch/project7_launch.py` starts the Grid Node, the Landmark Node and the Viewer Node
with the parameters in `config/project7.yaml` (15 m grid, snapshots at 0, 1, 2 and 3 m).
All save to the folder given by `run`:

```bash
ros2 launch day6pkg project7_launch.py run:=run_a
```

Ctrl-C saves every node's files to `run_a`; start it again with `run:=run_b` for the way
back. `draw:=false` leaves the Viewer Node out: the grid and the landmarks are still
computed, published and saved, but nothing is drawn. `save_grid` (Grid Node), `save`
(Landmark Node) and `save_images` (Viewer Node) in the YAML file turn their files off
(rebuild after changing it).

The Landmark Node (`landmark_node.py`, algorithms in `landmarks.py`) looks for landmarks in
every new `/grid`, once per `period`:

1. Points: the centers of the cells with log-odds above `occupied` (1.0).
2. Wall segments by RANSAC (Day 7 slides 42-45, with scikit-image's `ransac`): two random
   points at most `sample_radius` apart, the points within `inlier_distance` of their line;
   the best line's longest unbroken run (no gap over `max_gap`) is a segment if it has
   `min_points` points and `min_length` m. Its points are removed and the search repeats.
3. Corners (slide 48): two segments crossing at `corner_angle` or more, both ending within
   `corner_distance` of the crossing.

It publishes the corners on `/landmarks` (`geometry_msgs/PoseArray`) and the segments on
`/wall_segments` (`visualization_msgs/Marker`, a line list), both in the grid's frame, and
on Ctrl-C saves the last ones as `landmarks.npz` (`corners` K × 2, `segments` N × 2 × 2,
m). The Viewer Node draws them on the grid, so its images (`grid_final.png` too) show the
landmarks. RViz can show all these topics as well (Map, Path, PoseArray, Marker).

It needs scikit-image (`python3-skimage`, see [Build](#build)).

## Testing without the robot

- Unit tests of the grid and the landmark search, from this package's directory
  (`robotics_ws/src/day6pkg`); the RANSAC tests are skipped without scikit-image:

  ```bash
  python3 -m pytest test/test_grid.py test/test_landmarks.py
  ```

- Record the topics once on the robot and replay them later into the Grid Node:

  ```bash
  ros2 bag record /scan /odom /tf /tf_static     # on the robot's network, while driving
  ros2 bag play <bag folder>                     # later, with the Grid Node running
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
| The grid and the path are published | `timeout 5 ros2 topic echo /grid --once --no-arr`, `timeout 5 ros2 topic echo /grid_path --once` |
| The landmarks and segments are published | `timeout 5 ros2 topic echo /landmarks --once`, `timeout 5 ros2 topic echo /wall_segments --once` |
| The installed scikit-image version | `python3 -c "import skimage; print(skimage.__version__)"` |
