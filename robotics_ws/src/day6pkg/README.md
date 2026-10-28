# day6pkg

ROS 2 package for CS 5335 Projects 6 and 7. The **Grid Node** builds an occupancy grid
from the TurtleBot's LiDAR: it reads `/scan`, places each scan using `/odom` and the
LiDAR's mounting on the robot (from TF), and publishes the grid. The **Landmark Node**
(Project 7) finds corner landmarks in it. Both only compute and publish, for any node that
needs their data (later, to decide where to drive); the **Viewer Node** is the only one
that draws. None of them **ever moves the robot**.

The nodes are in `nodes/`; what they use, without ROS (so it can be tested on its own),
is in `helpers/`:

```
day6pkg/
├── nodes/                   ROS 2 nodes: subscribe, publish, save
│   ├── grid_node.py         Grid Node        (ros2 run day6pkg grid_node)
│   ├── landmark_node.py     Landmark Node    (ros2 run day6pkg landmark_node)
│   └── viewer_node.py       Viewer Node      (ros2 run day6pkg viewer_node)
└── helpers/                 no ROS
    ├── occupancy_grid.py    the grid: log-odds cells, ray casting, saving and loading
    ├── landmarks.py         RANSAC wall segments and corner landmarks
    ├── render.py            drawing the grid, the path and the landmarks
    └── files.py             saving files whole (temporary file, then rename)
```

```
 /scan       LaserScan ──┐                  /grid ──────┬──────────────> Viewer Node ──> window
 /odom       Odometry  ──┼──> Grid Node ──> /grid_path ─┼──────────────>  viewer_node.py   + result/grid/*.png
 /tf_static  LiDAR pose ─┘   grid_node.py      │        └> Landmark Node ──> /landmarks ──┘
                                               ▼            landmark_node.py  /wall_segments
                                   logs/<run>_grid.npz                 logs/<run>_landmarks.npz
```

Without the Viewer Node the others run the same and nothing is drawn; without the
Landmark Node the Viewer Node draws the grid alone.

## Install dependencies

Once, on the Ubuntu machine that runs the nodes (Ubuntu 24.04, ROS 2 Jazzy):

| Needed by | Python module | Ubuntu package |
| --- | --- | --- |
| All three nodes | `numpy` | `python3-numpy` |
| Viewer Node (drawing, window, PNG files) | `cv2` | `python3-opencv` |
| Landmark Node (RANSAC) | `skimage` | `python3-skimage` |
| Unit tests | `pytest` | `python3-pytest` |

`rclpy`, the message packages, `tf2_ros` and `launch` come with ROS 2 Jazzy. All of these
are declared in `package.xml`, so `rosdep` installs whatever is missing. From the workspace
root (`robotics_ws`):

```bash
sudo rosdep init      # only the first time on this machine; skip it if it says it already exists
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
```

Run it on the whole `src` folder, not just `src/day6pkg`: `day6pkg` lists `day3pkg` (for
`day6_launch.py`), which rosdep only skips when it finds it in the folder.

Without rosdep, install them with apt:

```bash
sudo apt install python3-numpy python3-opencv python3-skimage python3-pytest
```

Use apt, not `pip`: the nodes run on the system Python that ROS uses, and Ubuntu 24.04
refuses `pip install` into it. Check that all three import:

```bash
python3 -c "import numpy, cv2, skimage; print(numpy.__version__, cv2.__version__, skimage.__version__)"
```

`day6_launch.py` (Project 6) also starts `day3pkg`'s nodes, which need
`irobot_create_msgs` and `xterm`; see the [day3pkg README](../day3pkg/README.md#build-and-run).
`project7_launch.py` needs only this package.

## Build

From the workspace root (`robotics_ws`), after every change to a Python, launch or YAML
file:

```bash
colcon build --packages-select day6pkg
source install/setup.bash
```

For `day6_launch.py`, build `day3pkg` too:
`colcon build --packages-select day3_interfaces day3pkg day6pkg`.

## Run

The robot must be off the dock (the LiDAR may be off while it is docked), with clear floor
in front of it:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

### Project 7: landmarks while driving

`launch/project7_launch.py` starts the Grid Node, the Landmark Node and the Viewer Node
with the parameters in `config/project7.yaml` (15 m grid, snapshots at 0, 1, 2 and 3 m):

```bash
ros2 launch day6pkg project7_launch.py
```

Drive the robot from another terminal (see [Driving](#driving)), then stop with Ctrl-C;
launch it again for the way back. Each launch is one run, named after the time it started,
and its files are named after the run like log files (see [Saved files](#saved-files)), so
nothing needs naming and no run overwrites another. `run_id:=<name>` names a run yourself.
`draw:=false` leaves the Viewer Node out: the grid and the landmarks are still computed,
published and saved, but nothing is drawn. `save_grid` (Grid Node), `save` (Landmark Node)
and `save_images` (Viewer Node) in the YAML file turn their files off, and `save_period`
sets how often they are saved (rebuild after changing it).

The Landmark Node needs up to `period` (1 s) plus its search time after a grid update to
publish that grid's landmarks, so the images the Viewer Node saves at a snapshot distance
show the landmarks of the update before. Wait about 2 s after the robot stops before
pressing Ctrl-C, so that the last `<run>_landmarks.npz` and `<run>_grid_final.png` hold the
landmarks of the last grid.

### Project 6: occupancy grid while driving 1 m

```bash
ros2 launch day6pkg day6_launch.py
```

This starts the Grid Node and the Viewer Node with `day3pkg`'s Driving and Executive nodes.
Wait for `Update 1` (the start grid, Task 1), type `m 1.0` in the Executive's `xterm`
window, and after `All snapshots saved` type `q` to stop everything. It needs a desktop
(the `xterm` and the grid window); over SSH, run the nodes in separate terminals instead
and drive as below. Each one started on its own names its run after its own start time;
give both the same `run_id` so their files match:

```bash
ros2 run day6pkg viewer_node --ros-args -p run_id:=day6_test
ros2 run day6pkg grid_node --ros-args -p run_id:=day6_test
```

### Driving

The nodes here never move the robot. Drive slowly with the robot's built-in action (closed
loop on odometry, stops on cliffs):

```bash
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 1.0, max_translation_speed: 0.1}"
```

or with the Executive in [`day3pkg`](../day3pkg/README.md): `ros2 launch day3pkg day3_launch.py`,
then `m 1.0`.

### Saved files

The files are saved like logs: every run adds its own files to the same two folders, named
after the run (`<run>`: the launch's start time, e.g. `2026-10-27_16-40-12`, or `run_id`).
The data goes to `logs/` and the images to `result/grid/`, both in the directory the nodes
were started from (`log_dir`, `result_dir`), created if missing:

```
robotics_ws/
├── logs/
│   ├── 2026-10-27_16-40-12_grid.npz           ← the way there
│   ├── 2026-10-27_16-40-12_landmarks.npz
│   ├── 2026-10-27_16-45-03_grid.npz           ← the way back
│   └── 2026-10-27_16-45-03_landmarks.npz
└── result/
    └── grid/
        ├── 2026-10-27_16-40-12_grid_0.00m.png  … _grid_3.00m.png
        ├── 2026-10-27_16-40-12_grid_summary.png
        ├── 2026-10-27_16-40-12_grid_final.png
        └── 2026-10-27_16-45-03_grid_…png
```

It is `logs/`, not `log/`: colcon writes its own build logs to `robotics_ws/log/`. Git
ignores both `logs/` and `result/`.

| File | Saved by | Contents |
| --- | --- | --- |
| `result/grid/<run>_grid_<d>m.png` | Viewer Node | The image after `d` m of travel, one per `snapshot_distances` (`_grid_0.00m.png` is the start grid) |
| `result/grid/<run>_grid_summary.png` | Viewer Node | The snapshots so far, side by side |
| `result/grid/<run>_grid_final.png` | Viewer Node | The latest image |
| `logs/<run>_grid.npz` | Grid Node | Log-odds, resolution, the pose of each update and ^world T_odom |
| `logs/<run>_landmarks.npz` | Landmark Node | `corners` (K × 2), `segments` (N × 2 × 2), m, `resolution`, `frame` |

The snapshots are saved when their distance is reached. The other three are saved every
`save_period` seconds (10 s) while they change, and once more on Ctrl-C; `save_period: 0.0`
saves on Ctrl-C only. A file is written to a temporary file first and then renamed over the
old one, so a node killed while saving (or never stopped with Ctrl-C) leaves its last
complete file, at most `save_period` old.

## Nodes and topics

The robot publishes `/scan`, `/odom` and `/tf_static` on its own. Everything else comes from
the three nodes of this package:

| Node | Subscribes to | Publishes | Saves |
| --- | --- | --- | --- |
| Grid Node (`grid_node`) | `/scan`, `/odom`, `/tf_static` | `/grid`, `/grid_path` | `logs/<run>_grid.npz` (`save_grid`) |
| Landmark Node (`landmark_node`) | `/grid` | `/landmarks`, `/wall_segments` | `logs/<run>_landmarks.npz` (`save`) |
| Viewer Node (`viewer_node`) | `/grid`, `/grid_path`, `/landmarks`, `/wall_segments` | nothing | `result/grid/<run>_grid_*.png` (`save_images`) |

| Topic | From | To | Type | When |
| --- | --- | --- | --- | --- |
| `/scan` | robot (LiDAR) | Grid Node | `sensor_msgs/LaserScan` | Every LiDAR turn |
| `/odom` | robot (base) | Grid Node | `nav_msgs/Odometry` | Every odometry update |
| `/tf_static` | robot | Grid Node (looked up once) | TF | Once, at start-up |
| `/grid` | Grid Node | Landmark Node, Viewer Node | `nav_msgs/OccupancyGrid` | Every grid update (every 0.1 m) |
| `/grid_path` | Grid Node | Viewer Node | `nav_msgs/Path` | Every grid update, stamped like `/grid` |
| `/landmarks` | Landmark Node | Viewer Node | `geometry_msgs/PoseArray` | At most once per `period` (1 s), for a new grid |
| `/wall_segments` | Landmark Node | Viewer Node | `visualization_msgs/Marker` | With `/landmarks`, stamped alike |

The four topics of this package are in the grid's frame (`world`: the sensor's position and
the robot's heading at the Grid Node's first update) and keep their latest message
(transient local, depth 1), so a node started later gets each of them at once. RViz can
show all four (Map, Path, PoseArray, Marker). What they hold:

| Topic | Contents |
| --- | --- |
| `/grid` | `info`: `resolution` (0.05 m), `width` = `height` (cells per side), `origin` (the outer corner of the first cell, e.g. (−7.5, −7.5) m for 300 cells). `data`: one value per cell, occupancy in percent (0 to 100, −1 unknown), row by row from the −x, −y corner: cell (i, j) is `data[j * width + i]`, centered at `origin + ((i + 0.5), (j + 0.5)) * resolution` |
| `/grid_path` | `poses`: one per grid update so far, oldest first: the sensor's position (x, y) and the robot's heading (rotation about z) |
| `/landmarks` | `poses`: one per corner landmark, position (x, y) only, sorted by x then y (`L1`, `L2`, ... in the images); stamped like the grid they were found in |
| `/wall_segments` | A `LINE_LIST` marker: `points` holds two points per wall segment (start, end); width and color are for RViz |

## How it works

### The grid

| | |
| --- | --- |
| Size | `size` × `size` cells of 5 cm (200 = 10 m by default, 300 = 15 m in Project 7) |
| Origin | The LiDAR's position at the start, at the center of the grid |
| Axes | x = the robot's start heading (image **up**), y = its left (image **left**) |
| Cell value | Log-odds `L = log10(p / (1 − p))`, starting at 0 (p = 0.5, unknown) |
| Ray passes through a cell | `L += log10(0.1 / 0.9) ≈ −0.95` (free) |
| Ray ends in a cell | `L += log10(0.9 / 0.1) ≈ +0.95` (occupied) |
| Limits | `L` is clamped to ±3, so a cell can change state again later |

The Grid Node adds a scan at the start and then after every `update_distance` (0.1 m) of
travel, measured as the straight-line distance from the start position. Every ray with a
valid range is cast from the sensor's cell to the cell where it ends with Bresenham's line
algorithm (`helpers/occupancy_grid.py`); ranges outside `[range_min, range_max]`, `inf` and
`nan` are skipped. Each cell is updated at most once per scan.

The LiDAR is mounted **rotated +90° (yaw)** on the TurtleBot 4: scan angle 0 points to the
robot's left. The Grid Node reads this from TF (`base_link` → `rplidar_link`); ignoring it
rotates the map by 90° and smears the walls while driving.

In the Viewer Node's image, black is occupied, white is free and gray is unknown. The green
circle is the start, the blue dots are the sensor's position at each update, and the red
arrow is the robot's current heading. With the Landmark Node running, the wall segments
are orange lines and the corner landmarks red circles (`L1`, `L2`, ...). The bar in the
bottom-left corner is 1 m.

Example image:
<img width="1812" height="600" alt="grid_summary" src="https://github.com/user-attachments/assets/2010fb33-4cf8-4db3-a9eb-5d0574209ab3" />

### Landmarks

The Landmark Node (`nodes/landmark_node.py`, algorithms in `helpers/landmarks.py`) looks for
landmarks in every new `/grid`, once per `period`:

1. Points: the centers of the cells with log-odds above `occupied` (1.0).
2. Wall segments by RANSAC (Day 7 slides 42-45, with scikit-image's `ransac`): two random
   points at most `sample_radius` apart, the points within `inlier_distance` of their line;
   the best line's longest unbroken run (no gap over `max_gap`) is a segment if it has
   `min_points` points and `min_length` m. Its points are removed and the search repeats.
3. Corners (slide 48): two segments crossing at `corner_angle` or more, both ending within
   `corner_distance` of the crossing.

## Parameters

Set with `--ros-args -p name:=value`, e.g.
`ros2 run day6pkg viewer_node --ros-args -p result_dir:=/home/robo/result -p display:=false`,
or in a YAML file (as `config/project7.yaml`; rebuild after changing it).

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
| `save_grid` | `true` | Save the grid as `<run>_grid.npz` |
| `save_period` | `10.0` | Seconds between saves while the grid changes (and on Ctrl-C); `0.0`: on Ctrl-C only |
| `log_dir` | `logs` | Where the `.npz` file is saved (created if missing) |
| `run_id` | `''` | The run's name, which starts the file name; empty: the node's start time (the launch files set it) |

Landmark Node (`landmark_node`; Day 7 slides 45 and 48):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `occupied` | `1.0` | Log-odds above which a cell is a point (at least two more hits than misses) |
| `sample_radius` | `0.5` | The two random points of a RANSAC try are at most this far apart, m |
| `inlier_distance` | `0.06` | A point this close to a line is on it, m |
| `min_points` | `8` | Points a segment needs |
| `min_length` | `0.3` | Shortest segment kept, m |
| `max_gap` | `0.15` | A wider gap between points on a line splits it into segments, m |
| `trials` | `2000` | RANSAC tries per line |
| `corner_angle` | `60.0` | Two segments must cross at least at this angle to make a corner, degrees |
| `corner_distance` | `0.15` | Both segments must end this near their crossing, m |
| `period` | `1.0` | How often to look for a new grid, s |
| `save` | `true` | Save `<run>_landmarks.npz` |
| `save_period` | `10.0` | Seconds between saves while the landmarks change (and on Ctrl-C); `0.0`: on Ctrl-C only |
| `log_dir` | `logs` | Where the `.npz` file is saved (created if missing) |
| `run_id` | `''` | The run's name, which starts the file name; empty: the node's start time (the launch files set it) |

Viewer Node (`viewer_node`):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `snapshot_distances` | `[0.0, 0.5, 1.0]` | Travel at which to save an image, m |
| `distance_tolerance` | `0.03` | This far short of a snapshot distance still reaches it, m |
| `image_scale` | `3` | Image pixels per cell side |
| `display` | `true` | Show the image in a window (skipped without a display) |
| `save_images` | `true` | Save the snapshots and `<run>_grid_final.png` |
| `save_period` | `10.0` | Seconds between `<run>_grid_final.png` saves while the image changes (and on Ctrl-C); `0.0`: on Ctrl-C only |
| `result_dir` | `result` | The images are saved in its `grid/` folder (created if missing) |
| `run_id` | `''` | The run's name, which starts the file names; empty: the node's start time (the launch files set it) |

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

## Troubleshooting

| Problem | Check or fix |
| --- | --- |
| `ModuleNotFoundError: No module named 'skimage'` (or `cv2`) | Install the dependencies (see [Install dependencies](#install-dependencies)) |
| The Grid Node keeps logging `Waiting for transform base_link -> rplidar_link` | TF is not reaching the laptop: `ros2 run tf2_ros tf2_echo base_link rplidar_link`; or run with `-p use_tf:=false` to use `mount_yaw` (90°) |
| No grid updates | The robot's topics arrive: `ros2 topic list -t \| grep -E 'scan\|odom'`, `timeout 10 ros2 topic hz /scan` |
| Nothing drawn or no landmarks | This package's topics are published: `timeout 5 ros2 topic echo /grid --once --no-arr`, `timeout 5 ros2 topic echo /landmarks --once` |
