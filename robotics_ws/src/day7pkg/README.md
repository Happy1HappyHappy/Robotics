# day7pkg

Python package for CS 5335 Project 7 (Landmarks and Path Planning). It works on the
occupancy grids that [`day6pkg`](../day6pkg/README.md)'s Grid Node saves as `grid.npz`:

```
 Task 1   day6pkg Grid Node, run twice     ──> run_a/grid.npz, run_b/grid.npz
 Task 2   ros2 run day7pkg landmarks A B   ──> landmarks_a.png, landmarks_b.png,
                                               landmarks_overlay.png, landmarks.txt
 Task 3   ros2 run day7pkg plan A --goal   ──> path.png, path.txt
```

`landmarks` and `plan` are command-line tools, not ROS nodes: they read saved files, so
they run without the robot and never move it. The algorithms are in `landmarks.py`
(RANSAC, corners, lining up two grids) and `planner.py` (A*), the images in `draw.py`.

## Build

The tools need `day6pkg` (for the grid), NumPy and OpenCV. From the workspace root
(`robotics_ws`):

```bash
colcon build --packages-select day6pkg day7pkg
source install/setup.bash
```

Rebuild both whenever a Python file changes.

## Task 1: two occupancy grids

Drive at least 3 m away from the start (straight: 3.2 m), turn around 180 degrees, and drive
back along the same path. The Grid Node runs once for each way, so each run gets its own
grid and frame: origin at the sensor's position when that run started, x along the robot's
heading then. Grid B is therefore about grid A turned 180 degrees and moved 3.3 m.

Set up first:

- A straight strip of floor about 3.5 m long, in a room with walls and corners nearby.
- Two or three boxes 0.5 to 1 m beside the path, not on it. Their corners are landmarks
  for Task 2, and one of them is the obstacle for Task 3.
- Nobody walking in view of the LiDAR during the runs: people become obstacles and
  false landmarks.
- Mark the start on the floor with tape.

Then, with the robot undocked and on the tape facing along the path (one terminal for the
Grid Node, one for driving):

1. Terminal 1: start the Grid Node for the run there, and wait for `Update 1`:

   ```bash
   ros2 run day6pkg mapper --ros-args -p output_dir:=run_a -p size:=300 -p snapshot_distances:="[0.0, 1.0, 2.0, 3.0]"
   ```

2. Terminal 2: drive 3.2 m at 0.1 m/s:

   ```bash
   ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 3.2, max_translation_speed: 0.1}"
   ```

3. Terminal 1: when it has stopped, press Ctrl-C. The Grid Node saves `run_a/grid.npz`
   and `run_a/grid_final.png`. Stop it before turning, so the turn is not in grid A and
   grid B starts with its own frame.

4. Terminal 2: turn around:

   ```bash
   ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 3.14159, max_rotation_speed: 0.5}"
   ```

5. Terminal 1: start the Grid Node for the run back, and wait for `Update 1`:

   ```bash
   ros2 run day6pkg mapper --ros-args -p output_dir:=run_b -p size:=300 -p snapshot_distances:="[0.0, 1.0, 2.0, 3.0]"
   ```

6. Terminal 2: drive back the same 3.2 m (step 2's command again).

7. Terminal 1: Ctrl-C. Now `run_b/grid.npz` is saved too.

Notes:

- `size:=300` makes the grid 15 m across, so walls up to 7.5 m from the start still fit.
- The grid is updated after every 0.1 m of travel (the Grid Node's `update_distance`), with
  5 cm cells; the logs show `Update 1` to `Update 33`.
- Do not restart the robot or reset its pose between the runs: Task 2 compares the
  landmarks with odometry's estimate of where grid B sits in grid A, which needs both runs
  to share the odom frame.
- The Grid Node measures travel as the straight-line distance from the start, so a straight
  path is the simplest. A curved one would need that changed to the path length.
- Worth recording everything once, to redo the grids later without the robot:
  `ros2 bag record /scan /odom /tf /tf_static`.

## Task 2: landmarks

```bash
ros2 run day7pkg landmarks run_a/grid.npz run_b/grid.npz --output-dir results
```

How it finds and compares the landmarks:

1. Points: the centers of the cells with log-odds above `--occupied` (1.0: at least two
   more scans ended a ray in the cell than passed a ray through it).
2. Line segments by RANSAC (Day 7 slides 42-45): pick a random point and a random other
   point within `--sample-radius`, take the line through them, and collect the unbroken run
   of points within `--inlier-distance` of it. With at least `--min-points`, refit the line
   (least squares), collect the run again, and keep it if it is at least `--min-length`
   long. Remove its points and repeat until `--max-failures` tries in a row find nothing.
3. Corners (slide 48): two segments whose lines cross at `--corner-angle` (60 degrees) or
   more, and that both end within `--corner-distance` of the crossing. The crossing point
   is the landmark: salient, stationary, and localizable to the grid's resolution.
4. Lining up B with A, as the assignment's hint suggests: for each pair of landmarks (one
   from A, one from B), move B so the two coincide, try every rotation in 1-degree steps,
   and keep the one under which the most landmarks of B land within `--match-distance` of
   one of A (ties: the smallest total distance). Then refine it by a least-squares fit to
   the matched pairs. Only rotations within 30 degrees and translations within 1 m of
   odometry's estimate are tried, because a rectangular room matches just as well turned
   the wrong way round; `--no-prior` searches everything.
5. Comparing: landmarks of A and B that are each other's nearest neighbor and within
   `--match-distance` after lining up are the same landmark. The rest are only in one grid,
   e.g. a box corner seen from one direction only.

What it writes to `--output-dir`, for the instructor:

| File | Contents |
| --- | --- |
| `landmarks_a.png`, `landmarks_b.png` | Each grid with its RANSAC segments (orange) and landmarks (red, `A1`, `A2`, ... / `B1`, ...) |
| `landmarks_overlay.png` | Grid B moved into A's frame on top of A: occupied in B only = light blue, in both = purple; A's landmarks red circles, B's blue squares, matched pairs joined in green |
| `landmarks.txt` | The report below |

The report (here from the synthetic test runs, where odometry was given 5 cm and 2 degrees
of error):

```
Odometry:  B's frame in A's frame is at x=3.280 m, y=0.000 m, theta=180.0 deg
Landmarks: B's frame in A's frame is at x=3.327 m, y=-0.040 m, theta=181.9 deg (5 landmarks matched)
Difference (odometry error over the two runs): 0.062 m, 1.9 deg
...
Found in both (distance between the two positions in A's frame):
            lined up   odometry only
  A1  = B6    0.030 m       0.234 m
  ...
Mean difference in position: 0.022 m after lining up the landmarks (0.117 m with odometry only)
Largest difference: 0.039 m
Only in A: A4, A5
Only in B: B4
```

- Landmarks in B are listed in B's own frame and moved into A's frame.
- "lined up" is the mean average difference the assignment asks for: what is left after
  the best rigid transform, from the grid's resolution and from odometry drifting within
  each run.
- "odometry only" places grid B where odometry says, so its difference also shows the
  odometry error built up over the runs.
- If fewer than 2 landmarks match, the report says so and compares with odometry instead.

## Task 3: A* path

```bash
ros2 run day7pkg plan run_a/grid.npz --goal 2.2 0.7 --output-dir results
```

The goal is in grid A's frame (x = the robot's heading at the start, y = its left, in
meters). Pick one at least 2 m from the start, behind one of the boxes, so the path has to
bend; the start is the grid's origin, where the robot started.

How it plans (Day 7 slides 16-26):

1. Blocked cells: every cell not known to be free (unknown cells are avoided), and every
   cell within `--robot-radius` + `--margin` (0.17 + 0.05 m) of an occupied cell. Growing
   the obstacles by the robot's radius accounts for its width, so A* can plan for its
   center as a point.
2. A* over the free cells, 8-connected (side steps cost 1, diagonals sqrt 2, no cutting
   past a blocked corner), ordered by f = g + h with h the straight-line distance to the
   goal.
3. Path optimization: from each waypoint, go straight to the farthest later cell of the
   path that has a clear line to it.

It prints whether the straight line from start to goal is blocked, the A* path, the
shortened path with its bends, and the turns and drives that would follow it in
`day3pkg`'s Executive (`t <deg>`, `m <meters>`; the robot must face +x at the start). The
assignment only asks to show the plan, so driving it is optional.

`path.png`: the grid, the cells too close to an obstacle for the robot's center (light
red), the A* cells (light blue), the shortened path (green), the start (green circle) and
the goal (red cross). `path.txt` has the printed plan.

## Testing without the robot

The tests build synthetic grids with a simulated LiDAR in a room with two boxes
(`test/synthetic.py`). They check RANSAC, corners, lining up, A* and both tools. From this
package's directory (`robotics_ws/src/day7pkg`):

```bash
PYTHONPATH=../day6pkg:. python3 -m pytest test/test_landmarks.py test/test_planner.py test/test_cli.py
```

The same synthetic files can be written with `synthetic.save_pair(<folder>)` to try the
tools by hand. `colcon test --packages-select day7pkg` runs the tests together with the
ROS linters (flake8, pep257), as for the other packages.

## Check on the robot first

These were tested on synthetic grids only:

| What | How |
| --- | --- |
| The Grid Node saves `grid.npz` on Ctrl-C | `ls run_a` after step 3 of Task 1 |
| The turn goal's fields | `ros2 interface show irobot_create_msgs/action/RotateAngle` |
| Odometry is not reset by the turn | `timeout 5 ros2 topic echo /odom --once` before and after it |
| The TurtleBot 4 is about 0.34 m across (`--robot-radius 0.17`) | The TurtleBot 4 user manual, or measure it |
| RANSAC's defaults suit the real grids | Look at `landmarks_a.png`; with too many short or missing segments, try `--occupied`, `--min-points`, `--inlier-distance` |
