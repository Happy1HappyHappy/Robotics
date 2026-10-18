# Robotics math (CS 5335, Days 3–6)

Notation follows the slides. ξ = [x, y, θ] is a pose. ^A T_B is the transform that maps points
from frame {B} into frame {A}, which is also the pose of {B} expressed in {A}. Use NumPy
(`python3-numpy`) for matrices and `math` for scalars.

## Contents

- [2D poses and transforms](#2d-poses-and-transforms)
- [Motion relative to a start pose](#motion-relative-to-a-start-pose)
- [Quaternion and yaw](#quaternion-and-yaw)
- [Differential drive](#differential-drive)
- [Pose update](#pose-update)
- [Odometry closed-loop driving](#odometry-closed-loop-driving)
- [Error models](#error-models)
- [Kalman filter](#kalman-filter)
- [Occupancy grid from LiDAR](#occupancy-grid-from-lidar)

## 2D poses and transforms

A homogeneous 2D transform for the pose (t_x, t_y, θ):

```
        [ cos θ  -sin θ  t_x ]
^A T_B = [ sin θ   cos θ  t_y ]        ^A P = ^A T_B · ^B P     (P = [x, y, 1]^T)
        [   0       0     1  ]
```

- Chain transforms by post-multiplying: ^W T_C = ^W T_A · ^A T_B · ^B T_C. They do not commute.
- The inverse of a pose is the inverse matrix (not the negation):
  R^-1 = R^T and t' = −R^T t.
- Transforming **points** from {R} to {W} is the same as moving the **frame** from {W} to {R}.
  Transforming coordinates the other way uses the inverse. The slides warn that texts differ
  on this, so check which one a formula means.
- 3D (Day 4): 4×4 matrices with a 3×3 rotation. Roll, pitch and yaw rotate about x, y and z.
  ROS stores rotations as quaternions.

```python
import numpy as np

def transform(x, y, theta):
    """
    Homogeneous 3x3 matrix for a 2D pose (meters, radians)
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])
```

## Motion relative to a start pose

Given odometry poses (x0, y0, θ0) at the start and (x1, y1, θ1) at the end, the motion expressed
in the robot's **start** frame (the translation part of inv(T_start) · T_end, as in
`driver.py`'s `report_motion()`):

```
dx, dy  = x1 − x0, y1 − y0
forward =  cos θ0 · dx + sin θ0 · dy
lateral = −sin θ0 · dx + cos θ0 · dy
dyaw    = wrap(θ1 − θ0)            wrap(a) = (a + π) mod 2π − π
```

Errors are reported as odometry minus target in this frame.

## Quaternion and yaw

For 2D, where roll = pitch = 0:

```python
def yaw_from_quaternion(q):
    """
    Yaw (rad) from a geometry_msgs/Quaternion
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

# yaw -> quaternion: x = y = 0, z = sin(yaw / 2), w = cos(yaw / 2)
```

The course's way is `tf_transformations.euler_from_quaternion` / `quaternion_from_euler` (see
the TF2 section of the patterns reference). The helper above gives the same yaw without the
extra package.

## Differential drive

The robot frame has x forward and y left. b = wheel separation = **0.233 m** on the Create 3.
The wheels move at v_L and v_R (m/s).

**Forward kinematics** (wheel speeds for time t, to motion):

```
v = (v_R + v_L) / 2          linear speed          D = v t
ω = (v_R − v_L) / b          angular speed         θ = ω t
c = D / θ = v / ω            turn radius (∞ if v_R = v_L; 0 if v_L = −v_R)
```

**Inverse kinematics** (desired motion, to commands):

| Motion | Wheel speeds | Time |
| --- | --- | --- |
| Straight, distance D | v_R = v_L = v | t = D / v |
| Arc, radius c, angle θ | v_R = v (1 + b / 2c), v_L = v (1 − b / 2c) | t = c θ / v |
| Turn in place, angle θ | v_R = −v_L = ω b / 2 | t = θ / ω |

**`cmd_vel` mapping**: `linear.x` = v and `angular.z` = ω. The robot converts these to
`v_L = v − ω b / 2` and `v_R = v + ω b / 2`. For an arc, send v and ω = ±v / c (+ is left). If
v / c would exceed 1.9 rad/s, lower v. Also check v_R ≤ 0.306 m/s.

## Pose update

Over one small step with distance ΔD and turn Δθ (from commands or odometry), the standard
midpoint update is:

```
x ← x + ΔD cos(θ + Δθ / 2)
y ← y + ΔD sin(θ + Δθ / 2)
θ ← wrap(θ + Δθ)
```

Even when you only command x, cross-track motion (y) and rotation happen, so track the full
pose.

## Odometry closed-loop driving

Open loop (sum the commands and time it) accumulates error quickly. Closed loop uses the
odometry sensor. The Day 5 algorithm for driving distance D at speed v, with odometry reports
every t_M (about 0.05 s for 20 Hz `/odom`):

1. Set `remaining = D` and command v. If `D < v · t_M`, also start a one-shot timer for
   `remaining / v`.
2. On each odometry report, subtract the distance moved since the last report (the Euclidean
   step, or the path length for curves). If `remaining < v · t_M`, start the timer for
   `remaining / v`.
3. When the timer fires, command v = 0.
4. Wait for the next odometry report, which confirms the stop, before starting the next
   operation.

For curved paths, measure the path length the same way. At each inflection point (where the
radius changes), use a timer to switch velocities without stopping.

The Create 3 actions (`drive_distance` etc.) already do this on the robot. Write your own loop
only when the assignment asks for it.

## Error models

- **Accuracy** means bias, how far the mean is from the truth. **Precision** means spread, the
  standard deviation.
- The motion error model is linear in distance: mean error = b + g·D and
  Var(D) = k1 + k2·D. Over the small steps of odometry, the constants b and k1 usually suffice.
- Use `statistics.mean` / `statistics.stdev` (sample std) over repeated runs, as `driver.py`
  does for its last-20 table.
- In dead reckoning, the uncertainty grows with the number of steps (variance is additive for
  independent steps). With zero-mean error the expected final position stays unbiased.
- Covariance: Cov(x, y) = E[xy] − E[x]E[y]. It can be negative.
  Var(x + y) = Var(x) + Var(y) + 2 Cov(x, y). The eigenvectors and eigenvalues of a covariance
  matrix give the directions and sizes of the largest and smallest spread.
- Gaussian: about 68% of samples fall within 1σ of the mean.

## Kalman filter

| Symbol | Meaning |
| --- | --- |
| x, P | State and its covariance |
| F | Process matrix |
| u, B | Control vector and matrix |
| Q | Process noise covariance |
| z, R | Measurement and its covariance |
| H | State-to-measurement matrix |
| K | Kalman gain |

```
Predict:  x⁻ = F x + B u
          P⁻ = F P Fᵀ + Q
Update:   K  = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹
          x  = x⁻ + K (z − H x⁻)
          P  = (I − K H) P⁻
```

The update puts the estimate between the prediction and the measurement, weighted toward
whichever has the smaller variance. **EKF**: predict with x⁻ = f(x, u) and use h(x⁻) in place
of H x⁻ in the innovation, with F and H being the **Jacobians** of f and h, evaluated each step.

```python
import numpy as np

def kf_step(x, P, u, z, F, B, Q, H, R):
    """
    One Kalman filter predict + update cycle; returns the new (x, P)
    """
    x = F @ x + B @ u
    P = F @ P @ F.T + Q
    K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
    x = x + K @ (z - H @ x)
    P = (np.eye(len(x)) - K @ H) @ P
    return x, P
```

## Occupancy grid from LiDAR

- The grid is a 2D NumPy array over the world (e.g. 5 m × 5 m at 1 cm gives 500 × 500). Each
  cell stores **log-odds** L = log(p / (1 − p)), starting at 0 (p = 0.5).
- For each scan ray (r, θ) with r inside [range_min, range_max]:
  1. Get the endpoint in the sensor frame: ^S P = (r cos θ, r sin θ).
  2. Transform the sensor origin and the endpoint into the world/odom frame with ^W T_S from
     TF2, at the scan's timestamp.
  3. Convert both to grid indices: `i = int((x − origin_x) / resolution)`, and the same for j.
  4. Ray-cast from the origin cell to the endpoint cell. Add the "empty" log-odds to the cells
     passed through and the "filled" log-odds to the endpoint.
- The slides use a base-10 log: empty p = 0.1 gives L += log10(0.1/0.9) ≈ −0.95; filled p = 0.9
  gives L += log10(9) ≈ +0.95. Recover the probability with p = 1 − 1 / (1 + 10^L).
- Clamp L to a range so moving objects can change state again later.
- **Bresenham** (integer-only ray casting). The slides show the |di| ≥ |dj| case; this version
  handles every direction:

```python
def bresenham(i0, j0, i1, j1):
    """
    Grid cells from (i0, j0) up to but not including (i1, j1)
    """
    cells = []
    di, dj = abs(i1 - i0), abs(j1 - j0)
    si, sj = (1 if i1 > i0 else -1), (1 if j1 > j0 else -1)
    err = di - dj
    i, j = i0, j0
    while (i, j) != (i1, j1):
        cells.append((i, j))
        e2 = 2 * err
        if e2 > -dj:
            err -= dj
            i += si
        if e2 < di:
            err += di
            j += sj
    return cells
```

- The slides' refinements: also update cells the beam just "nicks" (weighted by distance to the
  beam line), and remember that objects smaller than a cell can be missed.
- To display the grid: `cv2.normalize(grid, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)`,
  then `cv2.imshow(...)` and `cv2.waitKey(0)` (required), then `cv2.destroyAllWindows()`.
