"""
Landmarks in an occupancy grid (Project 7, Task 2): line segments found one at a time with
RANSAC, corners where two segments meet, and the rigid transform that lines up the corners
of two grids
Pure NumPy (no ROS), so it can be tested and used without the robot
Points and transforms are in meters and radians; a transform (x, y, theta) maps a point p
of one frame to R(theta) p + (x, y) in the other, like ^A T_B in the slides
"""

from dataclasses import dataclass
import itertools
import math

import numpy as np


@dataclass
class Segment:
    """
    A line segment found by RANSAC: its two end points (meters) and how many points lie on it
    """
    start: np.ndarray
    end: np.ndarray
    count: int

    @property
    def direction(self):
        """
        Unit vector from start to end
        """
        d = self.end - self.start
        return d / np.linalg.norm(d)

    @property
    def length(self):
        """
        Length in meters
        """
        return float(np.linalg.norm(self.end - self.start))


def occupied_points(grid, threshold=1.0):
    """
    World points (N x 2, meters) at the centers of the cells whose log-odds exceed threshold
    Each hit adds about 0.95 and each pass through takes it off again, so the default 1.0
    keeps cells with at least two more hits than passes, which drops most single stray hits
    """
    rows, cols = np.nonzero(grid.log_odds > threshold)
    x, y = grid.cell_to_world(rows, cols)
    return np.column_stack([x, y])


def fit_line(points):
    """
    Best-fit line through points (N x 2), as (a point on the line, unit direction)
    Total least squares: the direction in which the points spread the most, so vertical
    lines work as well as horizontal ones
    """
    center = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - center)
    return center, vt[0]


def _run_around(t, t0, max_gap):
    """
    Indices into t of the run of values that contains the value closest to t0, where a gap
    larger than max_gap between neighbors (in sorted order) ends the run
    Empty if no value is within max_gap of t0
    """
    order = np.argsort(t)
    ts = t[order]
    k = int(np.argmin(np.abs(ts - t0)))
    if abs(ts[k] - t0) > max_gap:
        return np.array([], dtype=int)
    lo, hi = k, k
    while lo > 0 and ts[lo] - ts[lo - 1] <= max_gap:
        lo -= 1
    while hi < len(ts) - 1 and ts[hi + 1] - ts[hi] <= max_gap:
        hi += 1
    return order[lo:hi + 1]


def _points_on_segment(points, origin, direction, inlier_distance, max_gap):
    """
    Indices of the points within inlier_distance of the line through origin along direction,
    keeping only the unbroken run of them around origin (a gap of more than max_gap along
    the line separates two segments of the same line, e.g. two walls on either side of a door)
    """
    d = direction / np.linalg.norm(direction)
    normal = np.array([-d[1], d[0]])
    rel = points - origin
    on = np.nonzero(np.abs(rel @ normal) <= inlier_distance)[0]
    if len(on) == 0:
        return on
    return on[_run_around(rel[on] @ d, 0.0, max_gap)]


def ransac_segments(points, sample_radius=0.5, inlier_distance=0.05, min_points=8,
                    min_length=0.3, max_gap=0.15, max_failures=300, seed=0):
    """
    Line segments in points (N x 2, meters), found one at a time by RANSAC (Day 7 slides 42-45):
    pick a random point and a random other point within sample_radius of it, take the line
    through them, and collect the unbroken run of points within inlier_distance of it; if
    the run has at least min_points, refit the line to them (least squares), collect the run
    again, and accept it if it is at least min_length long; remove its points and repeat
    Stops after max_failures tries in a row find nothing, or when too few points are left
    seed makes the random choices repeatable, so the same grid gives the same segments
    Returns the segments in the order found
    """
    rng = np.random.default_rng(seed)
    remaining = np.asarray(points, dtype=float)
    segments = []
    failures = 0
    while failures < max_failures and len(remaining) >= min_points:
        failures += 1
        i = rng.integers(len(remaining))
        d = np.linalg.norm(remaining - remaining[i], axis=1)
        near = np.nonzero((d > 0.0) & (d <= sample_radius))[0]
        if len(near) == 0:
            continue
        j = rng.choice(near)
        members = _points_on_segment(remaining, remaining[i], remaining[j] - remaining[i],
                                     inlier_distance, max_gap)
        if len(members) < min_points:
            continue
        # refit to the run, then collect the points on the refit line
        center, direction = fit_line(remaining[members])
        members = _points_on_segment(remaining, center, direction, inlier_distance, max_gap)
        if len(members) < min_points:
            continue
        center, direction = fit_line(remaining[members])
        t = (remaining[members] - center) @ direction
        if t.max() - t.min() < min_length:
            continue
        segments.append(Segment(center + t.min() * direction, center + t.max() * direction,
                                len(members)))
        remaining = np.delete(remaining, members, axis=0)
        failures = 0
    return segments


def find_corners(segments, min_angle=math.radians(60.0), max_distance=0.15):
    """
    Corners where two segments meet (Day 7 slide 48): the intersection of the two lines, kept
    if the lines cross at min_angle or more and each segment ends within max_distance of the
    intersection, so the data runs up to the corner on both sides
    Corners closer together than max_distance are merged into their average
    Returns an N x 2 array of corners (meters), sorted by x then y so the labels are repeatable
    """
    found = []
    for a, b in itertools.combinations(segments, 2):
        da, db = a.direction, b.direction
        cross = da[0] * db[1] - da[1] * db[0]       # sin of the angle between the lines
        if abs(cross) < math.sin(min_angle):
            continue
        # a.start + s * da = b.start + u * db, solved for s with 2D cross products
        w = b.start - a.start
        p = a.start + (w[0] * db[1] - w[1] * db[0]) / cross * da
        if (min(np.linalg.norm(p - a.start), np.linalg.norm(p - a.end)) <= max_distance and
                min(np.linalg.norm(p - b.start), np.linalg.norm(p - b.end)) <= max_distance):
            found.append(p)

    clusters = []       # [sum of points, count]
    for p in found:
        for cluster in clusters:
            if np.linalg.norm(cluster[0] / cluster[1] - p) <= max_distance:
                cluster[0] = cluster[0] + p
                cluster[1] += 1
                break
        else:
            clusters.append([p.copy(), 1])
    corners = np.array([s / n for s, n in clusters]).reshape(-1, 2)
    return corners[np.lexsort((corners[:, 1], corners[:, 0]))]


def transform_points(transform, points):
    """
    Apply transform (x, y, theta) to points (N x 2): R(theta) p + (x, y)
    """
    x, y, theta = transform
    c, s = math.cos(theta), math.sin(theta)
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    return np.column_stack([c * p[:, 0] - s * p[:, 1] + x, s * p[:, 0] + c * p[:, 1] + y])


def compose(t1, t2):
    """
    Transform t1 after t2, both (x, y, theta): the 2D version of T1 @ T2
    """
    x, y = transform_points(t1, [t2[:2]])[0]
    return float(x), float(y), wrap_angle(t1[2] + t2[2])


def invert(transform):
    """
    Inverse of transform (x, y, theta)
    """
    x, y, theta = transform
    c, s = math.cos(theta), math.sin(theta)
    return -c * x - s * y, s * x - c * y, wrap_angle(-theta)


def wrap_angle(a):
    """
    Angle a wrapped to [-pi, pi)
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rigid_fit(src, dst):
    """
    Transform (x, y, theta) that best maps the points src onto the points dst (N x 2 each,
    N >= 2, row k of src goes with row k of dst), in the least-squares sense
    The rotation is the angle that best lines up the two point sets around their centroids;
    the translation then moves the rotated src centroid onto the dst centroid
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    s, d = src - sc, dst - dc
    theta = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]),
                       np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    x, y = dc - transform_points((0.0, 0.0, theta), [sc])[0]
    return float(x), float(y), theta


def match(a, b, max_distance):
    """
    Pairs (i, j, distance) of points a[i] and b[j] (both in the same frame) that are each
    other's nearest neighbor and at most max_distance apart
    """
    a = np.asarray(a, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1, 2)
    if len(a) == 0 or len(b) == 0:
        return []
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    pairs = []
    for i in range(len(a)):
        j = int(np.argmin(d[i]))
        if int(np.argmin(d[:, j])) == i and d[i, j] <= max_distance:
            pairs.append((i, j, float(d[i, j])))
    return pairs


def align(a, b, max_distance=0.3, angle_step=math.radians(1.0), prior=None,
          prior_angle=math.radians(30.0), prior_distance=1.0):
    """
    Transform (x, y, theta) that maps landmarks b (M x 2, in their own frame) onto landmarks
    a (N x 2), found the way the assignment's hint suggests: for every pair (a[i], b[j]),
    translate b so that b[j] lands on a[i], try every rotation about that point in steps of
    angle_step, and keep the one under which the most landmarks of b land within
    max_distance of a landmark of a (ties: the smallest total distance); then refine it with
    a least-squares fit to the matched pairs, re-matching until it settles
    prior: an estimate of the transform, e.g. from odometry; if given, only candidates within
    prior_angle and prior_distance of it are tried, so a symmetric room (a rectangle looks the
    same turned 180 degrees) cannot be matched the wrong way round
    Returns (transform, matches as in match()), or (None, []) if fewer than 2 landmarks match
    """
    a = np.asarray(a, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1, 2)
    angles = np.arange(0.0, 2.0 * math.pi, angle_step)
    c, s = np.cos(angles), np.sin(angles)
    if prior is not None:
        keep = np.abs([wrap_angle(t - prior[2]) for t in angles]) <= prior_angle
        angles, c, s = angles[keep], c[keep], s[keep]

    best = None     # (count, total distance, transform)
    for i, j in itertools.product(range(len(a)), range(len(b))):
        rel = b - b[j]
        # b turned by every angle about b[j], then moved so b[j] is on a[i]: K x M
        bx = c[:, None] * rel[:, 0] - s[:, None] * rel[:, 1] + a[i, 0]
        by = s[:, None] * rel[:, 0] + c[:, None] * rel[:, 1] + a[i, 1]
        # distance from each moved landmark of b to its nearest landmark of a: K x M
        nearest = np.hypot(bx[:, :, None] - a[:, 0], by[:, :, None] - a[:, 1]).min(axis=2)
        ok = nearest <= max_distance
        count = ok.sum(axis=1)
        total = np.where(ok, nearest, 0.0).sum(axis=1)
        for k in np.lexsort((total, -count)):
            # translation that puts b[j] on a[i] after turning by angles[k]
            tx = a[i, 0] - (c[k] * b[j, 0] - s[k] * b[j, 1])
            ty = a[i, 1] - (s[k] * b[j, 0] + c[k] * b[j, 1])
            if prior is None or math.hypot(tx - prior[0], ty - prior[1]) <= prior_distance:
                if best is None or (count[k], -total[k]) > (best[0], -best[1]):
                    best = (int(count[k]), float(total[k]), (tx, ty, float(angles[k])))
                break
    if best is None or best[0] < 2:
        return None, []

    transform = best[2]
    for _ in range(20):
        pairs = match(a, transform_points(transform, b), max_distance)
        if len(pairs) < 2:
            return None, []
        new = rigid_fit(b[[j for _, j, _ in pairs]], a[[i for i, _, _ in pairs]])
        settled = (math.hypot(new[0] - transform[0], new[1] - transform[1]) < 1e-6 and
                   abs(wrap_angle(new[2] - transform[2])) < 1e-9)
        transform = new
        if settled:
            break
    return transform, match(a, transform_points(transform, b), max_distance)
