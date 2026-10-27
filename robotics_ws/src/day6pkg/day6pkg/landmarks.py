"""
Corner landmarks in an occupancy grid (Project 7, Task 2; Day 7 slides 39-48)
Wall segments by RANSAC (scikit-image), then corners where two segments meet with data up
to the crossing; the landmark node runs this on every new grid
Pure NumPy and scikit-image (no ROS), so it can be tested and used without the robot
Points and segments are world (x, y) in meters
"""

import math

import numpy as np


def occupied_points(grid, threshold=1.0):
    """
    Centers (x, y) of the cells with log-odds above threshold, as M x 2
    threshold 1.0: at least two more scans ended a ray in the cell than passed through it
    """
    rows, cols = np.nonzero(grid.log_odds > threshold)
    half = grid.size // 2
    return np.column_stack([(half - 0.5 - rows) * grid.resolution,
                            (half - 0.5 - cols) * grid.resolution])


def ransac_line(points, rng, threshold, sample_radius, trials):
    """
    Best line through points by scikit-image's RANSAC: ((origin, direction), inlier mask),
    or (None, None) if there is none
    Each try takes two random points at most sample_radius apart (slide 45: 'nearby' points)
    and counts the points within threshold of the line through them
    """
    from skimage.measure import LineModelND, ransac   # only this function needs scikit-image

    def nearby(sample):
        return np.linalg.norm(sample[0] - sample[1]) <= sample_radius

    options = dict(min_samples=2, residual_threshold=threshold, max_trials=trials,
                   is_data_valid=nearby)
    try:
        model, inliers = ransac(points, LineModelND, rng=rng, **options)
    except TypeError:                                 # scikit-image before 0.21
        model, inliers = ransac(points, LineModelND, random_state=rng, **options)
    if not model or inliers is None:
        return None, None
    if hasattr(model, 'origin'):                      # scikit-image 0.26 and later
        return (np.asarray(model.origin), np.asarray(model.direction)), inliers
    origin, direction = model.params                  # earlier versions
    return (np.asarray(origin), np.asarray(direction)), inliers


def longest_run(points, line, mask, max_gap):
    """
    Mask of the longest unbroken run (no gap over max_gap) of the masked points along line
    RANSAC counts every point near the infinite line; a wall is one run of them
    """
    origin, direction = line
    idx = np.flatnonzero(mask)
    order = idx[np.argsort((points[idx] - origin) @ direction)]
    t = (points[order] - origin) @ direction
    runs = np.split(order, np.flatnonzero(np.diff(t) > max_gap) + 1)
    run = np.zeros(len(points), dtype=bool)
    run[max(runs, key=len)] = True
    return run


def find_segments(points, threshold=0.06, sample_radius=0.5, min_points=8, min_length=0.3,
                  max_gap=0.15, trials=2000, max_failures=5, seed=0):
    """
    Wall segments [(start, end), ...] in points (slides 42-45): find the best line, keep its
    longest unbroken run as a segment if it has at least min_points points and min_length m,
    remove the run's points, and repeat until max_failures runs in a row are too small
    seed: fixed, so the same grid always gives the same segments
    """
    rng = np.random.default_rng(seed)
    remaining = points
    segments = []
    failures = 0
    while len(remaining) >= min_points and failures < max_failures:
        line, inliers = ransac_line(remaining, rng, threshold, sample_radius, trials)
        if line is None:
            break
        run = longest_run(remaining, line, inliers, max_gap)
        origin, direction = line
        t = (remaining[run] - origin) @ direction
        if run.sum() >= min_points and t.max() - t.min() >= min_length:
            segments.append((origin + t.min() * direction, origin + t.max() * direction))
            failures = 0
        else:
            failures += 1
        remaining = remaining[~run]
    return segments


def find_corners(segments, min_angle=60.0, max_distance=0.15, merge_distance=0.1):
    """
    Corner landmarks (K x 2, sorted by x then y) where two segments cross at min_angle
    degrees or more, and both end within max_distance of the crossing, so that the data
    reaches the corner on both walls (slide 48); corners closer than merge_distance merge
    """
    corners = []
    for k, (a0, a1) in enumerate(segments):
        for b0, b1 in segments[k + 1:]:
            u, v = a1 - a0, b1 - b0
            sine = abs(u[0] * v[1] - u[1] * v[0]) / (np.linalg.norm(u) * np.linalg.norm(v))
            if sine < math.sin(math.radians(min_angle)):
                continue                              # too close to parallel
            s = np.linalg.solve(np.column_stack([u, -v]), b0 - a0)[0]
            x = a0 + s * u                            # where the two lines cross
            if (min(np.linalg.norm(x - a0), np.linalg.norm(x - a1)) <= max_distance and
                    min(np.linalg.norm(x - b0), np.linalg.norm(x - b1)) <= max_distance):
                corners.append(x)
    merged = []
    for c in corners:
        for group in merged:
            if np.linalg.norm(np.mean(group, axis=0) - c) < merge_distance:
                group.append(c)
                break
        else:
            merged.append([c])
    points = [np.mean(group, axis=0) for group in merged]
    return np.array(sorted(points, key=lambda p: (p[0], p[1]))).reshape(-1, 2)
