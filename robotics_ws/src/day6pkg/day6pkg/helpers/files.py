"""
Saving files so that a reader never finds half of one, even if the program is killed while
it writes: write a temporary file next to the target, then rename it over the target
(os.replace is atomic on one file system, so the old file stays until the new one is whole)
Each run's files are named after the run, like log files: <run>_grid_final.npz, ...
Pure NumPy (no ROS); images are saved with render.save_png
"""

import os
import time

import numpy as np

RUN_FORMAT = '%Y-%m-%d_%H-%M-%S'    # a run is named after its start: 2026-10-27_16-40-12


def new_run_name():
    """
    Name for a new run: the local time now (RUN_FORMAT), e.g. 2026-10-27_16-40-12, so two
    runs never share a name and the files sort by time
    """
    return time.strftime(RUN_FORMAT)


def run_name(run_id=''):
    """
    Name of this run, which starts every file it saves: run_id as given (the launch files
    name the run once with new_run_name(), so all its nodes share it), else a new name
    """
    return run_id or new_run_name()


def save_npz(path, **arrays):
    """
    Write the arrays to path as a NumPy .npz file, each under its name
    """
    tmp = f'{path}.tmp'
    try:
        with open(tmp, 'wb') as f:
            np.savez(f, **arrays)
        os.replace(tmp, path)
    except BaseException:           # e.g. the disk is full: keep the old file, drop the new
        if os.path.exists(tmp):
            os.remove(tmp)
        raise
