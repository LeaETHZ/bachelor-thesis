import numpy as np
import math
from environment import Cylinder


def load_cylinder_from_npy(path: str):
    """
    Load a semantic 2D numpy array (H x W) into a Cylinder environment.

    Map encoding:
        0 -> free space
        1 -> obstacle
        2 -> background (ignored)

    The map already has the correct resolution:
        - W determines cylinder circumference
        - H determines cylinder height
    """

    arr = np.load(path)
    arr = add_background_border_as_obstacles(arr, background_val=2, obstacle_val=1)


    # Allow RGB (just in case)
    if arr.ndim == 3 and arr.shape[2] == 3:
        arr = arr.mean(axis=2)

    if arr.ndim != 2:
        raise ValueError(f"Expected 2D array in {path}, got shape {arr.shape}")

    H, W = arr.shape

    # Physical dimensions directly from map size
    height = H
    radius = W / (2 * math.pi)

    # Create cylinder
    env = Cylinder(radius, height)

    # Grid ranges match map
    env.x_range = W
    env.y_range = H

    # Obstacles are exactly value == 1
    obstacles = {
        (x, y)
        for y in range(H)
        for x in range(W)
        if arr[y, x] == 1
    }

    env.obstacles = obstacles
    env.update(env.obstacles)

    return env




def add_background_border_as_obstacles(arr: np.ndarray, background_val=2, obstacle_val=1):
    bg = (arr == background_val)

    # pad so we can check 4-neighborhood without bounds issues
    bgp = np.pad(bg, 1, mode="constant", constant_values=False)

    # any cell that has a background neighbor (4-neighborhood)
    bg_neighbor = (
        bgp[1:-1, 0:-2] |  # left
        bgp[1:-1, 2:  ] |  # right
        bgp[0:-2, 1:-1] |  # up
        bgp[2:  , 1:-1]    # down
    )

    # border cells = non-background cells adjacent to background
    border_cells = (~bg) & bg_neighbor

    # set them to obstacles (keep background as background)
    out = arr.copy()
    out[border_cells] = obstacle_val
    return out