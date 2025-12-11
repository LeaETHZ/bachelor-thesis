import numpy as np
import math
from environment import Cylinder


def load_cylinder_from_npy(path: str, res_cm: float, obstacle_threshold: float = 0.5):
    """
    Load a 2D numpy array (H x W) into a Cylinder environment.

    Only resolution is needed.

    The map's width determines cylinder circumference:
        radius = (W * res) / (2π)

    The map's height determines cylinder height:
        height = H * res
    """

    arr = np.load(path)

    # Allow RGB
    if arr.ndim == 3 and arr.shape[2] == 3:
        arr = arr.mean(axis=2)

    if arr.ndim != 2:
        raise ValueError(f"Expected 2D array in {path}, got shape {arr.shape}")
    
    H, W = arr.shape

    # Compute physical dimensions from resolution
    height_cm = H * res_cm
    radius_cm = (W * res_cm) / (2 * math.pi)

    # Create cylinder with computed dimensions
    env = Cylinder(radius_cm, height_cm)

    # Override grid ranges to match map
    env.x_range = W
    env.y_range = H

    # Fill obstacles
    obstacles = {
        (x, y)
        for y in range(H)
        for x in range(W)
        if arr[y, x] > obstacle_threshold
    }

    env.obstacles = obstacles
    env.update(env.obstacles)

    return env
