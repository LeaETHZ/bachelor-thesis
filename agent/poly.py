import math
import numpy as np
from numpy.typing import NDArray






# This module provides geometric utilities for working with polygons in grid-based environments.
# It includes polygon representation, coordinate transforms, point-in-polygon tests, and footprint inflation for collision checking.

class Polygon:
    """
    Represents a 2D polygon by its list of vertices.
    Vertices are stored as a numpy array of shape (N, 2).
    """
    def __init__(self, polygon: list[tuple[float, float]]):
        self.vertices: NDArray[np.float64] = np.array(polygon, dtype=float)





def transform_polygon_local_to_world(world_pose: tuple[float, float, float], polygon: Polygon) -> NDArray[np.float64]:
    """
    Transform the polygon from the robot's local frame to world coordinates.

    Args:
        world_pose: (x, y, theta) pose of the robot in world frame
        polygon: Polygon in local frame
    Returns:
        Numpy array of transformed vertices in world frame
    """
    x, y, theta = world_pose
    rotation_matrix = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta),  math.cos(theta)]], dtype=float)
    return (polygon.vertices @ rotation_matrix.T) + np.array([x, y])



def contains_point(x: float, y: float, poly_world) -> bool:
    """
    Determine whether a point (x, y) lies inside a polygon using the ray-casting algorithm.
    Returns True if inside or on the boundary, False otherwise.
    """
    inside = False
    n = len(poly_world)
    eps = 1e-9

    for i in range(n):
        x1, y1 = poly_world[i]
        x2, y2 = poly_world[(i + 1) % n]

        # Boundary check: is point on segment?
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) <= eps:
            dot = (x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)
            if dot >= -eps:
                sq_len = (x2 - x1) ** 2 + (y2 - y1) ** 2
                if dot <= sq_len + eps:
                    return True  # on boundary

        # Ray-casting: does the edge cross the horizontal ray to the right of (x, y)?
        intersects = ((y1 > y) != (y2 > y)) and \
                    (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-15) + x1)
        if intersects:
            inside = not inside

    return inside



def footprint_cells(pose: tuple[float, float, float], polygon: Polygon, grid_width: int, grid_height: int) -> list[tuple[int, int]]:
    """
    Compute the set of grid cells covered by the polygon's footprint at a given pose.

    Args:
        pose: (x, y, theta) pose of the robot
        polygon: Polygon representing the robot's shape
        grid_width: Width of the grid (for wrapping)
        grid_height: Height of the grid
    Returns:
        List of (ix, iy) grid cells covered by the polygon
    """
    poly_world = transform_polygon_local_to_world(pose, polygon)

    xs = poly_world[:, 0]
    ys = poly_world[:, 1]

    # Compute bounding box of the polygon in grid coordinates
    min_ix = int(xs.min())
    max_ix = int(xs.max())
    min_iy = int(ys.min())
    max_iy = int(ys.max())

    # For each cell in the bounding box, check if its center is inside the polygon
    base = []
    for iy in range(min_iy, max_iy + 1):
        for ix in range(min_ix, max_ix + 1):
            if contains_point(ix, iy, poly_world):
                base.append(((ix) % grid_width, iy))
    return base

    

def padded_footprint(base: list[tuple[int, int]], grid_width: int, grid_height: int, res: float, pad: float) -> list[tuple[int, int]]:
    """
    Inflate a set of grid cells (the robot's base footprint) by a given padding (in cm).
    This is used to account for safety margins in collision checking.

    Args:
        base: List of (ix, iy) cells covered by the robot
        grid_width: Width of the grid (for wrapping)
        grid_height: Height of the grid
        res: Grid resolution (cm)
        pad: Padding (cm) to inflate
    Returns:
        List of (ix, iy) cells in the inflated (padded) footprint
    """
    pad_cells = math.ceil(pad / res)
    base_set = set(base)
    inflated = set(base)

    # For each border cell, add all cells within pad_cells in each direction
    for (ix, iy) in base:
        # Border cell: at least one neighbor not in base
        if any((ix + dx, iy + dy) not in base_set for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]):
            for dx in range(-pad_cells, pad_cells + 1):
                for dy in range(-pad_cells, pad_cells + 1):
                    nx, ny = ix + dx, iy + dy
                    if 0 <= ny < grid_height:
                        inflated.add((nx % grid_width, ny))
    return list(inflated)





