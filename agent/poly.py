import math
import numpy as np
from numpy.typing import NDArray





#in this class everything purely geometric with Polygons
class Polygon:
    def __init__(self, polygon : list[tuple[float, float]]):
        self.vertices: NDArray[np.float64] = np.array(polygon, dtype=float)
        





def transform_polygon_local_to_world(world_pose : tuple[float, float, float], polygon : Polygon) -> None:
    """Transform the polygon from the local frame to world coordinates."""
    x, y, theta = world_pose
    
    rotation_matrix = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta),  math.cos(theta)]], dtype=float)
    return (polygon.vertices @ rotation_matrix.T) + np.array([x, y])


def contains_point(x: float, y: float, poly_world) -> bool:
    """Determine whether a cell center lies inside a polygon using the ray-casting algorithm"""
    inside = False
    n = len(poly_world)
    eps = 1e-9

    for i in range(n):
        x1, y1 = poly_world[i]
        x2, y2 = poly_world[(i + 1) % n]

        # ---- NEW: boundary check (point on segment) ----
        # Check colinearity via cross product ~ 0
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) <= eps:
            # Check if within segment bounding box (using dot product)
            dot = (x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)
            if dot >= -eps:
                sq_len = (x2 - x1) ** 2 + (y2 - y1) ** 2
                if dot <= sq_len + eps:
                    return True  # on boundary → inside

        # ---- Original ray-casting logic ----
        intersects = ((y1 > y) != (y2 > y)) and \
                    (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-15) + x1)
        if intersects:
            inside = not inside

    return inside


def footprint_cells(pose : tuple[float, float, float], polygon : Polygon, grid_width : int, grid_height : int) -> list[tuple[int, int]]:
    """Compute the set of grid cells covered by the polygon's footprint at a given pose."""
    poly_world = transform_polygon_local_to_world(pose, polygon)
    
    
    # minx = int(math.floor(np.min(poly_world[:, 0])))
    # maxx = int(math.ceil (np.max(poly_world[:, 0])))
    # miny = max(int(math.floor(np.min(poly_world[:, 1]))), 0)
    # maxy = min(int(math.ceil (np.max(poly_world[:, 1]))), grid_height)


    xs = poly_world[:, 0]
    ys = poly_world[:, 1]
    #print("poly_world[:, 0]", poly_world[:, 0])


    #int cast only bc they are numpy floats, but this number is always exact as we already converted them to polygon
    min_ix = int(xs.min())
    max_ix = int(xs.max())
    min_iy = int(ys.min())
    max_iy = int(ys.max())

    #print("print min max indices: ", min_ix, min_iy, max_ix, max_iy)


    # base cells via center-in-polygon (fast)
    base = []
    
    for iy in range(min_iy, max_iy+1):
        for ix in range(min_ix, max_ix+1):
            if contains_point(ix, iy, poly_world):
                base.append(((ix)%grid_width, (iy)))
    
    return base

    
def padded_footprint(base: list[tuple[int,int]], grid_width : int, grid_height : int, res: float, pad : float):
    
    pad_cells = math.ceil(pad/res)
    
    base_set = set(base)
    inflated = set(base)

    # find outermost cells
    for (ix, iy) in base:
        # if any of the 4 neighbors are not in the base -> it's a border cell
        if any((ix + dx, iy + dy) not in base_set for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]):
            for dx in range(-pad_cells, pad_cells + 1):
                for dy in range(-pad_cells, pad_cells + 1):
                    nx, ny = ix + dx, iy + dy
                    if 0 <= ny < grid_height:
                        inflated.add((nx%grid_width, ny))

    return list(inflated)





