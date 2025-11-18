import math
import numpy as np
from numpy.typing import NDArray




#in this class everything purely geometric with Polygons
class Polygon:
    def __init__(self, polygon : list[tuple[float, float]]):
        self.vertices: NDArray[np.float64] = np.array(polygon, dtype=float)
        

    def transform_polygon_local_to_world(self, world_pose : tuple[float, float, float]) -> None:
        """Transform the polygon from the local frame to world coordinates."""
        x, y, theta = world_pose
        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta),  math.cos(theta)]], dtype=float)
        return (self.vertices @ rotation_matrix.T) + np.array([x, y])


    def contains_point(self, x: float, y: float, poly_world) -> bool:
        """Determine whether a cell center lies inside a polygon using the ray-casting algorithm"""
        inside = False
        n = len(poly_world)
        for i in range(n):
            x1, y1 = poly_world[i]
            x2, y2 = poly_world[(i + 1) % n]
            intersects = ((y1 > y) != (y2 > y)) and \
                         (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-15) + x1)
            if intersects:
                inside = not inside
        return inside
    

    def footprint_cells(self, pose : tuple[float, float, float], grid_width : int, grid_height : int) -> list[tuple[int, int]]:
        """Compute the set of grid cells covered by the polygon's footprint at a given pose."""
        poly_world = self.transform_polygon_local_to_world(pose)
        
       

        minx = max(int(math.floor(np.min(poly_world[:, 0]))), 0)
        maxx = min(int(math.ceil (np.max(poly_world[:, 0]))), grid_width)
        miny = max(int(math.floor(np.min(poly_world[:, 1]))), 0)
        maxy = min(int(math.ceil (np.max(poly_world[:, 1]))), grid_height)

        # base cells via center-in-polygon (fast)
        base = []
        for iy in range(miny, maxy):
            for ix in range(minx, maxx):
                if self.contains_point(ix + 0.5, iy + 0.5, poly_world):
                    base.append((ix, iy))
        
        return base

        
    def padded_footprint(base: list[tuple[int,int]], grid_width : int, grid_height : int, res: float=1.7, pad : float=1.7):
        
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
                        if 0 <= nx < grid_width and 0 <= ny < grid_height:
                            inflated.add((nx, ny))

        return list(inflated)

    



    