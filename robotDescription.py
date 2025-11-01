from __future__ import annotations

import python_motion_planning as pmp
from python_motion_planning.utils import Grid
from polygon import Polygon

Pose2D = tuple[float, float, float]
Cell = tuple[int, int]

#in this class everything that is robot description and its interaction with the environment
class RobotDescription(pmp.Robot):
    def __init__(self, pose : Pose2D, polygon_up : Polygon, polygon_right : Polygon, polygon_left : Polygon) -> None:
        self.pose = pose
        px,py,theta = pose
        super().__init__(px, py, theta, v = 1, w = 1)

        self.local_shape_up : Polygon = polygon_up
        self.local_shape_right : Polygon = polygon_right
        self.local_shape_left : Polygon = polygon_left
        

    
    @staticmethod
    def is_in_collision(pose : Pose2D, polygon: Polygon, env: Grid, obstacles : set[Cell] =None) -> bool: # obstacles is smth like this : {(10, 5), (11, 5), (12, 5), ...}
        """Return True if the robot footprint (polygon) overlaps any occupied cells in the environment."""

        grid_width, grid_height = env.x_range, env.y_range
        footprint = polygon.footprint_cells(pose, grid_width, grid_height)
        obs = obstacles if obstacles is not None else env.obstacles
        return any((ix, iy) in obs for (ix, iy) in footprint) #wenn mindestens eine zelle true dann gibt true zurück


