from __future__ import annotations
import python_motion_planning as pmp
from python_motion_planning.utils import Grid, Node
import matplotlib.pyplot as plt

from .poly import Polygon
from agent import poly

# Type alias for a 2D pose: (x, y, theta)
Pose2D = tuple[float, float, float]
# Type alias for a grid cell: (ix, iy)
Cell = tuple[int, int]

# PolygonAgent represents a robot with multiple polygonal shapes (for different postures/motions)
# and handles its interaction with the environment (collision checking, etc).
class PolygonAgent(pmp.Robot):
    def __init__(self,  pose: Pose2D | None,
                polygon_up : Polygon, polygon_right : Polygon, polygon_left : Polygon, polygon_crouched : Polygon,
                polygon_down : Polygon, polygon_diagonal_right : Polygon, polygon_diagonal_left : Polygon,
                polygon_up_2_3 : Polygon, polygon_up_1_3 : Polygon, polygon_right_2_3 : Polygon, polygon_right_1_3 : Polygon, polygon_left_2_3 : Polygon, polygon_left_1_3 : Polygon,
                motions : list[Node], resolution: float, padding: float) -> None:
        """
        Initialize a PolygonAgent with multiple possible shapes (for different robot postures),
        a list of allowed motions, and environment interaction parameters.

        Args:
            pose: Initial pose (x, y, theta) or None for default (0,0,0)
            polygon_*: Polygon objects for each possible robot posture
            motions: List of allowed motion primitives (as Node objects)
            resolution: Grid resolution (cm)
            padding: Safety padding (cm) around the robot for collision checking
        """
        if pose is None:
            pose = (0.0, 0.0, 0.0)
        self.pose = pose
        px, py, theta = pose

        # Initialize base robot with position and default velocities
        super().__init__(px, py, theta, v=1, w=1)

        # List of allowed motion primitives (e.g., up, right, diagonal, etc.)
        self.motions = motions

        # Store all possible local robot shapes for different postures/motions
        self.local_shape_up: Polygon = polygon_up
        self.local_shape_right: Polygon = polygon_right
        self.local_shape_left: Polygon = polygon_left
        self.local_shape_crouched: Polygon = polygon_crouched
        self.local_shape_down: Polygon = polygon_down
        self.local_shape_diagonal_right: Polygon = polygon_diagonal_right
        self.local_shape_diagonal_left: Polygon = polygon_diagonal_left
        self.local_shape_up_2_3: Polygon = polygon_up_2_3
        self.local_shape_up_1_3: Polygon = polygon_up_1_3
        self.local_shape_right_2_3: Polygon = polygon_right_2_3
        self.local_shape_right_1_3: Polygon = polygon_right_1_3
        self.local_shape_left_2_3: Polygon = polygon_left_2_3
        self.local_shape_left_1_3: Polygon = polygon_left_1_3

        # Padding and resolution for collision checking
        self.padding: float = padding
        self.resolution: float = resolution

        # For plotting the final target point (optional usage)
        self.target = 0

    @staticmethod
    def is_in_collision(pose: Pose2D, polygon: Polygon, env: Grid, res: float, pad: float) -> bool:
        """
        Check if the robot (with given pose and polygon shape) is in collision with any obstacles in the environment.

        Args:
            pose: Robot pose (x, y, theta)
            polygon: Polygon representing the robot's shape
            env: Grid environment (with obstacles)
            res: Grid resolution (cm)
            pad: Padding (cm) to expand the robot's footprint for safety

        Returns:
            True if any part of the (padded) robot footprint overlaps an obstacle cell, False otherwise.
        """
        grid_width, grid_height = env.x_range, env.y_range
        # Compute the set of grid cells covered by the robot's polygon at the given pose
        footprint = poly.footprint_cells(pose, polygon, grid_width, grid_height)
        # Expand the footprint by the given padding
        padded_footprint = poly.padded_footprint(footprint, grid_width, grid_height, res, pad)
        obs = env.obstacles
        # Return True if any padded footprint cell is occupied by an obstacle
        return any((ix, iy) in obs for (ix, iy) in padded_footprint)

    

