"""
cyl_env.py
----------
This module defines the Cylinder environment for motion planning on a discrete 3D cylinder grid map.

- The Cylinder class inherits from Env and represents a cylindrical world (with periodic x and bounded y).
- Handles obstacle initialization, edge crossing logic, and minimum distance calculations for cylindrical topology.
- Used as the environment for planners and agents operating on a cylinder (e.g., for robot path planning in a tube or ring).

Key methods:
- __init__(r, h): Initializes the cylinder with given radius and height, sets up boundary obstacles.
- init(): Adds boundary obstacles at the top and bottom of the cylinder.
- update(obstacles): Updates the set of obstacles and builds a KD-tree for fast queries.
- crossed_edge_check(x_1, x_2): Checks if a move crosses the left/right edge (wraps around).
- dx_min_node(node1, node2): Computes minimum x-distance between two nodes, considering wraparound.
- dx_min_int(x1, x2): Computes minimum x-distance between two x-coordinates, considering wraparound.
"""

from math import sqrt, pi
from abc import ABC, abstractmethod
from scipy.spatial import cKDTree
import numpy as np
from typing import Optional
from python_motion_planning.utils import Env, Node

class Cylinder(Env):
    """
    Class for discrete 3-d cylinder grid map.

    Parameters:
        r (int): radius of cylinder
        h (int): height of cylinder
    """
    def __init__(self, r: int, h: int) -> None:
        # Set x_range to the circumference (2*pi*r), y_range to height
        super().__init__(int(round(2*pi*r)), h)
        # Motions: dummy node for initialization (not used in planning)
        self.motions = [Node((0, 0), None, 1, None)]
        self.obstacles = None
        self.obstacles_tree = None
        self.init()  # Add boundary obstacles
    
    def init(self) -> None:
        """
        Initialize grid map by adding boundary obstacles at top and bottom.
        """
        obstacles = set()
        # Add obstacles at y=0 and y=max for all x (top and bottom boundaries)
        for i in range(self.x_range):
            obstacles.add((i, 0))
            obstacles.add((i, self.y_range - 1))
        self.update(obstacles)

    def update(self, obstacles):
        """
        Update the set of obstacles and build a KD-tree for fast queries.
        """
        self.obstacles = obstacles 
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))

    def crossed_edge_check(self, x_1: int, x_2: int) -> Optional[str]:
        """
        Check if a move from x_1 to x_2 crosses the left or right edge (wraps around).
        Returns 'left', 'right', or None.
        """
        dx = x_2 - x_1
        # If the direct distance is less than the wraparound, no edge crossed
        if abs(dx) < (self.x_range - abs(dx)):
            return None
        elif dx > 0:
            return "left"   # Crossed left edge (wraps from max to 0)
        else:
            return "right"  # Crossed right edge (wraps from 0 to max)

    def dx_min_node(self, node1: Node, node2: Node) -> float:
        """
        Compute minimum x-distance between two nodes, considering cylinder wraparound.
        """
        return min(abs(node2.x - node1.x), self.x_range - abs(node2.x - node1.x))
    
    def dx_min_int(self, x1: int, x2: int) -> float:
        """
        Compute minimum x-distance between two x-coordinates, considering cylinder wraparound.
        """
        return min(abs(x2 - x1), self.x_range - abs(x2 - x1))

