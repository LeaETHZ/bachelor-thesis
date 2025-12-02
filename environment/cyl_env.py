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
        super().__init__(int(round(2*pi*r)), h) # from here on: x_range = 2*pi*r (circumference), y_range = height
        # allowed motions
        self.motions = [Node((-1, 0), None, 1, None), Node((-1, 1),  None, sqrt(2), None),
                        Node((0, 1),  None, 1, None), Node((1, 1),   None, sqrt(2), None),
                        Node((1, 0),  None, 1, None), Node((1, -1),  None, sqrt(2), None),
                        Node((0, -1), None, 1, None), Node((-1, -1), None, sqrt(2), None)]
        # obstacles
        self.obstacles = None
        self.obstacles_tree = None
        self.init()
    
    def init(self) -> None:
        """
        Initialize grid map.
        """

        obstacles = set()

        # boundary of environment, only add on the top and bottom
        for i in range(self.x_range):
            obstacles.add((i, 0))
            obstacles.add((i, self.y_range - 1))

        self.update(obstacles)

    def update(self, obstacles):
        self.obstacles = obstacles 
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))

    def crossed_edge_check(self, x_1: int, x_2: int) -> Optional[str]:
        dx = x_2 - x_1

        if abs(dx) < (self.x_range - abs(dx)): # no edge was crossed
            return None

        elif dx > 0: # left edge was crossed
            return "left"
        
        else: # right edge was crossed
            return "right"

    def dx_min_node(self, node1: Node, node2: Node) -> float:
        return min(abs(node2.x - node1.x), self.x_range - abs(node2.x - node1.x))
    
    def dx_min_int(self, x1: int, x2: int) -> float:
        return min(abs(x2 - x1), self.x_range - abs(x2 - x1))
    
