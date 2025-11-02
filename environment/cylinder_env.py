from math import sqrt
from abc import ABC, abstractmethod
from scipy.spatial import cKDTree
import numpy as np
from python_motion_planning.utils import Env, Node

class Cylinder(Env):
    """
    Class for discrete 2-d grid map.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
    """
    def __init__(self, x_range: int, y_range: int) -> None:
        super().__init__(x_range, y_range)
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
        x, y = self.x_range, self.y_range
        obstacles = set()

        # boundary of environment
        # for i in range(x):
        #     obstacles.add((i, 0))
        #     obstacles.add((i, y - 1))
        # for i in range(y):
        #     obstacles.add((0, i))
        #     obstacles.add((x - 1, i))

        self.update(obstacles)

    def update(self, obstacles):
        self.obstacles = obstacles 
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))

    
    def getNeighbor(self, node: Node) -> list:
        """ Custom neighbor generation for cylindrical environment. Wraps around horizontally."""
        neighbors = []
        for motion in self.motions:
            new_node = node + motion
            x, y = new_node.current
            # Wrap x around cylinder
            x = x % self.x_range  
            wrapped = Node((x, y), node.current, new_node.g, new_node.h)
            if wrapped.current not in self.obstacles:
                neighbors.append(wrapped)
        return neighbors

