"""
poly_search.py

This module implements a polygon-based A* search planner for motion planning on a cylindrical grid.
It extends a generic graph searcher to support robots with polygonal footprints, custom motion sets, and collision checking.

Key Features:
- PolygonSearcher: A* planner for robots with arbitrary polygonal shapes.
- Cylinder-aware search: Handles horizontal wrapping (cylindrical topology) in the environment.
- Customizable heuristic (manhattan, euclidean) with goal tolerance region.
- Collision checking for each motion using the robot's local shape.
- Path extraction, neighbor expansion, and search visualization.

Typical Usage:
    searcher = PolygonSearcher(start, goal, env, robot)
    cost, path, expanded = searcher.plan()
    searcher.plot.animation(path, ...)
"""

import math
import heapq

from typing import Optional
from python_motion_planning.utils import Node, Grid
from python_motion_planning.global_planner.graph_search.graph_search import GraphSearcher

from agent import PolygonAgent 
from agent.presets import SHAPES_MM, MOTION_GROUPS_MM, RESOLUTION_CM, PADDING_GROUPS_CM, MOTIONS_CELLS
from plot import PolygonPlot
from environment import Cylinder
from helper import convert

import time


class PolygonSearcher(GraphSearcher):
    """
    A* search planner for robots with polygonal footprints on a cylindrical grid.
    Supports custom motion primitives, collision checking, and goal tolerance.
    """
    def __init__(self, start : tuple[int, int], goal : tuple[int, int], env : Grid, robot : PolygonAgent, heuristic_type : str ="euclidean", goal_tol_cells_x : int=0, goal_tol_cells_y : int=0) -> None:
        """
        Initialize the PolygonSearcher.
        """
        super().__init__(start, goal, env, heuristic_type)
        self.robot = robot  # instance of PolygonRobot
        self.plot = PolygonPlot(start, goal, env, robot)
        self.motions = robot.motions
        self.goal_tol_cells_x = goal_tol_cells_x
        self.goal_tol_cells_y = goal_tol_cells_y
        self.final_distance_cells = -1
        self.interim_distance = math.inf
        self.interim_closets_candidate_x = -1
        self.interim_closets_candidate_y = -1
        self.max_time_s = 200 # max time for regular motion sets (seconds)
        self.max_time_varying_s = 500 # max time for large motion sets (seconds)
        self.t_start = time.perf_counter() 
        # Precompute max step length for heuristic normalization
        self.max_step_cells = max(math.hypot(m.x, m.y) for m in self.motions)
        self.expanded_nodes = []
    
    def isCollision(self, node_from : Node, node_to : Node, motion : Node) -> bool:
        """
        Check if moving from node_from to node_to using the given motion causes a collision.
        Uses the robot's local shape for the specific motion.
        Returns True if collision occurs, False otherwise.
        """
        # Set robot pose to candidate location (keep orientation constant)
        x, y = node_from.current
        theta = self.robot.pose[2]
        self.robot.pose = (x, y, theta)
        # Decode motion type (e.g., up, diagonal, etc.)
        motion_name = self.motion_decode(motion.current)
        # Get the corresponding local shape for this motion
        attr_name = f"local_shape_{motion_name}"
        shape = getattr(self.robot, attr_name)
        # Check for collision with environment
        if self.robot.is_in_collision(self.robot.pose, shape, self.env, self.robot.resolution, self.robot.padding):
             return True
        return False
    
    def motion_decode(self, current_motion : tuple[int, int]) -> str:
        """
        Given a motion vector (dx, dy), return the corresponding motion name as defined in MOTIONS_CELLS.
        Raises ValueError if no match is found.
        """
        for name, motion in MOTIONS_CELLS.items():
            if motion == current_motion:
                return name
        raise ValueError(f"No motion found for motion {current_motion}")
    
    def getNeighbor(self, node: Node) -> list:
        """
        Generate all valid neighbor nodes for the current node, considering motion set, collision, and cylinder wrapping.
        Returns a list of neighbor Node objects, sorted by distance to goal.
        Raises TimeoutError if time limit is exceeded.
        """
        # Use longer timeout for large motion sets
        if len(self.motions) >= 9:
            if time.perf_counter() - self.t_start > self.max_time_varying_s:
                print("This has more than 9")
                raise TimeoutError("PolygonSearcher exceeded time limit")
        else:
            if time.perf_counter() - self.t_start > self.max_time_s:
                raise TimeoutError("PolygonSearcher exceeded time limit")
        neighbors = []
        if self.goal.current is None:
            print("No goal found")
            raise ValueError
        for motion in self.motions:
            candidate = node + motion
            if candidate.current is None:
                continue
            # Wrap x coordinate if environment is a cylinder
            if isinstance(self.env, Cylinder):
                x, y = candidate.current
                x = x % self.env.x_range
                candidate.current = (x, y)
            if self.isCollision(node, candidate, motion):
                continue
            neighbors.append(candidate)
        # Sort neighbors by distance to goal (closest first)
        neighbors.sort(key=self.neighbor_goal_distance)
        return neighbors
    
    def dist(self, node1: Node, node2: Node) -> float:
        """
        Compute the distance between two nodes, considering cylinder wrap in x.
        Returns the Euclidean distance in grid cells.
        """
        dx = abs(node2.x - node1.x)
        dy = abs(node2.y - node1.y)
        # Use shortest x distance around the cylinder
        if isinstance(self.env, Cylinder):
            dx = self.env.dx_min_node(node1, node2)
        return math.hypot(dx, dy)
    
    def h(self, node: Node, goal: Node) -> float:
        """
        Heuristic function for A* search.
        Returns the estimated cost from node to goal, considering goal tolerance region.
        Uses either 'manhattan' or 'euclidean' heuristic as configured.
        """
        x_min = abs(goal.x - node.x)
        if isinstance(self.env, Cylinder):
            x_min = self.env.dx_min_node(node, goal)
        dy = abs(goal.y - node.y)
        d_cells = math.hypot(x_min, dy)  # Euclidean distance in cells
        r_tol = math.hypot(self.goal_tol_cells_x, self.goal_tol_cells_y)  # Tolerance region radius
        d_eff = max(d_cells - r_tol, 0.0)  # Effective distance outside tolerance
        if self.heuristic_type == "manhattan":
            return x_min + abs(goal.y - node.y)
        elif self.heuristic_type == "euclidean":
            return d_eff / self.max_step_cells
    
    def extractPath(self, closed_list: dict) -> tuple:
        """
        Extract the path from the CLOSED list after search completion.
        Returns:
            cost (float): the cost of planned path
            path (list): the planning path
        """
        cost = 0
        node = closed_list[self.goal.current]
        path = [node.current]
        while node != self.start:
            node_parent = closed_list[node.parent]
            cost += 1  # Each step has unit cost
            node = node_parent
            path.append(node.current)
        return cost, path
    
    def plan(self) -> tuple:
        """
        Run the A* motion planning algorithm.
        Returns:
            cost (float): path cost
            path (list): planning path
            expand (list): all nodes that planner has searched
        """
        OPEN = []  # Priority queue for open nodes
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()  # Hash table for closed nodes
        while OPEN:
            node = heapq.heappop(OPEN)
            if node.current in CLOSED:
                continue
            CLOSED[node.current] = node
            # Check if goal is reached (within tolerance)
            if self.within_goal_tolerance(node):
                dx = self.env.dx_min_node(node, self.goal)
                dy = abs(node.y - self.goal.y)
                self.final_distance_cells = math.hypot(dx, dy)
                reached_goal = self.goal
                self.goal = Node(node.current)
                cost, path = self.extractPath(CLOSED)
                self.goal = reached_goal
                return cost, path, list(CLOSED.values())
            for node_n in self.getNeighbor(node):
                if node_n.current in CLOSED:
                    continue
                node_n.parent = node.current
                node_n.h = self.h(node_n, self.goal)
                heapq.heappush(OPEN, node_n)
            # Track expanded nodes for visualization/debugging
            self.expanded_nodes.append(node)
        return [], [], list(CLOSED.values())
    
    def within_goal_tolerance(self, node: Node) -> bool:
        """
        Check if the node is within the goal tolerance region (rectangle around goal).
        Returns True if within tolerance, False otherwise.
        """
        goal_x, goal_y = self.goal.current
        dx = self.env.dx_min_node(node, self.goal)  # Shortest x on cylinder
        dy = abs(node.y - goal_y)
        return (dx <= self.goal_tol_cells_x) and (dy <= self.goal_tol_cells_y)
    
    def neighbor_goal_distance(self, node: Node) -> float:
        """
        Compute the cylinder-aware Euclidean distance from node to the true goal (in cells).
        Used for neighbor sorting.
        """
        dx = self.env.dx_min_node(node, self.goal)
        dy = abs(node.y - self.goal.y)
        return math.hypot(dx, dy)