"""
randomize.py

This module provides utility functions and a class for generating random obstacles and random start/goal positions
in a cylindrical grid environment for motion planning experiments. It supports both rectangular and elliptical obstacles,
with horizontal wrapping (cylindrical topology), and ensures that randomly chosen start/goal cells are collision-free
with respect to a given robot footprint and padding.

Key Components:
- build_obstacle_rectangle: Add a rectangular obstacle region to the environment, wrapping horizontally.
- build_obstacle_ellipse: Add an elliptical obstacle region to the environment, wrapping horizontally.
- Randomize class: Static methods to generate random obstacles and random, collision-free start/goal cells.

Typical Usage:
    env = ...  # Grid environment
    env = Randomize.random_obstacles_rectangle(env, n_obstacles=5, max_size_cm=40, min_size_cm=10)
    start, goal = Randomize.random_start_and_goal(env, res=1.0, pad=5.0)

All functions assume a cylindrical world (x wraps, y does not).
"""

import sys, os
import math as math
import numpy as np
import random
from typing import Optional, Tuple
import python_motion_planning as pmp
from python_motion_planning.utils import Grid, Map, SearchFactory

from agent import PolygonAgent, Polygon 
from agent.presets import RESOLUTION_CM
from helper import convert


def build_obstacle_rectangle(x_range: tuple[int, int], y_range: tuple[int, int], env: Grid):
    """
    Add a rectangular obstacle to the environment, wrapping horizontally (cylindrical topology).
    """
    x_min, x_max = x_range
    y_min, y_max = y_range
    W, H = env.x_range, env.y_range   # width & height of grid
    for x in range(x_min, x_max):
        x_wrapped = x % W            # wrap horizontally (cylinder)
        for y in range(y_min, y_max):
            if 0 <= y < H:           # no vertical wrapping
                env.obstacles.add((x_wrapped, y))
    env.update(env.obstacles)
    return env

def build_obstacle_ellipse(x_center: int, y_center: int, a: int, b: int, env: Grid):
    """
    Add an elliptical obstacle to the environment, wrapping horizontally (cylindrical topology).
    """
    # Ellipse equation: x^2/a^2 + y^2/b^2 = 1
    W, H = env.x_range, env.y_range
    # Only check the bounding box of the ellipse
    x_min = x_center - b
    x_max = x_center + b
    y_min = y_center - a
    y_max = y_center + a
    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            dx = (x - x_center) / b
            dy = (y - y_center) / a
            if dx * dx + dy * dy <= 1.0:
                env.obstacles.add((x % W, y)) # add to obstacles with wrapped x
    env.update(env.obstacles)
    return env


class Randomize:
    """
    Utility class for generating random obstacles and random, collision-free start/goal cells
    in a cylindrical grid environment. All methods are static and operate directly on the provided Grid.
    """
    @staticmethod
    def random_obstacles_rectangle(env: Grid, n_obstacles: int, max_size_cm: int, min_size_cm: int) -> Grid:
        """
        Generate random rectangular obstacles on a cylindrical world (x wraps).
        """
        max_size = int(max_size_cm/RESOLUTION_CM)
        min_size = int(min_size_cm/RESOLUTION_CM)
        W, H = env.x_range, env.y_range
        for _ in range(n_obstacles):
            w = random.randint(min_size, max_size)  # random width
            h = random.randint(min_size, max_size)  # random height
            x_min = random.randint(0, W - 1)        # random x position
            y_min = random.randint(0, H - h - 1)    # random y position
            build_obstacle_rectangle((x_min, x_min + w), (y_min, y_min + h), env)
        env.update(env.obstacles)
        return env

    @staticmethod
    def random_obstacles_ellipse(env: Grid, n_obstacles: int, max_size_cm: int, min_size_cm: int) -> Grid:
        """
        Generate random elliptical obstacles on a cylindrical world (x wraps).
        """
        max_size = int(max_size_cm/RESOLUTION_CM)
        min_size = int(min_size_cm/RESOLUTION_CM)
        W, H = env.x_range, env.y_range
        for _ in range(n_obstacles):
            a = random.randint(min_size, max_size) # ellipse width
            b = random.randint(min_size, max_size) # ellipse height
            x_center = random.randint(0, W - 1)    # random center x
            y_center = random.randint(0, H - b - 1)# random center y
            build_obstacle_ellipse(x_center, y_center, a, b, env)
        env.update(env.obstacles)
        return env
    
    @staticmethod
    def random_start_cell(env: Grid, res: float, pad: float, start_bound: int ) -> Tuple[int, int]:
        """
        Pick a random cell (x, y) in the lower part of the grid (y <= start_bound) where the robot's footprint
        is collision-free. Used for randomizing start positions.
        """
        max_tries = 50
        # Use crouched polygon shape for collision check
        shape = Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (-160, 525), (-90, 215), (-68, 215)])
        shape = convert.ScalePolygon(shape, RESOLUTION_CM)
        W, H = env.x_range, env.y_range
        for _ in range(max_tries):
            x = random.randint(0, W - 1)
            y = random.randint(1 + math.ceil(pad/res), start_bound) # y_min = lower border obst + pad > 2
            pose = (float(x), float(y), 0.0)
            if not PolygonAgent.is_in_collision(pose, shape, env, res, pad):
                return (x, y)
        raise ValueError("No Goal Found")

    def random_goal_cell(env: Grid, res: float, pad: float, goal_bound: int ) -> Tuple[int, int]:
        """
        Pick a random cell (x, y) in the upper part of the grid (y >= y_max - goal_bound) where the robot's footprint
        is collision-free. Used for randomizing goal positions.
        """
        max_tries = 50
        shape = Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (-160, 525), (-90, 215), (-68, 215)])
        shape = convert.ScalePolygon(shape, RESOLUTION_CM)
        W, H = env.x_range, env.y_range
        for _ in range(max_tries):
            x = random.randint(0, W - 1)
            y = random.randint(H-goal_bound, H - math.ceil((78.3+pad)/res)) # y in upper region
            pose = (float(x), float(y), 0.0)
            if not PolygonAgent.is_in_collision(pose, shape, env, res, pad):
                return (x, y)
        raise ValueError("No Goal Found")

    @staticmethod
    def random_start_and_goal(env : Grid, res: float, pad: float, start_bound_cm : float = 10, goal_bound_cm : float = 100) -> tuple[tuple[int, int], tuple[int, int]]:
        """
        Generate a random, collision-free start and goal cell for the robot.
        The start is chosen from the lower part of the grid (y <= start_bound_cm),
        and the goal from the upper part (y >= y_max - goal_bound_cm).
        """
        start_bound_cells = math.ceil(start_bound_cm/res)
        goal_bound_cells = math.ceil(goal_bound_cm/res)         
        start = Randomize.random_start_cell(env, res, pad, start_bound_cells)
        goal = Randomize.random_goal_cell( env, res, pad, goal_bound_cells)
        return start, goal



