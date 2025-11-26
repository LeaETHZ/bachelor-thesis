import sys, os
import math as math
import numpy as np
import random
from typing import Optional, Tuple
import python_motion_planning as pmp
from python_motion_planning.utils import Grid, Map, SearchFactory

from agent import PolygonAgent, Polygon , presets
from helper import convert


def build_obstacle_rectangle(x_range: tuple[int, int], y_range: tuple[int, int], env: Grid):

    x_min, x_max = x_range
    y_min, y_max = y_range

    W, H = env.x_range, env.y_range   # width & height of grid

    for x in range(x_min, x_max):
        x_wrapped = x % W            # <-- wrap horizontally (cylinder)
        for y in range(y_min, y_max):
            if 0 <= y < H:           # no vertical wrapping
                env.obstacles.add((x_wrapped, y))

    env.update(env.obstacles)
    return env

def build_obstacle_ellipse(x_center: int, y_center: int, a: int, b: int, env: Grid):

    # Ellipse equation: x^2/a^2 + y^2/b^2 = 1

    W, H = env.x_range, env.y_range

    # We only need to check the bounding box of the ellipse
    x_min = x_center - b
    x_max = x_center + b
    y_min = y_center - a
    y_max = y_center + a

    for x in range(x_min, x_max + 1):

        for y in range(y_min, y_max + 1):

                # Check ellipse condition
                dx = (x - x_center) / b
                dy = (y - y_center) / a

                if dx * dx + dy * dy <= 1.0:
                    env.obstacles.add((x % W, y)) # if cell is part of ellipse, add to obstacles with wrapped x

    env.update(env.obstacles)
    return env


class Randomize:

    @staticmethod
    def random_obstacles_rectangle(env: Grid, n_obstacles: int, max_size: int = 25, min_size: int = 10) -> Grid:
        """Generate random rectangular obstacles on a cylindrical world (x wraps)."""

        W, H = env.x_range, env.y_range

        for _ in range(n_obstacles):
            # random size
            w = random.randint(min_size, max_size)
            h = random.randint(min_size, max_size)

            # random bottom-left corner:
            x_min = random.randint(0, W - 1)
            y_min = random.randint(0, H - h - 1)

            build_obstacle_rectangle((x_min, x_min + w), (y_min, y_min + h), env)

        env.update(env.obstacles)
        return env

    @staticmethod
    def random_obstacles_ellipse(env: Grid, n_obstacles: int, max_size: int = 10, min_size: int = 5) -> Grid:
        """Generate random elliptic obstacles on a cylindrical world (x wraps)."""

        W, H = env.x_range, env.y_range

        for _ in range(n_obstacles):
            # random size for the ellipse axes
            a = random.randint(min_size, max_size) # width
            b = random.randint(min_size, max_size) # height

            # random center of ellipse:
            x_center = random.randint(0, W - 1)
            y_center = random.randint(0, H - b - 1)

            build_obstacle_ellipse(x_center, y_center, a, b, env)

        env.update(env.obstacles)
        return env
    
    @staticmethod
    def random_start_cell(env: Grid, res: float, pad: float, start_bound: int ) -> Tuple[int, int]:
        """Pick a random cell (x,y) where the given robot footprint at (x,y,theta) is collision-free. It should be between y_min and y = start_bound"""
        
        max_tries = 50

        shape = Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (-160, 525), (-90, 215), (-68, 215)]) # crouched position
        shape = convert.ScalePolygon(shape, 1.7)
        
        W, H = env.x_range, env.y_range
    
        for _ in range(max_tries):
            x = random.randint(0, W - 1)
            y = random.randint(1 + math.ceil(pad/res), start_bound) # y_min = lower border obst + pad > 2
            pose = (float(x), float(y), 0.0)
            if not PolygonAgent.is_in_collision(pose, shape, env, res, pad):
                return (x, y)
        
        raise ValueError("No Goal Found")

            
    
    def random_goal_cell(env: Grid, res: float, pad: float, goal_bound: int ) -> Tuple[int, int]:
        """Pick a random cell (x,y) where the given robot footprint at (x,y,theta) is collision-free. It should be between y = goal_bound and y_max"""
        
        max_tries = 50

        shape = Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (-160, 525), (-90, 215), (-68, 215)]) # crouched position
        shape = convert.ScalePolygon(shape, 1.7)
        
        W, H = env.x_range, env.y_range
    
        for _ in range(max_tries):
            x = random.randint(0, W - 1)
            y = random.randint(H-goal_bound, H - math.ceil((52.5+pad)/res)) # find a random y s.t. it is between (y_max - bound, y_max - height of courched shape incl. pad)
            pose = (float(x), float(y), 0.0)
            if not PolygonAgent.is_in_collision(pose, shape, env, res, pad):
                return (x, y)
            
        raise ValueError("No Goal Found")



    @staticmethod
    def random_start_and_goal(env : Grid, res: float = 1.7, pad: float = 1.7, start_bound_cm : float = 10, goal_bound_cm : float = 100) -> tuple[tuple[int, int], tuple[int, int]]:
         
        start_bound_cells = math.ceil(start_bound_cm/res)
        goal_bound_cells = math.ceil(goal_bound_cm/res)         

        
        start = Randomize.random_start_cell(env, res, pad, start_bound_cells)
        goal = Randomize.random_goal_cell( env, res, pad, goal_bound_cells)

        return start, goal

         
   
   