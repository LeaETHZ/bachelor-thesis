import sys, os
import math as math
import numpy as np
import random
from typing import Optional, Tuple
import python_motion_planning as pmp
from python_motion_planning.utils import Grid, Map, SearchFactory

from agent import PolygonAgent, Polygon , presets
from helper import convert


def build_obstacle(x_range: tuple[int, int], y_range: tuple[int, int], env: Grid):

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

class Randomize:

    @staticmethod
    # def random_obstacles(env: Grid, n_obstacles: int, max_size: int = 15, min_size: int = 10) -> Grid:
    #     """ Generate a number of random rectangular obstacles and add them to the environment. """
    #     width, height = env.x_range, env.y_range

    #     for _ in range(n_obstacles):
    #         # choose random size
    #         w = random.randint(min_size, max_size)
    #         h = random.randint(min_size, max_size)

    #         # choose random bottom-left corner (make sure obstacle fits)
    #         x_min = random.randint(0, width - w - 1)
    #         y_min = random.randint(0, height - h - 1)

    #         # build obstacle
    #         build_obstacle((x_min, x_min + w), (y_min, y_min + h), env)
    #         env.update(env.obstacles)

    #     return env
    @staticmethod
    def random_obstacles(
        env: Grid,
        n_obstacles: int,
        max_size: int = 15,
        min_size: int = 10,
        min_gap: int = 0,   # fixed spacing between obstacles
    ) -> Grid:
        """Generate random rectangular obstacles on a cylindrical world (x wraps)."""

        W, H = env.x_range, env.y_range

        obstacles_added = 0
        max_attempts = n_obstacles * 30  # avoid infinite loops
        attempts = 0

        while obstacles_added < n_obstacles and attempts < max_attempts:
            attempts += 1

            # random size
            w = random.randint(min_size, max_size)
            h = random.randint(min_size, max_size)

            # random bottom-left corner:
            # x can be anywhere (wrap), y must fit vertically (no wrap in y)
            x_min = random.randint(0, W - 1)
            y_min = random.randint(0, H - h - 1)

            # extended region incl. min_gap, with wrap in x
            x0 = x_min - min_gap
            x1 = x_min + w + min_gap
            y0 = max(0, y_min - min_gap)
            y1 = min(H, y_min + h + min_gap)

            too_close = False

            for x in range(x0, x1):
                x_wrapped = x % W  # wrap horizontally
                for y in range(y0, y1):
                    if (x_wrapped, y) in env.obstacles:
                        too_close = True
                        break
                if too_close:
                    break

            if too_close:
                # try another obstacle position
                continue

            # Place obstacle – build_obstacle already wraps in x
            build_obstacle((x_min, x_min + w), (y_min, y_min + h), env)
            obstacles_added += 1

        env.update(env.obstacles)
        return env
    
    @staticmethod
    def random_start_cell(env: Grid, res: float, pad: float) -> Tuple[int, int]:
        """Pick a random cell (x,y) where the given robot footprint at (x,y,theta) is collision-free."""
        
        max_tries = 50

        shape = Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (-160, 525), (-90, 215), (-68, 215)]) # crouched position
        shape = convert.ScalePolygon(shape, 1.7)
        
        W, H = env.x_range, env.y_range
    
        for _ in range(max_tries):
            x = random.randint(0, W - 1)
            y = random.randint(2, H - 1) # obstacles at y = 0, padding is at least 1 grid, start looking for y > 2
            pose = (float(x), float(y), 0.0)
            if not PolygonAgent.is_in_collision(pose, shape, env, res, pad):
                return (x, y)
            
    
    def random_goal_cell(env: Grid) -> Tuple[int, int]:
        """Pick a random cell (x,y) that is collision-free."""
        
        max_tries = 50
        
        W, H = env.x_range, env.y_range


        #NEW: do we want this?
        Hight_Robot = convert.ScaleVertex((0,783), 1.7)[1]
        H = H - Hight_Robot
    
        for _ in range(max_tries):
            x = random.randint(0, W - 1)
            y = random.randint(0, H - 1)

            if (x, y) not in env.obstacles:
                return (x, y)



    @staticmethod
    def random_start_and_goal(env : Grid, res: float = 1.7, pad: float = 1.7) -> tuple[tuple[int, int], tuple[int, int]]:
         max_tries = 100

         for _ in range(max_tries):
              start = Randomize.random_start_cell(env, res, pad)
              goal = Randomize.random_goal_cell( env)

              if goal[1] >= start[1] and (pow((start[0]-goal[0]),2) +  pow(start[1]-goal[1],2))> (math.ceil(100/res))**2: 
                   return start, goal

         
   
   