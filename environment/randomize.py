import sys, os
import python_motion_planning as pmp
from python_motion_planning.utils import Grid, Map, SearchFactory
import agent.poly_agent as rd
import math as math
import planner.a_star_ext as ASE
import random
from typing import Optional, Tuple
from agent.poly_agent import PolygonAgent
from agent.poly import Polygon
import numpy as np


def build_obstacle(x_range: tuple[int, int], y_range: tuple[int, int], env: Grid):

        x_min, x_max = x_range
        y_min, y_max = y_range

        for x in range(x_min, x_max):
                for y in range(y_min, y_max):
                    env.obstacles.add((x, y)) 
        env.update(env.obstacles) 
        return env


class Randomize:

    @staticmethod
    def random_obstacles(env: Grid, n_obstacles: int, max_size: int = 15, min_size: int = 10) -> Grid:
        """ Generate a number of random rectangular obstacles and add them to the environment. """
        width, height = env.x_range, env.y_range

        for _ in range(n_obstacles):
            # choose random size
            w = random.randint(min_size, max_size)
            h = random.randint(min_size, max_size)

            # choose random top-left corner (make sure obstacle fits)
            x_min = random.randint(0, width - w - 1)
            y_min = random.randint(0, height - h - 1)

            # build obstacle
            build_obstacle((x_min, x_min + w), (y_min, y_min + h), env)

        return env
    
    @staticmethod
    def random_free_cell(env: Grid) -> Tuple[int, int]:
        """Pick a random cell (x,y) where the given robot footprint at (x,y,theta) is collision-free."""
        
        max_tries = 50

        shape = Polygon([(-4, 8), (4, 8), (4, 0), (8, -8), (-8, -8), (-4, 0)])
        
        W, H = env.x_range, env.y_range
    
        for _ in range(max_tries):
            x = random.randint(0, W - 1)
            y = random.randint(0, H - 1)
            pose = (float(x), float(y), 0.0)
            if not PolygonAgent.is_in_collision(pose, shape, env):
                return (x, y)


    @staticmethod
    def random_start_and_goal( env : Grid) -> tuple[tuple[int, int], tuple[int, int]]:
         max_tries = 50

         for _ in range(max_tries):
              start = Randomize.random_free_cell(env)
              goal = Randomize.random_free_cell(env)

              if goal[1] >= start[1]:
                   return start, goal

         
    @staticmethod
    def random_goal(env : Grid):
         return Randomize.random_free_cell(env)