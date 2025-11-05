import sys, os
import math 
import python_motion_planning as pmp
from python_motion_planning.utils import Grid

from agent import PolygonAgent, Polygon
from planner import AStarExtension 
from environment import Randomize, Cylinder

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))





if __name__ == '__main__':
    '''
    path searcher constructor
    '''

    # build environment
    env = Cylinder(16, 120) # radius and height input
    Randomize.random_obstacles(env, 3)
    start, goal = Randomize.random_start_and_goal(env)

    start = (5, 40)
    goal = (85, 40)
    # env = Cylinder(16, 120)
    # Randomize.random_obstacles(env, 3)

    # randomize.build_obstacle((30,40),(50,60), env)
    # randomize.build_obstacle((10,34),(33,50), env)
    # randomize.build_obstacle((58,70),(30,50), env)


    robot_shape_up = Polygon([(-4,0),(-4,8),(-7,16),(7,16),(4,8), (4,0)])
    robot_shape_right = Polygon([(-4,0),(-4,16),(10,16),(4,8),(4,0)])
    robot_shape_left = Polygon([(-4,0),(-4,8),(-10,16),(4,16),(4,0)])

    robot = PolygonAgent(pose=(start[0], start[1], math.radians(0)), polygon_up=robot_shape_up, polygon_right=robot_shape_right, polygon_left=robot_shape_left)

       
    planner = AStarExtension(start, goal, env=env, robot=robot, allowed_moves=[(1,0), (0,1), (-1,0)], step_cells=8, goal_tol_cells= 5)

    
    cost, path, expand = planner.plan()
    print("path = ", path)
    print("cost = ", cost)

    planner.plot.animation(path, "Shaped A*", cost, expand  = None)



    