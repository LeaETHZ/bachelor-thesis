import sys, os
import python_motion_planning as pmp
from python_motion_planning.utils import Grid
import robotDescription as rd
import math 
import AStar_extended as ASE
from randomize import Randomize
from polygon import Polygon
from environment import Cylinder

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))





if __name__ == '__main__':
    '''
    path searcher constructor
    '''

    # build environment
    # env = Cylinder(100, 120)
    # Randomize.random_obstacles(env, 3)
    # start, goal = Randomize.random_start_and_goal(env)

    start = (95, 30)
    goal = (3, 60)
    env = Cylinder(100, 120)
    Randomize.random_obstacles(env, 3)

    robot_shape_up = Polygon([(-4, 8), (4, 8), (4, 0), (8, -8), (-8, -8), (-4, 0)])
    robot_shape_right = Polygon([(-4, 8), (4, 8), (4, 0), (10, -8), (-4, -8), (-4, 0)])
    robot_shape_left = Polygon([(4, 8), (-4, 8), (-4, 0), (-10, -8), (4, -8), (4, 0)])


    robot = rd.RobotDescription(pose=(10, 7, math.radians(0)), polygon_up=robot_shape_up, polygon_right=robot_shape_right, polygon_left=robot_shape_left)


       
    planner = ASE.AStarExtended(start, goal, env=env, robot=robot, allowed_moves=[(1,0), (0,1), (-1,0)], step_cells=8, goal_tol_cells= 5)

    
    cost, path, expand = planner.plan()
    print("path = ", path)
    print("cost = ", cost)

    planner.plot.animation(path, "Shaped A*", cost, expand  = None)



    