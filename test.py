import sys, os
import math 

import matplotlib
import matplotlib.pyplot as plt 
import python_motion_planning as pmp
from python_motion_planning.utils import Grid
import numpy as np

from agent import PolygonAgent, Polygon, presets, robot_factory
from planner import AStarExtension 
from environment import Randomize, Cylinder, randomize
from helper import convert
from agent.presets import SHAPES_MM, MOTION_GROUPS_MM, PADDING_GROUPS_CM, RESOLUTION_CM
from evaluation import load_data


sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))





if __name__ == '__main__':
    '''
    path searcher constructor
    '''
    
    robot = robot_factory.build_robot('ExactRobot', 'UpSidewaysDiagonal','NoPad')
    pad = PADDING_GROUPS_CM["NoPad"]   

    # build environment
    CYL_RADIUS_CM = 20
    CYL_HEIGHT_CM = 500

    cyl_radius_cells = np.round(CYL_RADIUS_CM/RESOLUTION_CM)
    cyl_height_cells = np.round(CYL_HEIGHT_CM/RESOLUTION_CM)
    env = Cylinder(cyl_radius_cells, cyl_height_cells)
   
    # Randomize.random_obstacles_rectangle(env, 4, 50, 5)
    # Randomize.random_obstacles_ellipse(env, 1, 25, 2.5)
    start, goal = Randomize.random_start_and_goal(env, RESOLUTION_CM, pad)
    #randomize.build_obstacle_rectangle([10,20],[20,25], env)

    # MAP_PATH = os.path.join(os.path.dirname(__file__), "newnewarray.npy")

    # env = load_data.load_cylinder_from_npy(MAP_PATH, RESOLUTION_CM, obstacle_threshold=0.5)


    
    

    # start = (40, 30)
    # goal = (53, 45)


    #collision of start cell does not get checked by planner
    #REPLACE THE SHAPE WITH THE CROUCHED SHAPES AND NOT THE MERGED SHAPES
    if PolygonAgent.is_in_collision((start[0], start[1], 0),robot.local_shape_crouched, env, robot.resolution, robot.padding):
        raise ValueError("START position is in collision.")
    # if PolygonAgent.is_in_collision((goal[0], goal[1], 0),robot.local_shape_up, env):
    #     raise ValueError("GOAL position is in collision.")
    
    goal_tol_cells_x = math.ceil(8.5/RESOLUTION_CM) 
    goal_tol_cells_y = math.ceil(10/RESOLUTION_CM)

    # print("goal_tol_cells_x = ", goal_tol_cells_x)
    # print("goal_tol_cells_y = ", goal_tol_cells_y)
    


    planner = AStarExtension(start, goal, env, robot, goal_tol_cells_x = goal_tol_cells_x, goal_tol_cells_y = goal_tol_cells_y)
    
    cost, path, expand = planner.plan()
    print("start = ", start)
    print("goal = ", goal)
    print("path = ", path)
    # steps = len(path)-1
    # print("Step count: ",steps)

    # print("cost = ", cost)
    print ("final distance = ", planner.final_distance_cells)

    planner.plot.animation(path, "Shaped A*", cost, planner.final_distance_cells, expand  = None)
    
    






    