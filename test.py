import sys, os
import math 
import python_motion_planning as pmp
from python_motion_planning.utils import Grid

from agent import PolygonAgent, Polygon, presets, robot_factory
from planner import AStarExtension 
from environment import Randomize, Cylinder, randomize
from helper import convert
from agent.presets import SHAPES_MM, MOTION_GROUPS_MM, RESOLUTION_GROUPS_CM, PADDING_GROUPS_CM


sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))





if __name__ == '__main__':
    '''
    path searcher constructor
    '''

    # build environment
    env = Cylinder(12, 200) # radius and height input
    #start, goal = (25,25), (25,155)


    # Randomize.random_obstacles_rectangle(env, 4)
    Randomize.random_obstacles_ellipse(env, 1)
    
    robot = robot_factory.build_robot('ExactRobot', 'UpSideways', 'RegularRes','LowPad')
    res = RESOLUTION_GROUPS_CM["RegularRes"]
    pad = PADDING_GROUPS_CM["LowPad"]   
    start, goal = Randomize.random_start_and_goal(env, res, pad)

    # start = (20, 2)
    # goal = (40, 100)


    #collision of start cell does not get checked by planner
    #REPLACE THE SHAPE WITH THE CROUCHED SHAPES AND NOT THE MERGED SHAPES
    if PolygonAgent.is_in_collision((start[0], start[1], 0),robot.local_shape_crouched, env, robot.resolution, robot.padding):
        raise ValueError("START position is in collision.")
    # if PolygonAgent.is_in_collision((goal[0], goal[1], 0),robot.local_shape_up, env):
    #     raise ValueError("GOAL position is in collision.")
    
    goal_tol_cells_x = math.ceil(8.5/res) 
    goal_tol_cells_y = math.ceil(10/res)

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
    print ("final distance = ", planner.final_distance)

    planner.plot.animation(path, "Shaped A*", cost, planner.final_distance, expand  = None)
    
    






    