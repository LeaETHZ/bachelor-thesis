import sys, os
import math 
import python_motion_planning as pmp
from python_motion_planning.utils import Grid

from agent import PolygonAgent, Polygon, robot_factory, presets
from planner import AStarExtension 
from environment import Randomize, Cylinder, randomize
from helper import convert


sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))





if __name__ == '__main__':
    '''
    path searcher constructor
    '''

    # build environment
    env = Cylinder(12, 200) # radius and height input
    #start, goal = (25,25), (25,155)

    Randomize.random_obstacles(env, 2)
    #randomize.build_obstacle((25,35),(60,70),env)
    #randomize.build_obstacle((30,70),(70,80),env)
    randomize.build_obstacle((29,70),(20,30),env)
    
    
    robot = robot_factory.build_robot('ExactRobot', 'VaryingLength', 'RegularRes')

    start, goal = Randomize.random_start_and_goal(env)


    #collision of start cell does not get checked by planner
    #REPLACE THE SHAPE WITH THE CROUCHED SHAPES AND NOT THE MERGED SHAPES
    if PolygonAgent.is_in_collision((start[0], start[1], 0),robot.local_shape_up, env):
        raise ValueError("START position is in collision.")
    # if PolygonAgent.is_in_collision((goal[0], goal[1], 0),robot.local_shape_up, env):
    #     raise ValueError("GOAL position is in collision.")

       
    planner = AStarExtension(start, goal, env, robot, goal_tol_cells= 10)
    
    cost, path, expand = planner.plan()
    print("path = ", path)
    steps = len(path)
    print("Step count: ",steps)

    print("cost = ", cost)
    

    planner.plot.animation(path, "Shaped A*", cost, expand  = None)



    