import sys, os
import math 
import python_motion_planning as pmp
from python_motion_planning.utils import Grid

from agent import PolygonAgent, Polygon, robot_factory, presets
from planner import AStarExtension 
from environment import Randomize, Cylinder
from helper import convert

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))





if __name__ == '__main__':
    '''
    path searcher constructor
    '''

    # build environment
    env = Cylinder(12, 200) # radius and height input
    start, goal = Randomize.random_start_and_goal(env)

    Randomize.random_obstacles(env, 2)
    
    robot = robot_factory.build_robot('ExactRobot', 'UpAndSideways', 'RegularRes', (start,goal, 0))

       
    planner = AStarExtension(start, goal, env, robot, goal_tol_cells= 10)
    
    cost, path, expand = planner.plan()
    print("path = ", path)
    print("cost = ", cost)
    

    planner.plot.animation(path, "Shaped A*", cost, expand  = None)



    