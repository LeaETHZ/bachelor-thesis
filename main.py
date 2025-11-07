import sys, os
import python_motion_planning as pmp
from python_motion_planning.utils import Grid
import agent.poly_agent as rd
import math 
import planner.a_star_ext as ASE
from environment.randomize import Randomize
from agent.poly import Polygon
from environment import Cylinder
from helper import convert

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))





if __name__ == '__main__':
    '''
    path searcher constructor
    '''

    # build environment
    env = Cylinder(12, 200) # radius and height input
    #Randomize.random_obstacles(env, 3)
    start, goal = Randomize.random_start_and_goal(env)

    # start = (85, 40)
    # goal = (8, 40)
    # env = Cylinder(16, 120)
    # Randomize.random_obstacles(env, 3)

    # randomize.build_obstacle((30,40),(50,60), env)
    # randomize.build_obstacle((10,34),(33,50), env)
    # randomize.build_obstacle((58,70),(30,50), env)

    res = 1.7 # in cm


    robot_shape_up = convert.convertPolygonToCells(Polygon([(-68,0),(68,0),(68,215),(90,215),(232,783),(-232,783), (-90,215),(-68,215)]), res) #in mm
    robot_shape_right = convert.convertPolygonToCells(Polygon([(239,0),(-68,0),(-68,576),(413,576),(280,364),(261,279),(239,279)]),res)
    robot_shape_left = convert.convertPolygonToCells(Polygon([(-239,0),(68,0),(68,576),(-413,576),(-280,364),(-261,279),(-239,279)]),res)

    #robot = rd.PolygonAgent((start[0], start[1], math.radians(0)), robot_shape_up, robot_shape_right, robot_shape_left, motions=[(0, 14), (10, 0),(-10, 0)])
    robot = rd.PolygonAgent((start[0], start[1], math.radians(0)), robot_shape_up, robot_shape_right, robot_shape_left, motions=[convert.convertToCells((0, 200), res), convert.convertToCells((170, 0), res),convert.convertToCells((-170, 0),res)])

    #self.motions = [Node((0, 14)), Node((10, 0)), Node((-10, 0))]

       
    planner = ASE.AStarExtension(start, goal, env=env, robot=robot, step_cells=14, goal_tol_cells= 5)

    
    cost, path, expand = planner.plan()
    print("path = ", path)
    print("cost = ", cost)

    planner.plot.animation(path, "Shaped A*", cost, expand  = None)



    