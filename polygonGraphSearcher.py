import python_motion_planning as pmp
import plotPolygon
from python_motion_planning.utils import Node , Grid
from python_motion_planning.global_planner.graph_search.graph_search import GraphSearcher
from robotDescription import RobotDescription
from typing import Optional



class PolygonGraphSearcher(GraphSearcher):
    def __init__(self, start : tuple[int, int], goal : tuple[int, int], env : Grid, robot : RobotDescription, heuristic_type : str ="euclidean", allowed_moves : Optional[list[tuple[int, int]]]=None, step_cells : int =1, goal_tol_cells : int=0) -> None:
        super().__init__(start, goal, env, heuristic_type)
        self.robot = robot  # instance of PolygonRobot
        self.plot = plotPolygon.PlotPolygon(start, goal, env, robot)
        

        if allowed_moves is None:
            allowed_moves = [(1,0),(0,1),(-1,0),(0,-1)]
        
        self.motions = [Node((dx * step_cells, dy * step_cells)) for (dx, dy) in allowed_moves]
        self.goal_tol_cells = int(goal_tol_cells)

    
    def isCollision(self, node_from : Node, node_to : Node, motion : Node) -> bool:
        # keep original A* grid bounds & wall collisions
        if super().isCollision(node_from, node_to):
            return True

        # move robot to candidate location
        x, y = node_to.current
        theta = self.robot.pose[2]   # keep orientation constant 
        self.robot.pose = (x, y, theta)

        dx, dy = motion.current   # e.g. (3, 0), (0, -3), etc.

        # normalize sign (since step_cells could be >1)
        if dx > 0:
            direction = "right"
        elif dx < 0:
            direction = "left"
        elif dy > 0:
            direction = "up"
        elif dy < 0:
            direction = "down"
        else:
            direction = "none"

        if direction in ("right"):
            shape = self.robot.local_shape_right

        if direction in ("left"):
            shape = self.robot.local_shape_left
        

        else:
            shape = self.robot.local_shape_up

        # polygon footprint collision    
        if self.robot.is_in_collision(self.robot.pose, shape, self.env):
             return True
    

    #need this function to set a tolerance
    def getNeighbor(self, node: Node) -> list:
        neighbors = []
        goal_x, goal_y = self.goal.current
        tol_squared = self.goal_tol_cells * self.goal_tol_cells  # squared tol for quick check

        for motion in self.motions:
            candidate = node + motion
            if self.isCollision(node, candidate, motion):
                continue

            # If within tolerance, snap to exact goal so A* equality triggers
            if self.goal_tol_cells > 0:
                dx = candidate.current[0] - goal_x
                dy = candidate.current[1] - goal_y
                if dx*dx + dy*dy <= tol_squared:
                    candidate.current = (goal_x, goal_y)

            neighbors.append(candidate)
        return neighbors


