import python_motion_planning as pmp
import plotPolygon
from python_motion_planning.utils import Node , Grid
from python_motion_planning.global_planner.graph_search.graph_search import GraphSearcher
from robotDescription import RobotDescription
from typing import Optional
from environment import Cylinder
import math



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

            # wrap x coordinate if env is cylinder
            if isinstance(self.env, Cylinder):
                x, y = candidate.current
                x = x % self.env.x_range # wrap around x
                candidate.current = (x,y)

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

    
    def extractPath(self, closed_list: dict) -> tuple:
        """
        Extract the path based on the CLOSED list.

        Parameters:
            closed_list (dict): CLOSED list

        Returns:
            cost (float): the cost of planned path
            path (list): the planning path
        """

        cost = 0
        node = closed_list[self.goal.current]
        path = [node.current]
        while node != self.start:
            node_parent = closed_list[node.parent]
            cost += self.dist(node, node_parent)
            node = node_parent
            path.append(node.current)
        return cost, path
    
    def dist(self, node1: Node, node2: Node) -> float:
        
        dx = abs(node2.x - node1.x)
        dy =  abs(node2.y - node1.y)

        if isinstance(self.env, Cylinder):  # Handle wrap-around in the x direction
            dx = min(dx , self.env.x_range - dx)  # shortest path around the cylinder

        return math.hypot(dx, dy)
    
    def h(self, node: Node, goal: Node) -> float:
        """
        Calculate heuristic.

        Parameters:
            node (Node): current node
            goal (Node): goal node

        Returns:
            h (float): heuristic function value of node
        """
        if self.heuristic_type == "manhattan":
            x_min = min(abs(goal.x - node.x), abs(self.env.x_range - abs(goal.x - node.x))) # check if x distance is closer across grid or across edge
            return x_min + abs(goal.y - node.y) 
        
        elif self.heuristic_type == "euclidean":
            x_min = min(abs(goal.x - node.x), abs(self.env.x_range - abs(goal.x - node.x))) # check if x distance is closer across grid or across edge
            return math.hypot(x_min, goal.y - node.y)
