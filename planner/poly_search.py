import math
import heapq

from typing import Optional
from python_motion_planning.utils import Node, Grid
from python_motion_planning.global_planner.graph_search.graph_search import GraphSearcher

from agent import PolygonAgent 
from plot import PolygonPlot
from environment import Cylinder

import time




class PolygonSearcher(GraphSearcher):
    def __init__(self, start : tuple[int, int], goal : tuple[int, int], env : Grid, robot : PolygonAgent, heuristic_type : str ="euclidean", goal_tol_cells_x : int=0, goal_tol_cells_y : int=0) -> None:
        super().__init__(start, goal, env, heuristic_type)
        self.robot = robot  # instance of PolygonRobot
        self.plot = PolygonPlot(start, goal, env, robot)
        
        self.motions = robot.motions
        self.goal_tol_cells_x = goal_tol_cells_x
        self.goal_tol_cells_y = goal_tol_cells_y
        self.final_distance_cells = -1
        self.interim_distance = math.inf
        self.interim_closets_candidate_x = -1
        self.interim_closets_candidate_y = -1

        self.max_time_s = 300 #s
        self.t_start = time.perf_counter() 

        #need this for heuristic
        self.max_step_cells = max(math.hypot(m.x, m.y) for m in self.motions)
        print("self.max_step_cells", self.max_step_cells)


        self.expanded_nodes = []
       

    
    def isCollision(self, node_from : Node, node_to : Node, motion : Node) -> bool:
        # DO WE NEED THIS?!
        # keep original A* grid bounds & wall collisions
        # if super().isCollision(node_from, node_to):
        #     return True

        # move robot to candidate location


        #node_from or node_to???
        x, y = node_from.current
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

        if direction == "right":
            shape = self.robot.local_shape_right

        elif direction == "left":
            shape = self.robot.local_shape_left
        

        else:
            shape = self.robot.local_shape_up


        # polygon footprint collision    
        if self.robot.is_in_collision(self.robot.pose, shape, self.env, self.robot.resolution, self.robot.padding):
             return True
        return False
    

    #need this function to set a tolerance
    def getNeighbor(self, node: Node) -> list:

        if time.perf_counter() - self.t_start > self.max_time_s:
                raise TimeoutError("PolygonSearcher exceeded time limit")


        neighbors = []
        if self.goal.current == None:
            print("No goal found")
            raise ValueError
        
        goal_x, goal_y = self.goal.current

        for motion in self.motions:
            candidate = node + motion

            if candidate.current is None:
                continue

            # wrap x coordinate if env is cylinder
            if isinstance(self.env, Cylinder):
                x, y = candidate.current
                x = x % self.env.x_range # wrap around x
                candidate.current = (x,y)

            if self.isCollision(node, candidate, motion):
                continue

            # If within tolerance, snap to exact goal so A* equality triggers
            dx = self.env.dx_min_node(candidate, self.goal)
            dy = abs(candidate.y - goal_y)
            if (dx <= self.goal_tol_cells_x) and (dy <= self.goal_tol_cells_y): # find closets neighbor among all neighbors that are within tolerance and update variables
                if math.hypot(dx, dy) < self.interim_distance: 
                    self.interim_distance = math.hypot(dx,dy)
                    self.interim_closets_candidate_x, self.interim_closets_candidate_y = x,y
            
            neighbors.append(candidate) # append all neighbors

        if self.interim_closets_candidate_x > 0: # check if any of the neighbors is within tolerance, otherwise nothing needs to be changed
            self.final_distance_cells = math.hypot(self.env.dx_min_int(self.interim_closets_candidate_x, self.goal.x), self.interim_closets_candidate_y - self.goal.y) # calculate distance between last path point and original goal point
            self.robot.target = self.goal # save original goal coordinates for later purposes    
            self.goal = Node((self.interim_closets_candidate_x, self.interim_closets_candidate_y)) # snapped goal coordinates to closest neighbor within tolerance

        return neighbors

    
    def dist(self, node1: Node, node2: Node) -> float:
        
        dx = abs(node2.x - node1.x)
        dy =  abs(node2.y - node1.y)

        if isinstance(self.env, Cylinder):  # Handle wrap-around in the x direction
            dx = self.env.dx_min_node(node1, node2)  # shortest path around the cylinder

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
        x_min = abs(goal.x - node.x) 
       

        if isinstance(self.env, Cylinder):
            x_min = self.env.dx_min_node(node, goal)

        # Euclidean distance in cells
        dy = abs(goal.y - node.y)
        d_cells = math.hypot(x_min, dy)

        # Radius (in cells) of the tolerated goal region
        r_tol = math.hypot(self.goal_tol_cells_x, self.goal_tol_cells_y)
        d_eff = max(d_cells - r_tol, 0.0)

        if self.heuristic_type == "manhattan":
            return x_min + abs(goal.y - node.y) 
        
        elif self.heuristic_type == "euclidean":
            return d_eff/self.max_step_cells
        
            #old heuristic
            #return math.hypot(x_min, dy)

    def extractPath(self, closed_list: dict) -> tuple:
        """
        Extract the path based on the CLOSED list.

        Parameters:
            closed_list (dict): CLOSED list

        Returns:
            cost (float): the cost of planned path
            path (list): the planning path
        """
        # --- No path found: goal not in CLOSED ---
       
        cost = 0
        node = closed_list[self.goal.current]
        path = [node.current]
        while node != self.start:
            node_parent = closed_list[node.parent]
            # cost += self.dist(node, node_parent)
            cost += 1
            node = node_parent
            path.append(node.current)
        return cost, path
    
    def plan(self) -> tuple:
        """
        A* motion plan function.

        Returns:
            cost (float): path cost
            path (list): planning path
            expand (list): all nodes that planner has searched
        """
        # OPEN list (priority queue) and CLOSED list (hash table)
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()

        while OPEN:
            node = heapq.heappop(OPEN)

            # exists in CLOSED list
            if node.current in CLOSED:
                continue

            # goal found
            if node == self.goal:
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            for node_n in self.getNeighbor(node):                
                # exists in CLOSED list
                if node_n.current in CLOSED:
                    continue
                
                node_n.parent = node.current
                node_n.h = self.h(node_n, self.goal)

                # goal found
                if node_n == self.goal:
                    heapq.heappush(OPEN, node_n)
                    break
                
                # update OPEN list
                heapq.heappush(OPEN, node_n)

            CLOSED[node.current] = node

            #we need this in case of time out
            self.expanded_nodes.append(node)

        return [], [], list(CLOSED.values())