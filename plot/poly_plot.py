import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import numpy as np
from python_motion_planning.utils.plot.plot import Plot
from python_motion_planning.utils import Grid, Map

from agent import PolygonAgent, Polygon
from environment import Cylinder
import random
from agent.poly import transform_polygon_local_to_world, footprint_cells, padded_footprint





class PolygonPlot(Plot):
    def __init__(self, start : tuple[int,int], goal : tuple[int,int], env : Grid, robot : PolygonAgent) -> None:
        super().__init__(start, goal, env)
        self.robot = robot
        self._robot_patch = None
        self.current_shape = self.robot.local_shape_crouched
        self.color = "red"


    def drawRobotPolygon(self, pose : tuple[float, float, float]) -> None:
        self.robot.pose = pose
        poly_world = transform_polygon_local_to_world(pose, self.current_shape)

        for art in list(self.ax.artists):
            if isinstance(art, patches.Polygon):
                art.remove()

        poly = patches.Polygon(poly_world, closed=True, fill=False, edgecolor=self.color, linewidth=2)
        self.ax.add_patch(poly)

    def drawFootprint(self):
        footprint = footprint_cells(self.robot.pose, self.current_shape, self.env.x_range, self.env.y_range)
        footprint_padded = padded_footprint(footprint, self.env.x_range,self.env.y_range, self.robot.resolution, self.robot.padding)
        #footprint_padded = footprint
        
        for (ix, iy) in footprint_padded:
            self.ax.plot(ix,iy, marker = "o")
    
    def drawPadding(self):
        footprint = footprint_cells(self.current_shape, self.robot.pose, self.env.x_range, self.env.y_range)
        footprint_padded = padded_footprint(footprint, self.env.x_range,self.env.y_range, self.robot.resolution, self.robot.padding)
        
        base = set(footprint)
        padded = set(footprint_padded)

        padding_only = padded - base

        for (ix, iy) in padding_only:
            self.ax.plot(ix,iy, marker = "o")
        
    
    def drawPoint(self, x: float, y: float, color: str = "red", size: int = 6) -> None:
        """Draw a point on the current plot (useful for marking waypoints)."""
        self.ax.plot(x, y, marker="o", color=color, markersize=size)

 
    def animation(self, path, name, cost=None, final_distance = None, expand=None,
                  history_pose=None, predict_path=None,
                  lookahead_pts=None, cost_curve=None, ellipse=None):
        # Re-implement the base animation so we can insert our polygon BEFORE show()
        
        step_counter = 0

        
        if path != None:
            step_counter = len(path)-1

        
        name = (
        f"{name}\n"
            f"cost: {cost}\n"
            f"step counter: {step_counter}\n"
            f"final distance: {final_distance}"
            )
        
        # draw environment, expansions, path (same as base class)
        self.plotEnv(name)

        self.drawStartPos()
        self.drawGoalPos()

        if expand is not None:
            self.plotExpand(expand)
        if history_pose is not None:
            self.plotHistoryPose(history_pose, predict_path, lookahead_pts)
        if path is not None:
            self.plotPath(path)
        if cost_curve:
            plt.figure("cost curve")
            self.plotCostCurve(cost_curve, name)
        if ellipse is not None:
            self.plotEllipse(ellipse)

        if path:
            # downsample trail if path is long (avoid clutter)
            path.reverse()
            self.drawRobotPolygon((self.start.x, self.start.y, 0))
            for i in range(0, len(path)-1):
                x, y = path[i]

                next_x, next_y = path[i+1]
                theta = 0
                dx, dy = next_x - x, next_y - y

                crossed_edge = self.env.crossed_edge_check(x, next_x) # check if we cross edge

                if crossed_edge != None: 
                    if crossed_edge == "right": 
                        self.current_shape = self.robot.local_shape_right 
                    else: 
                        self.current_shape = self.robot.local_shape_left 

                else: # no edges were crossed
                    if dx > 0:
                        self.current_shape = self.robot.local_shape_right
                    
                    elif dx < 0: 
                        self.current_shape = self.robot.local_shape_left
                    
                    elif dy > 0:
                        self.current_shape = self.robot.local_shape_up
                    
                    else:
                        self.current_shape = self.robot.local_shape_up

                self.color = (random.random(), random.random(), random.random())
                self.drawRobotPolygon((x, y, theta))
                self.drawPoint(x, y, color="#888888", size=4)
                #self.drawFootprint()
                #self.drawPadding()
            #draw final point that triggers goal
            self.drawPoint(path[-1][0], path[-1][1], "green")
            #This is the real end
            

        plt.show()

    def plotEnv(self, name: str) -> None:
        '''
        Plot environment with static obstacles.

        Parameters
        ----------
        name: Algorithm name or some other information
        '''
        plt.plot(self.start.x, self.start.y, marker="s", color="#ff0000", markersize=8)
        plt.text(self.start.x + 5.0, self.start.y + 0.3, "START", color="red", fontsize=8)

        plt.plot(self.goal.x, self.goal.y, marker="s", color="#1155cc", markersize=8)
        plt.text(self.goal.x + 5.0, self.goal.y + 0.3, "GOAL", color="#1155cc", fontsize=8)

        if isinstance(self.env, Grid):
            obs_x = [x[0] for x in self.env.obstacles]
            obs_y = [x[1] for x in self.env.obstacles]
            plt.plot(obs_x, obs_y, "sk")

        if isinstance(self.env, Map):
            ax = self.fig.add_subplot()
            # boundary
            for (ox, oy, w, h) in self.env.boundary:
                ax.add_patch(patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='black',
                        fill=True
                    )
                )
            # rectangle obstacles
            for (ox, oy, w, h) in self.env.obs_rect:
                ax.add_patch(patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )
            # circle obstacles
            for (ox, oy, r) in self.env.obs_circ:
                ax.add_patch(patches.Circle(
                        (ox, oy), r,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )

        if isinstance(self.env, Cylinder):
            obs_x = [x[0] for x in self.env.obstacles]
            obs_y = [x[1] for x in self.env.obstacles]
            plt.plot(obs_x, obs_y, "sk")
    
            plt.axvline(0, color='gray', linestyle='--', linewidth=0.5)
            plt.axvline(self.env.x_range-1, color='gray', linestyle='--', linewidth=0.5)

        plt.title(name)
        plt.axis("equal")

        #plotting the grid
        self.ax.set_xticks(np.arange(0, self.env.x_range+1, 1))
        self.ax.set_yticks(np.arange(0, self.env.y_range+1, 1))

       

        self.ax.grid(which="both", color="lightgray", linewidth=0.3)
 
    def plotPath(self, path: list, path_color: str='#13ae00', path_style: str="-") -> None:
        '''
        Plot path in global planning.

        Parameters
        ----------
        path: Path found in global planning
        '''

        for i in range(len(path) - 1):
            x1, y1 = path[i] 
            x2, y2 = path[i + 1] 

            if isinstance(self.env, Cylinder):    
                crossed_edge = self.env.crossed_edge_check(x2,x1)
                if crossed_edge != None: 
                    if crossed_edge == "left": # crossing left edge
                        x_cross_first_edge = 0
                        x_cross_second_edge = self.env.x_range-1
                    
                    else: # crossing right edge
                        x_cross_first_edge = self.env.x_range-1
                        x_cross_second_edge = 0
                    
                    y_cross = y1 + (((y2 - y1)/ (x2 - x1)) * (x_cross_first_edge - x1))
                    plt.plot([x2, x_cross_first_edge], [y2, y_cross], path_style, linewidth=4, color=path_color) # plot line from (x2,y2) to first edge
                    plt.plot([x_cross_second_edge, x1], [y_cross, y1], path_style, linewidth=4, color=path_color) # plot line from second edge to (x1,y1)
                
                else: # no edge was crossed
                    plt.plot([x1, x2], [y1, y2], path_style, linewidth=4, color=path_color)

            else: 
                plt.plot([x1, x2], [y1, y2], path_style, linewidth=4, color=path_color)

        # plot start and goal markers
        #plt.plot(self.start.x, self.start.y, marker="s", color="#ff0000")
        

    
    def drawStartPos(self):
        """Draw crouched polygon + padded footprint at the start position."""
        start_pose = (self.start.x, self.start.y, 0)

        # Fixed crouched shape
        crouched_shape = self.robot.local_shape_crouched
        
        # Polygon in world
        poly_world = transform_polygon_local_to_world(start_pose, crouched_shape)
        poly = patches.Polygon(poly_world, closed=True, fill=False, edgecolor="red", linewidth=3)
        self.ax.add_patch(poly)

        # Footprint
        footprint = footprint_cells(start_pose, crouched_shape, self.env.x_range, self.env.y_range)

        # Padding
        padded = padded_footprint(
            footprint,
            self.env.x_range,
            self.env.y_range,
            self.robot.resolution,
            self.robot.padding
        )

        base = set(footprint)
        extra = set(padded) - base

        # Base footprint cells (blue)
        # for (ix, iy) in base:
        #     self.ax.plot(ix, iy, marker="o", color="blue")

        # Padding-only cells (cyan)
        # for (ix, iy) in extra:
        #     self.ax.plot(ix, iy, marker="o", color="cyan")

        # Start marker
        self.ax.plot(self.start.x, self.start.y, marker="s", color="red", markersize=10)


    def drawGoalPos(self):

        """Draw crouched polygon + padded footprint at the start position."""
        start_pose = (self.goal.x, self.goal.y, 0)

        # Fixed crouched shape
        crouched_shape = self.robot.local_shape_crouched
        
        # Polygon in world
        poly_world = transform_polygon_local_to_world(start_pose, crouched_shape)
        poly = patches.Polygon(poly_world, closed=True, fill=False, edgecolor="red", linewidth=3)
        self.ax.add_patch(poly)

        # Footprint
        footprint = footprint_cells(start_pose, crouched_shape, self.env.x_range, self.env.y_range)

        # Padding
        padded = padded_footprint(
            footprint,
            self.env.x_range,
            self.env.y_range,
            self.robot.resolution,
            self.robot.padding
        )

        base = set(footprint)
        extra = set(padded) - base

        # Base footprint cells (blue)
        # for (ix, iy) in base:
        #     self.ax.plot(ix, iy, marker="o", color="blue")

        # Padding-only cells (cyan)
        # for (ix, iy) in extra:
        #     self.ax.plot(ix, iy, marker="o", color="cyan")

        # Start marker
        self.ax.plot(self.start.x, self.start.y, marker="s", color="red", markersize=10)

    def plotExpand(self, expand: list) -> None:
            """
            Plot expanded nodes for Cylinder (and optionally Grid/Map).
            Works in headless mode (Agg) – no pauses.
            """
            if not expand:
                return

            # Remove start/goal if they are Node-like
            try:
                if self.start in expand:
                    expand.remove(self.start)
                if self.goal in expand:
                    expand.remove(self.goal)
            except Exception:
                pass

            # Handle Cylinder like a Grid: just plot their x,y
            if isinstance(self.env, Cylinder):
                for n in expand:
                    # Node from PMP usually has .x, .y
                    plt.plot(n.x, n.y, color="#c10808", marker="s", markersize=10)

            # (Optional) keep old behaviour for Grid
            elif isinstance(self.env, Grid):
                for n in expand:
                    plt.plot(n.x, n.y, color="#dddddd", marker='s', markersize=3)

            # (Optional) keep old behaviour for Map
            elif isinstance(self.env, Map):
                for n in expand:
                    if n.parent:
                        plt.plot([n.parent[0], n.x], [n.parent[1], n.y],
                                color="#dddddd", linestyle="-")

            # No plt.pause() here – we’re saving static images


