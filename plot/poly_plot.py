"""
poly_plot.py

This module provides visualization tools for motion planning with polygonal robots on grid and cylindrical environments.
It extends the base Plot class to support drawing robot polygons, padded footprints, paths, expanded nodes, and animation of planning results.

Key Features:
- PolygonPlot: Visualizes robot shape, path, and search process for polygonal robots.
- Supports both Grid and Cylinder environments, including edge wrapping.
- Draws robot at each path step, with correct local shape for each motion.
- Plots obstacles, start/goal, expanded nodes, and cost curves.
- Used for experiment result visualization and debugging.

Typical Usage:
    plot = PolygonPlot(start, goal, env, robot)
    plot.animation(path, name, cost, ...)
"""

import matplotlib
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
from agent.presets import MOTIONS_CELLS

class PolygonPlot(Plot):
    """
    Visualization class for polygonal robot planning on grid/cylinder environments.
    Extends Plot to support robot polygons, padded footprints, and search animation.
    """
    def __init__(self, start : tuple[int,int], goal : tuple[int,int], env : Grid, robot : PolygonAgent) -> None:
        """
        Initialize the PolygonPlot.
        """
        super().__init__(start, goal, env)
        self.robot = robot
        self._robot_patch = None
        self.current_shape = self.robot.local_shape_crouched
        self.color = "red"

    def drawRobotPolygon(self, pose : tuple[float, float, float]) -> None:
        """
        Draw the robot's polygonal footprint at the given pose.
        """
        self.robot.pose = pose
        poly_world = transform_polygon_local_to_world(pose, self.current_shape)
        # Remove any previous robot polygons from the plot
        for art in list(self.ax.artists):
            if isinstance(art, patches.Polygon):
                art.remove()
        poly = patches.Polygon(poly_world, closed=True, fill=False, edgecolor=self.color, linewidth=2)
        self.ax.add_patch(poly)

    def drawFootprint(self):
        """
        Draw the padded footprint cells of the robot at its current pose.
        """
        footprint = footprint_cells(self.robot.pose, self.current_shape, self.env.x_range, self.env.y_range)
        footprint_padded = padded_footprint(footprint, self.env.x_range,self.env.y_range, self.robot.resolution, self.robot.padding)
        for (ix, iy) in footprint_padded:
            self.ax.plot(ix,iy, marker = "o")
    
    def drawPadding(self):
        """
        Draw only the padding cells (not the base footprint) for the robot at its current pose.
        """
        footprint = footprint_cells(self.current_shape, self.robot.pose, self.env.x_range, self.env.y_range)
        footprint_padded = padded_footprint(footprint, self.env.x_range,self.env.y_range, self.robot.resolution, self.robot.padding)
        base = set(footprint)
        padded = set(footprint_padded)
        padding_only = padded - base
        for (ix, iy) in padding_only:
            self.ax.plot(ix,iy, marker = "o")
    
    def drawPoint(self, x: float, y: float, color: str = "red", size: int = 6) -> None:
        """
        Draw a point on the current plot (useful for marking waypoints).
        """
        self.ax.plot(x, y, marker="o", color=color, markersize=size)

    def animation(self, path, name, cost=None, final_distance = None, expand=None,
                  history_pose=None, predict_path=None,
                  lookahead_pts=None, cost_curve=None, ellipse=None):
        """
        Animate the planning result, drawing the robot at each path step, obstacles, expansions, and optional overlays.
        """
        step_counter = 0
        if path is not None:
            step_counter = len(path)-1
        name = (
        f"{name}\n"
            f"cost: {cost}\n"
            f"step counter: {step_counter}\n"
            f"final distance: {final_distance}"
            )
        # Draw environment, expansions, path, start/goal, overlays
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
            # Draw robot at each step along the path, using the correct local shape for each motion
            path.reverse()
            self.drawRobotPolygon((self.start.x, self.start.y, 0))
            for i in range(0, len(path)-1):
                x, y = path[i]
                next_x, next_y = path[i+1]
                theta = 0
                # Check if we crossed the cylinder edge and adjust coordinates
                crossed_edge = self.env.crossed_edge_check(x, next_x)
                if crossed_edge == "right":
                    next_x += self.env.x_range
                elif crossed_edge == "left":
                    x += self.env.x_range
                current_motion = next_x - x, next_y - y
                # Find the motion name for this step
                for name, motion in MOTIONS_CELLS.items():
                    if motion == current_motion:
                        motion_name = name
                # Assign the correct local shape for this motion
                attr_name = f"local_shape_{motion_name}"
                self.current_shape = getattr(self.robot, attr_name)
                # Use a random color for each step
                self.color = (random.random(), random.random(), random.random())
                self.drawRobotPolygon((x, y, theta))
                self.drawPoint(x, y, color="#888888", size=4)
            # Draw final point that triggers goal
            self.drawPoint(path[-1][0], path[-1][1], "green")

    def plotEnv(self, name: str) -> None:
        """
        Plot the environment, including obstacles, start/goal, and grid/cylinder boundaries.
        """
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
            # Draw map boundaries and obstacles
            for (ox, oy, w, h) in self.env.boundary:
                ax.add_patch(patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='black',
                        fill=True
                    )
                )
            for (ox, oy, w, h) in self.env.obs_rect:
                ax.add_patch(patches.Rectangle(
                        (ox, oy), w, h,
                        edgecolor='black',
                        facecolor='gray',
                        fill=True
                    )
                )
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
        self.ax.set_xticks(np.arange(0, self.env.x_range+1, 1))
        self.ax.set_yticks(np.arange(0, self.env.y_range+1, 1))
        self.ax.grid(which="both", color="lightgray", linewidth=0.3)
 
    def plotPath(self, path: list, path_color: str='#13ae00', path_style: str="-") -> None:
        """
        Plot the planned path on the environment, handling cylinder edge wrapping if needed.
        """
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
                    # Draw line from (x2,y2) to first edge, then from second edge to (x1,y1)
                    plt.plot([x2, x_cross_first_edge], [y2, y_cross], path_style, linewidth=4, color=path_color)
                    plt.plot([x_cross_second_edge, x1], [y_cross, y1], path_style, linewidth=4, color=path_color)
                else: # no edge was crossed
                    plt.plot([x1, x2], [y1, y2], path_style, linewidth=4, color=path_color)
            else: 
                plt.plot([x1, x2], [y1, y2], path_style, linewidth=4, color=path_color)

    def drawStartPos(self):
        """
        Draw the robot's crouched polygon and padded footprint at the start position.
        """
        start_pose = (self.start.x, self.start.y, 0)
        crouched_shape = self.robot.local_shape_crouched
        poly_world = transform_polygon_local_to_world(start_pose, crouched_shape)
        poly = patches.Polygon(poly_world, closed=True, fill=False, edgecolor="red", linewidth=3)
        self.ax.add_patch(poly)
        footprint = footprint_cells(start_pose, crouched_shape, self.env.x_range, self.env.y_range)
        padded = padded_footprint(
            footprint,
            self.env.x_range,
            self.env.y_range,
            self.robot.resolution,
            self.robot.padding
        )
        base = set(footprint)
        extra = set(padded) - base
        self.ax.plot(self.start.x, self.start.y, marker="s", color="red", markersize=10)

    def drawGoalPos(self):
        """
        Draw the robot's crouched polygon and padded footprint at the goal position.
        """
        start_pose = (self.goal.x, self.goal.y, 0)
        crouched_shape = self.robot.local_shape_crouched
        poly_world = transform_polygon_local_to_world(start_pose, crouched_shape)
        poly = patches.Polygon(poly_world, closed=True, fill=False, edgecolor="red", linewidth=3)
        self.ax.add_patch(poly)
        footprint = footprint_cells(start_pose, crouched_shape, self.env.x_range, self.env.y_range)
        padded = padded_footprint(
            footprint,
            self.env.x_range,
            self.env.y_range,
            self.robot.resolution,
            self.robot.padding
        )
        base = set(footprint)
        extra = set(padded) - base
        self.ax.plot(self.start.x, self.start.y, marker="s", color="red", markersize=10)

    def plotExpand(self, expand: list) -> None:
        """
        Plot expanded nodes for Cylinder (and optionally Grid/Map).
        """
        if not expand:
            return
        try:
            if self.start in expand:
                expand.remove(self.start)
            if self.goal in expand:
                expand.remove(self.goal)
        except Exception:
            pass
        if isinstance(self.env, Cylinder):
            for n in expand:
                plt.plot(n.x, n.y, color="#c10808", marker="s", markersize=10)
        elif isinstance(self.env, Grid):
            for n in expand:
                plt.plot(n.x, n.y, color="#dddddd", marker='s', markersize=3)
        elif isinstance(self.env, Map):
            for n in expand:
                if n.parent:
                    plt.plot([n.parent[0], n.x], [n.parent[1], n.y],
                            color="#dddddd", linestyle="-")
        # No plt.pause() here – we’re saving static images

