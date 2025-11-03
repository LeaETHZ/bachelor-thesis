import matplotlib.pyplot as plt
import matplotlib.patches as patches
from python_motion_planning.utils.plot.plot import Plot
import math 
from python_motion_planning.utils import Grid, Map, SearchFactory
from robotDescription import RobotDescription
from environment import Cylinder


class PlotPolygon(Plot):
    def __init__(self, start : tuple[int,int], goal : tuple[int,int], env : Grid, robot : RobotDescription) -> None:
        super().__init__(start, goal, env)
        self.robot = robot
        self._robot_patch = None
        self.current_shape = self.robot.local_shape_up


    def drawRobotPolygon(self, pose : tuple[float, float, float]) -> None:
        self.robot.pose = pose
        poly_world = self.current_shape.transform_polygon_local_to_world(pose)

        for art in list(self.ax.artists):
            if isinstance(art, patches.Polygon):
                art.remove()

        poly = patches.Polygon(poly_world, closed=True, fill=False, edgecolor="purple", linewidth=2)
        self.ax.add_patch(poly)
    
    def drawPoint(self, x: float, y: float, color: str = "red", size: int = 6) -> None:
        """Draw a point on the current plot (useful for marking waypoints)."""
        self.ax.plot(x, y, marker="o", color=color, markersize=size)

    def animation(self, path, name, cost=None, expand=None,
                  history_pose=None, predict_path=None,
                  lookahead_pts=None, cost_curve=None, ellipse=None):
        # Re-implement the base animation so we can insert our polygon BEFORE show()
        name = name + "\ncost: " + str(cost) if cost else name

        # draw environment, expansions, path (same as base class)
        self.plotEnv(name)


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
            for i in range(1, len(path)-1):
                x, y = path[i]

                next_x, next_y = path[i+1]
                theta = math.pi
                dx, dy = next_x - x, next_y - y
                
                if dx > 0:
                    self.current_shape = self.robot.local_shape_right
                
                elif dx < 0: 
                    self.current_shape = self.robot.local_shape_left
                
                elif dy > 0:
                    self.current_shape = self.robot.local_shape_up
                
                else:
                    self.current_shape = self.robot.local_shape_up

                    
                self.drawRobotPolygon((x, y, theta))
                self.drawPoint(x, y)

        plt.show()

    def plotEnv(self, name: str) -> None:
        '''
        Plot environment with static obstacles.

        Parameters
        ----------
        name: Algorithm name or some other information
        '''
        plt.plot(self.start.x, self.start.y, marker="s", color="#ff0000")
        plt.plot(self.goal.x, self.goal.y, marker="s", color="#1155cc")

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
 
    def plotPath(self, path: list, path_color: str='#13ae00', path_style: str="-") -> None:
        '''
        Plot path in global planning.

        Parameters
        ----------
        path: Path found in global planning
        '''

        for i in range(len(path) - 1):
            x1, y1 = path[i] # step closer to target, path is target -> start
            x2, y2 = path[i + 1] # step closer to start
            dx = abs(x2 - x1)

            if isinstance(self.env, Cylinder) and (dx > (self.env.x_range -dx)):    # check whether we crossed borders

                if x1 > x2: # crossing left edge
                    x_cross = 0
                    y_cross = y1 + (((y2 - y1)/ (x2 - x1)) * (x_cross - x1))
                    plt.plot([x2, x_cross], [y2, y_cross], path_style, linewidth=2, color=path_color) # plot line from (x2,y2) to left border

                    x_cross = self.env.x_range-1
                    plt.plot([x_cross, x1], [y_cross, y1], path_style, linewidth=2, color=path_color) # plot line from right border to (x1,y1)
                    print("crossing left edge")
                
                else: # crossing right edge
                    x_cross = self.env.x_range-1
                    y_cross = y1 + (((y2 - y1)/ (x2 - x1)) * (x_cross - x1))
                    plt.plot([x2, x_cross], [y2, y_cross], path_style, linewidth=2, color=path_color) # plot line from (x2,y2) to right border

                    x_cross = 0
                    plt.plot([x_cross, x1], [y_cross, y1], path_style, linewidth=2, color=path_color) # plot line from left border to (x1,y1)
                    print("crossing right edge")

            else:
                # normal connection
                plt.plot([x1, x2], [y1, y2], path_style, linewidth=2, color=path_color)

        # plot start and goal markers
        plt.plot(self.start.x, self.start.y, marker="s", color="#ff0000")
        plt.plot(self.goal.x, self.goal.y, marker="s", color="#1155cc")
