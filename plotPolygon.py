import matplotlib.pyplot as plt
import matplotlib.patches as patches
from python_motion_planning.utils.plot.plot import Plot
import math 
from python_motion_planning.utils import Grid, Map, SearchFactory
from robotDescription import RobotDescription


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