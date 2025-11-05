import python_motion_planning as pmp
import plot.poly_plot as poly_plot
from python_motion_planning.utils import Node


from python_motion_planning.utils import Grid, Map, SearchFactory
from planner.poly_search import PolygonSearcher



class AStarExtension(PolygonSearcher, pmp.AStar):
    pass

