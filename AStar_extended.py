import python_motion_planning as pmp
import plotPolygon
from python_motion_planning.utils import Node


from python_motion_planning.utils import Grid, Map, SearchFactory
from polygonGraphSearcher import PolygonGraphSearcher



class AStarExtended(PolygonGraphSearcher, pmp.AStar):
    pass

