import python_motion_planning as pmp

from .poly_search import PolygonSearcher


class AStarExtension(PolygonSearcher, pmp.AStar):
    pass

