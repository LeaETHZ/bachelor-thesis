from dataclasses import dataclass
from typing import Dict, List, Tuple

from agent import Polygon 


@dataclass(frozen=True)
class ShapeSetMM:
    """A set of polygons for different headings (all in mm)."""
    up: Polygon
    right: Polygon
    left: Polygon


SHAPES_MM: Dict[str, ShapeSetMM] = {
    "ExactRobot": ShapeSetMM(
    up=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (232, 783), (-232, 783), (-90, 215), (-68, 215)]),
    right=Polygon([(239, 0), (-68, 0), (-68, 576), (413, 576), (280, 364), (261, 279), (239, 279)]),
    left=Polygon([(-239, 0), (68, 0), (68, 576), (-413, 576), (-280, 364), (-261, 279), (-239, 279)]),
    ),
}



MOTION_GROUPS_MM: Dict[str, List[Tuple[int, int]]] = {
    "UpSideways": [(0, 200), (170, 0), (-170, 0)],
    "UpSidewaysDown": [(0, 200), (170, 0), (-170, 0), (0,-200)],
    "UpSidewaysDiagonal": [(0, 200), (170, 0), (-170, 0),(141,141),(-141,141)],
    "VaryingLenght": [(0, 200), (170, 0), (-170, 0)]
}

RESOLUTION_GROUPS_CM: Dict[str, float] = {
    "RegularRes": 1.7,
}

