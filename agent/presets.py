from dataclasses import dataclass
from typing import Dict, List, Tuple

from agent import Polygon 




def varying_length(motions: list[tuple], number: int) -> list[tuple]:
    varying_length_motions = motions.copy()
    for motion in motions:
        for i in range(1,number+1):
            if motion[0] != 0:
                varying_length_motions.append((int((motion[0]/float(number)*i)), 0))
            if motion[1] != 0:
                varying_length_motions.append((0,int((motion[1]/float(number)*i))))

    return varying_length_motions



@dataclass(frozen=True)
class ShapeSetMM:
    """A set of polygons for different headings (all in mm)."""
    up: Polygon
    right: Polygon
    left: Polygon
    crouched: Polygon


SHAPES_MM: Dict[str, ShapeSetMM] = {
    "ExactRobot": ShapeSetMM(
    up=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (232, 783), (-232, 783), (-90, 215), (-68, 215)]),
    right=Polygon([(239, 0), (-68, 0), (-68, 576), (413, 576), (280, 364), (261, 279), (239, 279)]),
    left=Polygon([(-239, 0), (68, 0), (68, 576), (-413, 576), (-280, 364), (-261, 279), (-239, 279)]),
    crouched=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (-160, 525), (-90, 215), (-68, 215)]) 
    ),

    "RectangleRobot": ShapeSetMM(
    up=Polygon([(-196, 0), (196, 0), (196, 780), (-196, 780)]),
    right=Polygon([(374, 0), (-68, 0), (-68, 576), (374, 576)]),
    left=Polygon([(-374, 0), (68, 0), (68, 576), (-374, 576)]),
    crouched=Polygon([(-139, 0), (139, 0), (139, 525), (-139, 525)]) 
    ),
}



MOTION_GROUPS_MM: Dict[str, List[Tuple[int, int]]] = {
    "UpSideways": [(0, 200), (170, 0), (-170, 0)],
    "UpSidewaysDown": [(0, 200), (170, 0), (-170, 0), (0,-200)],
    "UpSidewaysDiagonal": [(0, 200), (170, 0), (-170, 0),(141,141),(-141,141)],
    "VaryingLength": varying_length([(0, 200), (170, 0), (-170, 0)],3)
}

RESOLUTION_GROUPS_CM: Dict[str, float] = {
    "RegularRes": 1.7,
    "HighRes": 0.85,
    "LowRes": 3.4
    #comparing resolutions is diffeicult bc of the problem that to get a good start node we need to know robot size to check that start is not in collision
}

PADDING_GROUPS_CM: Dict[str, int] = {
    "RegularPad": 1.7,
    "HighPad": 3.4,
    "LowPad" : 0.85,
}




