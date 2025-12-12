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
    right=Polygon([(239, 0), (-68, 0), (-68,215), (-90, 215), (-172, 576), (413, 576), (280, 364), (261, 279), (239, 279)]),
    left=Polygon([(-239, 0), (68, 0), (68,215), (90, 215), (172, 576), (-413, 576), (-280, 364), (-261, 279), (-239, 279)]),
    crouched=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (-160, 525), (-90, 215), (-68, 215)]),

    down=Polygon([(-68, -200), (68, -200), (68, 15), (90, 15), (232, 525), (-232, 525), (-90, 15), (-68, 15)]),
    diagonal_right=Polygon([(68, 0), (-68, 0), (-68, 215), (-90, 215), (-160, 525), (-19, 717), (385, 717), (252, 505), (231, 420), (209, 420), (209, 141)]),
    diagonal_left=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (19, 717), (-385, 717), (-252, 505), (-231, 420), (-209, 420), (-209, 141)]),

    up_2_3=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (213, 716), (-213, 716), (-90, 215), (-68, 215)]),
    up_1_3=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (193, 649), (-193, 649), (-90, 215), (-68, 215)]),
    right_2_3=Polygon([(182, 0), (-68, 0), (-68,215), (-90, 215), (-172, 576), (356, 576), (223, 364), (204, 279), (187, 279)]),
    right_1_3=Polygon([(125, 0), (-68, 0), (-68,215), (-90, 215), (-172, 576), (299, 576), (166, 364), (147, 279), (125, 279)]),
    left_2_3=Polygon([(-182, 0), (68, 0), (68,215), (90, 215), (172, 576), (-356, 576), (-223, 364), (-204, 279), (-187, 279)]),
    left_1_3=Polygon([(-125, 0), (68, 0), (68,215), (90, 215), (172, 576), (-299, 576), (-166, 364), (-147, 279), (-125, 279)]),
    ),

    "RectangleRobot": ShapeSetMM(
    up=Polygon([(-232, 0), (232, 0), (232, 783), (-232, 783)]),
    right=Polygon([(413, 0), (-68, 0), (-68, 576), (413, 576)]),
    left=Polygon([(-413, 0), (68, 0), (68, 576), (-413, 576)]),
    crouched=Polygon([(-160, 0), (160, 0), (160, 525), (-160, 525)]) 
    ),
}


MOTION_GROUPS_MM: Dict[str, List[Tuple[int, int]]] = {
    "UpSideways": [(0, 200), (170, 0), (-170, 0)],
    "UpSidewaysDown": [(0, 200), (170, 0), (-170, 0), (0,-200)],
    "UpSidewaysDiagonal": [(0, 200), (170, 0), (-170, 0),(141,141),(-141,141)],
    "VaryingLength": varying_length([(0, 200), (170, 0), (-170, 0)],3)
}


RESOLUTION_CM: float = 1.7        # The ONLY resolution value


PADDING_GROUPS_CM: Dict[str, int] = {
    "higher20" : 20,
    "higher15" : 15,
    "higher10" : 10,
    "higher7" : 7,
    "higher" : 5,
    "HighPad": 3.4,
    "RegularPad": 1.7,
    "LowPad" : 0.85,
    "NoPad" : 0
}




