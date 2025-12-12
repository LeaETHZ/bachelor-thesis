from dataclasses import dataclass
from typing import Dict, List, Tuple

from agent import Polygon 
from helper import convert




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

    down: Polygon
    diagonal_right: Polygon
    diagonal_left: Polygon

    up_2_3: Polygon
    up_1_3: Polygon
    right_2_3: Polygon
    right_1_3: Polygon
    left_2_3: Polygon
    left_1_3: Polygon

SHAPES_MM: Dict[str, ShapeSetMM] = {
    "ExactRobot": ShapeSetMM(
    up=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (232, 783), (-232, 783), (-90, 215), (-68, 215)]),
    right=Polygon([(239, 0), (-68, 0), (-68,215), (-90, 215), (-172, 576), (413, 576), (280, 364), (261, 279), (239, 279)]),
    left=Polygon([(-239, 0), (68, 0), (68,215), (90, 215), (172, 576), (-413, 576), (-280, 364), (-261, 279), (-239, 279)]),
    crouched=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (-160, 525), (-90, 215), (-68, 215)]),

    down=Polygon([(-68, -258), (68, -258), (68, -43), (90, -43), (232, 525), (-232, 525), (-90, -43), (-68, -43)]),
    diagonal_right=Polygon([(68, 0), (-68, 0), (-68, 215), (-90, 215), (-160, 525), (-19, 717), (385, 717), (252, 505), (231, 420), (209, 420), (209, 141)]),
    diagonal_left=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (19, 717), (-385, 717), (-252, 505), (-231, 420), (-209, 420), (-209, 141)]),

    up_2_3=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (213, 716), (-213, 716), (-90, 215), (-68, 215)]),
    up_1_3=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (193, 649), (-193, 649), (-90, 215), (-68, 215)]),
    right_2_3=Polygon([(182, 0), (-68, 0), (-68,215), (-90, 215), (-172, 576), (356, 576), (223, 364), (204, 279), (187, 279)]),
    right_1_3=Polygon([(125, 0), (-68, 0), (-68,215), (-90, 215), (-172, 576), (299, 576), (166, 364), (147, 279), (125, 279)]),
    left_2_3=Polygon([(-182, 0), (68, 0), (68,215), (90, 215), (172, 576), (-356, 576), (-223, 364), (-204, 279), (-187, 279)]),
    left_1_3=Polygon([(-125, 0), (68, 0), (68,215), (90, 215), (172, 576), (-299, 576), (-166, 364), (-147, 279), (-125, 279)])
    ),

    "RectangleRobot": ShapeSetMM(
    up=Polygon([(-232, 0), (232, 0), (232, 783), (-232, 783)]),
    right=Polygon([(413, 0), (-172, 0), (-172, 576), (413, 576)]),
    left=Polygon([(-413, 0), (172, 0), (172, 576), (-413, 576)]),
    crouched=Polygon([(-160, 0), (160, 0), (160, 525), (-160, 525)]), 

    down=Polygon([(-232, -258),(232, -258), (232, 525), (-232,525)]),
    diagonal_right=Polygon([(385, 0), (-160, 0), (-160, 717), (385, 717)]),
    diagonal_left=Polygon([(-385, 0), (160, 0), (160, 717), (-385, 717)]),
                           
    up_2_3=Polygon([(-213, 0), (213, 0), (213, 716), (-213, 716)]),
    up_1_3=Polygon([(-193, 0), (193, 0), (193, 649), (-193, 649)]),
    right_2_3=Polygon([(356, 0), (-172, 0), (-172, 576), (356, 576)]),
    right_1_3=Polygon([(299, 0), (-172, 0), (-172, 576), (299, 576)]),
    left_2_3=Polygon([(-356, 0), (172, 0), (172, 576), (-356, 576)]),
    left_1_3=Polygon([(-299, 0), (172, 0), (172, 576), (-299, 576)]),
    )
}


MOTION_GROUPS_MM: Dict[str, List[Tuple[int, int]]] = {
    "UpSideways": [(0, 200), (170, 0), (-170, 0)],
    "UpSidewaysDown": [(0, 200), (170, 0), (-170, 0), (0,-200)],
    "UpSidewaysDiagonal": [(0, 200), (170, 0), (-170, 0),(141,141),(-141,141)],
    "VaryingLength": varying_length([(0, 200), (170, 0), (-170, 0)],3) 
    # VaryingLength = [(0, 200), (170, 0), (-170, 0), (0, 66), (0, 133), (0, 200), (56, 0), (113, 0), (170, 0), (-56, 0), (-113, 0), (-170, 0)]
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


# Dictionary assignin motion name to motion tuple given in cells
MOTIONS_CELLS: Dict[str, Tuple[int, int]] = {
    "up" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSideways"][0], RESOLUTION_CM),
    "right" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSideways"][1], RESOLUTION_CM),
    "left" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSideways"][2], RESOLUTION_CM),
    
    "down" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSidewaysDown"][3], RESOLUTION_CM),
    "diagonal_right" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSidewaysDiagonal"][3], RESOLUTION_CM),
    "diagonal_left" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSidewaysDiagonal"][4], RESOLUTION_CM),

    "up_2_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][4], RESOLUTION_CM),
    "up_1_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][3], RESOLUTION_CM),
    "right_2_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][7], RESOLUTION_CM),
    "right_1_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][6], RESOLUTION_CM),
    "left_2_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][10], RESOLUTION_CM),
    "left_1_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][9], RESOLUTION_CM),
    }


