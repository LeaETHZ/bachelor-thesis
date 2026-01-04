"""
This module defines robot shape, motion, and padding presets for experiments.
It provides:
 - ShapeSetMM: dataclass for robot polygon shapes in different postures
 - SHAPES_MM: dictionary of named robot shape sets
 - MOTION_GROUPS_MM: dictionary of named motion primitive sets
 - RESOLUTION_CM: the grid resolution used throughout the project
 - PADDING_GROUPS_CM: named padding values for collision checking
 - MOTIONS_CELLS: mapping from motion names to scaled grid cell moves
 - varying_length: helper to generate motion primitives of different lengths
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple

from agent import Polygon
from helper import convert


def varying_length(motions: list[tuple], number: int) -> list[tuple]:
    """
    Generate a set of motion primitives with varying lengths based on the input motions.
    For each motion, creates 'number' scaled versions (except for zero components).
    """
    varying_length_motions = motions.copy()
    for motion in motions:
        for i in range(1, number):
            if motion[0] != 0:
                varying_length_motions.append((int((motion[0] / float(number) * i)), 0))
            if motion[1] != 0:
                varying_length_motions.append((0, int((motion[1] / float(number) * i))))
    return varying_length_motions


@dataclass(frozen=True)
class ShapeSetMM:
    """
    A set of robot polygons (in mm) for different headings/postures.
    Used to describe the robot's shape for each possible motion direction.
    """
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

# Dictionary of named robot shape sets (ExactRobot, RectangleRobot, etc.)
SHAPES_MM: Dict[str, ShapeSetMM] = {
    "ExactRobot": ShapeSetMM(
    up=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (245, 783), (-245, 783), (-90, 215), (-68, 215)]),
    right=Polygon([(268, 0), (-68, 0), (-68,215), (-90, 215), (-172, 576), (442, 576), (309, 364), (290, 279), (268, 279)]),
    left=Polygon([(-268, 0), (68, 0), (68,215), (90, 215), (172, 576), (-442, 576), (-309, 364), (-290, 279), (-268, 279)]),
    crouched=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (-160, 525), (-90, 215), (-68, 215)]),

    down=Polygon([(-68, -258), (68, -258), (68, -43), (90, -43), (245, 525), (-245, 525), (-90, -43), (-68, -43)]),
    diagonal_right=Polygon([(68, 0), (-68, 0), (-68, 215), (-90, 215), (-160, 525), (-19, 717), (385, 717), (252, 505), (231, 420), (209, 420), (209, 141)]),
    diagonal_left=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (160, 525), (19, 717), (-385, 717), (-252, 505), (-231, 420), (-209, 420), (-209, 141)]),

    up_2_3=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (227, 716), (-227, 716), (-90, 215), (-68, 215)]),
    up_1_3=Polygon([(-68, 0), (68, 0), (68, 215), (90, 215), (208, 649), (-208, 649), (-90, 215), (-68, 215)]),
    right_2_3=Polygon([(201, 0), (-68, 0), (-68,215), (-90, 215), (-172, 576), (375, 576), (242, 364), (223, 279), (201, 279)]),
    right_1_3=Polygon([(134, 0), (-68, 0), (-68,215), (-90, 215), (-172, 576), (308, 576), (175, 364), (156, 279), (134, 279)]),
    left_2_3=Polygon([(-201, 0), (68, 0), (68,215), (90, 215), (172, 576), (-375, 576), (-242, 364), (-223, 279), (-201, 279)]),
    left_1_3=Polygon([(-134, 0), (68, 0), (68,215), (90, 215), (172, 576), (-308, 576), (-175, 364), (-156, 279), (-134, 279)])
    ),

    "RectangleRobot": ShapeSetMM(
    up=Polygon([(-245, 0), (245, 0), (245, 783), (-245, 783)]),
    right=Polygon([(442, 0), (-172, 0), (-172, 576), (442, 576)]),
    left=Polygon([(-442, 0), (172, 0), (172, 576), (-442, 576)]),
    crouched=Polygon([(-160, 0), (160, 0), (160, 525), (-160, 525)]), 

    down=Polygon([(-245, -258),(245, -258), (245, 525), (-245,525)]),
    diagonal_right=Polygon([(385, 0), (-160, 0), (-160, 717), (385, 717)]),
    diagonal_left=Polygon([(-385, 0), (160, 0), (160, 717), (-385, 717)]),
                           
    up_2_3=Polygon([(-227, 0), (227, 0), (227, 716), (-227, 716)]),
    up_1_3=Polygon([(-208, 0), (208, 0), (208, 649), (-208, 649)]),
    right_2_3=Polygon([(375, 0), (-172, 0), (-172, 576), (375, 576)]),
    right_1_3=Polygon([(308, 0), (-172, 0), (-172, 576), (308, 576)]),
    left_2_3=Polygon([(-375, 0), (172, 0), (172, 576), (-375, 576)]),
    left_1_3=Polygon([(-308, 0), (172, 0), (172, 576), (-308, 576)]),
    )
}

# Dictionary of named sets of motion primitives (in mm)
MOTION_GROUPS_MM: Dict[str, List[Tuple[int, int]]] = {
    "UpSideways": [(0, 240), (200, 0), (-200, 0)],
    "UpSidewaysDown": [(0, 240), (200, 0), (-200, 0), (0,-240)],
    "UpSidewaysDiagonal": [(0, 240), (200, 0), (-200, 0),(170,170),(-170,170)],
    "VaryingLength": varying_length([(0, 240), (200, 0), (-200, 0)],3), 
    # VaryingLength = [(0, 240), (200, 0), (-200, 0), (0, 80), (0, 160), (66, 0), (133, 0), (-66, 0), (-133, 0)]
    "DiagonalAndVaryingLength": varying_length([(0, 240), (200, 0), (-200, 0)],3) + [(170,170),(-170,170)]
    # DiagonalAndVaryingLength = [(0, 240), (200, 0), (-200, 0), (0, 80), (0, 160), (66, 0), (133, 0), (-66, 0), (-133, 0), (170, 170), (-170, 170)]
}

RESOLUTION_CM: float = 1.7        # The ONLY resolution value (cm per grid cell)

# Named padding values (in cm) for collision checking
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

# Dictionary assigning motion name to motion tuple (scaled to grid cells)
MOTIONS_CELLS: Dict[str, Tuple[int, int]] = {
    "up" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSideways"][0], RESOLUTION_CM),
    "right" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSideways"][1], RESOLUTION_CM),
    "left" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSideways"][2], RESOLUTION_CM),
    
    "down" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSidewaysDown"][3], RESOLUTION_CM),
    "diagonal_right" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSidewaysDiagonal"][3], RESOLUTION_CM),
    "diagonal_left" : convert.ScaleVertex(MOTION_GROUPS_MM["UpSidewaysDiagonal"][4], RESOLUTION_CM),

    "up_2_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][4], RESOLUTION_CM),
    "up_1_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][3], RESOLUTION_CM),
    "right_2_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][6], RESOLUTION_CM),
    "right_1_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][5], RESOLUTION_CM),
    "left_2_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][8], RESOLUTION_CM),
    "left_1_3" : convert.ScaleVertex(MOTION_GROUPS_MM["VaryingLength"][7], RESOLUTION_CM),
    }


