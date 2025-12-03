import math
from typing import Tuple
from python_motion_planning.utils import Node

from agent import PolygonAgent
from helper import convert
from .presets import SHAPES_MM, MOTION_GROUPS_MM, RESOLUTION_CM, PADDING_GROUPS_CM, ShapeSetMM
from .poly import Polygon





def build_robot(shape_key: str, motion_key: str, padding_key: str) -> PolygonAgent:
    """
    Create a PolygonAgent from preset names.

    Args:
    shape_key: key inside SHAPES_MM 
    motion_key: key inside MOTION_GROUPS_MM 
    start_xy: (x, y) start position in grid cells
    res_cm: grid resolution in cm (same as used by your convert helpers)
    heading_deg: initial yaw in degrees

    Returns:
    Configured PolygonAgent
    """
    # --- lookup presets ---
    if shape_key not in SHAPES_MM:
        raise KeyError(f"Shape '{shape_key}' not found. Available: {list(SHAPES_MM.keys())}")
    if motion_key not in MOTION_GROUPS_MM:
        raise KeyError(f"Motion group '{motion_key}' not found. Available: {list(MOTION_GROUPS_MM.keys())}")
    if padding_key not in PADDING_GROUPS_CM:
        raise KeyError(f"Padding '{padding_key}' not found. Available: {list(PADDING_GROUPS_CM.keys())}")


    shapes_mm: ShapeSetMM = SHAPES_MM[shape_key]
    motions_mm = MOTION_GROUPS_MM[motion_key]
    padding_cm = PADDING_GROUPS_CM[padding_key]

    # --- convert polygons (mm -> cells) ---
    scaledShapeUp = convert.ScalePolygon(shapes_mm.up, RESOLUTION_CM)
    scaledShapeRight = convert.ScalePolygon(shapes_mm.right, RESOLUTION_CM)
    sacledShapeLeft = convert.ScalePolygon(shapes_mm.left, RESOLUTION_CM)
    sacledShapeCrouched = convert.ScalePolygon(shapes_mm.crouched, RESOLUTION_CM)
    

    # --- convert motions (mm -> cells) ---
    motions_cells = [convert.ScaleVertex(m, RESOLUTION_CM) for m in motions_mm]


    # all motions have same cost
    motions = [Node((x,y), None, 1, None) for (x,y) in motions_cells]
   
        

    # --- construct agent ---
    agent = PolygonAgent(None, scaledShapeUp, scaledShapeRight, sacledShapeLeft, sacledShapeCrouched, motions, RESOLUTION_CM, padding_cm)
    return agent


