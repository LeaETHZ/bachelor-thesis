import math
from typing import Tuple
from python_motion_planning.utils import Node

from agent import PolygonAgent
from helper import convert
from .presets import SHAPES_MM, MOTION_GROUPS_MM, RESOLUTION_GROUPS_CM, ShapeSetMM
from .poly import Polygon





def build_robot(shape_key: str, motion_key: str, resolution_key: str) -> PolygonAgent:
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
        raise KeyError(f"shape '{shape_key}' not found. Available: {list(SHAPES_MM.keys())}")
    if motion_key not in MOTION_GROUPS_MM:
        raise KeyError(f"motion group '{motion_key}' not found. Available: {list(MOTION_GROUPS_MM.keys())}")
    if resolution_key not in RESOLUTION_GROUPS_CM:
        raise KeyError(f"motion group '{motion_key}' not found. Available: {list(MOTION_GROUPS_MM.keys())}")

    shapes_mm: ShapeSetMM = SHAPES_MM[shape_key]
    motions_mm = MOTION_GROUPS_MM[motion_key]
    resolution_cm = RESOLUTION_GROUPS_CM[resolution_key]

    # --- convert polygons (mm -> cells) ---
    scaledShapeUp = convert.ScalePolygon(shapes_mm.up, resolution_cm)
    scaledShapeRight = convert.ScalePolygon(shapes_mm.right, resolution_cm)
    sacledShapeLeft = convert.ScalePolygon(shapes_mm.left, resolution_cm)
    

    # --- convert motions (mm -> cells) ---
    motions_cells = [convert.ScaleVertex(m, resolution_cm) for m in motions_mm]


    #diagonal motions different cost
    motions = [Node((x,y), None, 1, None) for (x,y) in motions_cells]
    for motion in motions:
        if motion.current[0] != 0 and motion.current[1] != 0:
            motion.g = math.sqrt(2)
        

    # --- construct agent ---
    agent = PolygonAgent(None, scaledShapeUp, scaledShapeRight, sacledShapeLeft, motions)
    return agent


