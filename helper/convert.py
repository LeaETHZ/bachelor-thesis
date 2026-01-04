"""
convert.py

This module provides helper functions for converting robot and obstacle geometry from millimeter units to grid cell coordinates.
It is used to scale polygonal shapes (e.g., robot footprints) to the resolution of the planning grid.

Functions:
- ScaleVertex: Convert a single vertex from mm to grid cell coordinates.
- ScalePolygon: Convert an entire Polygon from mm to grid cell coordinates.
"""

from agent.poly import Polygon
import numpy as np
from numpy.typing import NDArray
import math


def ScaleVertex(vertex_mm :tuple[int,int], cell_size_cm : float):   
    """
    Convert a vertex from millimeter coordinates to grid cell coordinates.
    Args:
        vertex_mm: (x_mm, y_mm) tuple in millimeters.
        cell_size_cm: Size of one grid cell in centimeters.
    Returns:
        (cx, cy): Tuple of grid cell coordinates (rounded to nearest integer).
    """
    cell_mm = cell_size_cm * 10.0  # 1 cm = 10 mm
    x_mm, y_mm = vertex_mm
    cx = np.round(x_mm / cell_mm)
    cy = np.round(y_mm / cell_mm)
    return (cx, cy)
        

def ScalePolygon(polygon: Polygon, resolution : int):
    """
    Convert a Polygon from millimeter coordinates to grid cell coordinates.
    Args:
        polygon: Polygon object with vertices in millimeters.
        resolution: Grid cell size in centimeters.
    Returns:
        Polygon: New Polygon object with vertices in grid cell coordinates.
    """
    cell_vertices = []  # build as list first (efficient)
    for vertex in polygon.vertices: 
        cell_vertices.append(ScaleVertex(vertex, resolution))
    return Polygon(cell_vertices)