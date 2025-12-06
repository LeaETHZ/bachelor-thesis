from agent.poly import Polygon
import numpy as np
from numpy.typing import NDArray
import math




def ScaleVertex(vertex_mm :tuple[int,int], cell_size_cm : float):   
        cell_mm = cell_size_cm * 10.0  # 1 cm = 10 mm
        x_mm, y_mm = vertex_mm
        cx = np.round(x_mm / cell_mm)
        cy = np.round(y_mm / cell_mm)
        return (cx, cy)
            

def ScalePolygon(polygon: Polygon, resolution : int):
        cell_vertices = []  # build as list first (efficient)
        
        for vertex in polygon.vertices: 
            cell_vertices.append(ScaleVertex(vertex, resolution))
        
        return Polygon(cell_vertices)