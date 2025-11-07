from agent.poly import Polygon
import numpy as np
from numpy.typing import NDArray
import math




def convertToCells(vertex :tuple[int,int], cell_size_cm : float):   
        cell_mm = cell_size_cm * 10.0  # 1 cm = 10 mm
        x_mm, y_mm = vertex
        # choose one of: floor / round / ceil (floor is common for grid indexing)
        cx = math.floor(x_mm / cell_mm)
        cy = math.floor(y_mm / cell_mm)
        return (cx, cy)
            

def convertPolygonToCells(polygon: Polygon, resolution : int):
        cell_vertices = []  # build as list first (efficient)
        
        for vertex in polygon.vertices: 
            cell_vertices.append(convertToCells(vertex, resolution))
        
        return Polygon(cell_vertices)