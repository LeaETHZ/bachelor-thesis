from python_motion_planning.common import *
from python_motion_planning.path_planner import *
from python_motion_planning.controller import *

"""-----------------------------------------------create environment, define start/ end point-----------------------------------------------"""

tree = Grid(bounds=[[0, 101], [0, 61]]) 

# tree.fill_boundary_with_obstacles() # makes border of grid an obstacle
tree.type_map[45:55, 30:36] = TYPES.OBSTACLE
tree.type_map[75:95, 10:20] = TYPES.OBSTACLE

# tree.inflate_obstacles(radius=0) # adds extra margin around obstacles

start = (50, 0)
goal = (50, 60)

tree.type_map[start] = TYPES.START
tree.type_map[goal] = TYPES.GOAL

"""-----------------------------------------------create planner-----------------------------------------------"""

planner = AStar(map_=tree, start=start, goal=goal)
path, path_info = planner.plan()
print(path)
print(path_info)

"""-----------------------------------------------visualize-----------------------------------------------"""

vis = Visualizer("Path Visualizer")
vis.plot_grid_map(tree)
vis.plot_path(path, style="--", color="C4")
#vis.plot_expand_tree(path_info["expand"])
vis.show()
vis.close()