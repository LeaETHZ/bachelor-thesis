#!/usr/bin/env python3
import argparse
import math
import random
from typing import Iterable, Optional, Tuple, List, Dict

import numpy as np
import matplotlib.pyplot as plt
import heapq

# ---------------------- Helpers & Core Model ----------------------

Coord = Tuple[int, int]

def wrap_idx(i: int, n: int) -> int:
    return i % n

def angular_diff_min(a_idx: int, b_idx: int, n: int) -> int:
    """Minimal wrap-around integer difference between two indices on a ring of size n."""
    d = abs(a_idx - b_idx) % n
    return min(d, n - d)

def grid_spacing(radius: float, height: float, n_theta: int, n_z: int) -> Tuple[float, float, float, float]:
    dtheta = 2 * math.pi / n_theta
    dz = height / (n_z - 1) if n_z > 1 else height
    s_theta = radius * dtheta   # arc length between adjacent theta indices
    s_z = dz
    return s_theta, s_z, dtheta, dz

def idx_to_xyz(i: int, j: int, radius: float, height: float, n_theta: int, n_z: int) -> Tuple[float, float, float]:
    dtheta = 2 * math.pi / n_theta
    dz = height / (n_z - 1) if n_z > 1 else height
    theta = i * dtheta
    z = j * dz
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    return x, y, z

def neighbors(i: int, j: int, n_theta: int, n_z: int, eight: bool=False) -> Iterable[Coord]:
    steps = [(-1,0),(1,0),(0,-1),(0,1)]
    if eight:
        steps += [(-1,-1),(-1,1),(1,-1),(1,1)]
    for di, dj in steps:
        ni = wrap_idx(i + di, n_theta)     # wrap around theta
        nj = j + dj
        if 0 <= nj < n_z:
            yield ni, nj

def heuristic(a: Coord, b: Coord, radius: float, height: float, n_theta: int, n_z: int) -> float:
    """Euclidean distance on cylinder surface between two grid nodes (with theta wrap)."""
    i1, j1 = a
    i2, j2 = b
    s_theta, s_z, _, _ = grid_spacing(radius, height, n_theta, n_z)
    d_i = angular_diff_min(i1, i2, n_theta)
    d_j = abs(j1 - j2)
    return math.hypot(d_i * s_theta, d_j * s_z)

def edge_cost(a: Coord, b: Coord, radius: float, height: float, n_theta: int, n_z: int) -> float:
    i1, j1 = a
    i2, j2 = b
    s_theta, s_z, _, _ = grid_spacing(radius, height, n_theta, n_z)
    d_i = angular_diff_min(i1, i2, n_theta)
    d_j = abs(j1 - j2)
    return math.hypot(d_i * s_theta, d_j * s_z)

def astar(start: Coord, goal: Coord, blocked: np.ndarray,
          radius: float, height: float, n_theta: int, n_z: int,
          eight: bool=False) -> Tuple[Optional[List[Coord]], Optional[float]]:
    """A* on cylindrical grid with wrap-around in theta."""
    if blocked[start] or blocked[goal]:
        return None, None

    open_heap: List[Tuple[float, int, Coord]] = []
    heapq.heappush(open_heap, (0.0, 0, start))
    came_from: Dict[Coord, Optional[Coord]] = {start: None}
    g_cost: Dict[Coord, float] = {start: 0.0}
    counter = 1

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            # reconstruct path
            path: List[Coord] = []
            c: Optional[Coord] = current
            while c is not None:
                path.append(c)
                c = came_from[c]
            path.reverse()
            return path, g_cost[current]

        for nb in neighbors(current[0], current[1], n_theta, n_z, eight=eight):
            if blocked[nb]:
                continue
            new_g = g_cost[current] + edge_cost(current, nb, radius, height, n_theta, n_z)
            if new_g < g_cost.get(nb, float("inf")):
                g_cost[nb] = new_g
                came_from[nb] = current
                f = new_g + heuristic(nb, goal, radius, height, n_theta, n_z)
                heapq.heappush(open_heap, (f, counter, nb))
                counter += 1

    return None, None

# ---------------------- Obstacles ----------------------

def make_obstacles(n_theta: int, n_z: int, density: float, forbid: Optional[set]=None) -> np.ndarray:
    """Return boolean mask (True=blocked) with given density; never block any 'forbid' coordinates."""
    if forbid is None:
        forbid = set()
    mask = np.zeros((n_theta, n_z), dtype=bool)
    total = n_theta * n_z
    target = int(density * total)
    candidates = [(i,j) for i in range(n_theta) for j in range(n_z) if (i,j) not in forbid]
    target = min(target, len(candidates))
    if target > 0:
        blocked = set(random.sample(candidates, target))
        for (i,j) in blocked:
            mask[i, j] = True
    return mask

# ---------------------- Visualization ----------------------

def plot_cylinder_with_grid(blocked: np.ndarray, path: Optional[List[Coord]] = None,
                            start: Optional[Coord] = None, goal: Optional[Coord] = None,
                            radius: float = 1.0, height: float = 3.0, n_theta: int = 64, n_z: int = 60,
                            save_path3d: Optional[str] = None) -> None:
    """3D cylinder with grid nodes (light), obstacles (squares), and path."""
    theta_lin = np.linspace(0, 2*np.pi, 128)
    z_lin = np.linspace(0, height, 128)
    T, Z = np.meshgrid(theta_lin, z_lin)
    X = radius * np.cos(T)
    Y = radius * np.sin(T)

    fig = plt.figure(figsize=(9, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(X, Y, Z, alpha=0.15, linewidth=0, antialiased=False)

    xs, ys, zs = [], [], []
    xo, yo, zo = [], [], []
    for i in range(n_theta):
        for j in range(n_z):
            x, y, z = idx_to_xyz(i, j, radius, height, n_theta, n_z)
            if blocked[i, j]:
                xo.append(x); yo.append(y); zo.append(z)
            else:
                xs.append(x); ys.append(y); zs.append(z)
    ax.scatter(xs, ys, zs, s=3, alpha=0.35)
    ax.scatter(xo, yo, zo, s=12, marker='s')

    if start is not None:
        x, y, z = idx_to_xyz(*start, radius, height, n_theta, n_z)
        ax.scatter([x],[y],[z], s=60, marker='o')
    if goal is not None:
        x, y, z = idx_to_xyz(*goal, radius, height, n_theta, n_z)
        ax.scatter([x],[y],[z], s=60, marker='^')

    if path is not None:
        px, py, pz = [], [], []
        for (i,j) in path:
            x,y,z = idx_to_xyz(i, j, radius, height, n_theta, n_z)
            px.append(x); py.append(y); pz.append(z)
        ax.plot(px, py, pz, linewidth=3)

    ax.set_box_aspect([1,1,height/(2*radius)])
    ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
    ax.set_title('Path Planning on a Cylindrical Grid (3D)')
    plt.tight_layout()
    if save_path3d:
        plt.savefig(save_path3d, dpi=150, bbox_inches='tight')
    plt.show()

def unwrap_path_segments(path: List[Coord], n_theta: int) -> List[List[Coord]]:
    """Split path into segments when it crosses the unwrap seam to keep 2D lines short."""
    if not path:
        return []
    segs = [[path[0]]]
    for a, b in zip(path, path[1:]):
        i1, _ = a
        i2, _ = b
        if angular_diff_min(i1, i2, n_theta) > 1 and abs(i1 - i2) > n_theta // 2:
            segs.append([b])
        else:
            segs[-1].append(b)
    return segs

def plot_unwrapped(blocked: np.ndarray, path: Optional[List[Coord]] = None,
                   start: Optional[Coord] = None, goal: Optional[Coord] = None,
                   radius: float = 1.0, height: float = 3.0, n_theta: int = 64, n_z: int = 60,
                   save_path2d: Optional[str] = None) -> None:
    """Unwrapped rectangle plot (x = arc length along theta, y = z)."""
    s_theta, s_z, dtheta, dz = grid_spacing(radius, height, n_theta, n_z)

    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111)

    # Grid
    for i in range(n_theta):
        ax.plot([i*s_theta, i*s_theta], [0, height], linewidth=0.3, alpha=0.4)
    for j in range(n_z):
        ax.plot([0, n_theta*s_theta], [j*dz, j*dz], linewidth=0.3, alpha=0.4)

    # Obstacles
    oy, oz = [], []
    for i in range(n_theta):
        for j in range(n_z):
            if blocked[i, j]:
                oy.append(i*s_theta); oz.append(j*dz)
    ax.scatter(oy, oz, s=12, marker='s')

    # Start / Goal
    if start is not None:
        ax.scatter([start[0]*s_theta], [start[1]*dz], s=60, marker='o')
    if goal is not None:
        ax.scatter([goal[0]*s_theta], [goal[1]*dz], s=60, marker='^')

    # Path: draw in segments so it doesn't wrap across the whole width
    if path is not None:
        for seg in unwrap_path_segments(path, n_theta):
            px = [i*s_theta for (i,_) in seg]
            py = [j*dz for (_,j) in seg]
            ax.plot(px, py, linewidth=2)

    ax.set_xlim(0, n_theta*s_theta)
    ax.set_ylim(0, height)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('arc length (theta)')
    ax.set_ylabel('z')
    ax.set_title('Path Planning on a Cylindrical Grid (Unwrapped)')
    plt.tight_layout()
    if save_path2d:
        plt.savefig(save_path2d, dpi=150, bbox_inches='tight')
    plt.show()

# ---------------------- Main ----------------------

def parse_pair(s: str) -> Coord:
    a, b = s.split(',')
    return int(a), int(b)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--radius', type=float, default=1.0)
    ap.add_argument('--height', type=float, default=3.0)
    ap.add_argument('--n-theta', type=int, default=64)
    ap.add_argument('--n-z', type=int, default=60)
    ap.add_argument('--obst-density', type=float, default=0.18)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--start', type=parse_pair, default="0,0")
    ap.add_argument('--goal', type=parse_pair, default=None)
    ap.add_argument('--eight-connected', type=int, default=0, help="1 to allow diagonals on grid")
    ap.add_argument('--out-prefix', type=str, default='cylinder_grid_demo')
    args = ap.parse_args()

    R = args.radius
    H = args.height
    NT = args.n_theta
    NZ = args.n_z
    DENS = args.obst_density
    SEED = args.seed
    START = args.start
    GOAL = args.goal if args.goal is not None else (NT//2, NZ-1)
    EIGHT = bool(args.eight_connected)
    PREFIX = args.out_prefix

    if SEED is not None:
        random.seed(SEED)
        np.random.seed(SEED)

    forbid = {START, GOAL}
    blocked = make_obstacles(NT, NZ, DENS, forbid=forbid)

    path, cost = astar(START, GOAL, blocked, R, H, NT, NZ, eight=EIGHT)

    # save path if present
    if path is not None:
        with open(PREFIX + "_path.txt", "w") as f:
            for i,j in path:
                f.write(f"{i},{j}\n")

    # plots
    plot_cylinder_with_grid(blocked, path=path, start=START, goal=GOAL,
                            radius=R, height=H, n_theta=NT, n_z=NZ,
                            save_path3d=PREFIX+"_3d.png")

    plot_unwrapped(blocked, path=path, start=START, goal=GOAL,
                   radius=R, height=H, n_theta=NT, n_z=NZ,
                   save_path2d=PREFIX+"_unwrapped.png")

    print("Done. Files:")
    print(PREFIX + "_3d.png")
    print(PREFIX + "_unwrapped.png")
    if path is not None:
        print(PREFIX + "_path.txt")
    else:
        print("No path found (start/goal blocked or disconnected).")

if __name__ == "__main__":
    main()
