#!/usr/bin/env python3
import argparse
import math
import random
from typing import Iterable, Optional, Tuple, List, Dict

import numpy as np
import matplotlib.pyplot as plt
import heapq

Coord = Tuple[int, int]

# ---------------------- Grid & Geometry ----------------------

def wrap_idx(i: int, n: int) -> int:
    return i % n

def angular_diff_min(a_idx: int, b_idx: int, n: int) -> int:
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

# ---------------------- Motion Primitives ----------------------

def build_motion_primitives(max_up_m: float, max_side_m: float,
                            radius: float, height: float, n_theta: int, n_z: int,
                            allow_down: bool=False, allow_standstill: bool=False) -> list[tuple[int,int]]:
    """
    Axis-aligned moves only with EXACT step sizes.
    One move = exactly round(max_side_m / s_theta) cells sideways OR
               exactly round(max_up_m   / s_z)     cells vertically.
    """

    #n_theta : number of grid cells around the circumference
    #n_z: number of gridcells along the hight
    s_theta = radius * (2 * math.pi / n_theta)              # arc length per theta cell (m)
    s_z = (height / (n_z - 1)) if n_z > 1 else height       # vertical per z cell (m)

    di_mag = max(1, int(round(max_side_m / s_theta))) #convert meters to exact cell count
    dj_mag = max(1, int(round(max_up_m   / s_z)))

    moves: list[tuple[int,int]] = [
        ( di_mag, 0), (-di_mag, 0),   # right / left
        (0, dj_mag)                   # up
    ]
    if allow_down:
        moves.append((0, -dj_mag))    # down

    if allow_standstill:
        moves.append((0, 0))

    return moves #list[(di,dj)] integre moves in grid indeces, di columns in the theta directions, dj rows in z

# ---------------------- Obstacles (Branch Patches) ----------------------

def make_branch_obstacles(n_theta: int, n_z: int,
                          radius: float, height: float,
                          branches: int = 1,
                          arc_span_m_range: Tuple[float, float] = (0.3, 1.2),
                          thickness_m_range: Tuple[float, float] = (0.05, 0.30),
                          seed: Optional[int] = None,
                          forbid: Optional[set] = None) -> np.ndarray:
    """Create boolean mask with 'branches' elliptical patches, each spanning a wide angular arc and
       some vertical thickness. Resembles branches encircling partially around the trunk.
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
    if forbid is None:
        forbid = set()

    s_theta, s_z, _, _ = grid_spacing(radius, height, n_theta, n_z)
    mask = np.zeros((n_theta, n_z), dtype=bool)

    for _ in range(branches):
        # center
        i0 = random.randrange(0, n_theta)
        j0 = random.randrange(0, n_z)

        arc_span_m = random.uniform(*arc_span_m_range)
        thick_m = random.uniform(*thickness_m_range)

        w_i = max(1, int(round(arc_span_m / s_theta / 2)))   # half-width in theta cells
        w_j = max(1, int(round(thick_m / s_z / 2)))          # half-thickness in z cells

        # fill an ellipse around (i0,j0) with radii (w_i,w_j), wrapping in theta
        for di in range(-w_i, w_i + 1):
            ii = (i0 + di) % n_theta
            for dj in range(-w_j, w_j + 1):
                jj = j0 + dj
                if 0 <= jj < n_z:
                    # ellipse test
                    val = (di / max(w_i,1))**2 + (dj / max(w_j,1))**2
                    if val <= 1.0 and (ii, jj) not in forbid:
                        mask[ii, jj] = True

    # Optionally, do slight dilation to make branches chunkier
    dilated = mask.copy()
    for i in range(n_theta):
        for j in range(n_z):
            if mask[i, j]:
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        ii = (i + di) % n_theta
                        jj = j + dj
                        if 0 <= jj < n_z:
                            dilated[ii, jj] = True
    return dilated

# ---------------------- Planner ----------------------

def is_primitive_collision_free(state, move, blocked, n_theta, n_z):
    """Return True if every intermediate cell along the primitive is free.
       Axis-aligned primitives only (no diagonals).
    """
    i, j = state
    di, dj = move

    if di != 0 and dj != 0:
        # we don't allow diagonals in this project
        return False

    if di != 0:
        step = 1 if di > 0 else -1
        for k in range(1, abs(di) + 1):
            ii = (i + k*step) % n_theta
            if blocked[ii, j]:
                return False
        return True

    if dj != 0:
        step = 1 if dj > 0 else -1
        for k in range(1, abs(dj) + 1):
            jj = j + k*step
            if not (0 <= jj < n_z):
                return False
            if blocked[i, jj]:
                return False
        return True

    # standstill
    return not blocked[i, j]


def edge_cost(a: Coord, b: Coord, radius: float, height: float, n_theta: int, n_z: int) -> float:
    i1, j1 = a
    i2, j2 = b
    s_theta, s_z, _, _ = grid_spacing(radius, height, n_theta, n_z)
    d_i = angular_diff_min(i1, i2, n_theta)
    d_j = abs(j1 - j2)
    return math.hypot(d_i * s_theta, d_j * s_z)

def heuristic(a: Coord, b: Coord, radius: float, height: float, n_theta: int, n_z: int) -> float:
    i1, j1 = a
    i2, j2 = b
    s_theta, s_z, _, _ = grid_spacing(radius, height, n_theta, n_z)
    d_i = angular_diff_min(i1, i2, n_theta)
    d_j = abs(j1 - j2)
    return math.hypot(d_i * s_theta, d_j * s_z)

def astar_with_primitives(start, goal, blocked,
                          radius, height, n_theta, n_z,
                          primitives):
    if blocked[start]:
        return None, None

    open_heap = []
    heapq.heappush(open_heap, (0.0, 0, start))
    came_from = {start: None}
    g_cost = {start: 0.0}
    counter = 1

    # track closest-to-goal node we've actually reached
    best_node = start
    best_h = heuristic(start, goal, radius, height, n_theta, n_z)

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        # update "closest reached node"
        h_cur = heuristic(current, goal, radius, height, n_theta, n_z)
        if h_cur < best_h:
            best_h = h_cur
            best_node = current

        if current == goal:
            # reconstruct exact-goal path
            path = []
            c = current
            while c is not None:
                path.append(c)
                c = came_from[c]
            path.reverse()
            return path, g_cost[current]

        i, j = current
        for di, dj in primitives:
            # reject moves that cross any blocked cell
            if not is_primitive_collision_free((i, j), (di, dj), blocked, n_theta, n_z):
                continue

            ni = (i + di) % n_theta
            nj = j + dj
            if not (0 <= nj < n_z):
                continue
            nb = (ni, nj)
            # end cell is free by construction, but keep this for safety:
            if blocked[nb]:
                continue

            new_g = g_cost[current] + edge_cost(current, nb, radius, height, n_theta, n_z)
            if new_g < g_cost.get(nb, float("inf")):
                g_cost[nb] = new_g
                came_from[nb] = current
                f = new_g + heuristic(nb, goal, radius, height, n_theta, n_z)
                heapq.heappush(open_heap, (f, counter, nb))
                counter += 1

    # Goal not reachable: return path to closest reached node
    if best_node is None:
        return None, None

    path = []
    c = best_node
    while c is not None:
        path.append(c)
        c = came_from[c]
    path.reverse()
    return path, g_cost.get(best_node, None)

# ---------------------- Visualization ----------------------

def plot_cylinder_with_grid(blocked: np.ndarray, path: Optional[List[Coord]] = None,
                            start: Optional[Coord] = None, goal: Optional[Coord] = None,
                            radius: float = 1.0, height: float = 3.0, n_theta: int = 64, n_z: int = 60,
                            save_path3d: Optional[str] = None) -> None:
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
    ax.scatter(xo, yo, zo, s=14, marker='s')

    if start is not None:
        x, y, z = idx_to_xyz(*start, radius, height, n_theta, n_z)
        ax.scatter([x],[y],[z], s=70, marker='o')
    if goal is not None:
        x, y, z = idx_to_xyz(*goal, radius, height, n_theta, n_z)
        ax.scatter([x],[y],[z], s=70, marker='^')

    if path is not None:
        px, py, pz = [], [], []
        for (i,j) in path:
            x,y,z = idx_to_xyz(i, j, radius, height, n_theta, n_z)
            px.append(x); py.append(y); pz.append(z)
        ax.plot(px, py, pz, linewidth=3)

    ax.set_box_aspect([1,1,height/(2*radius)])
    ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
    ax.set_title('Cylindrical Grid with Branch Obstacles (3D)')
    plt.tight_layout()
    if save_path3d:
        plt.savefig(save_path3d, dpi=150, bbox_inches='tight')
    plt.show()

def unwrap_path_segments(path: List[Coord], n_theta: int) -> List[List[Coord]]:
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
    s_theta, s_z, _, dz = grid_spacing(radius, height, n_theta, n_z)

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
    ax.scatter(oy, oz, s=14, marker='s')

    # Start / Goal
    if start is not None:
        ax.scatter([start[0]*s_theta], [start[1]*dz], s=70, marker='o')
    if goal is not None:
        ax.scatter([goal[0]*s_theta], [goal[1]*dz], s=70, marker='^')

    # Path segments
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
    ax.set_title('Unwrapped Cylindrical Grid (Branch Obstacles)')
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

    # Motion constraints
    ap.add_argument('--max-up', type=float, default=0.30, help='max vertical step per move (meters)')
    ap.add_argument('--max-side', type=float, default=0.20, help='max lateral arc step per move (meters)')
    ap.add_argument('--allow-down', action='store_true', help='permit downward steps up to max-up')

    # Obstacles (branches)
    ap.add_argument('--branches', type=int, default=10, help='number of branch patches')
    ap.add_argument('--branch-arc-min', type=float, default=0.3, help='min branch arc span (meters along circumference)')
    ap.add_argument('--branch-arc-max', type=float, default=1.2, help='max branch arc span (meters along circumference)')
    ap.add_argument('--branch-thick-min', type=float, default=0.05, help='min vertical thickness (meters)')
    ap.add_argument('--branch-thick-max', type=float, default=0.30, help='max vertical thickness (meters)')

    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--start', type=parse_pair, default="0,0")
    ap.add_argument('--goal', type=parse_pair, default=None)
    ap.add_argument('--out-prefix', type=str, default='cylinder_branches_demo')

    args = ap.parse_args()

    R = args.radius
    H = args.height
    NT = args.n_theta
    NZ = args.n_z

    START = args.start
    GOAL = args.goal if args.goal is not None else (NT//2, NZ-1)

    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)

    # Build motion primitives from physical limits
    primitives = build_motion_primitives(args.max_up, args.max_side, R, H, NT, NZ,
                                         allow_down=args.allow_down, allow_standstill=False)

    # Obstacles as branches
    forbid = {START, GOAL}
    blocked = make_branch_obstacles(
        NT, NZ, R, H,
        branches=args.branches,
        arc_span_m_range=(args.branch_arc_min, args.branch_arc_max),
        thickness_m_range=(args.branch_thick_min, args.branch_thick_max),
        seed=args.seed, forbid=forbid
    )

    # Plan
    path, cost = astar_with_primitives(START, GOAL, blocked, R, H, NT, NZ, primitives=primitives)

    # Save path
    if path is not None:
        with open(args.out_prefix + "_path.txt", "w") as f:
            for i,j in path:
                f.write(f"{i},{j}\n")

    # Visualize
    plot_cylinder_with_grid(blocked, path=path, start=START, goal=GOAL,
                            radius=R, height=H, n_theta=NT, n_z=NZ,
                            save_path3d=args.out_prefix+"_3d.png")
    plot_unwrapped(blocked, path=path, start=START, goal=GOAL,
                   radius=R, height=H, n_theta=NT, n_z=NZ,
                   save_path2d=args.out_prefix+"_unwrapped.png")

    print("Done. Files:")
    print(args.out_prefix + "_3d.png")
    print(args.out_prefix + "_unwrapped.png")
    if path is not None:
        print(args.out_prefix + "_path.txt")
    else:
        print("No path found. Try fewer branches, different seed, or coarser grid.")

if __name__ == "__main__":
    main()
