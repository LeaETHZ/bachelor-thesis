import os, json, time, random
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Tuple, List, Dict, Any
import math

import matplotlib
matplotlib.use("Agg")  # headless saves
import matplotlib.pyplot as plt

import python_motion_planning as pmp
from python_motion_planning.utils import Grid

from environment import Cylinder, Randomize, randomize
from planner import AStarExtension
from agent import robot_factory
from agent.presets import SHAPES_MM, MOTION_GROUPS_MM, RESOLUTION_GROUPS_CM, PADDING_GROUPS_CM



#python -m evaluation.evaluate

# ---------- Configs you want to compare ----------
VARIANTS = [
    #here we can add things like padding and stuff
    # ("ExactRobot", "VaryingLength", "RegularRes"),
    ("ExactRobot", "UpSideways", "RegularRes","RegularPad"),
    ("RectangleRobot", "UpSideways", "RegularRes", "RegularPad"),
    # ("ExactRobot", "UpSidewaysDiagonal", "RegularRes"),
]


# ---------- Evaluation parameters ----------
N_CASES = 3
CYL_RADIUS = 12
CYL_HEIGHT = 2000
N_RANDOM_OBS_BLOCKS = 8           # how many random obstacles per case
EXTRA_OBS_RECT = ((0,0), (0,0))  # optional fixed obstacle example
GOAL_TOL_CM_X = 8.5
GOAL_TOL_CM_Y = 10
MASTER_SEED = 70                # set None for non-deterministic


MAX_CASE_TIME_S = 30.0 

# ---------- Data containers ----------
@dataclass
class Scenario:
    seed: int
    start: Tuple[int, int]
    goal: Tuple[int, int]
    obstacles: List[Tuple[int, int]]  # if you need to serialize

@dataclass
class RunResult:
    case_id: int
    variant: str
    shape_key: str
    motion_key: str
    res_key: str
    pad_key: str
    success: bool
    cost: float
    steps: int
    expand_count: int
    runtime_ms: float
    img_path: str

# ---------- Utilities ----------
def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)

def save_scenario(path: Path, scenario: Scenario) -> None:
    with open(path, "w") as f:
        json.dump(asdict(scenario), f, indent=2)

def save_config(path: Path, varaints: List[Tuple[str, str, str, str]] = VARIANTS, n : int = N_CASES) -> None:
    data = {
        "variants": varaints,
        "n_cases": N_CASES,
        "cylinder": {"radius": CYL_RADIUS, "height": CYL_HEIGHT},
        "random_obstacles": N_RANDOM_OBS_BLOCKS,
        "extra_obstacle_rect": EXTRA_OBS_RECT,
        "goal_tol_cm_x": GOAL_TOL_CM_X,
        "goal_tol_cm_y": GOAL_TOL_CM_Y,
        "master_seed": MASTER_SEED,
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)

# OPTIONAL: if your Randomize has no seed, you can hook python's random
def seed_everything(seed: int | None):
    if seed is None: 
        return
    random.seed(seed)
    try:
        import numpy as np
        np.random.seed(seed)
    except ImportError:
        pass



# ---------- One run on one scenario / one variant ----------
def run_variant_on_scenario(case_id: int, env: Cylinder, start: Tuple[int,int], goal: Tuple[int,int],
                            variant: tuple[str,str,str,str], out_dir: Path) -> RunResult:
    shape_key, motion_key, res_key, pad_key = variant
    variant_name = f"{shape_key}_{motion_key}_{res_key}_{pad_key}"

    # Build robot
    robot = robot_factory.build_robot(shape_key, motion_key, res_key, pad_key)

    # (Optional) pre-check start collision
    # from agent import PolygonAgent
    # if PolygonAgent.is_in_collision((start[0], start[1], 0), robot.local_shape_up, env):
    #     return RunResult(case_id, variant_name, shape_key, motion_key, res_key, False, float("inf"), 0, 0, 0.0, "")

    # convert goal_tol_x/y from cm into res
    goal_tol_cells_x = math.ceil(GOAL_TOL_CM_X/RESOLUTION_GROUPS_CM[variant[2]])
    goal_tol_cells_y = math.ceil(GOAL_TOL_CM_Y/RESOLUTION_GROUPS_CM[variant[2]])    

    planner = AStarExtension(start, goal, env, robot, goal_tol_cells_x = goal_tol_cells_x, goal_tol_cells_y= goal_tol_cells_y)

    t0 = time.perf_counter()
    try:
        cost, path, expand = planner.plan()

    except TimeoutError:
        dt = (time.perf_counter() - t0) * 1000.0
        print(f"[TIMEOUT] case {case_id}, variant {variant_name} exceeded time limit")
        cost = None
        path = None
        expand = None

    dt = (time.perf_counter() - t0) * 1000.0

    success = bool(path)
    steps = len(path)-1 if success else 0 
    expand_count = len(expand) if expand else 0
    cost_val = float(cost) if success else float("inf")

    img_path = out_dir / f"{variant_name}.png"
    title = f"case {case_id:04d} - {variant_name} - {'OK' if success else 'FAIL'}"

    # Use your PolygonPlot-based animation instead of static plot
    planner.plot.animation(path, title, cost, expand)

    # ---- RESIZE IN METRIC UNITS ----
    fig = plt.gcf()

    cm = 1 / 2.54   # centimeters → inches
    # Set your desired size IN CENTIMETERS:
    fig.set_size_inches(50 * cm, 120 * cm)  # example: 40 cm width × 120 cm height

    # ---- SAVE HIGH-RES ----
    fig.savefig(img_path, dpi=200, bbox_inches="tight")

    # (Optional) also save vector:
    #fig.savefig(img_path.with_suffix(".pdf"), bbox_inches="tight")

    plt.close(fig)

    return RunResult(
        case_id=case_id,
        variant=variant_name,
        shape_key=shape_key,
        motion_key=motion_key,
        res_key=res_key,
        pad_key=pad_key,
        success=success,
        cost=cost_val,
        steps=steps,
        expand_count=expand_count,
        runtime_ms=dt,
        img_path=str(img_path),
    )

# ---------- Build one scenario ----------
def build_random_scenario(case_seed: int, res: float, pad: float) -> tuple[Cylinder, Tuple[int,int], Tuple[int,int], Scenario]:
    seed_everything(case_seed)

    env = Cylinder(CYL_RADIUS, CYL_HEIGHT)
    

    # Your own random obstacle builder(s); keep deterministic under seed
    Randomize.random_obstacles(env, N_RANDOM_OBS_BLOCKS)
    # Optional fixed rectangle:
    randomize.build_obstacle(EXTRA_OBS_RECT[0], EXTRA_OBS_RECT[1], env)

    start, goal = Randomize.random_start_and_goal(env, res, pad)

    # Serialize obstacles if you need them (convert set->list)
    obs_list = list(env.obstacles)
    scen = Scenario(seed=case_seed, start=start, goal=goal, obstacles=obs_list)
    return env, start, goal, scen

# ---------- Main experiment ----------
def run_experiment(n_cases=N_CASES, variants=VARIANTS):
    ts_dir = Path("runs") / time.strftime("%Y-%m-%d_%H-%M-%S")
    ensure_dir(ts_dir)
    save_config(ts_dir / "config.json", variants, n_cases)

    results: List[RunResult] = []
    failed_cases: List[int] = []

    # find min padding and min res throughout all variants
    min_res = min(RESOLUTION_GROUPS_CM[variant[2]] for variant in variants)
    min_pad = min(PADDING_GROUPS_CM[variant[3]] for variant in variants)

    for i in range(1, n_cases + 1):
        case_dir = ts_dir / f"case_{i:04d}"
        ensure_dir(case_dir)
        print(f"\n=== RUN CASE {i}/{n_cases} ===")
        t_case_start = time.perf_counter()  #
        # Use MASTER_SEED + i to make each case reproducible & distinct
        case_seed = (MASTER_SEED or 0) + i
        env, start, goal, scen = build_random_scenario(case_seed, min_res, min_pad)
        save_scenario(case_dir / "scenario.json", scen)


        # Run all variants on the SAME scenario
        for v in variants:

            res = run_variant_on_scenario(i, env.copy() if hasattr(env, "copy") else env, start, goal, v, case_dir)
            results.append(res)
            if not res.success:
                failed_cases.append(i)
        
        t_case_end = time.perf_counter()  #end timer
        print(f"Case {i} finished in {(t_case_end - t_case_start):.2f} seconds")

    # Write CSV
    import csv
    with open(ts_dir / "results.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["case_id","variant","shape","motion","res","success","cost","steps","expand_count","runtime_ms","img_path"])
        for r in results:
            w.writerow([r.case_id, r.variant, r.shape_key, r.motion_key, r.res_key,
                        int(r.success), r.cost, r.steps, r.expand_count, round(r.runtime_ms,2), r.img_path])

    # Print a tiny summary

    summary_lines = []

    by_variant: Dict[str, Dict[str, Any]] = {}
    for r in results:
        s = by_variant.setdefault(r.variant, {"runs":0, "success":0, "costs":[]})
        s["runs"] += 1
        s["success"] += int(r.success)
        if r.success: s["costs"].append(r.cost)

    print("\n=== Summary ===")
    summary_lines.append("\n=== Summary ===")

    for v, s in by_variant.items():
        sr = 100.0 * s["success"] / s["runs"]
        avg = (sum(s["costs"])/len(s["costs"])) if s["costs"] else float("nan")
        line = f"{v}: success {sr:.1f}% | avg cost {avg:.2f} over {s['runs']} cases"
        print(line)
        summary_lines.append(line)

    print("\n=== Failed Cases ===")
    summary_lines.append("\n=== Failed Cases ===")

    if failed_cases:
        fail_line = "Cases with no success: " + ", ".join(map(str, sorted(set(failed_cases))))
        print(fail_line)
        summary_lines.append(fail_line)
    else:
        print("All cases successful!")
        summary_lines.append("All cases successful!")
    
    summary_path = ts_dir / "summary.txt"
    with open(summary_path, "w") as f:
        f.write("\n".join(summary_lines))
    print(f"Summary saved to {summary_path}")


if __name__ == "__main__":
    run_experiment()
