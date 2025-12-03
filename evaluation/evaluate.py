import os, json, time, random
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Tuple, List, Dict, Any
import math
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing
import numpy as np




import matplotlib
matplotlib.use("Agg")  # headless saves
import matplotlib.pyplot as plt

import python_motion_planning as pmp
from python_motion_planning.utils import Grid

from environment import Cylinder, Randomize, randomize
from planner import AStarExtension
from agent import robot_factory
from agent.presets import SHAPES_MM, MOTION_GROUPS_MM, PADDING_GROUPS_CM, RESOLUTION_CM



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
CYL_RADIUS_CM = 20
CYL_HEIGHT_CM = 1000
START_BOUND_CM = 100               # random start point is constrainted in y = (0, start_bound)
GOAL_BOUND_CM = 100                # random end point is constrainted in y = (height - goal_bound , height)
N_RANDOM_OBS_RECTANGLE = 0          # how many random rectangle obstacles per case
N_RANDOM_OBS_ELLIPSE = 1
EXTRA_OBS_RECT = ((0,0), (0,0))  # optional fixed obstacle example
GOAL_TOL_CM_X = 30 #8.5
GOAL_TOL_CM_Y = 30 #10
MASTER_SEED = 36                # set None for non-deterministic
RESOLUTION = RESOLUTION_CM


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
    pad_key: str
    success: bool
    cost: float
    steps: int
    final_distance : float
    expand_count: int
    runtime_ms: float
    img_path: str

# ---------- Utilities ----------
def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)

def save_scenario(path: Path, scenario: Scenario) -> None:
    with open(path, "w") as f:
        json.dump(asdict(scenario), f, indent=2)

def save_config(path: Path, varaints: List[Tuple[str, str, str]] = VARIANTS, n : int = N_CASES) -> None:
    data = {
        "variants": varaints,
        "n_cases": N_CASES,
        "cylinder": {"radius": CYL_RADIUS_CM, "height": CYL_HEIGHT_CM},
        "random_obstacles_rectangle": N_RANDOM_OBS_RECTANGLE,
        "random_obstacles_ellipse": N_RANDOM_OBS_ELLIPSE,
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


def _run_single_case(case_id: int,
                     case_seed: int,
                     variants,
                     min_pad: float,
                     ts_dir: Path) -> list[RunResult]:
    """
    Run all variants for a single case_id.
    This will be executed in a separate process.
    """
    case_dir = ts_dir / f"case_{case_id:04d}"
    ensure_dir(case_dir)
    print(f"\n=== RUN CASE {case_id} ===")
    t_case_start = time.perf_counter()

    # build scenario (may raise ValueError if no start/goal is found)
    try:
        env, start, goal, scen = build_random_scenario(case_seed, min_pad)
    except ValueError as e:
        print(f"[SCENARIO {case_id}] could not sample start/goal: {e}")
        return []  # no results for this case

    save_scenario(case_dir / "scenario.json", scen)

    case_results: list[RunResult] = []

    for v in variants:
        res = run_variant_on_scenario(
            case_id,
            env.copy() if hasattr(env, "copy") else env,
            start,
            goal,
            v,
            case_dir,
        )
        case_results.append(res)

    t_case_end = time.perf_counter()
    print(f"Case {case_id} finished in {(t_case_end - t_case_start):.2f} seconds")

    return case_results


# ---------- One run on one scenario / one variant ----------
def run_variant_on_scenario(case_id: int, env: Cylinder, start: Tuple[int,int], goal: Tuple[int,int],
                            variant: tuple[str,str,str], out_dir: Path) -> RunResult:
    shape_key, motion_key, pad_key = variant
    variant_name = f"{shape_key}_{motion_key}_{pad_key}"

    # Build robot
    robot = robot_factory.build_robot(shape_key, motion_key, pad_key)

    # (Optional) pre-check start collision
    # from agent import PolygonAgent
    # if PolygonAgent.is_in_collision((start[0], start[1], 0), robot.local_shape_up, env):
    #     return RunResult(case_id, variant_name, shape_key, motion_key, False, float("inf"), 0, 0, 0.0, "")

    # convert goal_tol_x/y from cm into res
    goal_tol_cells_x = math.ceil(GOAL_TOL_CM_X/RESOLUTION)
    goal_tol_cells_y = math.ceil(GOAL_TOL_CM_Y/RESOLUTION)    

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
    final_distance_cm = planner.final_distance_cells * RESOLUTION_CM
    expand_count = len(expand) if expand else 0
    cost_val = float(cost) if success else float("inf")

    img_path = out_dir / f"{variant_name}.png"
    title = f"case {case_id:04d} - {variant_name} - {'OK' if success else 'FAIL'}"

    # Use your PolygonPlot-based animation instead of static plot
    planner.plot.animation(path, title, cost, final_distance_cm, expand)

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
        pad_key=pad_key,
        success=success,
        cost=cost_val,
        steps=steps,
        final_distance=final_distance_cm,
        expand_count=expand_count,
        runtime_ms=dt,
        img_path=str(img_path),
    )

# ---------- Build one scenario ----------
def build_random_scenario(case_seed: int, pad: float) -> tuple[Cylinder, Tuple[int,int], Tuple[int,int], Scenario]:
    seed_everything(case_seed)

    cyl_radius_cells = np.round(CYL_RADIUS_CM/RESOLUTION)
    cyl_height_cells = np.round(CYL_HEIGHT_CM/RESOLUTION)
    env = Cylinder(cyl_radius_cells, cyl_height_cells)
    

    # Your own random obstacle builder(s); keep deterministic under seed
    Randomize.random_obstacles_rectangle(env, N_RANDOM_OBS_RECTANGLE)
    Randomize.random_obstacles_ellipse(env, N_RANDOM_OBS_ELLIPSE)
    # Optional fixed rectangle:
    randomize.build_obstacle_rectangle(EXTRA_OBS_RECT[0], EXTRA_OBS_RECT[1], env)

    start, goal = Randomize.random_start_and_goal(env, RESOLUTION, pad, START_BOUND_CM, GOAL_BOUND_CM)

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

    # find min padding throughout all variants
    min_pad = min(PADDING_GROUPS_CM[variant[2]] for variant in variants)


    # ---------- PARALLEL CASE EXECUTION ----------
    num_cores = multiprocessing.cpu_count()
    max_workers = max(1, num_cores - 2)   # leave a couple cores free
    print(f"Using {max_workers} parallel workers out of {num_cores} cores")

    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = []
        for i in range(1, n_cases + 1):
            case_seed = (MASTER_SEED or 0) + i
            futures.append(
                executor.submit(
                    _run_single_case,
                    i,
                    case_seed,
                    variants,
                    min_pad,
                    ts_dir,
                )
            )

        # collect results as they complete
        for fut in as_completed(futures):
            case_results = fut.result()  # this is list[RunResult]
            results.extend(case_results)

    # for i in range(1, n_cases + 1):
    #     case_dir = ts_dir / f"case_{i:04d}"
    #     ensure_dir(case_dir)
    #     print(f"\n=== RUN CASE {i}/{n_cases} ===")
    #     t_case_start = time.perf_counter()  #
    #     # Use MASTER_SEED + i to make each case reproducible & distinct
    #     case_seed = (MASTER_SEED or 0) + i

    #     #if no start or goal can be found
    #     try:
    #         env, start, goal, scen = build_random_scenario(case_seed, RESOLUTION, min_pad)
    #     except ValueError as e:
    #         print(f"[SCENARIO {i}] could not sample start/goal: {e} — skipping case.")
    #         failed_cases.append(i)
    #         continue  # go to next case


    #     save_scenario(case_dir / "scenario.json", scen)

        



    #     # Run all variants on the SAME scenario
    #     for v in variants:

    #         res = run_variant_on_scenario(i, env.copy() if hasattr(env, "copy") else env, start, goal, v, case_dir)
    #         results.append(res)
    #         if not res.success:
    #             failed_cases.append(i)
        
    #     t_case_end = time.perf_counter()  #end timer
    #     print(f"Case {i} finished in {(t_case_end - t_case_start):.2f} seconds")

    # Write CSV
    import csv
    with open(ts_dir / "results.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["case_id","variant","shape","motion","success","cost","steps","final_distance","expand_count","runtime_ms","img_path"])
        for r in results:
            w.writerow([
                r.case_id, r.variant, r.shape_key, r.motion_key,
                int(r.success), r.cost, r.steps, r.final_distance, r.expand_count,
                round(r.runtime_ms, 2), r.img_path
            ])


    # Print a tiny summary

    summary_lines = []

    by_variant: Dict[str, Dict[str, Any]] = {}
    for r in results:
        s = by_variant.setdefault(r.variant, {"runs":0, "success":0, "costs":[], "final_distances":[]})
        s["runs"] += 1
        s["success"] += int(r.success)
        if r.success: 
            s["costs"].append(r.cost)
            s["final_distances"].append(r.final_distance)

    print("\n=== Summary ===")
    summary_lines.append("\n=== Summary ===")

    summary_env = (
    f"Cylinder radius: {CYL_RADIUS_CM} cm | "
    f"Cylinder height: {CYL_HEIGHT_CM} cm | "
    f"Random seed: {MASTER_SEED}"
    )
    print(summary_env)
    summary_lines.append(summary_env)

    obs_info = (
    f"Obstacles per case: "
    f"{N_RANDOM_OBS_RECTANGLE} rectangular and "
    f"{N_RANDOM_OBS_ELLIPSE} elliptical obstacles "
    f"(target numbers in generator)."
    )
    print(obs_info)
    summary_lines.append(obs_info)

    for v, s in by_variant.items():
        sr = 100.0 * s["success"] / s["runs"]
        avg_cost = (sum(s["costs"])/len(s["costs"])) if s["costs"] else float("nan")
        avg_final_dist = (sum(s["final_distances"]) / len(s["final_distances"])) if s["final_distances"] else float("nan")
        line = f"{v}: success {sr:.1f}% | avg cost {avg_cost:.2f} | avg final dist {avg_final_dist:.2f} over {s['runs']} cases"
        print(line)
        summary_lines.append(line)

    print("\n=== Case Outcome Classification ===")
    summary_lines.append("\n=== Case Outcome Classification ===")

    all_case_ids = set(range(1, n_cases + 1))

    # For each case → list of success booleans (one per variant)
    case_success_map: dict[int, list[bool]] = {}

    for r in results:
        case_success_map.setdefault(r.case_id, []).append(r.success)

    # Cases where scenario failed entirely (no variants ran → map never filled)
    cases_no_results = sorted(all_case_ids - set(case_success_map.keys()))

    # Cases where ALL variants succeeded
    cases_all_success = sorted([cid for cid, succ_list in case_success_map.items()
                                if all(succ_list)])

    # Cases where AT LEAST one variant succeeded BUT NOT all
    cases_partial_success = sorted([cid for cid, succ_list in case_success_map.items()
                                    if any(succ_list) and not all(succ_list)])

    # Cases where NO variant succeeded
    cases_all_failed = sorted([cid for cid, succ_list in case_success_map.items()
                            if not any(succ_list)])

    # -------- Print results --------
    print(f"Cases with ALL variants successful: {cases_all_success}")
    summary_lines.append(f"Cases with ALL variants successful: {cases_all_success}")

    print(f"Cases with AT LEAST one success BUT NOT all: {cases_partial_success}")
    summary_lines.append(f"Cases with AT LEAST one success BUT NOT all: {cases_partial_success}")

    print(f"Cases with NO successful variant: {cases_all_failed}")
    summary_lines.append(f"Cases with NO successful variant: {cases_all_failed}")

    if cases_no_results:
        print(f"Cases where NO variants could run (start/goal not found): {cases_no_results}")
        summary_lines.append(f"Cases where NO variants could run (start/goal not found): {cases_no_results}")
        
    summary_path = ts_dir / "summary.txt"
    with open(summary_path, "w") as f:
        f.write("\n".join(summary_lines))
    print(f"Summary saved to {summary_path}")


if __name__ == "__main__":
    run_experiment()
