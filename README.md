# Bachelor Thesis — Experiments & Results

This repository contains all code and data for running and evaluating experiments for the bachelor thesis on global path planning.


## How to Run, Evaluate, and Plot Results

1. **Configure your experiment:**
	- Set all experiment parameters (such as variant, number of cases, etc.) directly at the top of `main.py`. 
	- Also set evaluation and plotting parameters (such as `CYL_RADIUS_CM`, `N_CASES`, etc.) at the top of `evaluation/evaluate.py` as needed for your analysis and plots. You can explicitly set the random seed for reproducibility.
	- No external config files are needed; simply edit the variables in these files.

2. **Run the experiment:**
	```bash
	python main.py
	```
	This will execute the experiment, evaluate, and generate plots with your chosen settings.

3. **View your results:**
	- Results (plots and data) are saved in the `runs/` directory.
	- For each case, you will find a PNG plot of the solution and a visualization of the scenario.
	- At the end of all cases for a run, you will also find the configuration used (`config`), a `results.csv` file with all results, and a `summary.txt` with a summary of the run.
	- Open these images and outputs in VS Code or your preferred viewer.

## Project Structure

- `main.py` — Main entry point. Configure and run experiments here.
- `evaluation/evaluate.py` — Set evaluation parameters and run evaluation/plotting.
- `agent/`, `environment/`, `planner/` — Core modules for agents, environments, and planners.
- `plot/` — Additional plotting utilities.
- `runs/`, `Archive_Cases/` — (Optional) Output and archive folders for experiment data.

## Typical Workflow

1. Edit `main.py` to set up your experiment.
2. Run `python main.py` to generate, evaluate, and plot results.
3. View results in the `runs/` directory using VS Code or your image viewer.



## Reproducibility

To reproduce any experiment or figure:
- Set the same parameters in `main.py` and `evaluation/evaluate.py` as used originally.
- Run the scripts as above; you should obtain the same plots and metrics.





## Planner Workflow and Details

This project uses an extended A* search for global path planning. Here is how the planner workflow operates:

1. **Experiment Setup**
	- Configure experiment and evaluation parameters in `main.py` and `evaluation/evaluate.py`.
	- Select robot shape, motion model, padding, number of cases, and random seed.

2. **Scenario Generation**
	- For each case, a random scenario is generated (start/goal, obstacles) using the random seed for reproducibility.
	- Obstacles (cylinders, rectangles, ellipses) are created using helpers in `environment/`.

3. **Planner Initialization**
	- The planner (`AStarExtension` in `planner/a_star_ext.py`) is initialized with the environment, robot, motion model, and start/goal.

4. **Presets**
	- Presets in `agent/presets.py` and `evaluation/variant_presets.py` define robot shapes, motion groups, padding, and resolution.
	- These allow quick switching between experiment configurations.

5. **A * Search Algorithm**
	- The planner uses A* to find the shortest path from start to goal on a grid.
	- It expands nodes with the lowest estimated total cost (cost-so-far + heuristic).
	- The heuristic is usually Euclidean or Manhattan distance.
	- For each node, possible moves (from the motion model) are expanded, collision-checked, and added to the search if valid.

6. **Collision Checking**
	- Before moving, the planner checks if the robot would collide with obstacles using the robot's shape and environment obstacles.
	- Collision checking is handled by functions in `environment/` and the robot's collision model.
	- Invalid moves are discarded.

7. **Path Output and Evaluation**
	- If a path is found, it is saved and plotted as a PNG, along with the scenario.
	- Metrics (success, cost, steps, etc.) are recorded.
	- All results are saved in the `runs/` directory, including config, `results.csv`, and `summary.txt`.

8. **Plotting and Analysis**
	- After all cases, review plots and summary files in `runs/`.
	- Each case has a visual output; the run folder contains all results and configuration.

For more details on any step, see the relevant code in the `planner/`, `agent/`, and `environment/` folders.