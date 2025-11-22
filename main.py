from evaluation import *
from evaluation.variant_presets import *


# Default parameters for all variant sets
SHAPE  = "ExactRobot"
MOTION = "UpSideways"
RES    = "RegularRes"
PAD    = "RegularPad"

# ---------------------------------------------------------------------
# ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓  USER CONFIGURATION BELOW  ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
# ---------------------------------------------------------------------

SELECTED_VARIANTS = "motion_groups"   # "shapes", "motion_groups", "padding", "resolution"
N_CASES = 2                     # how many random scenarios to run

# ---------------------------------------------------------------------
# ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑  USER CONFIGURATION ABOVE  ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
# ---------------------------------------------------------------------

if SELECTED_VARIANTS == "shapes":
    variants = variant_set_shapes(MOTION, RES, PAD)

elif SELECTED_VARIANTS == "motion_groups":
    variants = variant_set_motion_groups(SHAPE, RES, PAD)


elif SELECTED_VARIANTS == "resolution":
    variants = variant_set_resolution(SHAPE, MOTION, PAD)

elif SELECTED_VARIANTS == "padding":
    variants = variant_set_padding(SHAPE, MOTION, RES)

else:
    raise ValueError(f"Unknown variant set: {SELECTED_VARIANTS}")




if __name__ == "__main__":
    run_experiment(n_cases=N_CASES, variants=variants)