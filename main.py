from evaluation import *
from evaluation.variant_presets import *


# Default parameters for all variant sets
SHAPE  = "ExactRobot"
MOTION = "UpSideways"
PAD    = "RegularPad"

# ---------------------------------------------------------------------
# ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓  USER CONFIGURATION BELOW  ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
# ---------------------------------------------------------------------

SELECTED_VARIANTS = "motion_groups"   # "base_variant", "shapes", "motion_groups", "padding"
N_CASES = 100                 # how many random scenarios to run
# ---------------------------------------------------------------------
# ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑  USER CONFIGURATION ABOVE  ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
# ---------------------------------------------------------------------

if SELECTED_VARIANTS == "shapes":
    variants = variant_set_shapes(MOTION, PAD)

elif SELECTED_VARIANTS == "motion_groups":
    variants = variant_set_motion_groups(SHAPE, PAD)

elif SELECTED_VARIANTS == "padding":
    variants = variant_set_padding(SHAPE, MOTION)

elif SELECTED_VARIANTS == "base_variant":
    variants = [("ExactRobot", "UpSideways", "RegularPad")]

else:
    raise ValueError(f"Unknown variant set: {SELECTED_VARIANTS}")




if __name__ == "__main__":
    run_experiment(n_cases=N_CASES, variants=variants)