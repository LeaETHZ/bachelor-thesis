# evaluation/variant_presets.py

def variant_set_shapes(motion: str, res: str, pad: str):
    """Compare robot shapes, keep motion/res/pad fixed."""
    return [
        ("ExactRobot", motion, res, pad),
        ("RectangleRobot", motion, res, pad),
    ]

def variant_set_motion_groups(shape: str, res: str, pad: str):
    """Compare different motion groups, keep robot shape/res/pad fixed."""
    return [
        (shape, "UpSideways", res, pad),
        (shape, "UpSidewaysDown", res, pad),
        (shape, "UpSidewaysDiagonal", res, pad),
        (shape, "VaryingLength", res, pad),
    ]


def variant_set_resolution(shape: str, motion: str, pad: str):
    """Compare resolutions."""
    return [
        (shape, motion, "RegularRes", pad),
        (shape, motion, "HighRes", pad),
        (shape, motion, "LowRes", pad),
    ]

def variant_set_padding(shape: str, motion: str, res: str):
    """Compare different padding settings."""
    return [
        (shape, motion, res, "RegularPad"),
        (shape, motion, res, "HighPad"),
        (shape, motion, res, "LowPad"),
    ]