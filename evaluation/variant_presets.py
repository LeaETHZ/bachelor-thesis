# evaluation/variant_presets.py

def variant_set_shapes(motion: str, pad: str):
    """Compare robot shapes, keep motion/res/pad fixed."""
    return [
        ("ExactRobot", motion, pad),
        ("RectangleRobot", motion, pad),
    ]

def variant_set_motion_groups(shape: str, pad: str):
    """Compare different motion groups, keep robot shape/res/pad fixed."""
    return [
        (shape, "UpSideways", pad),
        (shape, "UpSidewaysDown", pad),
        (shape, "UpSidewaysDiagonal", pad),
        (shape, "VaryingLength", pad),
    ]

def variant_set_padding(shape: str, motion: str):
    """Compare different padding settings."""
    return [
        (shape, motion, "RegularPad"),
        (shape, motion, "HighPad"),
        (shape, motion, "LowPad"),
    ]