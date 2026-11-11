"""The joint-2 experiment, expressed without ROS-specific message classes."""
import math

JOINTS = tuple(f'joint_{i}' for i in range(1, 7))
WAYPOINTS = ((8, 0.1), (11, 0.1), (27, -0.1), (30, -0.1), (38, 0.0))


def home_error(names, positions, velocities):
    """Return a reason to refuse motion, or None when near home and stopped."""
    if len(set(names)) != len(names):
        return 'Joint names are duplicated.'
    if len(names) != len(positions) or len(names) != len(velocities):
        return 'Joint position or velocity data is incomplete.'
    state = dict(zip(names, zip(positions, velocities)))
    for index, name in enumerate(JOINTS):
        if name not in state:
            return f'Missing feedback for {name}.'
        position, velocity = state[name]
        if not math.isfinite(position) or not math.isfinite(velocity):
            return f'Non-finite feedback for {name}.'
        # Joint 1 is linear; all remaining joints are rotary.
        tolerance = 0.002 if index == 0 else 0.01
        if abs(position) > tolerance or abs(velocity) > tolerance:
            return f'{name} is not near home and stopped: q={position:.5f}, v={velocity:.5f}'
    return None
