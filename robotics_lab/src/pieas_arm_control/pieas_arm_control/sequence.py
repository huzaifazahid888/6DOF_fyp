"""Thirty-second configured-range simulation test, not a collision planner."""
import math

# The rail uses metres. All other coordinates use radians.
JOINTS = tuple(f'joint_{i}' for i in range(1, 7))
HOME = (0.0,) * 6
# These match the bounded joints in the URDF. Joint 6 is continuous, so its
# +/-pi targets are chosen test angles, not physical end stops.
UPPER = (0.5, 3.14159265359, 3.14159265359, 3.14159265359, 3.14159265359, math.pi)
LOWER = tuple(-q for q in UPPER)

# Each row is (arrival seconds, absolute joint coordinates, stage label).
# Halfway points keep continuous-joint segments below pi so the commanded
# direction is unambiguous. Zero endpoint velocities make each segment stop.
# All joints move together; testing them separately would take longer.
WAYPOINTS = (
    (4, tuple(q / 2 for q in UPPER), 'upper sweep: halfway'),
    (8, UPPER, 'upper sweep: configured limits'),
    (11, tuple(q / 2 for q in UPPER), 'return from upper: halfway'),
    (15, HOME, 'return from upper: home'),
    (19, tuple(q / 2 for q in LOWER), 'lower sweep: halfway'),
    (22, LOWER, 'lower sweep: configured limits'),
    (26, tuple(q / 2 for q in LOWER), 'return from lower: halfway'),
    (30, HOME, 'return from lower: home'),
)
DURATION = WAYPOINTS[-1][0]


# Our own helper, called by JointSequence.run before sending any motion.
# Inputs are parallel arrays from a JointState message. It returns an error
# string on failure, or None when the starting-state checks pass.
def home_error(names, positions, velocities):
    """Return a reason to refuse motion, or None when near home and stopped."""
    # A set removes duplicates. Different lengths reveal repeated joint names.
    if len(set(names)) != len(names):
        return 'Joint names are duplicated.'
    # Every name needs one position and one velocity; effort is not needed here.
    if len(names) != len(positions) or len(names) != len(velocities):
        return 'Joint position or velocity data is incomplete.'
    # The inner zip makes (position, velocity) pairs. The outer zip pairs each
    # name with its values. dict makes a lookup such as joint_2 -> (0.0, 0.0).
    # This handles a publisher that sends joints in a different array order.
    state = dict(zip(names, zip(positions, velocities)))
    # Check every required joint. enumerate supplies its index and name.
    for index, name in enumerate(JOINTS):
        if name not in state:
            return f'Missing feedback for {name}.'
        # Unpack the two stored values for this named joint.
        position, velocity = state[name]
        # Reject invalid measurements instead of letting NaN bypass comparisons.
        if not math.isfinite(position) or not math.isfinite(velocity):
            return f'Non-finite feedback for {name}.'
        # Joint 1 is linear; all remaining joints are rotary.
        # For joint 1: position 0.002 metres and speed 0.002 metres/second.
        # For rotary joints: position 0.01 radians and speed 0.01 radians/second.
        # We reuse the numeric threshold for different units; these are lesson checks,
        # not a claim about actuator precision or hardware safety limits.
        tolerance = 0.002 if index == 0 else 0.01
        # Require both near-zero position and near-zero speed. abs ignores the sign.
        if abs(position) > tolerance or abs(velocity) > tolerance:
            return f'{name} is not near home and stopped: q={position:.5f}, v={velocity:.5f}'
    # All six joints passed. None tells run() it may proceed.
    return None
