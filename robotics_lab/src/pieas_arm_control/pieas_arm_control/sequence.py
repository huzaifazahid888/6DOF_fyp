# This file contains experiment data and a helper, with no ROS communication.
# Editing WAYPOINTS changes the requested sequence. It does not move the robot
# until joint_sequence.py is run again. It currently varies joint_2 only.

"""The joint-2 experiment, expressed without ROS-specific message classes."""
# Python standard library; isfinite rejects NaN and positive/negative infinity.
import math

# range(1, 7) produces 1 through 6. Each f-string builds a joint name.
# A tuple is an immutable sequence. This order defines the outgoing arrays.
JOINTS = tuple(f'joint_{i}' for i in range(1, 7))
# Each pair is (seconds from start, target angle of joint_2 in radians).
# 8: reach +0.1; 11: finish hold; 27: reach -0.1; 30: finish hold; 38: home.
# These are cumulative times, not durations to add together.
# joint_sequence.py fills the other five target coordinates with zero.
WAYPOINTS = ((8, 0.1), (11, 0.1), (27, -0.1), (30, -0.1), (38, 0.0))


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
