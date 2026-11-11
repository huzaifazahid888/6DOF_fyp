"""Small-range six-joint demonstration. No ROS communication in this file."""
import math

# Array order is always rail first (metres), then five angles (radians).
JOINTS = tuple(f'joint_{i}' for i in range(1, 7))
HOME = (0.0,) * 6


def build_waypoints():
    """Return (arrival seconds, six absolute positions, stage description)."""
    points = []
    elapsed = 0

    def add(duration, positions, label):
        # Convert a segment duration into time from the beginning of the goal.
        nonlocal elapsed
        elapsed += duration
        points.append((elapsed, tuple(positions), label))

    # Isolate each joint so its axis and effect on the tool are easy to observe.
    # Negative flexion angles lift the links in this model's home configuration.
    # These are demonstration distances, NOT the mechanical travel limits.
    excursions = (0.02, 0.10, -0.10, -0.10, -0.10, 0.15)
    for index, excursion in enumerate(excursions):
        pose = list(HOME)
        pose[index] = excursion
        label = f'{JOINTS[index]} only'
        add(8, pose, label + ': move out')
        add(2, pose, label + ': hold')
        add(8, HOME, label + ': return home')
        add(2, HOME, label + ': home hold')

    # Coordinated motion: all six coordinates change in the same time segment.
    # The controller interpolates JOINT coordinates. This is not a straight-line
    # tool-tip command, inverse kinematics, or collision-aware motion planning.
    pose_a = (0.02, 0.10, -0.10, -0.08, -0.05, 0.15)
    pose_b = (-0.02, -0.10, -0.06, -0.12, -0.08, -0.15)
    add(12, pose_a, 'coordinated: lift and move along rail')
    add(2, pose_a, 'coordinated: hold pose A')
    add(12, pose_b, 'coordinated: reverse rail and change arm pose')
    add(2, pose_b, 'coordinated: hold pose B')
    add(12, pose_a, 'coordinated: return to pose A')
    add(2, pose_a, 'coordinated: hold pose A again')
    add(12, HOME, 'all joints: return home')
    add(2, HOME, 'all joints: final home hold')
    return tuple(points)


WAYPOINTS = build_waypoints()
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
