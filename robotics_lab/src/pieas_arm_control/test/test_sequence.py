import math
from pieas_arm_control.sequence import JOINTS, WAYPOINTS, home_error


def test_home_accepts_shuffled_joint_order():
    assert home_error(list(reversed(JOINTS)), [0.0] * 6, [0.0] * 6) is None


def test_missing_or_invalid_feedback_is_rejected():
    assert home_error(JOINTS[:-1], [0.0] * 5, [0.0] * 5)
    assert home_error(JOINTS, [0.0] * 6, [])
    assert home_error(JOINTS, [math.nan] + [0.0] * 5, [0.0] * 6)
    assert home_error(JOINTS, [0.0] * 6, [math.inf] + [0.0] * 5)
    assert home_error([JOINTS[0]] * 6, [0.0] * 6, [0.0] * 6)


def test_not_home_or_still_moving_is_rejected():
    assert home_error(JOINTS, [0.003] + [0.0] * 5, [0.0] * 6)
    assert home_error(JOINTS, [0.0, 0.1, 0.0, 0.0, 0.0, 0.0], [0.0] * 6)
    assert home_error(JOINTS, [0.0] * 6, [0.0, 0.02, 0.0, 0.0, 0.0, 0.0])


def test_all_segments_fit_model_limits_and_exercise_every_joint():
    import xml.etree.ElementTree as ET
    from pathlib import Path
    model = Path(__file__).resolve().parents[2] / 'pieas_arm_description/urdf/pieas_control.urdf.in'
    joints = {j.attrib['name']: j for j in ET.parse(model).getroot().findall('joint')}
    previous_time, previous_pose = 0.0, (0.0,) * 6
    moved = set()
    for seconds, pose, label in WAYPOINTS:
        assert seconds > previous_time and label
        assert len(pose) == 6 and all(math.isfinite(q) for q in pose)
        for index, name in enumerate(JOINTS):
            limit = joints[name].find('limit')
            if 'lower' in limit.attrib:
                assert float(limit.attrib['lower']) <= pose[index] <= float(limit.attrib['upper'])
            # Rest-to-rest cubic interpolation peaks at 1.5 * displacement / time.
            speed = 1.5 * abs(pose[index] - previous_pose[index]) / (seconds - previous_time)
            assert speed < float(limit.attrib['velocity'])
            if pose[index] != 0.0:
                moved.add(name)
        previous_time, previous_pose = seconds, pose
    assert moved == set(JOINTS)
    assert previous_pose == (0.0,) * 6


def test_full_configured_range_and_thirty_second_finish():
    from pieas_arm_control.sequence import UPPER, LOWER, HOME, DURATION
    assert DURATION == 30
    assert WAYPOINTS[1][1] == UPPER
    assert WAYPOINTS[5][1] == LOWER
    assert WAYPOINTS[3][1] == WAYPOINTS[-1][1] == HOME
    previous = HOME
    for _, pose, _ in WAYPOINTS:
        # Avoid ambiguous +/-pi steps for the continuous wrist joint.
        assert abs(pose[5] - previous[5]) < math.pi
        previous = pose
