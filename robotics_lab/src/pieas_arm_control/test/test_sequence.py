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


def test_cubic_segments_fit_joint_two_velocity_limit():
    # Rest-to-rest cubic interpolation peaks at 1.5 * displacement / duration.
    previous_time, previous_angle = 0.0, 0.0
    for seconds, angle in WAYPOINTS:
        assert seconds > previous_time
        assert abs(angle) <= 0.1
        peak_speed = 1.5 * abs(angle - previous_angle) / (seconds - previous_time)
        assert peak_speed < 0.0349065850399
        previous_time, previous_angle = seconds, angle
    assert previous_angle == 0.0
