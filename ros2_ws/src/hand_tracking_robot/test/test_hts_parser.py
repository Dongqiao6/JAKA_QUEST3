import numpy as np
import pytest

from hand_tracking_robot.hts_parser import (
    HandState,
    matrix_to_quat,
    parse_hts_line,
    quat_rotate,
    quat_to_matrix,
)


def test_parse_wrist_and_landmarks():
    line = "Right wrist:, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0"
    assert parse_hts_line(line) == ("right", "wrist", (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0))

    values = ",".join("0.0" for _ in range(63))
    parsed = parse_hts_line("Left landmarks:, " + values)
    assert parsed[0] == "left"
    assert parsed[1] == "landmarks"
    assert len(parsed[2]) == 63


def test_parse_debug_header():
    line = "Right wrist | f = 12 | t = 123456:, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0"
    parsed = parse_hts_line(line)
    assert parsed[0] == "right"
    assert parsed[1] == "wrist"
    assert parsed[2][:3] == (0.1, 0.2, 0.3)


def test_parse_ignores_head_pose():
    assert parse_hts_line("Head pose:, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0") is None


def test_quaternion_helpers_roundtrip():
    q = np.array([0.0, 0.0, 0.0, 1.0])
    recovered = matrix_to_quat(quat_to_matrix(q))
    assert np.allclose(np.abs(recovered), np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-9)


def test_hand_pinch_distance():
    hand = HandState("right")
    hand.update_wrist((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
    landmarks = np.zeros((21, 3))
    landmarks[4] = [0.02, 0.0, 0.0]   # thumb tip
    landmarks[8] = [0.05, 0.0, 0.0]   # index tip
    hand.update_landmarks(tuple(landmarks.flatten()))
    assert hand.pinch_distance() == pytest.approx(0.03)


def test_quat_rotate_identity():
    points = np.array([[1.0, 0.0, 0.0]])
    rotated = quat_rotate(points, np.array([0.0, 0.0, 0.0, 1.0]))
    assert np.allclose(rotated, points)
