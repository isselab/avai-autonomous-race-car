import math
import numpy as np
from avai_lab.utils import (
    to_unit_vec,
    angle_between,
    quat_to_rot_vec,
    get_direction_vec,
    is_right,
)


def test_to_unit_vec_nonzero():
    v = np.array([3, 4])
    unit = to_unit_vec(v)
    expected = np.array([0.6, 0.8])
    np.testing.assert_allclose(unit, expected)


def test_to_unit_vec_zero():
    v = np.array([0, 0])
    unit = to_unit_vec(v)
    np.testing.assert_array_equal(unit, v)


def test_angle_between_orthogonal():
    v1 = np.array([1, 0])
    v2 = np.array([0, 1])
    angle = angle_between(v1, v2)
    expected = math.pi / 2
    assert math.isclose(angle, expected)


def test_angle_between_parallel():
    v1 = np.array([1, 1])
    v2 = np.array([2, 2])
    angle = angle_between(v1, v2)
    expected = 0.0
    assert math.isclose(angle, expected, abs_tol=1e-7)


def test_quat_to_rot_vec_zero():
    rad = quat_to_rot_vec(0, 1)
    assert math.isclose(rad, 0.0)


def test_quat_to_rot_vec_pi():
    rad = quat_to_rot_vec(1, 0)
    assert math.isclose(rad, math.pi)


def test_quat_to_rot_vec_negative():
    rad = quat_to_rot_vec(-1, 0)
    assert math.isclose(rad, math.pi)


def test_get_direction_vec():
    start = np.array([1, 2])
    end = np.array([4, 6])
    vec = get_direction_vec(start, end)
    expected = np.array([3, 4])
    np.testing.assert_array_equal(vec, expected)


def test_is_right():
    a = np.array([0, 0])
    b = np.array([1, 0])
    c_right = np.array([1, -1])
    c_left = np.array([1, 1])
    assert is_right(a, b, c_right) is True
    assert is_right(a, b, c_left) is False
