import numpy.typing as npt
import numpy as np
from scipy.spatial.transform import Rotation


def to_unit_vec(v: np.ndarray):
    """Return the normalized unit vector (length = 1) of a given vector"""
    if v.sum() == 0:
        return v
    return v / np.linalg.norm(v)


def angle_between(v1: np.ndarray, v2: np.ndarray):
    """Return the angle between two vectors in radian"""
    return np.arccos(np.clip(np.dot(to_unit_vec(v1), to_unit_vec(v2)), -1.0, 1.0))


def quat_to_rot_vec(z: float, w: float) -> float:
    """Calculate the rotation in radians on the z-axis given the (z, w) component of a quaternion"""
    rad = 2 * np.arctan2(z, w)
    if rad < 0:
        rad += 2 * np.pi
    return rad


def get_direction_vec(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """Calculate the vector between a start and an end point"""
    return end - start


def rot_from_vec(v: np.ndarray):
    """Calculate the rotation in radian from a vector"""
    rad = np.arctan2(*v[::-1])
    if rad < 0:
        rad += 2 * np.pi
    return rad


def is_right(a: np.ndarray, b: np.ndarray, c: np.ndarray):
    """
    Note that the line is directional!
    If the line goes from b to a, the answer will be the inverse of the
    line going from a to b.
    """
    return bool(((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) < 0)


def angle_between_three(a: npt.NDArray, b: npt.NDArray, c: npt.NDArray):
    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    radians = np.arccos(cosine_angle)
    return radians
