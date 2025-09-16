# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
import numpy as np
import pytest
from avai_lab.track_gen import (
    graham_scan,
    generate_track,
)


def test_graham_scan_square():
    # Define square and one point inside square
    points = np.array([
        [0, 0],
        [0, 1],
        [1, 0],
        [1, 1],
        [0.5, 0.5]
    ])
    hull = graham_scan(points)
    # Convex hull points should be square edges
    expected = {(0, 0), (0, 1), (1, 0), (1, 1)}
    hull_tuples = {tuple(np.round(pt, 5)) for pt in hull}
    assert hull_tuples == expected


def test_graham_scan_few_points():
    # For two points, should simply return them.
    points = np.array([[0, 0], [1, 1]])
    hull = graham_scan(points)
    np.testing.assert_array_equal(hull, points)


def test_generate_track_invalid_n():
    # If n < 4, generate_track should assert.
    with pytest.raises(AssertionError):
        generate_track(3)


def test_generate_track_invalid_intensity():
    # intensity must be between 0 and 1.
    with pytest.raises(AssertionError):
        generate_track(10, intensity=1.5)
