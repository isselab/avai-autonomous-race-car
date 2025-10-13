# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
import math

import numpy as np
from shapely.ops import polygonize
from shapely.geometry import MultiLineString, Polygon
from shapely import unary_union, affinity
from scipy.spatial import Delaunay
from avai_lab.utils import is_right


def graham_scan(X: np.ndarray):
    """Graham scan separates the problem into the lower/upper convex hull
    First all the points are sorted by their x-coordinate and the upper convex hull is
    calculated from the leftmost point. The algorithm goes through all points and appends
    them to the convex hull in order. After each point it will check whether the last three points
    form a right turn. If they are not then the second last point does not belong to the convex hull
    and is discarded. The check is then repeated for the last three points of the convex hull again
    until the condition is satisfied.
    The lower hull can be calculated by simple inverting the problem.
    This runs in O(n log n) because of the sorting.
    """
    if len(X) <= 2:
        pass
        return X

    def partial_convex_hull(P: np.ndarray):
        convex_hull = []
        for p in P:
            convex_hull.append(p)
            while len(convex_hull) >= 3 and is_right(convex_hull[-3], convex_hull[-2], convex_hull[-1]):
                convex_hull.pop(-2)
        return np.array(convex_hull)

    P = X[np.argsort(X[:, 0])]
    upper_hull = partial_convex_hull(P)
    lower_hull = partial_convex_hull(P[::-1])
    return np.concatenate((upper_hull[1:], lower_hull[1:]))


def alpha_shape(coords: np.ndarray, alpha: float = 1.0):
    """
    Compute the alpha shape (concave hull) of a set of points.

    Credit goes to: https://github.com/dwyerk/boundaries
    """
    if len(coords) < 4:
        return graham_scan(coords)

    def add_edge(edges, edge_points, coords, i, j):
        """Add a line between the i-th and j-th points, if not in the list already"""
        if (i, j) in edges or (j, i) in edges:
            # already added
            return
        edges.add((i, j))
        edge_points.append(coords[[i, j]])

    tri = Delaunay(coords)
    edges = set()
    edge_points = []
    # loop over triangles:
    # ia, ib, ic = indices of corner points of the triangle
    for ia, ib, ic in tri.simplices:
        pa = coords[ia]
        pb = coords[ib]
        pc = coords[ic]

        # Lengths of sides of triangle
        a = math.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2)
        b = math.sqrt((pb[0] - pc[0]) ** 2 + (pb[1] - pc[1]) ** 2)
        c = math.sqrt((pc[0] - pa[0]) ** 2 + (pc[1] - pa[1]) ** 2)

        # Semiperimeter of triangle
        s = (a + b + c) / 2.0

        # Area of triangle by Heron's formula
        area = math.sqrt(s * (s - a) * (s - b) * (s - c))
        circum_r = a * b * c / (4.0 * area)

        # Here's the radius filter.
        # print circum_r
        if circum_r < 1.0 / alpha:
            add_edge(edges, edge_points, coords, ia, ib)
            add_edge(edges, edge_points, coords, ib, ic)
            add_edge(edges, edge_points, coords, ic, ia)

    m = MultiLineString(edge_points)
    triangles = list(polygonize(m))
    return unary_union(triangles), edge_points


def chaikins_corner_cutting(coords: np.ndarray, refinements: int = 5):
    """Create two new points for each corner with a reduced angle resulting in 2 * N points"""
    coords = np.array(coords)
    for _ in range(refinements):
        L = coords.repeat(2, axis=0)
        R = np.empty_like(L)
        R[0] = L[0]
        R[2::2] = L[1:-1:2]
        R[1:-1:2] = L[2::2]
        R[-1] = L[-1]
        coords = L * 0.75 + R * 0.25
    return coords


def resample_polygon(xy: np.ndarray, n_points: int = 100) -> np.ndarray:
    # credit to Tankred: https://stackoverflow.com/questions/66833406/resample-polygon-coordinates-to-space-them-evenly
    # Cumulative Euclidean distance between successive polygon points.
    # This will be the "x" for interpolation
    d = np.cumsum(np.r_[0, np.sqrt((np.diff(xy, axis=0) ** 2).sum(axis=1))])

    # get linearly spaced points along the cumulative Euclidean distance
    d_sampled = np.linspace(0, d.max(), n_points)

    # interpolate x and y coordinates
    xy_interp = np.c_[
        np.interp(d_sampled, d, xy[:, 0]),
        np.interp(d_sampled, d, xy[:, 1]),
    ]
    return xy_interp


def generate_track(n: int, size_x: float = 10, size_y: float = 10, track_width: int = 1,
                   refinements: int = 2, alpha: int = 1, n_points: int = 100, dents: int = 0,
                   intensity: float = 0.5):
    """Generate a track based on the convex hull of a normal distribution"""
    assert n >= 4, "The minimum number of points to generate a track is 4"
    assert intensity <= 1 and intensity >= 0, "Intensity must be from the interval [0, 1]"
    X = np.random.normal(size=(n, 2))
    outlier_idx = np.abs(X) > 1
    valid_indices = np.where(~outlier_idx)[0]
    X = X[valid_indices]
    concave_hull, _ = alpha_shape(X, alpha=alpha)
    # Add dents for stronger turns
    dented_hull = np.array(concave_hull.exterior.coords)
    assert dents <= len(dented_hull) / 2, "Shrinking more than every second point would result in a scaling operation"
    dent_multiplier = 1 - intensity
    for i in range(0, len(dented_hull), len(dented_hull) // (dents + 1)):
        dented_hull[i] *= dent_multiplier

    track_inner = affinity.scale(Polygon(dented_hull).normalize(), size_x, size_y).buffer(1)
    track_middle = track_inner.buffer(track_width / 2, join_style="mitre")
    track_outer = track_inner.buffer(track_width, join_style="mitre")

    track_ref_inner = chaikins_corner_cutting(np.array(track_inner.exterior.coords), refinements=refinements)
    track_ref_middle = chaikins_corner_cutting(np.array(track_middle.exterior.coords), refinements=refinements)
    track_ref_outer = chaikins_corner_cutting(np.array(track_outer.exterior.coords), refinements=refinements)
    return (resample_polygon(track_ref_inner, n_points),
            resample_polygon(track_ref_middle, n_points),
            resample_polygon(track_ref_outer, n_points))
