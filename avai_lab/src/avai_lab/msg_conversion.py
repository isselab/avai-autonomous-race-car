# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
import numpy as np
import numpy.typing as npt

from racecar_msgs.msg import SemanticGrid


def semantic_grid_to_np(grid: SemanticGrid) -> npt.NDArray:
    width = grid.info.width
    height = grid.info.height
    labels = np.empty((height, width), dtype=int)
    for i, cell in enumerate(grid.cells):
        row = i // width
        col = i % width
        labels[row, col] = cell.label
    return labels
