import math
import pytest
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import MapMetaData
from racecar_msgs.msg import SemanticGrid, SemanticCell
from visualization_msgs.msg import Marker
from unittest.mock import MagicMock

from race_car_ws.src.test_package.test_package.nodes.semantic_grid_visualizer_node import SemanticGridVisualizerNode
from avai_lab import enums


@pytest.fixture
def semantic_visualizer_node(ros_setup):
    node = SemanticGridVisualizerNode()
    node.marker_pub = MagicMock()
    node._published_markers = None
    node.marker_pub.publish = lambda msg: setattr(node, "_published_markers", msg)

    yield node
    node.destroy_node()


def test_get_color_yellow(semantic_visualizer_node):
    color = semantic_visualizer_node.get_color(enums.YELLOW_CONE)
    assert math.isclose(color.r, 1.0)
    assert math.isclose(color.g, 1.0)
    assert math.isclose(color.b, 0.0)
    assert math.isclose(color.a, 1.0)


def test_get_color_blue(semantic_visualizer_node):
    color = semantic_visualizer_node.get_color(enums.BLUE_CONE)
    assert math.isclose(color.r, 0.0)
    assert math.isclose(color.g, 0.0)
    assert math.isclose(color.b, 1.0)
    assert math.isclose(color.a, 1.0)


def test_get_color_orange(semantic_visualizer_node):
    color = semantic_visualizer_node.get_color(enums.ORANGE_CONE)
    assert math.isclose(color.r, 1.0)
    assert math.isclose(color.g, 0.5)
    assert math.isclose(color.b, 0.0)
    assert math.isclose(color.a, 1.0)


def test_get_color_unknown(semantic_visualizer_node):
    color = semantic_visualizer_node.get_color(enums.UNKNOWN_CONE)
    assert math.isclose(color.r, 0.5)
    assert math.isclose(color.g, 0.5)
    assert math.isclose(color.b, 0.5)
    assert math.isclose(color.a, 1.0)


def test_get_color_default(semantic_visualizer_node):
    color = semantic_visualizer_node.get_color(999)
    assert math.isclose(color.a, 0.0)


def create_fake_semantic_grid(width, height, resolution, origin_x, origin_y, cell_values, frame_id="map"):
    sem_grid = SemanticGrid()
    header = Header()
    header.frame_id = frame_id
    sem_grid.header = header

    info = MapMetaData()
    info.width = width
    info.height = height
    info.resolution = resolution
    pose = Pose()
    pose.position.x = origin_x
    pose.position.y = origin_y
    pose.position.z = 0.0
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    info.origin = pose
    sem_grid.info = info

    cells = []
    for val in cell_values:
        cell = SemanticCell()
        cell.occupancy = 100 if val != 0 else 0
        cell.label = val
        cells.append(cell)
    sem_grid.cells = cells
    return sem_grid


def test_semantic_grid_callback_single_cluster(semantic_visualizer_node):
    """
    Create a 4x4 semantic grid with a 2x2 contiguous cluster of YELLOW_CONE cells.
    Verify that the node computes a marker at the correct centroid.
    """
    width, height = 4, 4
    resolution = 1.0
    origin_x, origin_y = 0.0, 0.0
    cell_values = [
        0, 0, 0, 0,
        0, enums.YELLOW_CONE, enums.YELLOW_CONE, 0,
        0, enums.YELLOW_CONE, enums.YELLOW_CONE, 0,
        0, 0, 0, 0
    ]
    sem_grid = create_fake_semantic_grid(width, height, resolution, origin_x, origin_y, cell_values)

    semantic_visualizer_node.semantic_grid_callback(sem_grid)

    marker_array = semantic_visualizer_node._published_markers
    assert marker_array is not None

    assert len(marker_array.markers) == 1
    marker = marker_array.markers[0]

    assert marker.header.frame_id == semantic_visualizer_node.frame_id
    assert marker.type == Marker.SPHERE

    assert math.isclose(marker.pose.position.x, 2.0)
    assert math.isclose(marker.pose.position.y, 2.0)
    assert math.isclose(marker.pose.position.z, 0.0)

    scale = semantic_visualizer_node.cluster_marker_scale
    assert math.isclose(marker.scale.x, scale)
    assert math.isclose(marker.scale.y, scale)
    assert math.isclose(marker.scale.z, scale)

    expected_color = semantic_visualizer_node.get_color(enums.YELLOW_CONE)
    assert math.isclose(marker.color.r, expected_color.r)
    assert math.isclose(marker.color.g, expected_color.g)
    assert math.isclose(marker.color.b, expected_color.b)
    assert math.isclose(marker.color.a, expected_color.a)


def test_semantic_grid_callback_no_clusters(semantic_visualizer_node):
    width, height = 3, 3
    resolution = 1.0
    origin_x, origin_y = 0.0, 0.0

    cell_values = [0] * (width * height)
    sem_grid = create_fake_semantic_grid(width, height, resolution, origin_x, origin_y, cell_values)

    semantic_visualizer_node.semantic_grid_callback(sem_grid)
    marker_array = semantic_visualizer_node._published_markers
    assert marker_array is not None
    assert len(marker_array.markers) == 0
