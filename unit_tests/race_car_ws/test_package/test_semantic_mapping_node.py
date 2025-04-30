import pytest
import numpy as np
from collections import defaultdict

from nav_msgs.msg import OccupancyGrid, MapMetaData
from racecar_msgs.msg import DetectedConeArray, DetectedCone
from geometry_msgs.msg import PointStamped, Point, Pose, Quaternion, TransformStamped
from std_msgs.msg import Header
from unittest.mock import MagicMock
import tf2_geometry_msgs
from rclpy.time import Time
from rclpy.clock import ClockType

from race_car_ws.src.test_package.test_package.nodes.semantic_mapping_node import SemanticMappingNode, UNKNOWN_CONE


def create_fake_occupancy_grid(width, height, resolution, data, frame_id="map", origin_x=0.0, origin_y=0.0):
    grid = OccupancyGrid()
    header = Header()
    header.frame_id = frame_id
    grid.header = header

    grid.info = MapMetaData()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution

    grid.info.origin = Pose()
    grid.info.origin.position.x = origin_x
    grid.info.origin.position.y = origin_y
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    grid.data = data
    return grid


def create_fake_detected_cone(cone_type, x, y, frame_id="map"):
    cone = DetectedCone()
    header = Header()
    header.frame_id = frame_id
    header.stamp.sec = 100
    header.stamp.nanosec = 0
    cone.header = header
    cone.type = cone_type
    cone.position = Point(x=x, y=y, z=0.0)
    return cone


@pytest.fixture
def semantic_node(ros_setup):
    node = SemanticMappingNode()
    node.get_logger = MagicMock()  # Mock get_logger to avoid actual logging

    node.filtered_map_pub = MagicMock()
    node.semantic_pub = MagicMock()
    node.tf_buffer = MagicMock()

    # Patch tf_buffer.lookup_transform to return an identity transform.
    def fake_lookup_transform(target_frame, source_frame, stamp, timeout=None):
        ts = TransformStamped()
        ts.header.frame_id = target_frame
        ts.child_frame_id = source_frame
        ts.transform.translation.x = 0.0
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = 0.0
        ts.transform.rotation.x = 0.0
        ts.transform.rotation.y = 0.0
        ts.transform.rotation.z = 0.0
        ts.transform.rotation.w = 1.0
        return ts

    node.tf_buffer.lookup_transform.side_effect = fake_lookup_transform

    # Patch tf2_geometry_msgs.do_transform_point to return the point with updated frame_id.
    def fake_do_transform_point(ps, transform_stamped):
        new_ps = PointStamped()
        new_ps.header = ps.header
        new_ps.header.frame_id = transform_stamped.header.frame_id
        new_ps.point = ps.point
        return new_ps

    tf2_geometry_msgs.do_transform_point = MagicMock(side_effect=fake_do_transform_point)

    yield node
    node.destroy_node()


def test_world_to_grid(semantic_node):
    """Verify world_to_grid conversion."""
    gx, gy = semantic_node.world_to_grid(2.0, 3.0, origin_x=1.0, origin_y=2.0, resolution=0.5)
    assert gx == 2
    assert gy == 2


def test_match_cluster_to_existing(semantic_node):
    """Test cluster matching."""
    semantic_node.clusters_dict = {
        0: {"centroid": (10.0, 10.0), "label_counts": defaultdict(int)},
        1: {"centroid": (20.0, 20.0), "label_counts": defaultdict(int)}
    }
    cid = semantic_node.match_cluster_to_existing(10.5, 10.5, threshold=2.0)
    assert cid == 0
    cid = semantic_node.match_cluster_to_existing(30.0, 30.0, threshold=2.0)
    assert cid is None


def test_cones_callback(semantic_node):
    """Test cones_callback."""
    width, height, resolution = 100, 100, 0.1
    data = [0] * (width * height)
    fake_map = create_fake_occupancy_grid(width, height, resolution, data, frame_id="map", origin_x=0.0, origin_y=0.0)
    semantic_node.current_map = fake_map

    cone = create_fake_detected_cone(cone_type=3, x=1.0, y=1.0, frame_id="camera_frame")  # Use a different frame
    dca = DetectedConeArray()
    dca.cones = [cone]

    semantic_node.cones_callback(dca)

    expected = (1.0, 1.0, 3)  # Expect coordinates in the map frame
    assert semantic_node.new_cones[0][:3] == expected


def test_filter_large_objects(semantic_node):
    """Test filtering of large objects."""
    width, height = 10, 10
    resolution = 0.1
    grid_array = np.zeros((height, width), dtype=int)
    grid_array[1:3, 1:3] = 100  # Small region
    grid_array[5:10, 5:10] = 100  # Large region
    data = grid_array.flatten().tolist()

    fake_map = create_fake_occupancy_grid(width, height, resolution, data, frame_id="map", origin_x=0.0, origin_y=0.0)
    semantic_node.max_cone_length_m = 0.3
    semantic_node.min_cone_area_m2 = 0.03

    filtered_map = semantic_node.filter_large_objects(fake_map)
    filtered_array = np.array(filtered_map.data).reshape((height, width))

    assert np.all(filtered_array[1:3, 1:3] == 100)
    assert np.all(filtered_array[5:10, 5:10] == -1)


def test_build_semantic_grid_no_cones(semantic_node):
    """Test semantic grid building with no cones."""
    width, height = 4, 4
    resolution = 0.1
    data_array = np.array([
        0, 0, 0, 0,
        0, 100, 100, 0,
        0, 100, 100, 0,
        0, 0, 0, 0
    ], dtype=int)
    fake_map = create_fake_occupancy_grid(width, height, resolution, data_array.tolist(), frame_id="map")
    semantic_node.new_cones = []
    semantic_grid = semantic_node.build_semantic_grid(fake_map)

    for cell in semantic_grid.cells:
        if cell.occupancy == 100:
            assert cell.label == UNKNOWN_CONE
        elif cell.occupancy == 0:
            assert cell.label == 0
        elif cell.occupancy == -1:
            assert cell.label == -1


def test_build_semantic_grid_with_cones(semantic_node):
    """Test semantic grid with cone detections."""
    width, height = 6, 6
    resolution = 0.1
    grid_array = np.zeros((height, width), dtype=int)
    grid_array[2:5, 2:5] = 100
    data = grid_array.flatten().tolist()
    fake_map = create_fake_occupancy_grid(width, height, resolution, data, frame_id="map")

    semantic_node.new_cones = [(0.35, 0.35, 1)]
    semantic_node.clusters_dict = {}
    semantic_grid = semantic_node.build_semantic_grid(fake_map)

    for row in range(2, 5):
        for col in range(2, 5):
            idx = row * width + col
            assert semantic_grid.cells[idx].label == 1


def test_map_callback(semantic_node):
    """Test map_callback."""
    width, height = 4, 4
    resolution = 0.1
    data = [0] * (width * height)
    data[5] = 100
    data[6] = 100
    fake_map = create_fake_occupancy_grid(width, height, resolution, data, frame_id="map")
    semantic_node.current_map = fake_map  # Set current_map
    fake_time = Time(seconds=10, nanoseconds=0, clock_type=ClockType.ROS_TIME)
    semantic_node.new_cones = [(1.0, 1.0, 3, fake_time)]

    semantic_node.map_callback(fake_map)

    # Verify that the publish methods were called.
    semantic_node.filtered_map_pub.publish.assert_called()
    semantic_node.semantic_pub.publish.assert_called()
    # Check if new_cones is cleared.
    assert semantic_node.new_cones == []
