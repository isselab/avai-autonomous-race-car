# SPDX-License-Identifier: MIT
# Authors: Kevin Losing

import math
import pytest
import rclpy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from racecar_msgs.msg import DetectedCone, DetectedConeArray
from visualization_msgs.msg import Marker
from avai_lab.enums import BLUE_CONE, ORANGE_CONE, YELLOW_CONE

import tf2_geometry_msgs

from race_car_ws.src.test_package.test_package.nodes.cone_marker_node import ConeMarkerNode

from unittest.mock import MagicMock


@pytest.fixture
def cone_marker_node(ros_setup):
    node = ConeMarkerNode()
    node._published_markers = None
    node.markers_pub = MagicMock()
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


def create_fake_cone(cone_type, x, y, frame_id="base_link"):
    """
    Create a fake DetectedCone message with the specified type and position.
    """
    cone = DetectedCone()
    header = Header()
    header.frame_id = frame_id
    header.stamp = rclpy.time.Time(seconds=100).to_msg()
    cone.header = header
    cone.type = cone_type
    cone.position = Point(x=x, y=y, z=0.0)
    return cone


def create_fake_cone_array(cones):
    """
    Create a fake DetectedConeArray message from a list of DetectedCone messages.
    """
    dca = DetectedConeArray()
    dca.cones = cones
    return dca


@pytest.mark.parametrize("cone_type, expected_color", [
    (YELLOW_CONE, (1.0, 1.0, 0.0)),
    (BLUE_CONE, (0.0, 0.0, 1.0)),
    (ORANGE_CONE, (1.0, 0.65, 0.0)),
    (999, (1.0, 0.0, 1.0))
])
def test_cone_color(cone_marker_node, cone_type, expected_color):
    color = cone_marker_node.cone_color(cone_type)
    assert math.isclose(color[0], expected_color[0])
    assert math.isclose(color[1], expected_color[1])
    assert math.isclose(color[2], expected_color[2])


def test_transform_cone_success(cone_marker_node):
    # Create a fake cone with header.frame_id "base_link"
    cone = create_fake_cone(YELLOW_CONE, 1.0, 2.0, frame_id="base_link")

    # Call transform_cone (patched with fake transform)
    pt_map = cone_marker_node.transform_cone(cone)

    # With fake transform the point should remain unchanged.
    assert pt_map is not None
    assert pt_map.header.frame_id == cone_marker_node.target_frame
    assert math.isclose(pt_map.point.x, cone.position.x)
    assert math.isclose(pt_map.point.y, cone.position.y)
    assert math.isclose(pt_map.point.z, cone.position.z)


def test_transform_cone_failure(cone_marker_node):
    # Simulate exception in transform
    cone_marker_node.tf_buffer.lookup_transform.side_effect = Exception("TF error")
    cone = create_fake_cone(BLUE_CONE, 1.0, 2.0, frame_id="base_link")
    pt_map = cone_marker_node.transform_cone(cone)
    assert pt_map is None


def test_cones_callback_all_valid(cone_marker_node):
    cone1 = create_fake_cone(YELLOW_CONE, 1.0, 2.0, frame_id="base_link")
    cone2 = create_fake_cone(BLUE_CONE, 3.0, 4.0, frame_id="base_link")
    dca = create_fake_cone_array([cone1, cone2])

    # Call cones_callback.
    cone_marker_node.cones_callback(dca)

    cone_marker_node.markers_pub.publish.assert_called_once()
    marker_array = cone_marker_node.markers_pub.publish.call_args[0][0]

    assert marker_array is not None
    # The first marker must be a DELETEALL marker.
    assert marker_array.markers[0].action == Marker.DELETEALL
    # Expect one DELETEALL marker plus one marker per valid cone.
    assert len(marker_array.markers) == 1 + 2

    marker1 = marker_array.markers[1]
    marker2 = marker_array.markers[2]
    assert marker1.header.frame_id == cone_marker_node.target_frame
    assert marker2.header.frame_id == cone_marker_node.target_frame
    assert marker1.ns == "cones"
    assert marker2.ns == "cones"
    assert marker1.id == 0
    assert marker2.id == 1

    # Verify marker colors match the cone_color() output.
    color1 = cone_marker_node.cone_color(YELLOW_CONE)
    color2 = cone_marker_node.cone_color(BLUE_CONE)
    assert math.isclose(marker1.color.r, color1[0])
    assert math.isclose(marker1.color.g, color1[1])
    assert math.isclose(marker1.color.b, color1[2])
    assert math.isclose(marker2.color.r, color2[0])
    assert math.isclose(marker2.color.g, color2[1])
    assert math.isclose(marker2.color.b, color2[2])

    # Verify marker scale (all axes).
    scale = 0.2
    for marker in (marker1, marker2):
        assert math.isclose(marker.scale.x, scale)
        assert math.isclose(marker.scale.y, scale)
        assert math.isclose(marker.scale.z, scale)


def test_cones_callback_with_invalid_transform(cone_marker_node):
    """
    Test that if transform_cone() fails for a cone in the list, that cone is skipped.
    """

    def selective_transform(cone):
        if cone.type == ORANGE_CONE:
            return None
        else:
            ps = PointStamped()
            ps.header = cone.header
            ps.header.frame_id = cone_marker_node.target_frame
            ps.point = cone.position
            return ps

    cone_marker_node.transform_cone = MagicMock(side_effect=selective_transform)

    cone1 = create_fake_cone(ORANGE_CONE, 5.0, 6.0, frame_id="base_link")  # Will fail transform.
    cone2 = create_fake_cone(YELLOW_CONE, 7.0, 8.0, frame_id="base_link")  # Valid.
    dca = create_fake_cone_array([cone1, cone2])

    cone_marker_node.cones_callback(dca)
    cone_marker_node.markers_pub.publish.assert_called_once()
    marker_array = cone_marker_node.markers_pub.publish.call_args[0][0]

    # Expect one DELETEALL marker plus only one valid cone marker.
    assert marker_array is not None
    assert len(marker_array.markers) == 2

    # Verify the published marker corresponds to the valid cone
    marker = marker_array.markers[1]
    expected_color = cone_marker_node.cone_color(YELLOW_CONE)
    assert math.isclose(marker.color.r, expected_color[0])
    assert math.isclose(marker.color.g, expected_color[1])
    assert math.isclose(marker.color.b, expected_color[2])
