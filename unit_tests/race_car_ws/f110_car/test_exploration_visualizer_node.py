# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
import math
import pytest
from unittest.mock import MagicMock
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from visualization_msgs.msg import Marker

from race_car_ws.src.f110_car.f110_car.exploration_visualizer_node import ExplorationVisualizerNode


@pytest.fixture
def node(ros_setup):
    node = ExplorationVisualizerNode()
    node.marker_publisher.publish = MagicMock()
    yield node
    node.destroy_node()


def create_fake_pose_stamped(frame_id="map", stamp_sec=100, stamp_nanosec=0, pos=(1.0, 2.0, 3.0)):
    msg = PoseStamped()
    header = Header()
    header.frame_id = frame_id
    header.stamp.sec = stamp_sec
    header.stamp.nanosec = stamp_nanosec
    msg.header = header
    pose = Pose()
    pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    msg.pose = pose
    return msg


def test_target_point_callback_creates_correct_marker(node):
    """
    Test that when a PoseStamped message is received by the target_point_callback,
    the node publishes a Marker with:
      - Header matching the input header.
      - Namespace "target_point" and id 0.
      - Type Marker.SPHERE and action Marker.ADD.
      - Pose with the same position as the input message and orientation with w=1.0.
      - Scale set to the marker_scale parameter.
      - Color set to red (r=1.0, g=0.0, b=0.0, a=1.0).
    """
    # Create a fake PoseStamped message.
    fake_msg = create_fake_pose_stamped(frame_id="map", stamp_sec=123, stamp_nanosec=456, pos=(5.0, 10.0, 0.0))
    # Call the callback.
    node.target_point_callback(fake_msg)

    # Get the mock call and the published message.
    mock_publish = node.marker_publisher.publish
    mock_publish.assert_called_once()
    marker = mock_publish.call_args[0][0]  # Get the first argument of the first call

    # Check header.
    assert marker.header.frame_id == fake_msg.header.frame_id
    assert marker.header.stamp.sec == fake_msg.header.stamp.sec
    assert marker.header.stamp.nanosec == fake_msg.header.stamp.nanosec

    # Check namespace and id.
    assert marker.ns == "target_point"
    assert marker.id == 0

    # Check type and action.
    assert marker.type == Marker.SPHERE
    assert marker.action == Marker.ADD

    # Check pose: position should match the input pose.
    assert math.isclose(marker.pose.position.x, fake_msg.pose.position.x)
    assert math.isclose(marker.pose.position.y, fake_msg.pose.position.y)
    assert math.isclose(marker.pose.position.z, fake_msg.pose.position.z)
    # Check orientation
    assert math.isclose(marker.pose.orientation.x, 0.0)
    assert math.isclose(marker.pose.orientation.y, 0.0)
    assert math.isclose(marker.pose.orientation.z, 0.0)
    assert math.isclose(marker.pose.orientation.w, 1.0)

    # Check scale: should equal the marker_scale parameter
    marker_scale = node.get_parameter("marker_scale").value
    assert math.isclose(marker.scale.x, marker_scale)
    assert math.isclose(marker.scale.y, marker_scale)
    assert math.isclose(marker.scale.z, marker_scale)

    # Check color: should be red.
    assert math.isclose(marker.color.r, 1.0)
    assert math.isclose(marker.color.g, 0.0)
    assert math.isclose(marker.color.b, 0.0)
    assert math.isclose(marker.color.a, 1.0)
