# SPDX-License-Identifier: MIT
# Authors: Kevin Losing

import pytest
import numpy as np
from unittest.mock import MagicMock, patch
from geometry_msgs.msg import PoseWithCovarianceStamped
from racecar_msgs.msg import SemanticGrid
from race_car_ws.src.f110_car.f110_car.exploration_node import ExplorationNode


@pytest.fixture
def node(ros_setup):
    """Create an instance of the ExplorationNode."""
    node = ExplorationNode()
    node.get_logger = MagicMock()
    yield node
    node.destroy_node()


def test_pose_callback(node):
    """Test that pose_callback updates the last_pose correctly."""
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    node.pose_callback(msg)
    assert node.last_pose.pose.pose.position.x == 1.0
    assert node.last_pose.pose.pose.position.y == 2.0


@patch("f110_car.exploration_node.utils.quat_to_rot_vec", return_value=0.0)
def test_get_car_projection_vector(mock_quat_to_rot_vec, node):
    """Test that the vehicle's projection vector is computed correctly."""
    node.last_pose = PoseWithCovarianceStamped()
    node.last_pose.pose.pose.orientation.z = 0.0
    node.last_pose.pose.pose.orientation.w = 1.0
    vec = node.get_car_projection_vector()
    assert np.allclose(vec, [1.0, 0.0], atol=1e-5)


@patch.object(ExplorationNode, "get_car_projection_vector", return_value=np.array([1, 0]))
def test_get_projected_point(mock_get_vector, node):
    """Test that the projected point is calculated correctly."""
    node.last_pose = PoseWithCovarianceStamped()
    node.last_pose.pose.pose.position.x = 5.0
    node.last_pose.pose.pose.position.y = 3.0
    projected_point = node.get_projected_point()
    assert np.allclose(projected_point, [6.0, 3.0], atol=1e-5)


def test_create_target_point_msg(node):
    """Test creation of PoseStamped target message."""
    msg = node.create_target_point_msg(1.5, 2.5)
    assert msg.pose.position.x == 1.5
    assert msg.pose.position.y == 2.5
    assert msg.header.frame_id == "map"


def test_semantic_grid_callback_no_pose(node):
    """Test that semantic_grid_callback does nothing if pose is not initialized."""
    msg = SemanticGrid()
    node.semantic_grid_callback(msg)
    node.get_logger().info.assert_called_with("Pose not initialized, skipping semantic grid callback")


def test_semantic_grid_callback_no_cones(node):
    """Test semantic grid callback when no cones are detected."""
    node.last_pose = PoseWithCovarianceStamped()
    msg = SemanticGrid()
    msg.info.width = 10
    msg.info.height = 10
    msg.cells = []
    node.semantic_grid_callback(msg)
    node.get_logger().info.assert_called_with("No cones detected, skipping target point creation")
