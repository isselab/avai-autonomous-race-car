import math
import pytest
import numpy as np
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from unittest.mock import MagicMock

from race_car_ws.src.f110_car.f110_car.m2p_node import M2P

from avai_lab import utils


@pytest.fixture
def m2p_node(monkeypatch):
    node = M2P()
    node.publisher = MagicMock()
    node._published_msg = None
    node.publisher.publish.side_effect = lambda msg: setattr(node, "_published_msg", msg)

    fake_pose = PoseWithCovarianceStamped()
    fake_pose.header.stamp.sec = 100
    fake_pose.header.stamp.nanosec = 0
    fake_pose.header.frame_id = "map"
    fake_pose.pose.pose.position = Point(x=10.0, y=20.0, z=0.0)
    fake_pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    node.last_pose_msg = fake_pose

    monkeypatch.setattr(utils, "get_direction_vec", lambda pos, target: target - pos)
    monkeypatch.setattr(utils, "quat_to_rot_vec", lambda z, w: 0.0)
    monkeypatch.setattr(utils, "rot_from_vec", lambda vec: math.atan2(vec[1], vec[0]))
    node.get_logger = MagicMock()
    yield node
    node.destroy_node()


def test_normalize_angle(m2p_node):
    """Test normalization of angles."""
    assert math.isclose(m2p_node.normalize_angle(0.0), 0.0, abs_tol=1e-5)
    assert math.isclose(m2p_node.normalize_angle(3 * math.pi), -math.pi)
    assert math.isclose(m2p_node.normalize_angle(-3 * math.pi), math.pi)
    assert math.isclose(m2p_node.normalize_angle(2 * math.pi), 0.0)


def test_param_callback_updates(m2p_node):
    params = [
        Parameter(name="max_steering_angle", value=0.8),
        Parameter(name="max_speed", value=1.2),
        Parameter(name="min_speed", value=0.3),
        Parameter(name="max_acceleration", value=0.6),
        Parameter(name="max_steering", value=0.9),
        Parameter(name="epsilon", value=0.4),
    ]
    result = m2p_node.param_callback(params)
    assert result.successful
    assert m2p_node.max_steering_angle == 0.8
    assert m2p_node.max_speed == 1.2
    assert m2p_node.min_speed == 0.3
    assert m2p_node.max_acceleration == 0.6
    assert m2p_node.max_steering == 0.9
    assert m2p_node.epsilon == 0.4


def test_point_callback_sets_target(m2p_node):
    """Test that the point_callback sets the target correctly."""
    msg = PoseStamped()
    msg.pose.position = Point(x=15.0, y=25.0, z=0.0)
    m2p_node.point_callback(msg)
    np.testing.assert_allclose(m2p_node.target, np.array([15.0, 25.0]))


def test_pose_callback_sets_last_pose(m2p_node):
    """Test that the pose_callback sets the last_pose_msg."""
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position = Point(x=12.0, y=22.0, z=0.0)
    msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.1, w=1.0)
    m2p_node.pose_callback(msg)
    assert m2p_node.last_pose_msg is msg


def test_create_drive_msg_returns_valid_message(m2p_node):
    """Test that create_drive_msg creates a valid AckermannDriveStamped message."""
    test_speed = 0.7
    steering_angle_input = 0.3
    drive_msg = m2p_node.create_drive_msg(steering_angle_input, test_speed)

    assert drive_msg.header.frame_id == "base_link"
    # Steering angle should be clipped to [-max_steering_angle, max_steering_angle].
    clipped_angle = np.clip(steering_angle_input, -m2p_node.max_steering_angle, m2p_node.max_steering_angle)
    assert math.isclose(drive_msg.drive.steering_angle, clipped_angle)
    assert math.isclose(drive_msg.drive.speed, test_speed)
    assert math.isclose(drive_msg.drive.jerk, m2p_node.max_acceleration)
    assert math.isclose(drive_msg.drive.acceleration, m2p_node.max_acceleration)
    assert math.isclose(drive_msg.drive.steering_angle_velocity, m2p_node.max_steering)


def test_timer_callback_no_target(m2p_node):
    """Test that the timer callback does nothing if no target is set."""
    m2p_node.target = None
    m2p_node.timer_callback()
    # Should not publish any message.
    m2p_node.publisher.publish.assert_not_called()


def test_timer_callback_no_pose(m2p_node):
    """Test timer callback when no pose is available."""
    # Set a target, but clear last_pose_msg.
    m2p_node.target = np.array([15.0, 25.0])
    m2p_node.last_pose_msg = None
    m2p_node.timer_callback()
    # Should publish a drive message with steering=0.0 and speed=0.5 (creep forward)
    m2p_node.publisher.publish.assert_called_once()
    published = m2p_node._published_msg
    assert published is not None
    assert math.isclose(published.drive.steering_angle, 0.0)
    assert math.isclose(published.drive.speed, 0.5)


def test_timer_callback_target_reached(m2p_node):
    """
    Test that when the distance to the target is less than epsilon,
    the timer callback publishes a drive message to stop (speed 0, steering 0).
    """
    # Set target very close to the vehicle.
    m2p_node.target = np.array([10.05, 20.0])
    # last_pose_msg remains as set in fixture (vehicle at (10,20))
    m2p_node.timer_callback()
    m2p_node.publisher.publish.assert_called_once()
    published = m2p_node._published_msg

    assert published is not None
    assert math.isclose(published.drive.speed, 0.0, abs_tol=1e-5)
    assert math.isclose(published.drive.steering_angle, 0.0, abs_tol=1e-5)


def test_timer_callback_with_target_and_pose(m2p_node):
    """
    Test the timer_callback when both target and last_pose_msg are available and the target is far.
    """
    m2p_node.target = np.array([10, 30])
    m2p_node.timer_callback()
    m2p_node.publisher.publish.assert_called_once()
    published = m2p_node._published_msg
    assert published is not None
    assert math.isclose(published.drive.steering_angle, m2p_node.max_steering_angle, rel_tol=1e-5)
    assert math.isclose(published.drive.speed, m2p_node.max_speed, rel_tol=1e-5)
