# SPDX-License-Identifier: MIT
# Authors: Kevin Losing

import math
import pytest
from unittest.mock import MagicMock, patch
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped

from race_car_ws.src.f110_car.f110_car.init_drive_node import InitDrive


@pytest.fixture
def init_drive_node(ros_setup):
    node = InitDrive()
    node.get_logger = MagicMock()
    yield node
    node.destroy_node()


def is_valid_drive_msg(msg: AckermannDriveStamped, expected_speed: float):
    """Helper to check that the drive message fields are as expected."""
    assert msg.header.frame_id == "base_link"
    assert math.isclose(msg.drive.steering_angle, 0.0)
    assert math.isclose(msg.drive.steering_angle_velocity, 0.0)
    assert math.isclose(msg.drive.speed, expected_speed)
    assert math.isclose(msg.drive.jerk, 1.0)
    assert math.isclose(msg.drive.acceleration, 1.0)


def test_create_drive_msg_returns_correct_msg(init_drive_node):
    """
    Test that create_drive_msg() returns an AckermannDriveStamped message with the correct fields
    when given a specific speed.
    """
    test_speed = 1.2
    drive_msg = init_drive_node.create_drive_msg(test_speed)
    assert drive_msg.header.stamp.sec > 0
    is_valid_drive_msg(drive_msg, test_speed)


@patch('race_car_ws.src.f110_car.f110_car.init_drive_node.InitDrive.create_drive_msg')
def test_node_initialization_publishes_drive_msg(mock_create_drive_msg, init_drive_node):
    """
    Test that upon initialization the node publishes a drive message with speed equal to the parameter value
    """
    expected_speed = init_drive_node.get_parameter("speed").value

    # Set up the mock to return a valid drive message when called.
    mock_msg = AckermannDriveStamped()
    mock_msg.header.frame_id = "base_link"
    mock_msg.drive.speed = expected_speed
    mock_msg.drive.steering_angle = 0.0
    mock_msg.drive.steering_angle_velocity = 0.0
    mock_msg.drive.jerk = 1.0
    mock_msg.drive.acceleration = 1.0
    mock_create_drive_msg.return_value = mock_msg

    node = InitDrive()
    node.get_logger = MagicMock()

    mock_create_drive_msg.assert_called_once_with(expected_speed)

    is_valid_drive_msg(mock_msg, expected_speed)
    node.destroy_node()


def test_parameter_override_affects_drive_msg(init_drive_node):
    """
    Test that if the 'speed' parameter is overridden before node creation,
    the published drive message uses the new value.
    """
    custom_speed = 2.0

    init_drive_node.set_parameters([Parameter("speed", value=custom_speed)])

    published_msg = init_drive_node.create_drive_msg(custom_speed)

    assert published_msg is not None
    assert isinstance(published_msg, AckermannDriveStamped)
    is_valid_drive_msg(published_msg, custom_speed)
