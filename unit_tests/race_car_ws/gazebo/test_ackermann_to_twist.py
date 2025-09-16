# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
import pytest
import math
from unittest.mock import MagicMock, patch
from ackermann_msgs.msg import AckermannDriveStamped

from race_car_ws.src.gazebo.gazebo_f110.gazebo_f110.ackermann_to_twist import AckermannToTwist


@pytest.fixture
def ackermann_to_twist_node(ros_setup):
    node = AckermannToTwist()
    node.get_logger = MagicMock()
    yield node
    node.destroy_node()


def create_fake_ackermann_msg(speed, steering_angle, frame_id="drive_frame", stamp_sec=123, stamp_nanosec=456):
    msg = AckermannDriveStamped()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nanosec
    msg.header.frame_id = frame_id
    msg.drive.speed = speed
    msg.drive.steering_angle = steering_angle
    return msg


def test_drive_callback_publishes_correct_twist(ackermann_to_twist_node):
    # Create a fake message.
    test_speed = 3.5
    test_steering = -0.2
    ack_msg = create_fake_ackermann_msg(test_speed, test_steering)

    with patch.object(ackermann_to_twist_node.twist_publisher, 'publish') as mock_publish:
        # Call the drive callback.
        ackermann_to_twist_node.drive_callback(ack_msg)

        # Assert that publish was called.
        mock_publish.assert_called_once()

        # Get the published Twist message from the mock call.
        published_twist = mock_publish.call_args[0][0]

    assert math.isclose(published_twist.linear.x, test_speed)
    assert math.isclose(published_twist.angular.z, test_steering)
    assert math.isclose(published_twist.linear.y, 0.0)
    assert math.isclose(published_twist.linear.z, 0.0)
    assert math.isclose(published_twist.angular.x, 0.0)
    assert math.isclose(published_twist.angular.y, 0.0)


def test_drive_callback_with_zero_values(ackermann_to_twist_node):
    ack_msg = create_fake_ackermann_msg(0.0, 0.0)

    with patch.object(ackermann_to_twist_node.twist_publisher, 'publish') as mock_publish:
        # Call the drive callback.
        ackermann_to_twist_node.drive_callback(ack_msg)
        # Assert that publish was called.
        mock_publish.assert_called_once()
        # Get the published message
        published_twist = mock_publish.call_args[0][0]

    assert math.isclose(published_twist.linear.x, 0.0)
    assert math.isclose(published_twist.angular.z, 0.0)
    assert math.isclose(published_twist.linear.y, 0.0)
    assert math.isclose(published_twist.linear.z, 0.0)
    assert math.isclose(published_twist.angular.x, 0.0)
    assert math.isclose(published_twist.angular.y, 0.0)
