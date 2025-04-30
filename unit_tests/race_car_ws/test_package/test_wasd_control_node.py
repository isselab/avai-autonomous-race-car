import pytest
from unittest.mock import MagicMock
from pynput.keyboard import KeyCode

from race_car_ws.src.test_package.test_package.nodes.wasd_control_node import WASDControl, DriveState
from ackermann_msgs.msg import AckermannDriveStamped


@pytest.fixture
def ros_node(ros_setup):
    node = WASDControl()
    yield node
    node.destroy_node()


def test_create_drive_msg(ros_node):
    """Test if drive messages are created correctly."""
    msg = ros_node._create_drive_msg()
    assert isinstance(msg, AckermannDriveStamped)
    assert msg.drive.speed == 0.0
    assert msg.drive.steering_angle == 0.0


def test_get_steering_angle(ros_node):
    """Test different drive states and their steering angles."""
    ros_node.drive_state = DriveState(False, False, False, False)
    assert ros_node.get_steering_angle() == 0.0

    ros_node.drive_state = DriveState(False, True, False, False)
    assert ros_node.get_steering_angle() == -ros_node.max_steering_angle

    ros_node.drive_state = DriveState(False, False, True, False)
    assert ros_node.get_steering_angle() == ros_node.max_steering_angle


def test_get_drive_speed(ros_node):
    """Test different drive states and their speeds."""
    ros_node.drive_state = DriveState(False, False, False, False)
    assert ros_node.get_drive_speed() == 0.0

    ros_node.drive_state = DriveState(True, False, False, False)
    assert ros_node.get_drive_speed() == ros_node.max_speed

    ros_node.drive_state = DriveState(False, False, False, True)
    assert ros_node.get_drive_speed() == -ros_node.max_speed


def test_on_press(ros_node):
    """Test keyboard input for movement."""
    ros_node._on_press(KeyCode(char="w"))
    assert ros_node.drive_state.forward

    ros_node._on_press(KeyCode(char="a"))
    assert ros_node.drive_state.left

    ros_node._on_press(KeyCode(char="d"))
    assert ros_node.drive_state.right

    ros_node._on_press(KeyCode(char="s"))
    assert ros_node.drive_state.backwards


def test_on_release(ros_node):
    """Test keyboard release input to stop movement."""
    ros_node.drive_state = DriveState(True, True, True, True)

    ros_node._on_release(KeyCode(char="w"))
    assert not ros_node.drive_state.forward

    ros_node._on_release(KeyCode(char="a"))
    assert not ros_node.drive_state.left

    ros_node._on_release(KeyCode(char="d"))
    assert not ros_node.drive_state.right

    ros_node._on_release(KeyCode(char="s"))
    assert not ros_node.drive_state.backwards


def test_publish_control(ros_node):
    """Test that publish_control sends a message."""
    ros_node.publisher.publish = MagicMock()

    ros_node.publish_control()
    ros_node.publisher.publish.assert_called_once()

    published_msg = ros_node.publisher.publish.call_args[0][0]
    assert isinstance(published_msg, AckermannDriveStamped)
