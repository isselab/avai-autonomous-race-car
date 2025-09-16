# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
import pytest
import math
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from unittest.mock import MagicMock

from race_car_ws.src.gazebo.gazebo_f110.gazebo_f110.world_pose_to_odom import WorldPoseToOdom


@pytest.fixture
def world_pose_to_odom_node(ros_setup):
    node = WorldPoseToOdom()
    node.get_logger = MagicMock()
    node.odom_publisher.publish = MagicMock()
    yield node
    node.destroy_node()


def create_fake_pose_array(header_stamp_sec, header_stamp_nanosec, positions, orientations, frame_id="test_frame"):
    pa = PoseArray()
    header = Header()
    header.stamp.sec = header_stamp_sec
    header.stamp.nanosec = header_stamp_nanosec
    header.frame_id = frame_id
    pa.header = header
    for pos, orient in zip(positions, orientations):
        p = Pose()
        p.position = Point(x=pos[0], y=pos[1], z=pos[2])
        p.orientation = Quaternion(x=orient[0], y=orient[1], z=orient[2], w=orient[3])
        pa.poses.append(p)
    return pa


def test_pose_array_callback_single_pose(world_pose_to_odom_node):
    pa = create_fake_pose_array(
        header_stamp_sec=100,
        header_stamp_nanosec=0,
        positions=[(1.0, 2.0, 3.0)],
        orientations=[(0.0, 0.0, 0.0, 1.0)],
        frame_id="dummy_frame"
    )

    world_pose_to_odom_node.pose_array_callback(pa)
    world_pose_to_odom_node.odom_publisher.publish.assert_called_once()
    msg = world_pose_to_odom_node.odom_publisher.publish.call_args[0][0]

    assert msg.header.stamp.sec == pa.header.stamp.sec
    assert msg.header.stamp.nanosec == pa.header.stamp.nanosec

    first_pose = pa.poses[0]
    assert math.isclose(msg.pose.pose.position.x, first_pose.position.x)
    assert math.isclose(msg.pose.pose.position.y, first_pose.position.y)
    assert math.isclose(msg.pose.pose.position.z, first_pose.position.z)
    assert math.isclose(msg.pose.pose.orientation.x, first_pose.orientation.x)
    assert math.isclose(msg.pose.pose.orientation.y, first_pose.orientation.y)
    assert math.isclose(msg.pose.pose.orientation.z, first_pose.orientation.z)
    assert math.isclose(msg.pose.pose.orientation.w, first_pose.orientation.w)


def test_pose_array_callback_multiple_poses(world_pose_to_odom_node):
    pa = create_fake_pose_array(
        header_stamp_sec=200,
        header_stamp_nanosec=100,
        positions=[(4.0, 5.0, 6.0), (7.0, 8.0, 9.0)],
        orientations=[(0.0, 0.0, 0.7071, 0.7071), (0.0, 0.0, 0.0, 1.0)],
        frame_id="dummy_frame"
    )

    world_pose_to_odom_node.pose_array_callback(pa)
    world_pose_to_odom_node.odom_publisher.publish.assert_called_once()
    msg = world_pose_to_odom_node.odom_publisher.publish.call_args[0][0]

    first_pose = pa.poses[0]
    assert math.isclose(msg.pose.pose.position.x, first_pose.position.x)
    assert math.isclose(msg.pose.pose.position.y, first_pose.position.y)
    assert math.isclose(msg.pose.pose.position.z, first_pose.position.z)
    assert math.isclose(msg.pose.pose.orientation.x, first_pose.orientation.x)
    assert math.isclose(msg.pose.pose.orientation.y, first_pose.orientation.y)
    assert math.isclose(msg.pose.pose.orientation.z, first_pose.orientation.z)
    assert math.isclose(msg.pose.pose.orientation.w, first_pose.orientation.w)
