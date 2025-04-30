from race_car_ws.src.gazebo.gazebo_f110.gazebo_f110.pose_transformer import GazeboPoseTransformer
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros


@pytest.fixture(scope='function')
def rclpy_node(ros_setup):
    node = GazeboPoseTransformer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    yield node
    node.destroy_node()


@pytest.fixture(scope='function')
def publisher(rclpy_node):
    return rclpy_node.create_publisher(PoseStamped, '/points_to_translate', 10)


@pytest.fixture(scope='function')
def subscriber(rclpy_node):
    received_msgs = []

    def callback(msg):
        received_msgs.append(msg)

    sub = rclpy_node.create_subscription(
        PoseStamped, '/target_points', callback, 10
    )

    return received_msgs


@pytest.fixture(scope='function')
def static_tf_broadcaster(rclpy_node):
    """Fixture to publish a static transform required for transformation tests."""
    broadcaster = tf2_ros.StaticTransformBroadcaster(rclpy_node)

    static_transform = TransformStamped()
    static_transform.header.stamp = rclpy_node.get_clock().now().to_msg()
    static_transform.header.frame_id = "gazebo_frame"
    static_transform.child_frame_id = "0"
    static_transform.transform.translation.x = 0.0
    static_transform.transform.translation.y = 0.0
    static_transform.transform.translation.z = 0.0
    static_transform.transform.rotation.x = -0.7071067811865475
    static_transform.transform.rotation.y = 0.7071067811865475
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 0.0

    broadcaster.sendTransform(static_transform)
    return broadcaster


@pytest.mark.parametrize("transformation_enabled", [False, True])
def test_pose_transformation(rclpy_node, publisher, subscriber, static_tf_broadcaster, transformation_enabled):
    """Test transformation behavior when enabled/disabled."""
    rclpy_node.set_parameters([
        rclpy.parameter.Parameter('transformation_enabled', rclpy.Parameter.Type.BOOL, transformation_enabled)
    ])

    input_pose = PoseStamped()
    input_pose.header.frame_id = 'gazebo_frame'
    input_pose.pose.position.x = 1.0
    input_pose.pose.position.y = 2.0
    input_pose.pose.position.z = 3.0

    publisher.publish(input_pose)

    for _ in range(10):
        rclpy.spin_once(rclpy_node, timeout_sec=0.5)

    assert len(subscriber) > 0
    output_pose = subscriber[0]

    if transformation_enabled:
        assert output_pose.pose.position.x != 1.0
    else:
        assert output_pose.pose.position.x == 1.0


def test_tf_lookup_failure(rclpy_node, publisher, subscriber):
    """Test behavior when TF lookup fails."""
    rclpy_node.tf_buffer.lookup_transform = lambda *args, **kwargs: (_ for _ in ()).throw(
        tf2_ros.LookupException("TF not found"))

    input_pose = PoseStamped()
    input_pose.header.frame_id = 'invalid_frame'
    input_pose.pose.position.x = 1.0
    input_pose.pose.position.y = 0.0
    input_pose.pose.position.z = 0.0

    publisher.publish(input_pose)

    for _ in range(5):
        rclpy.spin_once(rclpy_node, timeout_sec=0.5)

    assert len(subscriber) == 0
