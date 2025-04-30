import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.duration import Duration
import rclpy.time

import tf2_ros
import tf2_geometry_msgs


class GazeboPoseTransformer(Node):
    """This node conditionally transforms target points from Gazebo coordinates
       to ROS coordinates based on a boolean parameter 'transformation_enabled'."""

    def __init__(self):
        super().__init__("gazebo_pose_transformer")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Use ros2 param set /gazebo_pose_transformer transformation_enabled false to toggle it.
        self.declare_parameter("transformation_enabled", True)
        self.transformation_enabled = self.get_parameter("transformation_enabled").value
        self.add_on_set_parameters_callback(self.on_param_change)

        self.sub_pose = self.create_subscription(
            PoseStamped,
            "/points_to_translate",
            self.pose_callback_transformer,
            10
        )
        self.pub_pose = self.create_publisher(
            PoseStamped,
            "/target_points",
            10
        )

    def on_param_change(self, params):
        """Parameter callback to update transformation_enabled at runtime."""
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'transformation_enabled':
                self.transformation_enabled = param.value
                self.get_logger().info(f"Set 'transformation_enabled' to {param.value}")
        return SetParametersResult(successful=True)

    def pose_callback_transformer(self, msg_in: PoseStamped):
        """Callback to transform target points if enabled, otherwise pass them through."""
        self.get_logger().info("pose_callback_transformer triggered.")

        if not self.transformation_enabled:
            self.get_logger().info("Transformation is disabled -> publishing unmodified pose.")
            self.pub_pose.publish(msg_in)
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                '0',  # target_frame
                msg_in.header.frame_id,  # source_frame
                now,
                timeout=Duration(seconds=1.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return

        point_in = PointStamped()
        point_in.header.stamp = now.to_msg()
        point_in.header.frame_id = msg_in.header.frame_id
        point_in.point.x = msg_in.pose.position.x
        point_in.point.y = msg_in.pose.position.y
        point_in.point.z = msg_in.pose.position.z
        self.get_logger().info(f"Gazebo Pose: {point_in.point}")

        transformed_point = tf2_geometry_msgs.do_transform_point(point_in, trans)
        self.get_logger().info(f"ROS Pose: {transformed_point.point}")

        transformed_pose_stamped = PoseStamped()
        transformed_pose_stamped.header = transformed_point.header
        transformed_pose_stamped.pose.position.x = transformed_point.point.x
        transformed_pose_stamped.pose.position.y = transformed_point.point.y
        transformed_pose_stamped.pose.position.z = transformed_point.point.z

        self.pub_pose.publish(transformed_pose_stamped)
        self.get_logger().info("Published transformed pose on /transformed_points.")


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
