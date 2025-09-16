# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class ExplorationVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("exploration_visualizer_node")
        self.declare_parameter("target_point_topic", "/target_point")
        self.declare_parameter("marker_topic", "/target_point_marker")
        self.declare_parameter("marker_scale", 0.2)
        self.target_point_subscriber = self.create_subscription(PoseStamped,
                                                                self.get_parameter("target_point_topic").value,
                                                                self.target_point_callback, 10)
        self.marker_publisher = self.create_publisher(Marker, self.get_parameter("marker_topic").value, 10)

    def target_point_callback(self, msg: PoseStamped):
        self.get_logger().info("Create marker for target point")
        marker = Marker()
        marker.header = msg.header
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.pose.position
        marker.pose.orientation.w = 1.0
        marker_scale = self.get_parameter("marker_scale").value
        marker.scale.x = marker_scale
        marker.scale.y = marker_scale
        marker.scale.z = marker_scale
        color = ColorRGBA()
        color.r = 1.0
        color.g = 0.0
        color.b = 0.0
        color.a = 1.0
        marker.color = color
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
