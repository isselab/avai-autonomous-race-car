#!/usr/bin/env python3

from avai_lab.enums import BLUE_CONE, ORANGE_CONE, YELLOW_CONE
import rclpy
from rclpy.node import Node

from racecar_msgs.msg import DetectedConeArray, DetectedCone
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration

import tf2_ros
import tf2_geometry_msgs


class ConeMarkerNode(Node):
    """
    Subscribes to a DetectedConeArray (e.g. from YOLO),
    transforms each cone's position into the 'map' frame,
    and visualizes them as markers in RViz.
    """

    def __init__(self):
        super().__init__('cone_marker_node')

        self.declare_parameter('detected_cones_topic', '/yolo_cones')
        self.declare_parameter('marker_topic', '/cone_markers')
        self.declare_parameter('target_frame', 'base_link')

        self.detected_cones_topic = self.get_parameter('detected_cones_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.cones_sub = self.create_subscription(
            DetectedConeArray,
            self.detected_cones_topic,
            self.cones_callback,
            10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"ConeMarkerNode started. Subscribing to {self.detected_cones_topic}")

    def cones_callback(self, cone_array_msg: DetectedConeArray):
        """
        Receive a DetectedConeArray, transform each cone into `target_frame`,
        and publish as a MarkerArray for visualization.
        """
        marker_array = MarkerArray()

        # Clear old markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Process each cone
        for i, cone in enumerate(cone_array_msg.cones):
            # Transform the cone's position to the map frame
            pt_map = self.transform_cone(cone)
            if pt_map is None:
                continue

            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cones"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = pt_map.point.x
            marker.pose.position.y = pt_map.point.y
            marker.pose.position.z = pt_map.point.z

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            (r, g, b) = self.cone_color(cone.type)
            marker.color.a = 1.0
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.markers_pub.publish(marker_array)

    def transform_cone(self, cone: DetectedCone) -> PointStamped:
        """
        Transform a Cone from its original frame to a target frame using TF.
        
        :param cone: The Cone to transform
        :return: The transformed point or None if the transform failed.
        """

        try:
            stamp_time = rclpy.time.Time.from_msg(cone.header.stamp)
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                cone.header.frame_id,
                stamp_time,
                timeout=Duration(seconds=0.5)
            )
            ps = PointStamped()
            ps.header = cone.header
            ps.point = cone.position
            pt_map = tf2_geometry_msgs.do_transform_point(ps, transform_stamped)
            return pt_map
        except Exception as e:
            self.get_logger().warn(f"TF transform failed for cone {cone.type}: {e}")
            return None

    def cone_color(self, cone_type: int):
        """
        Return an (r,g,b) for each cone type.
        """
        if cone_type == YELLOW_CONE:
            return (1.0, 1.0, 0.0)  # yellow
        elif cone_type == BLUE_CONE:
            return (0.0, 0.0, 1.0)  # blue
        elif cone_type == ORANGE_CONE:
            return (1.0, 0.65, 0.0)  # orange
        else:
            return (1.0, 0.0, 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ConeMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
