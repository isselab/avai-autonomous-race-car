# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from racecar_msgs.msg import SemanticGrid, SemanticCell
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from scipy.ndimage import label
from std_msgs.msg import ColorRGBA

# Import the cone label enums.
from avai_lab import enums


class SemanticGridVisualizerNode(Node):
    """
    This node visualizes labeled clusters from a semantic grid.
    For each cone cluster a marker is placed at the cluster's position.
    """

    def __init__(self):
        super().__init__('semantic_grid_visualizer_node')

        self.declare_parameter('semantic_grid_topic', '/semantic_map')
        self.declare_parameter('marker_topic', '/semantic_grid_cluster_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('cluster_marker_scale', 0.2)  # marker size

        self.semantic_grid_topic = self.get_parameter('semantic_grid_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.cluster_marker_scale = self.get_parameter('cluster_marker_scale').value

        self.create_subscription(SemanticGrid, self.semantic_grid_topic, self.semantic_grid_callback, 1)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 1)

        self.get_logger().info("SemanticGridVisualizerNode started.")

    def semantic_grid_callback(self, semantic_grid_msg: SemanticGrid):
        """
        Processes the semantic grid, computes clusters for each cone label,
        and publishes markers at the cluster centroids.
        
        :param semantic_grid_msg: The semantic occupancy grid.
        :return: None
        """
        marker_array = MarkerArray()
        header = semantic_grid_msg.header
        header.frame_id = self.frame_id

        # Retrieve grid information
        width = semantic_grid_msg.info.width
        height = semantic_grid_msg.info.height
        resolution = semantic_grid_msg.info.resolution
        origin_x = semantic_grid_msg.info.origin.position.x
        origin_y = semantic_grid_msg.info.origin.position.y

        # Convert the list of SemanticCell messages into a 2D numpy array of labels.
        labels = np.empty((height, width), dtype=int)
        for i, cell in enumerate(semantic_grid_msg.cells):
            row = i // width
            col = i % width
            labels[row, col] = cell.label

        # Only consider cells with cone labels
        valid_labels = [
            enums.YELLOW_CONE,
            enums.BLUE_CONE,
            enums.ORANGE_CONE,
            enums.UNKNOWN_CONE
        ]

        marker_id = 0
        for cone_label in valid_labels:
            # Create a binary mask for this label.
            mask = (labels == cone_label)
            # Find connected clusters in the binary mask.
            labeled_mask, num_clusters = label(mask)

            for cluster in range(1, num_clusters + 1):
                # Get indices (rows, cols) for cells belonging to this cluster.
                indices = np.where(labeled_mask == cluster)
                if len(indices[0]) == 0:
                    continue
                # Compute the centroid in grid coordinates.
                centroid_row = np.mean(indices[0])
                centroid_col = np.mean(indices[1])
                # Convert grid coordinates to world coordinates.
                centroid = Point()
                centroid.x = origin_x + (centroid_col + 0.5) * resolution
                centroid.y = origin_y + (centroid_row + 0.5) * resolution
                centroid.z = 0.0

                # Create a marker for this cluster.
                marker = Marker()
                marker.header = header
                marker.ns = "semantic_grid_clusters"
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = centroid
                marker.pose.orientation.w = 1.0
                marker.scale.x = self.cluster_marker_scale
                marker.scale.y = self.cluster_marker_scale
                marker.scale.z = self.cluster_marker_scale
                marker.color = self.get_color(cone_label)

                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_pub.publish(marker_array)

    def get_color(self, label_value: int):
        """
        Returns a color based on the label value.
        
        :param label_value: The label from which one wants to get the color from.
        :return: None
        """
        color = ColorRGBA()
        color.a = 1.0
        if label_value == enums.YELLOW_CONE:
            color.r = 1.0
            color.g = 1.0
            color.b = 0.0
        elif label_value == enums.BLUE_CONE:
            color.r = 0.0
            color.g = 0.0
            color.b = 1.0
        elif label_value == enums.ORANGE_CONE:
            color.r = 1.0
            color.g = 0.5
            color.b = 0.0
        elif label_value == enums.UNKNOWN_CONE:
            color.r = 0.5
            color.g = 0.5
            color.b = 0.5
        else:
            color.r = 0.0
            color.g = 0.0
            color.b = 0.0
            color.a = 0.0
        return color


def main(args=None):
    rclpy.init(args=args)
    node = SemanticGridVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
