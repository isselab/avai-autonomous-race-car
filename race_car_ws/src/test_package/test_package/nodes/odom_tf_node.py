# SPDX-License-Identifier: MIT
# Authors: Kevin Losing
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfPublisher(Node):
    """
    A ROS2 node that subscribes to odometry messages and broadcasts the corresponding transform
    between the odom and base_link frame.
    """

    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        ts = TransformStamped()
        ts.header = msg.header
        ts.header.frame_id = 'odom'
        ts.child_frame_id = 'base_link'

        ts.transform.translation.x = msg.pose.pose.position.x
        ts.transform.translation.y = msg.pose.pose.position.y
        ts.transform.translation.z = msg.pose.pose.position.z
        ts.transform.rotation = msg.pose.pose.orientation
        ts.header.stamp = msg.header.stamp

        self.tf_broadcaster.sendTransform(ts)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
