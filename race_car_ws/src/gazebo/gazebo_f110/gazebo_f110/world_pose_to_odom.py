#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

CAR_ID = 4


class WorldPoseToOdom(Node):
    """Node that subscribes to the "/drive" topic, collects all AckermannDriveStamped msgs and
    converts them to Twist msgs which are published to the "/cmd_vel" topic.
    """

    def __init__(self):
        super().__init__("WorldPoseToOdom")  # "NodeName" will be displayed in rqt_graph
        self.drive_subscriber = self.create_subscription(PoseArray, "/world/car_world/pose/info",
                                                         self.pose_array_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)

    def pose_array_callback(self, pose_arr: PoseArray):
        pose = pose_arr.poses[0]

        msg = Odometry()
        msg.header.stamp = pose_arr.header.stamp
        msg.pose.pose.position.x = pose.position.x
        msg.pose.pose.position.y = pose.position.y
        msg.pose.pose.position.z = pose.position.z
        msg.pose.pose.orientation.x = pose.orientation.x
        msg.pose.pose.orientation.y = pose.orientation.y
        msg.pose.pose.orientation.z = pose.orientation.z
        msg.pose.pose.orientation.w = pose.orientation.w
        self.odom_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = WorldPoseToOdom()
    rclpy.spin(node)  # used to loop the node

    rclpy.shutdown()


if __name__ == "__main__":
    main()
