# SPDX-License-Identifier: MIT
# Authors: Nicklas Osmers
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


class AckermannToTwist(Node):
    """Node that subscribes to the "/drive" topic, collects all AckermannDriveStamped msgs and
    converts them to Twist msgs which are published to the "/cmd_vel" topic.
    """

    def __init__(self):
        super().__init__("AckermannToTwist")  # "NodeName" will be displayed in rqt_graph
        self.drive_subscriber = self.create_subscription(AckermannDriveStamped, "/drive",
                                                         self.drive_callback, 10)
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def drive_callback(self, drive_msg: AckermannDriveStamped):
        msg = Twist()
        msg.linear.x = drive_msg.drive.speed
        msg.angular.z = drive_msg.drive.steering_angle
        self.twist_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = AckermannToTwist()
    rclpy.spin(node)  # used to loop the node

    rclpy.shutdown()


if __name__ == "__main__":
    main()
