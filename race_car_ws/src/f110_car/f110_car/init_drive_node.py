# SPDX-License-Identifier: MIT
# Authors: Nicklas Osmers, Kevin Losing
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class InitDrive(Node):
    """
    Node that subscribes to the "/drive" topic, collects all AckermannDriveStamped msgs and
    converts them to Twist msgs which are published to the "/cmd_vel" topic.
    """

    def __init__(self):
        super().__init__("m2p_node")  # "NodeName" will be displayed in rqt_graph
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        self.declare_parameter('speed', 0.5)  # m/s
        speed = self.get_parameter("speed").value
        self.get_logger().info(f"Starting with speed: {speed}")
        self.publisher.publish(self.create_drive_msg(speed))

    def create_drive_msg(self, speed: float) -> AckermannDriveStamped:
        t = self.get_clock().now()
        msg = AckermannDriveStamped()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = 0.0
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.speed = speed
        msg.drive.jerk = 1.0
        msg.drive.acceleration = 1.0
        return msg


def main(args=None):
    rclpy.init(args=args)

    InitDrive()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
