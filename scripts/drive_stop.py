from time import sleep

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class DriveOnceNode(Node):
    def __init__(self):
        super().__init__("DriveOnce")
        self.publisher = Node.create_publisher(AckermannDriveStamped, "/drive", 10)
        sleep(2)
        self.publisher.publish(self._create_drive_msg(1.0))
        sleep(2)
        self.publisher.publish(self._create_drive_msg(-1.0))
        self.publisher.publish(self._create_drive_msg(0.0))

    def _create_drive_msg(self, speed: float) -> AckermannDriveStamped:
        """Create a new AckermannDriveStamped message using the parameters of self.drive_state"""
        t = self.get_clock().now()
        msg = AckermannDriveStamped()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = 0.0
        msg.drive.steering_angle_velocity = 1.0
        msg.drive.speed = speed
        msg.drive.jerk = 1.0
        msg.drive.acceleration = 1.0
        return msg


def main():
    rclpy.init()
    DriveOnceNode()
