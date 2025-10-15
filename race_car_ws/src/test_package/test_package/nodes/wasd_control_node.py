# SPDX-License-Identifier: MIT
# Authors: Nicklas Osmers
#!/usr/bin/env python3
from dataclasses import dataclass

from pynput.keyboard import Key, Listener, KeyCode
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

from avai_lab.config import load_config


## Driving command for a car-like vehicle using Ackermann steering.
#  $Id$

# Assumes Ackermann front-wheel steering. The left and right front
# wheels are generally at different angles. To simplify, the commanded
# angle corresponds to the yaw of a virtual wheel located at the
# center of the front axle, like on a tricycle.  Positive yaw is to
# the left. (This is *not* the angle of the steering wheel inside the
# passenger compartment.)
#
# Zero steering angle velocity means change the steering angle as
# quickly as possible. Positive velocity indicates a desired absolute
# rate of change either left or right. The controller tries not to
# exceed this limit in either direction, but sometimes it might.
#
# float32 steering_angle          # desired virtual angle (radians)
# float32 steering_angle_velocity # desired rate of change (radians/s)

# Drive at requested speed, acceleration and jerk (the 1st, 2nd and
# 3rd derivatives of position). All are measured at the vehicle's
# center of rotation, typically the center of the rear axle. The
# controller tries not to exceed these limits in either direction, but
# sometimes it might.
#
# Speed is the desired scalar magnitude of the velocity vector.
# Direction is forward unless the sign is negative, indicating reverse.
#
# Zero acceleration means change speed as quickly as
# possible. Positive acceleration indicates a desired absolute
# magnitude; that includes deceleration.
#
# Zero jerk means change acceleration as quickly as possible. Positive
# jerk indicates a desired absolute rate of acceleration change in
# either direction (increasing or decreasing).
#
# float32 speed                   # desired forward speed (m/s)
# float32 acceleration            # desired acceleration (m/s^2)
# float32 jerk                    # desired jerk (m/s^3)

@dataclass
class DriveState:
    """Dataclass representation for the input vector"""
    forward: bool
    right: bool
    left: bool
    backwards: bool


class WASDControl(Node):
    """This node can be used to send AckermannDriveStamped messages in a given interval to the /drive topic.
    Actions:
        W - Move Forward
        A - Move Left
        D - Move Right
        S - Move Backwars

    """

    def __init__(self):
        super().__init__("WASDControl")  # "NodeName" will be displayed in rqt_graph
        self.timer = self.create_timer(0.5, self.publish_control)
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.get_logger().info("WASD control node started")
        self.get_logger().info("Press any other key to interrupt")
        self.drive_state = DriveState(False, False, False, False)
        self.config = load_config()
        self.get_logger().info(f"Used config:\n{str(self.config)}")
        self.max_steering_angle = self.config.car_platform.max_steering_angle  # radians
        self.max_speed = self.config.car_platform.max_speed  # m/s
        self.max_acceleration = self.config.car_platform.max_acceleration  # m/s
        self.max_steering = self.config.car_platform.max_steering_angle  # radians/s
        self.msg_id = 0
        self.current_msg = self._create_drive_msg()
        self.listener = Listener(
            on_press=self._on_press,
            on_release=self._on_release)
        self.listener.start()
        self._stop_node = False  # Can be used from a thread to stop the spinning of the node

    def _create_drive_msg(self) -> AckermannDriveStamped:
        """Create a new AckermannDriveStamped message using the parameters of self.drive_state"""
        t = self.get_clock().now()
        msg = AckermannDriveStamped()
        msg.header.stamp = t.to_msg()
        # msg.header.seq = self.msg_id
        msg.header.frame_id = "0"
        msg.drive.steering_angle = self.get_steering_angle()
        msg.drive.steering_angle_velocity = self.max_steering
        msg.drive.speed = self.get_drive_speed()
        msg.drive.jerk = self.max_acceleration
        msg.drive.acceleration = self.max_acceleration
        # msg.jerk = self.max_steering
        return msg

    def get_steering_angle(self) -> float:
        """Return the correct steering angle based on the configured max values and the drive state"""
        match self.drive_state:
            case DriveState(_, False, False, _):
                return 0.0
            case DriveState(_, True, False, _):
                return -self.max_steering_angle
            case DriveState(_, False, True, _):
                return self.max_steering_angle
        return 0.0

    def get_drive_speed(self) -> float:
        """Return the correct speed based on the configured max values and the drive state"""
        match self.drive_state:
            case DriveState(False, _, _, False):
                return 0.0
            case DriveState(True, _, _, False):
                return self.max_speed
            case DriveState(False, _, _, True):
                return -self.max_speed
        return 0.0

    def _on_press(self, key: Key | KeyCode | None):
        """callback function for the keyboard listener"""
        match key:
            case KeyCode(char="w"):
                self.drive_state.forward = True
            case KeyCode(char="d"):
                self.drive_state.right = True
            case KeyCode(char="a"):
                self.drive_state.left = True
            case KeyCode(char="s"):
                self.drive_state.backwards = True
            case _:
                pass
                # self.listener.stop()
                # self._stop_node = True
        return True

    def _on_release(self, key: Key | KeyCode | None):
        """callback function for the keyboard listener"""
        match key:
            case KeyCode(char="w"):
                self.drive_state.forward = False
            case KeyCode(char="d"):
                self.drive_state.right = False
            case KeyCode(char="a"):
                self.drive_state.left = False
            case KeyCode(char="s"):
                self.drive_state.backwards = False
            case _:
                pass
                # self.listener.stop()
                # self._stop_node = True
        return True

    def publish_control(self):
        """Callback function for the timer to publish new drive messages"""
        self.current_msg = self._create_drive_msg()
        if self._stop_node:
            raise KeyboardInterrupt("Program terminated by user")

        self.publisher.publish(self.current_msg)
        self.msg_id += 1


def main(args=None):
    rclpy.init(args=args)

    node = WASDControl()
    try:
        rclpy.spin(node)  # used to loop the node
    except KeyboardInterrupt as e:
        print(str(e))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
