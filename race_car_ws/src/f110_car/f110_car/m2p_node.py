# SPDX-License-Identifier: MIT
# Authors: Kevin Losing, Ahmad Al Shihabi, Julia Dawicki
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import numpy as np

from avai_lab.utils import get_direction_vec, quat_to_rot_vec, rot_from_vec

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class M2P(Node):
    """
    Node that subscribes to the "/drive" topic, collects all AckermannDriveStamped msgs and
    converts them to Twist msgs which are published to the "/cmd_vel" topic.
    """

    def __init__(self):
        super().__init__("m2p_node")  # "NodeName" will be displayed in rqt_graph
        self.odom_subscriber = self.create_subscription(PoseWithCovarianceStamped, "/pose",
                                                        self.pose_callback, 10)
        self.point_subscriber = self.create_subscription(PoseStamped, "/target_point",
                                                         self.point_callback, 100)
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.create_timer(0.1, self.timer_callback)

        self.declare_parameter('max_steering_angle', 0.5)  # radians
        self.declare_parameter('max_speed', 0.5)  # m/s
        self.declare_parameter('min_speed', 0.25)  # m/s
        self.declare_parameter('max_acceleration', 0.25)  # m/s²
        self.declare_parameter('max_steering', 0.5)  # radians/s
        self.declare_parameter('epsilon', 0.5)  # m // distance to the target where the car will stop

        self.max_steering_angle = self.get_parameter('max_steering_angle').value  # radians
        self.max_speed = self.get_parameter('max_speed').value  # m/s
        self.min_speed = self.get_parameter('min_speed').value  # m/s
        self.max_acceleration = self.get_parameter('max_acceleration').value  # m/s²
        self.max_steering = self.get_parameter('max_steering').value  # radians/s
        self.epsilon = self.get_parameter('epsilon').value
        self.target = None
        self.last_pose_msg = None

        self.add_on_set_parameters_callback(self.param_callback)

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        return math.remainder(angle, 2 * math.pi)

    def param_callback(self, params: Parameter):
        """
        Callback to handle parameter updates.
        """
        for param in params:
            if param.name == 'max_steering_angle':
                self.max_steering_angle = param.value
            elif param.name == 'max_speed':
                self.max_speed = param.value
            elif param.name == 'min_speed':
                self.min_speed = param.value
            elif param.name == 'max_acceleration':
                self.max_acceleration = param.value
            elif param.name == 'max_steering':
                self.max_steering = param.value
            elif param.name == 'epsilon':
                self.epsilon = param.value
        return SetParametersResult(successful=True)

    def point_callback(self, point_msg: PoseStamped):
        """
        Callback to handle new target points. Pushes received point to target stack.
        """
        new_point = np.array([point_msg.pose.position.x, point_msg.pose.position.y])
        self.target = new_point
        self.get_logger().info(f"Received new target point: {new_point}")

    def pose_callback(self, pose_msg: PoseWithCovarianceStamped):
        self.last_pose_msg = pose_msg
        self.get_logger().info(f"Received new position: {pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y}")

    def timer_callback(self):
        """
        Callback to process odometry updates and navigate to current target point.
        """
        if self.target is None:
            return

        if self.last_pose_msg is None:
            # If there is no information yet, creep forward slowly
            self.get_logger().info("Creepin forward...")
            self.publisher.publish(self.create_drive_msg(0.0, 0.5))
            return

        pose_msg = self.last_pose_msg
        # Extract target and vehicle state
        pos = np.array(
            [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])  # We only need to calculate in 2D
        direction_vec = get_direction_vec(pos, self.target)
        orientation_rot = quat_to_rot_vec(pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w)

        # Calculate distance and steering angle
        distance = np.linalg.norm(direction_vec)
        direction_rot = rot_from_vec(direction_vec)
        steering_angle = self.normalize_angle(direction_rot - orientation_rot)

        speed = max(float(self.min_speed), min(float(distance), float(
            self.max_speed)))  # Set the speed to the distance from the point (when we steer we should reduce that)

        # Target switching
        if distance < self.epsilon:
            msg = self.create_drive_msg(0.0, 0.0)
        else:
            msg = self.create_drive_msg(steering_angle, speed)

        self.publisher.publish(msg)

    def create_drive_msg(self, steering_angle: float, speed: float) -> AckermannDriveStamped:
        t = self.get_clock().now()
        msg = AckermannDriveStamped()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        msg.drive.steering_angle_velocity = self.max_steering
        msg.drive.speed = speed
        msg.drive.jerk = self.max_acceleration
        msg.drive.acceleration = self.max_acceleration
        return msg


def main(args=None):
    rclpy.init(args=args)

    node = M2P()
    rclpy.spin(node)  # used to loop the node

    rclpy.shutdown()


if __name__ == "__main__":
    main()
