# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
#!/usr/bin/env python3
from typing import List
from enum import Enum
from collections import deque
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

import numpy as np
import numpy.typing as npt

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from racecar_msgs.msg import SemanticGrid


class State(Enum):
    START = 0
    LEFT_START = 1
    FINISHED_LAP = 2
    GLOBAL_PLAN = 3


class GlobalPlanningNode(Node):
    """
    Uses the immediate surroundings as orientation points to calculate target points with a short distance
    to the vehicle that lie inside of a track.
    The node subscribes to a semantic grid which holds the position of cones on a map alongside their label.
    The algorithm to calculate a new target point creates a projection point in front of the vehicle (in driving direction)
    and calculates the distances to all cones on the semantic grid. It then uses the closest blue cone to the left
    and the closest yellow cone to the right to calculate the gravitational center between the two and use that as
    a new target point.
    
    projection_point_distance: Distance from the projection point to the vehicle in meters
    """

    def __init__(self):
        super().__init__("global_planning_node")  # "NodeName" will be displayed in rqt_graph
        self.declare_parameter("map_pose_topic", "/pose")
        self.declare_parameter("semantic_grid_topic", "/semantic_map")
        self.declare_parameter("target_point_topic", "/target_point")
        self.declare_parameter("start_point_epsilon", 1)
        self.declare_parameter("point_epsilon", 1)
        self.declare_parameter("planning_speed", 1.0)
        self.declare_parameter("path_topic", "/path")

        self.map_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped,
                                                            self.get_parameter("map_pose_topic").value,
                                                            self.pose_callback, 10)
        self.semantic_grid_subscriber = self.create_subscription(SemanticGrid,
                                                                 self.get_parameter("semantic_grid_topic").value,
                                                                 self.semantic_grid_callback, 10)
        self.target_point_subscriber = self.create_subscription(PoseStamped,
                                                                self.get_parameter("target_point_topic").value,
                                                                self.target_point_callback, 10)
        self.target_point_publisher = self.create_publisher(PoseStamped, self.get_parameter("target_point_topic").value,
                                                            10)
        self.path_publisher = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.start_point_epsilon = self.get_parameter("start_point_epsilon").value
        self.next_point_threshold = self.get_parameter("point_epsilon").value
        self.path = []
        self.left_starting_area = False
        self.start_position = None
        self.state = State.START

        self.exploration_node_cli = self.create_client(SetParameters, 'f110/exploration_node/set_parameters')
        self.m2p_cli = self.create_client(SetParameters, 'f110/move_to_point/set_parameters')
        self.get_logger().info("Waiting for Exploration node service...")
        while not self.exploration_node_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info("Waiting for Move to Point node service...")
        while not self.m2p_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info("Initialized")

    def deactivate_exploration(self):
        req = SetParameters.Request()
        param = Parameter()
        param.name = "active"
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value = ParameterValue(bool_value=False, type=ParameterType.PARAMETER_BOOL)
        req.parameters.append(param)

        future = self.exploration_node_cli.call_async(req)

    def increase_speed_of_vehicle(self, speed: float):
        self.get_logger().info(f"Set move to point node max speed to {speed}")
        req = SetParameters.Request()
        param = Parameter()
        param.name = "max_speed"
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value = ParameterValue(double_value=speed, type=ParameterType.PARAMETER_DOUBLE)
        req.parameters.append(param)

        future = self.m2p_cli.call_async(req)

    def publish_path(self):
        msg = Path()
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "map"
        msg.poses = self.path
        self.path_publisher.publish(msg)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if self.start_position is None:
            self.start_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        vehicle_location = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        distance_to_start = np.linalg.norm((self.start_position, vehicle_location))
        match self.state:
            case State.START:
                if distance_to_start > self.start_point_epsilon:
                    self.state = State.LEFT_START
                self.path.append(self.create_pose_stamped_from_pose_with_covariance(msg))
            case State.LEFT_START:
                self.path.append(self.create_pose_stamped_from_pose_with_covariance(msg))
                if distance_to_start < self.start_point_epsilon:
                    self.state = State.FINISHED_LAP
                    start_target_point_msg = self.create_target_point_msg(*self.start_position)
                    self.target_point_publisher.publish(start_target_point_msg)
                    self.get_logger().info("Back at start")
            case State.FINISHED_LAP:
                # calculate optimal path?
                self.optimal_path = deque(deepcopy(self.path[::10]))
                self.state = State.GLOBAL_PLAN
                self.get_logger().info("Deactivating Exploration node")
                self.deactivate_exploration()
                p = self.get_next_point(vehicle_location)
                target_msg = self.create_target_point_msg(*p)
                self.target_point_publisher.publish(target_msg)
                self.increase_speed_of_vehicle(self.get_parameter("planning_speed").value)
            case State.GLOBAL_PLAN:
                p = self.get_next_point(vehicle_location)
                target_msg = self.create_target_point_msg(*p)
                self.target_point_publisher.publish(target_msg)
        self.publish_path()

    def get_next_point(self, vehicle_location: npt.NDArray) -> npt.NDArray:
        while True:
            current_point = self.optimal_path[0]
            p = np.array([current_point.pose.position.x, current_point.pose.position.y])
            distance = np.linalg.norm(p - vehicle_location)
            if distance < self.next_point_threshold:
                self.optimal_path.rotate(-1)
            else:
                break
        return p

    def target_point_callback(self, msg: PoseStamped):
        pass

    def create_pose_stamped_from_pose_with_covariance(self, cov_msg: PoseWithCovarianceStamped) -> PoseStamped:
        msg = PoseStamped()
        msg.header = cov_msg.header
        msg.pose = cov_msg.pose.pose
        msg.pose = cov_msg.pose.pose
        return msg

    def create_target_point_msg(self, x: float, y: float) -> PoseStamped:
        msg = PoseStamped()
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        return msg

    def semantic_grid_callback(self, msg: SemanticGrid):
        """Convert the grid into a numpy array and calculate a new target point
           currently bypassed
        """
        return


def main(args=None):
    rclpy.init(args=args)

    node = GlobalPlanningNode()
    rclpy.spin(node)  # used to loop the node

    rclpy.shutdown()


if __name__ == "__main__":
    main()
