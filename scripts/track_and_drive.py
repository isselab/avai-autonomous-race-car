"""
This scripts spawns a track into gazebo
and publishes the cone positions to the 
move to point node
"""
# SPDX-License-Identifier: MIT
# Authors: Nicklas Osmers

from pathlib import Path
import os
from time import sleep

import numpy as np

from avai_lab.track_gen import generate_track, resample_polygon
from avai_lab.gazebo.msg import EntityFactory, EntityFactory_V, Pose, Vector3d, WorldControl
from avai_lab.gazebo.service import set_pose, spawn_entities, world_control

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

WORLD = "car_world"
CAR_ID = 4  # important to set the car position
cone_model = Path(os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")) / "cone.sdf"
assert cone_model.exists(), "Cone model does not exist"


class TrackPlanner(Node):
    def __init__(self, points: np.ndarray):
        super().__init__("TrackPlanner")  # "NodeName" will be displayed in rqt_graph
        self.publisher = self.create_publisher(PoseStamped, "/target_points", 100)
        for (x, y) in points:
            t = self.get_clock().now()
            msg = PoseStamped()
            msg.header.stamp = t.to_msg()
            msg.header.frame_id = "0"
            msg.pose.position.x = x
            msg.pose.position.y = y
            self.publisher.publish(msg)


def main():
    inner_track, middle_track, outer_track = generate_track(30, size_x=5, size_y=5, refinements=2, n_points=40,
                                                            track_width=6)
    left_most_point = middle_track[:, 1].argmax()
    x, y = middle_track[left_most_point]
    inner_track[:, 1] -= y
    middle_track[:, 1] -= y
    outer_track[:, 1] -= y
    cones = []
    cone_positions = np.concatenate((inner_track, outer_track))
    for idx, pos in enumerate(cone_positions):
        x, y = pos
        entity_factory = EntityFactory(sdf_filename=str(cone_model), name=f"{idx}",
                                       pose=Pose(position=Vector3d(x=x, y=y)))
        cones.append(entity_factory)
    entity_factory_v = EntityFactory_V(data=cones)
    spawn_entities(WORLD, entity_factory_v)
    start_point = middle_track[left_most_point]
    set_pose(WORLD, Pose(id=4, position=Vector3d(x=start_point[0], y=start_point[1], z=0.25)))
    sleep(5)
    rclpy.init()
    world_control(WORLD, WorldControl(pause=False, step=True))


if __name__ == "__main__":
    main()
