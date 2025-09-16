# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
"""
This script generates a track using the track gen function
and spawns it using the standard cone model into gazebo
"""
import os
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

from avai_lab.gazebo.service import remove_entity, spawn_entities, set_pose
from avai_lab.gazebo.msg import Entity, EntityFactory, EntityFactory_V, EntityType, Pose, Vector3d
from avai_lab.track_gen import generate_track

WORLD = "car_world"
CAR_ID = 4
cone_model_path = Path(os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")) / "cone"
assert cone_model_path.exists(), f"Cone model path does not exist: {cone_model_path}"
blue_cone_model = cone_model_path / "blue-cone.sdf"
orange_cone_model = cone_model_path / "orange-cone.sdf"
assert blue_cone_model.exists(), "Blue cone model does not exist"
assert orange_cone_model.exists(), "Blue cone model does not exist"


def main():
    inner_track, middle_track, outer_track = generate_track(30, size_x=10, size_y=10, refinements=2, n_points=120,
                                                            track_width=3, dents=3)
    left_most_point = middle_track[:, 1].argmax()
    x, y = middle_track[left_most_point]
    inner_track[:, 1] -= y
    middle_track[:, 1] -= y
    outer_track[:, 1] -= y
    cones = []
    cone_models = [orange_cone_model, blue_cone_model]
    cone_id = 0
    for i, track_boundary in enumerate((inner_track, outer_track)):
        for pos in track_boundary:
            x, y = pos
            entity_factory = EntityFactory(sdf_filename=str(cone_models[i]), name=f"{cone_id}",
                                           pose=Pose(position=Vector3d(x=x, y=y)))
            cone_id += 1
            cones.append(entity_factory)
    entity_factory_v = EntityFactory_V(data=cones)
    spawn_entities(WORLD, entity_factory_v)
    x, y = middle_track[left_most_point]
    set_pose(WORLD, Pose(id=CAR_ID, position=Vector3d(x=x, y=y, z=0.25)))

    return


if __name__ == "__main__":
    main()
