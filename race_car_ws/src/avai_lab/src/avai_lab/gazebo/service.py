# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
import subprocess

from .msg import Entity, EntityFactory, EntityFactory_V, IgnitionMsg, Pose, WorldControl


def world_service_factory(world: str):
    class WorldService:
        CREATE = f"/world/{world}/create"
        CREATE_MULTIPLE = f"/world/{world}/create_multiple"
        SET_POSE = f"/world/{world}/set_pose"
        REMOVE_ENTITY = f"/world/{world}/remove"
        WORLD_CONTROL = f"/world/{world}/control"

    return WorldService


def cmd_factory(service: str, request: IgnitionMsg, reptype: str, timeout: int = 1000):
    cmd = [
        "ign", "service",
        "-s", service,
        "--reqtype", f"ignition.msgs.{request.__class__.__name__}",
        "--reptype", f"ignition.msgs.{reptype}",
        "--timeout", f"{timeout}",
        "--req", f'{request.dump_request()}',
    ]
    return cmd


def spawn_entity(world: str, entity_factory: EntityFactory):
    """Spawn an entity in the provided gazebo world.

    Note that the service /world/<world>/create only exists if the ignition-gazebo-user-commands-system is loaded
    """
    cmd = cmd_factory(world_service_factory(world).CREATE, entity_factory, "Boolean")
    r = subprocess.run(cmd)
    return r


def spawn_entities(world: str, entity_factory_v: EntityFactory_V):
    cmd = cmd_factory(world_service_factory(world).CREATE_MULTIPLE, entity_factory_v, "Boolean")
    r = subprocess.run(cmd)
    return r


def set_pose(world: str, pose: Pose):
    cmd = cmd_factory(world_service_factory(world).SET_POSE, pose, "Boolean")
    r = subprocess.run(cmd)
    return r


def remove_entity(world: str, entity: Entity):
    cmd = cmd_factory(world_service_factory(world).REMOVE_ENTITY, entity, "Boolean", timeout=10000)
    r = subprocess.run(cmd)
    return r


def world_control(world: str, world_control: WorldControl):
    cmd = cmd_factory(world_service_factory(world).WORLD_CONTROL, world_control, "Boolean", timeout=10000)
    r = subprocess.run(cmd)
    return r
