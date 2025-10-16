# SPDX-License-Identifier: MIT
# Authors: Nicklas Osmers

from typing import Any, List, Optional
from enum import Enum, auto

from pydantic import BaseModel, ConfigDict


class EntityType(Enum):
    NONE = 0
    LIGHT = auto()
    MODEL = auto()
    LINK = auto()
    VISUAL = auto()
    COLLISION = auto()
    SENSOR = auto()
    JOINT = auto()
    ACTOR = auto()
    WORLD = auto()


class IgnitionMsg(BaseModel):

    @staticmethod
    def get_attr_string(name: str, value: Any):
        if isinstance(value, str):
            value = f"\"{value}\""
        if isinstance(value, Enum):
            value = value.value
        if isinstance(value, IgnitionMsg):
            return f"{name}:" + " { " + value.dump_request() + " }"
        if isinstance(value, list):
            return " ".join([IgnitionMsg.get_attr_string(name, e) for e in value])
        else:
            return (f"{name}: {value}")

    def dump_request(self):
        attributes = []  # strings of "name: value" or if instance is a model "name: {{ value }}"
        for field_name, _ in self.model_fields.items():
            value = getattr(self, field_name)
            if value is None:
                continue
            attributes.append(self.get_attr_string(field_name, value))
        return " ".join(attributes)


class Vector3d(IgnitionMsg):
    x: float = 0
    y: float = 0
    z: float = 0


class Quaternion(IgnitionMsg):
    x: float = 0
    y: float = 0
    z: float = 0
    w: float = 0


class Pose(IgnitionMsg):
    """Mimic for ignition.msgs.Pose"""
    id: Optional[int] = None
    position: Optional[Vector3d] = None
    orientation: Optional[Quaternion] = None


class EntityFactory(IgnitionMsg):
    """Used to spawn a single entity using an SDF file"""
    name: Optional[str] = None
    sdf_filename: str
    pose: Optional[Pose] = None
    allow_renaming: Optional[bool] = None


class EntityFactory_V(IgnitionMsg):
    """Used to spawn multiple entities in one batch"""
    data: List[EntityFactory]


class Entity(IgnitionMsg):
    id: Optional[int] = None
    name: Optional[str] = None
    type: Optional[EntityType] = None


class WorldReset(IgnitionMsg):
    all: Optional[bool] = None
    time_only: Optional[bool] = None
    model_only: Optional[bool] = None

    model_config = ConfigDict(protected_namespaces=())


class Time(IgnitionMsg):
    sec: int
    nsec: int = 0


class WorldControl(IgnitionMsg):
    pause: bool
    step: bool
    multi_step: Optional[int] = None
    reset: Optional[WorldReset] = None
    run_to_sim_time: Optional[Time] = None
