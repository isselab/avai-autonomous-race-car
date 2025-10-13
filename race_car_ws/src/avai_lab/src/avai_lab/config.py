# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
from pathlib import Path
from typing import Optional

from pydantic import BaseModel
import tomlkit

CONFIG_BASE_PATH = Path.home() / "buggy-busters-config.toml"


class CarPlatformConfig(BaseModel):
    """The car platform section includes all settings that are based on the car hardware itself
    like driving speeds, steering angles, NOT behavior"""
    max_speed: float  # m/s
    min_speed: float  # m/s
    max_steering_angle: float  # radian/s
    max_acceleration: float  # m/s^2
    max_steering_change: float  # radian/s^2


class Config(BaseModel):
    """The base model for the config file of the project. Can be loaded from a file using the `load_config` function.
    """
    car_platform: CarPlatformConfig


def load_config(path: Optional[Path] = None) -> Config:
    """Load a config file and return the contents as a Config object
    If no path is provided, the users home path is used.
    """
    if path is None:
        path = CONFIG_BASE_PATH
    with open(path, "r") as f:
        config_data = tomlkit.parse(f.read())
    return Config.model_validate(config_data)
