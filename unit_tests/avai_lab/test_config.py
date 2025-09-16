# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
import pytest
from pathlib import Path
from avai_lab.config import load_config, Config

def test_load_config(tmp_path: Path):
    # Create a temporary TOML file with valid content.
    config_content = """
[car_platform]
max_speed = 2.5
min_speed = 1.0
max_steering_angle = 0.5
max_acceleration = 1.2
max_steering_change = 0.3
"""
    config_file = tmp_path / "config.toml"
    config_file.write_text(config_content)
    
    config = load_config(config_file)
    assert isinstance(config, Config)
    assert config.car_platform.max_speed == 2.5
    assert config.car_platform.min_speed == 1.0
    assert config.car_platform.max_steering_angle == 0.5
    assert config.car_platform.max_acceleration == 1.2
    assert config.car_platform.max_steering_change == 0.3
