# SPDX-License-Identifier: MIT
# Authors: Nicolas Kuhl
import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def ros_setup():
    if not rclpy.ok():
        rclpy.init()
    yield
    try:
        rclpy.shutdown()
    except RuntimeError:
        pass
