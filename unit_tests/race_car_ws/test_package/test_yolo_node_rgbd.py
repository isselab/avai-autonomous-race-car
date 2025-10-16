# SPDX-License-Identifier: MIT
# Authors: Kevin Losing

import pytest
import math
import numpy as np
from sensor_msgs.msg import CameraInfo
from unittest.mock import MagicMock
from race_car_ws.src.test_package.test_package.nodes.yolo_node_rgbd import YoloConeDetectionNodeRGBD


class FakeBox:
    def __init__(self, cls, conf, xyxy):
        self.cls = [cls]
        self.conf = [conf]
        self.xyxy = [np.array(xyxy)]


class FakeResults:
    def __init__(self, boxes):
        self.boxes = boxes


class FakeModel:
    def __init__(self, names):
        self.names = names

    def __call__(self, img):
        return [FakeResults([])]


def create_fake_camera_info(width=640, height=480):
    cam_info = CameraInfo()
    cam_info.width = width
    cam_info.height = height
    cam_info.k = [100.0, 0.0, width / 2.0, 0.0, 100.0, height / 2.0, 0.0, 0.0, 1.0]
    cam_info.header.frame_id = "camera_frame"
    return cam_info


@pytest.fixture
def yolo_node(ros_setup):
    node = YoloConeDetectionNodeRGBD()
    node.conf_threshold = 0.5
    node.frame_id = "base_link"
    node.model = FakeModel({0: "yellow_cones", 1: "blue_cones", 2: "orange_cones"})
    node.get_logger = MagicMock()

    yield node
    node.destroy_node()


def test_parse_yolo_results_empty(yolo_node):
    detections = yolo_node.parse_yolo_results([])
    assert detections == []


def test_parse_yolo_results_threshold(yolo_node):
    box_low = FakeBox(cls=0, conf=0.4, xyxy=[10, 20, 30, 40])
    box_high = FakeBox(cls=0, conf=0.6, xyxy=[15, 25, 35, 45])
    fake_results = FakeResults(boxes=[box_low, box_high])
    detections = yolo_node.parse_yolo_results([fake_results])
    assert len(detections) == 1
    assert detections[0]["class_name"] == "yellow_cones"
    assert detections[0]["bbox"] == [15, 25, 35, 45]


def test_pixel_to_3d_valid(yolo_node):
    fake_depth = np.full((480, 640), 2.0, dtype=np.float32)
    cam_info = create_fake_camera_info(640, 480)
    result = yolo_node.pixel_to_3d(320, 240, fake_depth, cam_info)
    assert result is not None
    X, Y, Z = result
    assert math.isclose(X, 0.0, abs_tol=1e-5)
    assert math.isclose(Y, 0.0, abs_tol=1e-5)
    assert math.isclose(Z, 2.0, abs_tol=1e-5)


def test_pixel_to_3d_invalid(yolo_node):
    fake_depth = np.full((480, 640), 2.0, dtype=np.float32)
    cam_info = create_fake_camera_info(640, 480)
    result = yolo_node.pixel_to_3d(700, 500, fake_depth, cam_info)
    assert result is None
