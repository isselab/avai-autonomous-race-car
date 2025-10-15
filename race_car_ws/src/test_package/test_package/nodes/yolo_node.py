# SPDX-License-Identifier: MIT
# Authors: Nicklas Osmers, Kevin Losing, Ahmad Al Shihabi
#!/usr/bin/env python3

import math
from time import time
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from racecar_msgs.msg import DetectedCone, DetectedConeArray

from cv_bridge import CvBridge
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
import message_filters

from avai_lab.enums import string_to_label, UNKNOWN_CONE

import pyrealsense2 as rs
from realsense2_camera_msgs.msg import Extrinsics


class YoloConeDetectionNode(Node):
    """
    ROS2 node that uses YOLO to detect cones from camera sensors.
    """

    def __init__(self):
        super().__init__('yolo_cone_detection_node')
        self.declare_parameter('image_topic', '/camera/realsense2_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/realsense2_camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/realsense2_camera/depth/image_rect_raw')
        self.declare_parameter('detection_topic', '/yolo_cones')
        self.declare_parameter('model_path', f'{__file__}/../../../runs/detect/train/weights/best.pt')
        self.declare_parameter('confidence_threshold', 0.25)  # Confidence threshold for filtering yolo detections
        self.declare_parameter('frame_id', 'base_link')  # The target frame to which cone positions will be transformed.
        self.declare_parameter('patch_size', 5)  # The amount of pixels to average over for depth detection of the cone
        self.declare_parameter('depth_camera_info_topic', '/camera/realsense2_camera/depth/camera_info')
        self.declare_parameter('extrinsics_topic', '/camera/realsense2_camera/extrinsics/depth_to_color')
        self.declare_parameter('depth_scale', 0.001)  # to convert depth units (mm to m)
        self.declare_parameter('depth_min', 0.1)  # Minimum depth (m)
        self.declare_parameter('depth_max', 10.0)  # Maximum depth (m)

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.frame_id = self.get_parameter('frame_id').value
        self.patch_size = self.get_parameter('patch_size').value
        depth_camera_info_topic = self.get_parameter('depth_camera_info_topic').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.depth_min = self.get_parameter('depth_min').value
        self.depth_max = self.get_parameter('depth_max').value

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.cones_pub = self.create_publisher(DetectedConeArray, self.detection_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.extrinsics_R = None  # 3x3 rotation matrix
        self.extrinsics_t = None  # 3-element translation vector
        rotation = np.array([0.999, -0.0122, 0.000324, 0.01223, 0.999, 0.00525, 0.000388, -0.00524, 0.999])
        translation = np.array([0.01499, -0.000196, 0.000446])
        self.extrinsics_R = np.array(rotation).reshape(3, 3)
        self.extrinsics_t = np.array(translation)

        # Setup synchronized subscriptions for color image, camera info, depth image, and depth camera info.
        self.image_sub = message_filters.Subscriber(self, Image, image_topic)
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.depth_camera_info_sub = message_filters.Subscriber(self, CameraInfo, depth_camera_info_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub, self.depth_sub, self.depth_camera_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

        self.last_evaluated = time()
        self.get_logger().info("YOLO Cone Detection Node with synchronized topics started...")

    def extrinsics_callback(self, msg: Extrinsics):
        self.extrinsics_R = np.array(msg.rotation).reshape(3, 3)
        self.extrinsics_t = np.array(msg.translation)
        self.get_logger().debug(f"Received extrinsics: R={self.extrinsics_R}, t={self.extrinsics_t}")

    def synced_callback(self, image_msg: Image, camera_info_msg: CameraInfo, depth_msg: Image,
                        depth_camera_info_msg: CameraInfo):
        """
        Callback for synchronized image, camera info, depth and depth info message.
        
        :param image_msg: The color image message.
        :param camera_info_msg: The CameraInfo message containing color intrinsics.
        :param depth_msg: The depth image message.
        :depth_camera_info_msg: The CameraInfomessage containing depth intrinsics
        :return: None
        """
        if time() - self.last_evaluated < 0.1:
            return
        self.last_evaluated = time()
        if self.extrinsics_R is None or self.extrinsics_t is None:
            self.get_logger().warn("Extrinsics not received yet. Skipping callback.")
            return

        # Convert depth image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")
            return

        # Convert color image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert color image: {e}")
            return

        # Run YOLO inference on the color image.
        results = self.model(cv_image)
        detections = self.parse_yolo_results(results)

        # Prepare DetectedConeArray message
        cone_array_msg = DetectedConeArray()
        cone_array_msg.header = depth_msg.header
        cone_array_msg.header.frame_id = self.frame_id

        try:
            stamp_time = rclpy.time.Time.from_msg(depth_msg.header.stamp)
            transform_stamped = self.tf_buffer.lookup_transform(
                self.frame_id,
                depth_camera_info_msg.header.frame_id,
                stamp_time,
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not look up TF: {e}")
            return

        # Create intrinsics objects for color and depth using the CameraInfo messages.
        color_intrin = self.create_intrinsics(camera_info_msg)
        depth_intrin = self.create_intrinsics(depth_camera_info_msg)

        # Create extrinsics objects
        depth_to_color = self.create_extrinsics(self.extrinsics_R, self.extrinsics_t)
        color_to_depth = self.create_inverse_extrinsics(self.extrinsics_R, self.extrinsics_t)

        # For each yolo detection
        for det in detections:
            x_min, y_min, x_max, y_max = det["bbox"]
            # Use the center of the bounding box as the color pixel.
            u_color = int((x_min + x_max) / 2)
            v_color = int((y_min + y_max) / 2)
            from_pixel = [u_color, v_color]

            # Project the color pixel to depth using the ported function.
            to_pixel = self.ported_rs2_project_color_pixel_to_depth_pixel(
                depth_msg,
                self.depth_scale,
                self.depth_min,
                self.depth_max,
                depth_intrin,
                color_intrin,
                color_to_depth,
                depth_to_color,
                from_pixel
            )
            u_depth, v_depth = int(round(to_pixel[0])), int(round(to_pixel[1]))

            if u_depth < 0 or v_depth < 0 or u_depth >= depth_intrin.width or v_depth >= depth_intrin.height:
                self.get_logger().warn("Mapped depth pixel out of bounds.")
                continue

            pt_depth = self.pixel_to_3d(u_depth, v_depth, depth_image, depth_camera_info_msg)
            if pt_depth is None:
                continue
            ps_x, ps_y, ps_z = pt_depth
            self.get_logger().debug(f"Detected 3D in depth frame: x={ps_x}, y={ps_y}, z={ps_z}")

            ps = PointStamped()
            ps.header.frame_id = depth_camera_info_msg.header.frame_id
            ps.header.stamp = depth_msg.header.stamp
            ps.point.x = float(ps_x)
            ps.point.y = float(ps_y)
            ps.point.z = float(ps_z)

            # Transform the point to the target frame
            try:
                ps_transformed = tf2_geometry_msgs.do_transform_point(ps, transform_stamped)
            except Exception as e:
                self.get_logger().warn(f"Error transforming point: {e}")
                continue

            cone_msg = DetectedCone()
            cone_msg.header = depth_msg.header
            cone_msg.header.frame_id = self.frame_id
            cone_msg.type = string_to_label.get(det["class_name"], UNKNOWN_CONE)
            cone_msg.position.x = ps_transformed.point.x
            cone_msg.position.y = ps_transformed.point.y
            cone_msg.position.z = 0.0  # 2D race track, we can just set z to 0
            self.get_logger().debug(
                f"Transformed 3D to {self.frame_id}: x={ps_transformed.point.x}, y={ps_transformed.point.y}, z={ps_transformed.point.z}")

            cone_array_msg.cones.append(cone_msg)

        self.cones_pub.publish(cone_array_msg)

    def parse_yolo_results(self, results):
        """
        Parse YOLO inference results to extract detections.
        
        :param results: The results returned by the YOLO model.
        :return: A list of dictionaries; each dictionary contains:
                 - class_name: The label of the detected object.
                 - bbox: The bounding box [x_min, y_min, x_max, y_max].
        """
        detections = []
        if len(results) == 0:
            return detections

        res = results[0]
        boxes = res.boxes

        for box in boxes:
            cls_id = int(box.cls[0])
            class_name = self.model.names[cls_id]
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].tolist()

            if conf < self.conf_threshold:
                continue

            detections.append({
                "class_name": class_name,
                "bbox": xyxy
            })
        return detections

    def pixel_to_3d(self, u, v, depth_image, camera_info):
        """
        Converts a pixel coordinate (u, v) and its corresponding depth value to a 3D point in the camera frame.

        :param u: The u-coordinate (horizontal) of the pixel.
        :param v: The v-coordinate (vertical) of the pixel.
        :param depth_image: The depth image.
        :param camera_info: The CameraInfo message containing camera intrinsics.
        :return: A tuple (X, Y, Z) representing the 3D point in the camera frame, or None if the conversion fails.
        """
        # Get camera intrinsics from CameraInfo message
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        # Return None if extracted focal lengths are invalid
        if fx == 0.0 or fy == 0.0:
            return None

        # Average/Take the median over patch_size pixels around the pixel region
        half = self.patch_size // 2
        depths = []
        for dx in range(-half, half + 1):
            for dy in range(-half, half + 1):
                u_median = u + dx
                v_median = v + dy
                if 0 <= u_median < depth_image.shape[1] and 0 <= v_median < depth_image.shape[0]:
                    d = depth_image[v_median, u_median]
                    if np.isfinite(d) and d > 0:
                        depths.append(d)

        if not depths:
            return None

        depth = np.median(depths) * self.depth_scale
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth

        if not (math.isfinite(X) and math.isfinite(Y) and math.isfinite(Z)):
            return None

        return (X, Y, Z)

    def create_intrinsics(self, cam_info):
        intrin = rs.intrinsics()
        intrin.width = cam_info.width
        intrin.height = cam_info.height
        intrin.fx = cam_info.k[0]
        intrin.fy = cam_info.k[4]
        intrin.ppx = cam_info.k[2]
        intrin.ppy = cam_info.k[5]
        intrin.model = rs.distortion.none
        intrin.coeffs = [0, 0, 0, 0, 0]
        return intrin

    def create_extrinsics(self, R, t):
        ext = rs.extrinsics()
        ext.rotation = R.flatten().tolist()
        ext.translation = t.tolist()
        return ext

    def create_inverse_extrinsics(self, R, t):
        R_inv = R.T
        t_inv = -R_inv.dot(t)
        ext = rs.extrinsics()
        ext.rotation = R_inv.flatten().tolist()
        ext.translation = t_inv.tolist()
        return ext

    # Many thanks & credits to joernnilsson for these functions https://github.com/IntelRealSense/realsense-ros/issues/1785#issuecomment-1568730486
    def ported_adjust_2D_point_to_boundary(self, p, width, height):
        if (p[0] < 0):
            p[0] = 0
        if (p[0] > width):
            p[0] = float(width)
        if (p[1] < 0):
            p[1] = 0
        if (p[1] > height):
            p[1] = float(height)
        return p

    def ported_next_pixel_in_line(self, curr, start, end_p):
        line_slope = (end_p[1] - start[1]) / (end_p[0] - start[0])
        if (abs(end_p[0] - curr[0]) > abs(end_p[1] - curr[1])):
            curr[0] = curr[0] + 1 if end_p[0] > curr[0] else curr[0] - 1
            curr[1] = end_p[1] - line_slope * (end_p[0] - curr[0])
        else:
            curr[1] = curr[1] + 1 if end_p[1] > curr[1] else curr[1] - 1
            curr[0] = end_p[0] - ((end_p[1] + curr[1]) / line_slope)

    def ported_is_pixel_in_line(self, curr, start, end_p):
        return ((end_p[0] >= start[0] and end_p[0] >= curr[0] and curr[0] >= start[0]) or (
                end_p[0] <= start[0] and end_p[0] <= curr[0] and curr[0] <= start[0])) and \
            ((end_p[1] >= start[1] and end_p[1] >= curr[1] and curr[1] >= start[1]) or (
                    end_p[1] <= start[1] and end_p[1] <= curr[1] and curr[1] <= start[1]))

    def ported_rs2_project_color_pixel_to_depth_pixel(self,
                                                      data,
                                                      depth_scale,
                                                      depth_min,
                                                      depth_max,
                                                      depth_intrin,
                                                      color_intrin,
                                                      color_to_depth,
                                                      depth_to_color,
                                                      from_pixel
                                                      ):

        # Return value
        to_pixel = [0, 0]

        # Find line start pixel
        min_point = rs.rs2_deproject_pixel_to_point(color_intrin, from_pixel, depth_min)
        min_transformed_point = rs.rs2_transform_point_to_point(color_to_depth, min_point)
        start_pixel = rs.rs2_project_point_to_pixel(depth_intrin, min_transformed_point)

        # Find line end depth pixel
        max_point = rs.rs2_deproject_pixel_to_point(color_intrin, from_pixel, depth_max)
        max_transformed_point = rs.rs2_transform_point_to_point(color_to_depth, max_point)
        end_pixel = rs.rs2_project_point_to_pixel(depth_intrin, max_transformed_point)
        end_pixel = self.ported_adjust_2D_point_to_boundary(end_pixel, depth_intrin.width, depth_intrin.height)

        # search along line for the depth pixel that it's projected pixel is the closest to the input pixel
        min_dist = -1.0

        p = [start_pixel[0], start_pixel[1]]
        while self.ported_is_pixel_in_line(p, start_pixel, end_pixel):

            # This part assumes data is a ROS sensor_msgs/msg/Image
            x = int(p[0])
            y = int(p[1])
            step = data.step
            idx = y * step + x * 2
            byte_values = data.data[idx:idx + 2]
            int_value = int.from_bytes(byte_values, "big" if data.is_bigendian else "little")

            depth = depth_scale * int_value
            if (depth == 0):
                self.ported_next_pixel_in_line(p, start_pixel, end_pixel)
                continue

            point = rs.rs2_deproject_pixel_to_point(depth_intrin, p, depth)
            transformed_point = rs.rs2_transform_point_to_point(depth_to_color, point)
            projected_pixel = rs.rs2_project_point_to_pixel(color_intrin, transformed_point)

            new_dist = (float)(
                math.pow((projected_pixel[1] - from_pixel[1]), 2) + math.pow((projected_pixel[0] - from_pixel[0]), 2))
            if new_dist < min_dist or min_dist < 0:
                min_dist = new_dist
                to_pixel[0] = p[0]
                to_pixel[1] = p[1]

            self.ported_next_pixel_in_line(p, start_pixel, end_pixel)

        return to_pixel


def main(args=None):
    rclpy.init(args=args)
    node = YoloConeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
