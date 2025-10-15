# SPDX-License-Identifier: MIT
# Authors: Kevin Losing, Sai Tarun Bhyri
#!/usr/bin/env python3
import math
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


class YoloConeDetectionNodeRGBD(Node):
    """
    ROS2 node that uses YOLO to detect cones from RGBD camera sensors.
    """

    def __init__(self):
        super().__init__('yolo_cone_detection_node')
        self.declare_parameter('image_topic', '/camera/realsense2_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/realsense2_camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/realsense2_camera/depth/image_rect_raw')
        self.declare_parameter('detection_topic', '/yolo_cones')
        self.declare_parameter('model_path', '/home/tarunbhyri9/project_repo/avai-autonomous-race-car/race_car_ws/src/test_package/runs/detect/train/weights/best.pt')
        self.declare_parameter('confidence_threshold', 0.25)  # Confidence threshold for filtering yolo detections
        self.declare_parameter('frame_id', 'base_link')  # The target frame to which cone positions will be transformed.
        self.declare_parameter('patch_size', 5)  # The amount of pixels to average over for depth detection of the cone

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.frame_id = self.get_parameter('frame_id').value
        self.patch_size = self.get_parameter('patch_size').value

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.cones_pub = self.create_publisher(DetectedConeArray, self.detection_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Setup synchronized subscriptions for image, camera info, and depth topics
        self.image_sub = message_filters.Subscriber(self, Image, image_topic)
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info("YOLO Cone Detection Node with synchronized topics started...")

    def synced_callback(self, image_msg: Image, camera_info_msg: CameraInfo, depth_msg: Image):
        """
        Callback for synchronized image, camera info, and depth messages.
        
        :param image_msg: The color image message.
        :param camera_info_msg: The CameraInfo message containing intrinsics.
        :param depth_msg: The depth image message.
        :return: None
        """
        camera_frame = image_msg.header.frame_id

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

        # Run YOLO inference on the color image
        results = self.model(cv_image)
        detections = self.parse_yolo_results(results)

        # Prepare DetectedConeArray message
        cone_array_msg = DetectedConeArray()
        cone_array_msg.header = image_msg.header
        cone_array_msg.header.frame_id = self.frame_id

        # Determine scaling factors if image sizes differ
        rgb_h, rgb_w = cv_image.shape[:2]
        depth_h, depth_w = depth_image.shape[:2]
        scale_depth_image = (rgb_w != depth_w) or (rgb_h != depth_h)
        if scale_depth_image:
            scale_x = depth_w / float(rgb_w)
            scale_y = depth_h / float(rgb_h)

        try:
            stamp_time = rclpy.time.Time.from_msg(image_msg.header.stamp)
            transform_stamped = self.tf_buffer.lookup_transform(
                self.frame_id,
                camera_frame,
                stamp_time,
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not look up TF: {e}")
            return

        # For each yolo detection
        for det in detections:
            x_min, y_min, x_max, y_max = det["bbox"]

            # Determine pixel coordinates (u, v), u and v are centered in the bounding box
            if scale_depth_image:
                x_min_scaled = x_min * scale_x
                x_max_scaled = x_max * scale_x
                y_min_scaled = y_min * scale_y
                y_max_scaled = y_max * scale_y
                u = int((x_min_scaled + x_max_scaled) / 2)
                v = int((y_min_scaled + y_max_scaled) / 2)
            else:
                u = int((x_min + x_max) / 2)
                v = int((y_min + y_max) / 2)

            # Convert pixel coordinates to 3D point in camera frame
            pt_camera = self.pixel_to_3d(u, v, depth_image, camera_info_msg)
            if pt_camera is None:
                continue

            ps_x, ps_y, ps_z = pt_camera
            self.get_logger().debug(f"Detected 3D in {camera_frame}: x={ps_x}, y={ps_y}, z={ps_z}")

            # Create a PointStamped for the detected cone in the camera frame
            ps = PointStamped()
            ps.header.frame_id = camera_frame
            ps.header.stamp = image_msg.header.stamp
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
            cone_msg.header = image_msg.header
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
        # Check if pixel is valid
        if u < 0 or v < 0 or u >= camera_info.width or v >= camera_info.height:
            return None

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

        depth = np.median(depths)
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth

        if not (math.isfinite(X) and math.isfinite(Y) and math.isfinite(Z)):
            return None

        return (X, Y, Z)


def main(args=None):
    rclpy.init(args=args)
    node = YoloConeDetectionNodeRGBD()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
