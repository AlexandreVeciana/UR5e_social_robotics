#!/usr/bin/env python3
"""
Video Classification Node using Ultralytics YOLO
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import os

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


class VideoClassificationNode(Node):

    def __init__(self):
        super().__init__('video_classification_node')

        package_path = get_package_share_directory('ur5e_video_classification')

        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('model_path', os.path.join(package_path, 'models', 'best_model.pt'))
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('prediction_rate', 2.0)
        self.declare_parameter('frame_size', 112)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.prediction_rate = self.get_parameter('prediction_rate').value
        self.frame_size = self.get_parameter('frame_size').value

        self.bridge = CvBridge()
        self.latest_frame = None

        # Load YOLO model
        self.model = self.load_model()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.result_pub = self.create_publisher(String, '/prediction/result', 10)
        self.confidence_pub = self.create_publisher(Float32, '/prediction/confidence', 10)
        self.status_pub = self.create_publisher(String, '/prediction/status', 10)

        # Prediction timer
        self.prediction_timer = self.create_timer(
            1.0 / self.prediction_rate,
            self.prediction_callback
        )

        self.get_logger().info('=' * 50)
        self.get_logger().info('YOLO Classification Node Started')
        self.get_logger().info(f'Model: {self.model_path}')
        self.get_logger().info(f'Camera: {self.camera_topic}')
        self.get_logger().info(f'Classes: {self.model.names}')
        self.get_logger().info('=' * 50)

    def load_model(self):
        try:
            model = YOLO(self.model_path)
            self.get_logger().info('YOLO model loaded successfully')
            return model
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def prediction_callback(self):
        if self.latest_frame is None:
            status_msg = String()
            status_msg.data = 'Waiting for camera frames...'
            self.status_pub.publish(status_msg)
            return

        try:
            # Run YOLO inference
            results = self.model(self.latest_frame, verbose=False)
            result = results[0]

            # Get top classification result
            probs = result.probs  # for classification models
            predicted_idx = int(probs.top1)
            confidence = float(probs.top1conf)
            class_name = self.model.names[predicted_idx]

            # Publish
            result_msg = String()
            result_msg.data = class_name
            self.result_pub.publish(result_msg)

            conf_msg = Float32()
            conf_msg.data = confidence * 100.0
            self.confidence_pub.publish(conf_msg)

            status_msg = String()
            if confidence >= self.confidence_threshold:
                status_msg.data = f'{class_name} ({confidence*100:.1f}%)'
                self.get_logger().info(
                    f'Prediction: {class_name} ({confidence*100:.1f}%)',
                    throttle_duration_sec=1.0
                )
            else:
                status_msg.data = f'? {class_name} ({confidence*100:.1f}%) - LOW CONFIDENCE'
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Prediction error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VideoClassificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()