#!/usr/bin/env python3
"""
Hand Position Detection Node with YOLO Pose + Depth Camera

Detects hand position using YOLO Pose and depth camera,
then provides 3D coordinates for robot handshake.

Subscribes to:
  /camera/color/image_raw - RGB image
  /camera/depth/image_raw - Depth image
  /camera/depth/camera_info - Camera intrinsics

Publishes:
  /hand_position/detected (Bool) - Hand detected
  /hand_position/3d (geometry_msgs/Point) - Hand 3D position
  /hand_position/robot_command (String) - Robot move command
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from ultralytics import YOLO


class HandPositionNode(Node):
    """
    Detects hand position using YOLO Pose and depth camera.
    """
    
    def __init__(self):
        super().__init__('hand_position_node')
        
        # Parameters
        self.declare_parameter('yolo_model_path', './yolo11n-pose.pt')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('target_gesture', 'handshake')  # Which gesture to detect
        self.declare_parameter('hand_side', 'right')  # 'right' or 'left'
        
        self.yolo_model_path = self.get_parameter('yolo_model_path').value
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.target_gesture = self.get_parameter('target_gesture').value
        self.hand_side = self.get_parameter('hand_side').value
        
        # Load YOLO Pose model
        self.get_logger().info(f'Loading YOLO Pose model: {self.yolo_model_path}')
        self.yolo_model = YOLO(self.yolo_model_path)
        self.get_logger().info('YOLO Pose model loaded')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Latest images
        self.latest_rgb = None
        self.latest_depth = None
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.hand_detected_pub = self.create_publisher(Bool, '/hand_position/detected', 10)
        self.hand_3d_pub = self.create_publisher(Point, '/hand_position/3d', 10)
        self.robot_command_pub = self.create_publisher(String, '/ur5e/move_command', 10)
        self.debug_image_pub = self.create_publisher(Image, '/hand_position/debug_image', 10)
        
        # Timer for processing
        self.processing_timer = self.create_timer(0.1, self.process_callback)  # 10 Hz
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Hand Position Detection Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Target gesture: {self.target_gesture}')
        self.get_logger().info(f'Hand side: {self.hand_side}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info('=' * 60)
    
    def rgb_callback(self, msg: Image):
        """Store latest RGB image."""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')
    
    def depth_callback(self, msg: Image):
        """Store latest depth image."""
        try:
            # Depth is usually uint16 in millimeters
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsics."""
        if self.camera_matrix is None:
            # Camera intrinsic matrix
            # K = [fx  0  cx]
            #     [ 0 fy  cy]
            #     [ 0  0   1]
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            
            self.get_logger().info('✓ Camera intrinsics received')
            self.get_logger().info(f'  fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}')
            self.get_logger().info(f'  cx={self.camera_matrix[0,2]:.1f}, cy={self.camera_matrix[1,2]:.1f}')
    
    def process_callback(self):
        """Main processing loop."""
        if self.latest_rgb is None or self.latest_depth is None:
            return
        
        if self.camera_matrix is None:
            return
        
        try:
            # Run YOLO Pose detection
            results = self.yolo_model(self.latest_rgb, verbose=False)
            
            # Process detections
            hand_position_3d = self.detect_hand_position(results[0])
            
            if hand_position_3d is not None:
                # Publish detection
                detected_msg = Bool()
                detected_msg.data = True
                self.hand_detected_pub.publish(detected_msg)
                
                # Publish 3D position
                point_msg = Point()
                point_msg.x = hand_position_3d[0]
                point_msg.y = hand_position_3d[1]
                point_msg.z = hand_position_3d[2]
                self.hand_3d_pub.publish(point_msg)
                
                # Generate robot command
                command_msg = String()
                command_msg.data = self.generate_robot_command(hand_position_3d)
                self.robot_command_pub.publish(command_msg)
                
                self.get_logger().info(
                    f'Hand detected at: ({hand_position_3d[0]:.3f}, '
                    f'{hand_position_3d[1]:.3f}, {hand_position_3d[2]:.3f}) m',
                    throttle_duration_sec=1.0
                )
            else:
                # No hand detected
                detected_msg = Bool()
                detected_msg.data = False
                self.hand_detected_pub.publish(detected_msg)
            
            # Publish debug image
            debug_img = self.draw_debug_image(results[0])
            if debug_img is not None:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                self.debug_image_pub.publish(debug_msg)
        
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
    
    def detect_hand_position(self, result) -> np.ndarray:
        """
        Detect hand position from YOLO Pose results.
        
        Returns:
            3D position [x, y, z] in meters, or None if not detected
        """
        if result.keypoints is None or len(result.keypoints) == 0:
            return None
        
        # Get keypoints
        # YOLO Pose COCO format has 17 keypoints:
        # 0: nose, 1: left_eye, 2: right_eye, 3: left_ear, 4: right_ear
        # 5: left_shoulder, 6: right_shoulder, 7: left_elbow, 8: right_elbow
        # 9: left_wrist, 10: right_wrist, 11: left_hip, 12: right_hip
        # 13: left_knee, 14: right_knee, 15: left_ankle, 16: right_ankle
        
        keypoints = result.keypoints.xy[0].cpu().numpy()  # (17, 2)
        confidences = result.keypoints.conf[0].cpu().numpy()  # (17,)
        
        # Select wrist based on hand_side parameter
        if self.hand_side == 'right':
            wrist_idx = 10  # right_wrist
        else:
            wrist_idx = 9   # left_wrist
        
        # Check if wrist is detected with sufficient confidence
        if confidences[wrist_idx] < self.confidence_threshold:
            return None
        
        # Get 2D wrist position
        wrist_x = int(keypoints[wrist_idx, 0])
        wrist_y = int(keypoints[wrist_idx, 1])
        
        # Check bounds
        h, w = self.latest_depth.shape
        if wrist_x < 0 or wrist_x >= w or wrist_y < 0 or wrist_y >= h:
            return None
        
        # Get depth at wrist position (average over small region for stability)
        region_size = 5
        x_min = max(0, wrist_x - region_size)
        x_max = min(w, wrist_x + region_size)
        y_min = max(0, wrist_y - region_size)
        y_max = min(h, wrist_y + region_size)
        
        depth_region = self.latest_depth[y_min:y_max, x_min:x_max]
        
        # Filter out zero/invalid depths
        valid_depths = depth_region[depth_region > 0]
        if len(valid_depths) == 0:
            return None
        
        # Get median depth (more robust than mean)
        depth_mm = np.median(valid_depths)
        depth_m = depth_mm / 1000.0  # Convert to meters
        
        # Convert 2D pixel + depth to 3D position
        # Using camera intrinsics
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # 3D position in camera frame
        X = (wrist_x - cx) * depth_m / fx
        Y = (wrist_y - cy) * depth_m / fy
        Z = depth_m
        
        return np.array([X, Y, Z])
    
    def generate_robot_command(self, position_3d: np.ndarray) -> str:
        """
        Generate robot movement command.
        
        Args:
            position_3d: [x, y, z] in camera frame (meters)
            
        Returns:
            Command string for robot
        """
        # Camera: X=right, Y=down, Z=forward
        # Robot: X=forward, Y=left, Z=up
        
        # Transformation
        robot_x = -position_3d[0]      # Camera -X → Robot X (forward)
        robot_y = -position_3d[1]     # Camera -Y → Robot Y (left)
        robot_z = position_3d[2]     # Camera z → Robot Z (up)
        
        # Offset for robot base
        robot_x += 0.0  # Camera is 0m behind robot base
        robot_y += 0.26  #
        robot_z += 0.11  # Camera is 0.11m above robot base
        
        # Generate command
        command = f'move_to_position,{robot_x:.3f},{robot_y:.3f},{robot_z:.3f}'
        
        return command
    
    def draw_debug_image(self, result) -> np.ndarray:
        """Draw debug visualization."""
        if self.latest_rgb is None:
            return None
        
        debug_img = self.latest_rgb.copy()
        
        if result.keypoints is not None and len(result.keypoints) > 0:
            # Draw all keypoints
            keypoints = result.keypoints.xy[0].cpu().numpy()
            confidences = result.keypoints.conf[0].cpu().numpy()
            
            for i, (kp, conf) in enumerate(zip(keypoints, confidences)):
                if conf > self.confidence_threshold:
                    x, y = int(kp[0]), int(kp[1])
                    cv2.circle(debug_img, (x, y), 3, (0, 255, 0), -1)
            
            # Highlight target wrist
            wrist_idx = 10 if self.hand_side == 'right' else 9
            if confidences[wrist_idx] > self.confidence_threshold:
                x, y = int(keypoints[wrist_idx, 0]), int(keypoints[wrist_idx, 1])
                cv2.circle(debug_img, (x, y), 10, (0, 0, 255), 2)
                cv2.putText(debug_img, f'{self.hand_side.upper()} HAND', 
                           (x - 50, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (0, 0, 255), 2)
        
        return debug_img


def main(args=None):
    rclpy.init(args=args)
    node = HandPositionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
