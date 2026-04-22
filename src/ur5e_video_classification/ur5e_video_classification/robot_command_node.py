#!/usr/bin/env python3
"""
Robot Command Node for UR5e

Receives video classification predictions and sends commands to UR5e robot.

Topic subscriptions:
  /prediction/result (std_msgs/String)
  /prediction/confidence (std_msgs/Float32)

Topic publications:
  /ur5e/command (std_msgs/String) - Commands to robot
  /robot/status (std_msgs/String) - Robot status

Service clients:
  /ur5e/move_to_pose (custom) - Move robot to pose
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from collections import deque
import time


class RobotCommandNode(Node):
    """
    Sends commands to UR5e robot based on video predictions.
    """
    
    def __init__(self):
        super().__init__('robot_command_node')
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('stability_frames', 3)  # Require 3 consistent predictions
        self.declare_parameter('command_cooldown', 2.0)  # Seconds between commands
        
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.stability_frames = self.get_parameter('stability_frames').value
        self.command_cooldown = self.get_parameter('command_cooldown').value
        
        # State
        self.last_prediction = None
        self.last_confidence = 0.0
        self.prediction_history = deque(maxlen=self.stability_frames)
        self.last_command_time = time.time()
        
        # Action mappings (customize for your application)
        self.action_commands = {
            'Bow':          'bow_response',
            'Cross':        'cross_response',
            'GoldenOrder':  'golden_order_response',
            'HalfSun':      'half_sun_response',
            'Handshake':    'handshake_response',
            'NoAction':     'idle',
            'PointDown':    'point_down_response',
            'PraiseTheSun': 'praise_response',
            'SideLeg':      'side_leg_response',
            'Stop':         'stop_response',
            'Wave':         'wave_response',
        }
        
        # Subscribers
        self.result_sub = self.create_subscription(
            String,
            '/prediction/result',
            self.result_callback,
            10
        )
        
        self.confidence_sub = self.create_subscription(
            Float32,
            '/prediction/confidence',
            self.confidence_callback,
            10
        )
        
        # Publishers
        self.command_pub = self.create_publisher(String, '/ur5e/command', 10)
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        
        self.get_logger().info('Robot Command Node Started')
        self.get_logger().info(f'  Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'  Stability frames: {self.stability_frames}')
        self.get_logger().info(f'  Command cooldown: {self.command_cooldown}s')
    
    def result_callback(self, msg: String):
        """Process prediction result."""
        self.last_prediction = msg.data
        self.prediction_history.append(msg.data)
        
        # Check if we should send a command
        if self.should_send_command():
            self.send_robot_command()
    
    def confidence_callback(self, msg: Float32):
        """Update confidence."""
        self.last_confidence = msg.data
    
    def should_send_command(self) -> bool:
        """
        Determine if robot command should be sent.
        
        Conditions:
        1. Confidence above threshold
        2. Predictions are stable (same class for N frames)
        3. Cooldown period has passed
        """
        # Check confidence
        if self.last_confidence < self.confidence_threshold * 100:
            return False
        
        # Check stability
        if len(self.prediction_history) < self.stability_frames:
            return False
        
        if len(set(self.prediction_history)) != 1:  # Not all same
            return False
        
        # Check cooldown
        time_since_last = time.time() - self.last_command_time
        if time_since_last < self.command_cooldown:
            return False
        
        return True
    
    def send_robot_command(self):
        """Send command to robot based on prediction."""
        if self.last_prediction is None:
            return
        
        # Get command for this action
        command = self.action_commands.get(
            self.last_prediction,
            'unknown_action'
        )
        
        # Publish command
        cmd_msg = String()
        cmd_msg.data = command
        self.command_pub.publish(cmd_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Executing: {command} (detected: {self.last_prediction}, conf: {self.last_confidence:.1f}%)'
        self.status_pub.publish(status_msg)
        
        # Update last command time
        self.last_command_time = time.time()
        
        self.get_logger().info(
            f'Command sent: {command} (Action: {self.last_prediction}, '
            f'Confidence: {self.last_confidence:.1f}%)'
        )
        
        # Clear history to avoid repeated commands
        self.prediction_history.clear()


def main(args=None):
    rclpy.init(args=args)
    node = RobotCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()