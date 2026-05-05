"""ROS 2 node for rule-based gesture classification.

This node is intentionally independent from YOLO execution. It subscribes to a
JSON message produced by the perception stack and publishes a stable gesture
label for the rest of the social robot system.
"""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .gesture_rules import GestureRules, GestureThresholds
from .gesture_types import GESTURE_NONE
from .temporal_filter import MajorityVoteFilter


class GestureClassifierNode(Node):
    def __init__(self) -> None:
        super().__init__('gesture_classifier_node')

        self.declare_parameter('input_topic', '/social_robot_perception/keypoints_json')
        self.declare_parameter('output_topic', '/social_robot_gesture/gesture')
        self.declare_parameter('debug_topic', '/social_robot_gesture/debug')
        self.declare_parameter('window_size', 5)
        self.declare_parameter('min_count', 3)
        self.declare_parameter('min_kpt_confidence', 0.4)
        self.declare_parameter('highfive_wrist_above_shoulder_px', 20.0)
        self.declare_parameter('handshake_wrist_below_shoulder_px', 20.0)
        self.declare_parameter('handshake_wrist_to_torso_ratio', 0.6)

        thresholds = GestureThresholds(
            min_kpt_confidence=float(self.get_parameter('min_kpt_confidence').value),
            highfive_wrist_above_shoulder_px=float(
                self.get_parameter('highfive_wrist_above_shoulder_px').value
            ),
            handshake_wrist_below_shoulder_px=float(
                self.get_parameter('handshake_wrist_below_shoulder_px').value
            ),
            handshake_wrist_to_torso_ratio=float(
                self.get_parameter('handshake_wrist_to_torso_ratio').value
            ),
        )
        self.rules = GestureRules(thresholds)
        self.filter = MajorityVoteFilter(
            window_size=int(self.get_parameter('window_size').value),
            min_count=int(self.get_parameter('min_count').value),
        )

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)

        self.subscription = self.create_subscription(
            String,
            input_topic,
            self.keypoints_callback,
            10,
        )
        self.gesture_publisher = self.create_publisher(String, output_topic, 10)
        self.debug_publisher = self.create_publisher(String, debug_topic, 10)

        self.get_logger().info(f'Listening for gesture keypoints on: {input_topic}')
        self.get_logger().info(f'Publishing gesture labels on: {output_topic}')

    def keypoints_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Invalid JSON in keypoints message: {exc}')
            return

        raw_label = self.rules.classify(data)
        stable_label = self.filter.update(raw_label)

        output_msg = String()
        output_msg.data = stable_label
        self.gesture_publisher.publish(output_msg)

        debug_msg = String()
        debug_payload: dict[str, Any] = {
            'raw_gesture': raw_label,
            'stable_gesture': stable_label,
            'person_id': data.get('person_id', -1),
        }
        debug_msg.data = json.dumps(debug_payload)
        self.debug_publisher.publish(debug_msg)

        if stable_label != GESTURE_NONE:
            self.get_logger().info(f'Detected gesture: {stable_label}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GestureClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
