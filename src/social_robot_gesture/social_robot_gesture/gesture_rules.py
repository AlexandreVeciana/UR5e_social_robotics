"""Rule-based gesture classification from 2D body keypoints."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .gesture_types import GESTURE_HANDSHAKE, GESTURE_HIGHFIVE, GESTURE_NONE


@dataclass
class GestureThresholds:
    min_kpt_confidence: float = 0.4
    highfive_wrist_above_shoulder_px: float = 20.0
    handshake_wrist_below_shoulder_px: float = 20.0
    handshake_wrist_to_torso_ratio: float = 0.6
    min_image_width_px: float = 640.0


class GestureRules:
    """Simple heuristics for handshake/high-five classification.

    Expected input format (JSON already decoded into dict):
    {
        "image_width": 640,
        "image_height": 480,
        "person_id": 0,
        "keypoints": {
            "nose": {"x": 320, "y": 120, "conf": 0.9},
            "left_shoulder": {"x": 270, "y": 180, "conf": 0.9},
            "right_shoulder": {"x": 370, "y": 180, "conf": 0.9},
            "left_wrist": {"x": 220, "y": 250, "conf": 0.8},
            "right_wrist": {"x": 430, "y": 145, "conf": 0.9}
        }
    }
    """

    def __init__(self, thresholds: GestureThresholds | None = None) -> None:
        self.thresholds = thresholds or GestureThresholds()

    def classify(self, data: dict[str, Any]) -> str:
        image_width = float(data.get('image_width', self.thresholds.min_image_width_px))
        keypoints = data.get('keypoints', {})

        right_shoulder = keypoints.get('right_shoulder')
        left_shoulder = keypoints.get('left_shoulder')
        right_wrist = keypoints.get('right_wrist')
        left_wrist = keypoints.get('left_wrist')

        right_result = self._classify_arm(right_shoulder, left_shoulder, right_wrist, image_width, side='right')
        left_result = self._classify_arm(left_shoulder, right_shoulder, left_wrist, image_width, side='left')

        if GESTURE_HIGHFIVE in (right_result, left_result):
            return GESTURE_HIGHFIVE
        if GESTURE_HANDSHAKE in (right_result, left_result):
            return GESTURE_HANDSHAKE
        return GESTURE_NONE

    def _classify_arm(
        self,
        shoulder: dict[str, Any] | None,
        other_shoulder: dict[str, Any] | None,
        wrist: dict[str, Any] | None,
        image_width: float,
        side: str,
    ) -> str:
        if not self._is_valid_keypoint(shoulder) or not self._is_valid_keypoint(wrist):
            return GESTURE_NONE

        shoulder_x = float(shoulder['x'])
        shoulder_y = float(shoulder['y'])
        wrist_x = float(wrist['x'])
        wrist_y = float(wrist['y'])

        # High-five: wrist clearly above shoulder.
        if wrist_y < shoulder_y - self.thresholds.highfive_wrist_above_shoulder_px:
            return GESTURE_HIGHFIVE

        # Handshake: wrist roughly around chest level and extended laterally.
        wrist_is_lower = wrist_y > shoulder_y - self.thresholds.handshake_wrist_below_shoulder_px
        torso_reference_x = shoulder_x
        if self._is_valid_keypoint(other_shoulder):
            torso_reference_x = 0.5 * (shoulder_x + float(other_shoulder['x']))

        lateral_distance = abs(wrist_x - torso_reference_x)
        min_distance = self.thresholds.handshake_wrist_to_torso_ratio * max(1.0, image_width / self.thresholds.min_image_width_px) * 80.0

        if wrist_is_lower and lateral_distance > min_distance:
            if side == 'right' and wrist_x > torso_reference_x:
                return GESTURE_HANDSHAKE
            if side == 'left' and wrist_x < torso_reference_x:
                return GESTURE_HANDSHAKE

        return GESTURE_NONE

    def _is_valid_keypoint(self, keypoint: dict[str, Any] | None) -> bool:
        if not keypoint:
            return False
        return float(keypoint.get('conf', 0.0)) >= self.thresholds.min_kpt_confidence
