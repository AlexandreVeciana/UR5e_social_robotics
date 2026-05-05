# social_robot_gesture

ROS 2 package for gesture classification in the `UR5e_social_robotics` project.

## Responsibility

This package does **not** run YOLO directly.
It receives already detected human keypoints from the perception stack and classifies:
- `none`
- `handshake`
- `highfive`

This separation keeps the architecture clean:
- `social_robot_perception` -> vision / YOLO / body pose
- `social_robot_gesture` -> gesture interpretation
- `social_robot_behaviors` -> robot action selection

## Expected input topic

Topic: `/social_robot_perception/keypoints_json`
Type: `std_msgs/msg/String`

Expected JSON example:

```json
{
  "image_width": 640,
  "image_height": 480,
  "person_id": 0,
  "keypoints": {
    "left_shoulder": {"x": 270.0, "y": 180.0, "conf": 0.90},
    "right_shoulder": {"x": 370.0, "y": 180.0, "conf": 0.92},
    "left_wrist": {"x": 225.0, "y": 240.0, "conf": 0.81},
    "right_wrist": {"x": 435.0, "y": 145.0, "conf": 0.94}
  }
}
```

## Published topics

- `/social_robot_gesture/gesture` -> final stable label
- `/social_robot_gesture/debug` -> debug JSON with raw and filtered gesture

## Build

```bash
cd ~/UR5e_social_robotics
colcon build --packages-select social_robot_gesture
source install/setup.bash
```

## Run

```bash
ros2 launch social_robot_gesture gesture_classifier.launch.py
```
