## Social Robotics architecture

ROS 2 workspace for social robotics applications with UR5e.

## Packages

- `ur5e_social_motion`: motion execution and pose sequences
- `social_robot_hri`: HRI app, voice, GPT interface, robot controller
- `social_robot_perception`: hand detection and 3D target estimation
- `social_robot_behaviors`: high-level social behaviors

## Workspace structure

```text
UR5e_social_robotics/
└── src/
    ├── ur5e_social_motion/
    ├── social_robot_hri/
    ├── social_robot_perception/
    └── social_robot_behaviors/
```

The detailed structure is:

```text
UR5e_social_robotics/
└── src/
    ├── ur5e_social_motion/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/
    │   │   └── ur5e_social_motion
    │   ├── launch/
    │   ├── config/
    │   └── ur5e_social_motion/
    │       ├── __init__.py
    │       └── ur5e_move_to_pose_exe.py
    │
    ├── social_robot_hri/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/
    │   │   └── social_robot_hri
    │   └── social_robot_hri/
    │       ├── __init__.py
    │       ├── app_main.py
    │       ├── social_robot_app.py
    │       ├── robot_ros2_controller.py
    │       ├── voice_interface.py
    │       ├── face_identifier.py
    │       └── gpt_interpreter.py
    │
    ├── social_robot_perception/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/
    │   │   └── social_robot_perception
    │   ├── launch/
    │   └── social_robot_perception/
    │       ├── __init__.py
    │       ├── hand_detector_yolo.py
    │       ├── hand_pose_3d_estimator.py
    │       └── hand_target_selector.py
    │
    └── social_robot_behaviors/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── social_robot_behaviors
        └── social_robot_behaviors/
            ├── __init__.py
            ├── handshake_behavior.py
            ├── highfive_behavior.py
            └── behavior_manager.py
```

## ur5e_social_motion
This package contains the motion execution node `ur5e_move_to_pose_exe.py` which subscribes to target poses and commands the UR5e robot to move accordingly. It uses MoveIt for motion planning and execution.

The detailed structure is:

```text
ur5e_social_motion/
├── launch/
│   ├── ur5e_pose_sequence.launch.py
│   ├── ur5e_social_init.launch.py
│   ├── ur5e_social_handshake.launch.py
│   └── ur5e_social_highfive.launch.py
├── config/
│   ├── ur5e_social_init.yaml
│   ├── ur5e_social_handshake.yaml
│   └── ur5e_social_highfive.yaml
├── ur5e_social_motion/
│   ├── __init__.py
│   └── ur5e_move_to_pose_exe.py
├── resource/
├── package.xml
├── setup.py
└── setup.cfg
```

## Install pymoveit2

````xml
cd ~/UR5e_social_robotics/src
git clone https://github.com/AndrejOrsula/pymoveit2.git
cd ~/UR5e_social_robotics
rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths src
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
source install/setup.bash
````
````xml
cd ~/UR5e_social_robotics
git rm --cached -r src/pymoveit2
rm -rf src/pymoveit2/.git
git add src/pymoveit2
git commit -m "Vendor pymoveit2 inside workspace"
git push
````

To compile:
````bash
colcon build --merge-install
````

Verification:
````bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=0.0.0.0 use_fake_hardware:=true launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
ros2 launch ur5e_social_motion ur5e_social_init.launch.py
````