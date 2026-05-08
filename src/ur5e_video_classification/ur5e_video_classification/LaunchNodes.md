# Source ROS
source /opt/ros/humble/setup.bash

#  Go to your workspace root (not inside src/)
cd ~/your_ws
colcon build
source install/setup.bash

#  UR5e driver (real hardware)
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.4

# 2. MoveIt2
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true

Important to add on .bashrc an environment variable on Container:

export LD_PRELOAD=/usr/local/lib/librealsense2.so
Verify that the camera is detected in the container

lsusb
rs-enumerate-devices

# 3. CAMERA LAUNCH
ros2 launch realsense2_camera rs_launch.py \
rgb_camera.color_profile:=640x480x15 \
depth_module.depth_profile:=640x360x15 \
pointcloud.enable:=false

# 4. VIDEO CLASSIFICATION NODE
ros2 run ur5e_video_classification video_classification_node \
  --ros-args \
  -p camera_topic:=/camera/color/image_raw \
  -p confidence_threshold:=0.8 \
  -p prediction_rate:=5.0

# 5. ROBOT COMMAND
  ros2 run ur5e_video_classification robot_command_node \
  --ros-args \
  -p confidence_threshold:=0.75 \
  -p stability_frames:=5 \
  -p command_cooldown:=3.0 \
  -p ignore_no_action:=true

# TEST RUN
ros2 run ur5e_video_classification ur5e_move_to_pose_exe \
  --ros-args \
  -p step_name:=home \
  -p target_xyz_mm:="[350.0, 0.0, 500.0]" \
  -p target_rpy_deg:="[90.0, 0.0, 90.0]" \
  -p execute:=false \
  -p print_joints:=true

If IK succeeds and prints joint angles — swap execute:=false to execute:=true at speed 0.05

If that looks right in RViz, start Terminal 3 + 4 + 5 and test the full pipeline

# SIMULATION RUN
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  use_fake_hardware:=true \
  launch_rviz:=true


# USING LAUNCH FILE
ros2 launch ur5e_video_classification social_robot.launch.py \
  robot_ip:

ros2 launch ur5e_video_classification social_robot.launch.py \
  use_fake_hardware:=true

# FULL OVERRIDE
ros2 launch ur5e_video_classification social_robot.launch.py \
  robot_ip:=192.168.1.102 \
  confidence_threshold:=0.8 \
  command_threshold:=0.8 \
  prediction_rate:=5.0 \
  stability_frames:=5 \
  command_cooldown:=3.0 \
  launch_rviz:=true



# FULL SYSTEM TRYOUT 1

# Terminal 1 — UR driver
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.4

# Terminal 2 — MoveIt (wait for "You can start planning now!")
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=false

# Terminal 3 — Camera
ros2 launch realsense2_camera rs_launch.py

# Terminal 4 — Full system
ros2 launch ur5e_video_classification social_robot.launch.py \
  robot_ip:=192.168.1.4