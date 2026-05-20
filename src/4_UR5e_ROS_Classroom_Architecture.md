# UR5e ROS 2 Classroom Architecture

## Overview

This architecture is designed for a robotics laboratory or classroom environment where:

* One Ubuntu 22.04 teacher PC is directly connected to the UR5e robot via Ethernet.
* Multiple student PCs connect through a local WiFi network.
* Only the teacher PC communicates directly with the UR5e driver and MoveIt.
* Student PCs send high-level motion requests through ROS 2 services.

This architecture improves:

* Stability
* Safety
* Real-time communication reliability
* Classroom scalability
* Network robustness

---

# Recommended Network Topology

```text
                 WiFi Local Network

     Student PC 1
            \
     Student PC 2
              \
     Student PC 3  --->  Teacher PC  --->  Ethernet  --->  UR5e
              /
     Student PC 4
            /
     Student PC 5
```

---

# Why This Architecture?

The UR ROS 2 driver uses:

* RTDE communication
* Real-time trajectory execution
* Low-latency feedback loops
* MoveIt planning and execution

WiFi communication may introduce:

* Packet jitter
* Latency spikes
* Temporary disconnections
* DDS discovery instability

For this reason:

```text
The UR5e should be connected via Ethernet to a dedicated control PC.
```

Student PCs should NOT directly run:

* ur_robot_driver
* MoveIt
* Trajectory controllers
* RTDE communication

Instead, students should only send:

* High-level motion requests
* Poses
* Sequences
* Joint targets

---

# Teacher PC Responsibilities

The teacher PC is the central robot control server.

It executes:

## 1. UR Driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.1.4 \
    launch_rviz:=false
```

## 2. MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    launch_rviz:=true
```

## 3. Motion Server Nodes

Example:

```bash
ros2 run ur5e_motion_server ur5e_sequence_server
```

---

# Teach Pendant Configuration

The UR5e Teach Pendant External Control program must use:

```text
Teacher PC Ethernet IP address
```

Example:

```text
192.168.1.10
```

NOT the student PC IP addresses.

---

# Ubuntu Low-Latency Kernel Recommendation

For UR5e real-time communication with ROS 2, it is strongly recommended to use the Ubuntu low-latency kernel on all Ubuntu PCs involved in robot control.

This is especially important for:

* UR RTDE communication
* MoveIt trajectory execution
* Joint trajectory controllers
* Reduced scheduling jitter
* Improved ROS 2 timing stability

The standard Ubuntu generic kernel is optimized for general desktop usage.

The low-latency kernel improves:

* Thread scheduling responsiveness
* Real-time communication behavior
* Controller timing consistency
* Motion execution reliability

This is particularly useful when working with:

* UR robots
* ROS 2
* MoveIt
* Real-time trajectory execution
* Multi-node robotics systems

---

# Recommended Setup

## Teacher PC

The teacher PC SHOULD use:

```text
Ubuntu 22.04 + linux-lowlatency
```

because it executes:

* ur_robot_driver
* MoveIt
* RTDE communication
* Trajectory controllers
* Motion execution servers

---

## Student PCs

Student PCs can also use the low-latency kernel.

This improves:

* ROS 2 communication stability
* DDS timing
* Motion request responsiveness

Although it is less critical than on the teacher PC.

---

# Low-Latency Kernel Installation

Install the Ubuntu low-latency kernel:

```bash
sudo apt update
sudo apt install linux-lowlatency
sudo reboot
```

After reboot, verify the active kernel:

```bash
uname -r
```

Expected output example:

```text
6.8.0-71-lowlatency
```

---

# Real-Time Scheduling Permissions

It is also recommended to configure real-time scheduling permissions.

Create realtime group:

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
```

Create configuration file:

```bash
sudo nano /etc/security/limits.d/99-realtime.conf
```

Add:

```text
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft priority 99
@realtime hard priority 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
```

Reboot:

```bash
sudo reboot
```

---

# PREEMPT_RT vs Low-Latency

Ubuntu low-latency is generally sufficient for:

* Robotics education
* MoveIt experiments
* UR5e trajectory execution
* Classroom demonstrations
* ROS 2 development

PREEMPT_RT kernels provide stricter real-time behavior but are usually not necessary for classroom environments.

Important:

```text
A stable Ethernet connection to the UR5e is more important than PREEMPT_RT.
```

Recommended priority:

```text
1. Ethernet connection
2. lowlatency kernel
3. realtime permissions
4. PREEMPT_RT only if necessary
```

---

# ROS 2 Configuration

All PCs should use the same ROS_DOMAIN_ID.

Example:

```bash
export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

# Software Architecture

The system is separated into two ROS 2 packages:

```text
ur5e_interfaces
ur5e_motion_server
```

---

# Package: ur5e_interfaces

This package contains only ROS 2 interfaces.

It should contain:

```text
srv/
msg/
action/
```

No executable nodes should be placed in this package.

---

# Example Services

## RunSequence.srv

```text
string sequence_name
---
bool success
string message
```

Used to execute YAML-defined motion sequences.

Example:

```bash
ros2 service call /ur5e/run_sequence \
ur5e_interfaces/srv/RunSequence \
"{sequence_name: 'handshake.yaml'}"
```

---

## Possible Future Services

### RunPose.srv

```text
float64[3] target_xyz_mm
float64[3] target_rpy_deg
bool seed_from_joint_states
float64[6] seed_joints_deg
bool execute
---
bool success
string message
```

Purpose:

* Move the robot to a Cartesian pose
* Perform IK internally on the teacher PC
* Execute motion using MoveIt

---

### RunJoints.srv

```text
float64[6] joint_positions_deg
float64 duration
---
bool success
string message
```

Purpose:

* Direct joint-space motion execution
* Simple educational trajectory testing

---

### RunNamedSequence.srv

```text
string sequence_name
bool loop
---
bool success
string message
```

Purpose:

* Execute predefined reusable behaviors
* Social robotics demonstrations
* Teaching examples

---

# Package: ur5e_motion_server

This package contains executable ROS 2 nodes.

Examples:

```text
ur5e_sequence_server.py
ur5e_pose_server.py
ur5e_joint_server.py
```

These nodes:

* Run only on the teacher PC
* Receive requests from students
* Execute motions locally
* Interact with MoveIt and UR driver

---

# Recommended Execution Flow

## Example: Handshake Sequence

### Student PC

```bash
ros2 service call /ur5e/run_sequence \
ur5e_interfaces/srv/RunSequence \
"{sequence_name: 'handshake.yaml'}"
```

### Teacher PC

```text
ur5e_sequence_server
    ↓
Loads YAML sequence
    ↓
Computes IK
    ↓
Generates trajectories
    ↓
Sends trajectories to UR driver
    ↓
UR5e executes motion
```

---

# YAML Motion Sequences

Example structure:

```yaml
common:
  execute: true
  group_name: ur_manipulator
  ik_link: tool0

steps:
  - name: approach
    target_xyz: [-250, -350, 300]
    target_rpy: [90.0, 0.0, 0.0]

  - name: handshake
    target_xyz: [-250, -350, 200]
    target_rpy: [90.0, 0.0, 0.0]
```

Advantages:

* Human-readable
* Easy for students to modify
* Safe reusable motion definitions
* Simple debugging
* Easy version control with Git

---

# Safety Recommendations

## Recommended

* Only one active motion request at a time
* Add a "busy" flag in motion servers
* Limit maximum velocity and acceleration
* Use approved YAML sequences for students
* Keep the teacher PC as the only robot controller

## Avoid

* Multiple PCs controlling the robot simultaneously
* Running ur_robot_driver on student PCs
* WiFi-only robot connections
* Direct RTDE communication from student PCs

---

# Future Extensions

This architecture scales naturally toward:

* YOLO gesture recognition
* Human-robot interaction
* Voice commands
* GPT-generated behaviors
* Social robotics demonstrations
* Multi-user classroom experiments

Example:

```text
YOLO detects handshake gesture
        ↓
ROS2 service call
        ↓
/run_sequence
        ↓
handshake.yaml
        ↓
UR5e executes motion
```

---

# Summary

This architecture provides:

* Stable UR5e communication
* Safer classroom operation
* Reduced WiFi latency problems
* Centralized robot control
* Scalable ROS 2 design
* Clean separation between interfaces and execution
* Future compatibility with social robotics systems
