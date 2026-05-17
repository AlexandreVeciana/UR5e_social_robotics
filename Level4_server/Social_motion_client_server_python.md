# UR5e Classroom Architecture using Python Client-Server Communication

## Objective

In this first laboratory session, students will control the UR5e robot using a simple Python client-server architecture over the local WiFi network.

Instead of connecting every student computer directly to the robot, all robot commands are sent through the professor computer.

This architecture is simpler, safer, and easier to supervise during class.

---

# General Architecture

```text
Student PC 1 \
Student PC 2  \
Student PC 3   ---> WiFi ---> Professor PC ---> Ethernet ---> UR5e Robot
Student PC n  /
```

---

# Why use this architecture?

## Advantages

### 1. Only one computer controls the real robot

The UR5e receives commands only from the professor computer.

This avoids conflicts when multiple students send commands at the same time.

---

### 2. Better network reliability

The UR5e is connected using Ethernet cable to the professor computer.

Robot communication is more stable than direct WiFi control.

---

### 3. Safer robot operation

The professor computer can supervise all movements before execution.

Future versions may automatically validate workspace limits, speed limits, or collision risks.

---

### 4. Students learn motion programming

Students focus on:

- MoveJ motions
- MoveL motions
- Robot poses
- Joint configurations
- Motion parameters

without directly accessing low-level robot communication.

---

# Communication Flow

## Step 1 — Student creates a YAML motion sequence

Example:

```yaml
sequence_name: handshake_student_01

steps:
  - name: init
    motion: moveJ
    joints_deg: [0, -90, 90, -90, -90, 0]
    acceleration: 1.2
    velocity: 0.5
    time: 4.0

  - name: handshake_pose
    motion: moveL
    target_xyz_mm: [400, -250, 350]
    target_rpy_deg: [180, 0, 90]
    acceleration: 1.2
    velocity: 0.15
    time: 3.0
```

---

## Step 2 — Student sends the YAML file to the professor PC

The student runs:

```bash
python3 send_sequence.py handshake.yaml
```

The Python client sends the YAML file using a TCP socket over WiFi.

---

## Step 3 — Professor server receives the sequence

The professor computer runs:

```bash
python3 ur5e_server.py
```

The server:

- receives the YAML file
- loads the motion sequence
- converts poses to URScript format
- sends URScript commands to the UR5e

---

## Step 4 — UR5e executes the motion

The professor server sends commands such as:

```python
movej([...])
movel(p[...])
```

through a TCP socket to the robot controller.

---

# Motion Types

## moveJ

Joint motion.

The robot moves in joint space.

Example:

```yaml
motion: moveJ
joints_deg: [0, -90, 90, -90, -90, 0]
```

---

## moveL

Linear Cartesian motion.

The TCP moves linearly in space.

Example:

```yaml
motion: moveL
target_xyz_mm: [400, -250, 350]
target_rpy_deg: [180, 0, 90]
```

---

# Motion Parameters

## Velocity

Robot motion speed.

```yaml
velocity: 0.25
```

---

## Acceleration

Robot motion acceleration.

```yaml
acceleration: 1.2
```

---

## Time

Desired execution time.

```yaml
time: 3.0
```

The robot tries to complete the movement in approximately 3 seconds.

---

## Time = -1

If:

```yaml
time: -1
```

the robot ignores the time parameter and uses only:

- velocity
- acceleration

---

# Robot Pose Representation

Students define poses using:

```yaml
target_xyz_mm: [x, y, z]
target_rpy_deg: [roll, pitch, yaw]
```

Internally, the professor server converts these values into URScript angle-axis format using RoboDK:

```python
Pose_2_UR(...)
```

This simplifies robot programming for students.

---

# Technologies Used

## Python

Main programming language.

---

## TCP Sockets

Used for communication between:

- student PC
- professor PC
- UR5e robot

---

## YAML

Used to describe robot motion sequences.

---

## RoboDK

Used to:

- create robot poses
- convert orientations
- simplify URScript generation

---

## URScript

Native scripting language of Universal Robots.

Example:

```python
movej(...)
movel(...)
```

---

# Educational Objectives

Students learn:

- client-server communication
- TCP sockets
- robot motion programming
- Cartesian poses
- joint space motions
- YAML configuration files
- industrial robot control concepts

---

# Future Improvements

Future versions may include:

- automatic safety validation
- workspace limits
- collision checking
- motion simulation
- ROS 2 integration
- MoveIt 2 integration
- graphical interfaces
- multiple robot support

---

# Summary

This architecture allows students to safely control a real UR5e robot through a simple Python network architecture.

Students create motion sequences in YAML files and send them through WiFi to the professor computer, which acts as the central robot controller.

This is a simple and effective first step toward industrial robot programming and networked robotic systems.