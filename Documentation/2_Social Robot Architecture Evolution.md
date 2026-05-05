# Social Robot Architecture Evolution: From Monolithic Script to ROS 2

## Overview

This document explains three progressive software architectures for controlling a UR5e social robot:

1. **Level 1 – Single Python Script**
2. **Level 2 – Modular Python Application**
3. **Level 3 – ROS 2 Distributed Architecture**

The goal is to understand why robotics systems evolve toward ROS 2 when complexity increases.

---

# Level 1 – Single Python Script

## Architecture

All functionalities are implemented inside one file:

```
social_robot.py
```

Typical internal components:

* Robot control (URScript via sockets)
* Voice interface
* Face identification
* Command interpreter
* Application logic

Example structure:

```
social_robot.py
 ├── CONFIG
 ├── RobotController
 ├── VoiceInterface
 ├── ChatGPTInterpreter
 ├── FaceIdentifier
 └── SocialRobotApp
```

## Advantages

* Very easy to understand
* Fast prototyping
* Minimal setup required
* Good for first experiments with robot interaction

## Limitations

* All components tightly coupled
* Difficult to maintain as project grows
* Hard to reuse modules
* No distributed execution
* Custom communication protocol (URScript sockets)

---

# Level 2 – Modular Python Application

## Architecture

The application is split into independent modules.

Suggested structure aligned with ROS 2 package naming:

```
social_robot_app/
 ├── social_robot_hri.py
 ├── social_robot_perception.py
 ├── social_robot_behaviors.py
 ├── ur5e_social_motion.py
 └── main.py
```

Responsibilities:

* **social_robot_hri.py** → voice interface + command interpretation
* **social_robot_perception.py** → face recognition + 3D camera processing
* **social_robot_behaviors.py** → high-level interaction logic
* **ur5e_social_motion.py** → robot motion execution (URScript sockets)
* **main.py** → application entry point

## Advantages

* Clear separation of responsibilities
* Easier debugging
* Improved readability
* Code reuse possible
* Prepares students for ROS-style modular thinking

## Limitations

* Still a single process application
* No standard middleware
* Limited scalability
* Difficult integration with additional robotic subsystems

---

# Level 3 – ROS 2 Architecture

## Architecture

Each subsystem becomes a ROS 2 package and node.

Example workspace:

```
UR5e_social_robotics/
 ├── social_robot_hri/
 ├── social_robot_perception/
 ├── social_robot_behaviors/
 └── ur5e_social_motion/
```

Execution model:

```
Voice Node  ──▶ Behavior Node ──▶ Motion Node
Camera Node ──▶ Behavior Node
```

Robot execution handled through:

* Universal Robots ROS 2 Driver
* MoveIt 2 motion planning

## Advantages

* Distributed architecture
* Standard communication (topics, services, actions)
* Compatible with professional robotics pipelines
* Scalable to perception, navigation, and manipulation
* Hardware abstraction
* Easier integration with external sensors

## Limitations

* Higher initial complexity
* Requires ROS 2 knowledge
* More configuration effort

---

# Academic Motivation for Transition to ROS 2

The transition from a monolithic Python script to ROS 2 reflects a natural evolution in robotics system design.

Key academic reasons:

## Modularity

ROS 2 separates perception, interaction, and motion into independent components.

## Scalability

New capabilities can be added without rewriting the system.

Examples:

* 3D pose detection
* object recognition
* navigation
* multi-robot coordination

## Robustness

ROS 2 drivers manage communication with industrial robots reliably.

MoveIt 2 provides:

* trajectory planning
* collision checking
* controller abstraction

## Standardization

Students learn tools used in research labs and industry.

## Maintainability

Large robotics software projects require structured architectures.

ROS 2 supports long-term evolution of the system.

---

# Laboratory Session Script: Social Interaction with UR5e

## Objective

Implement voice-driven social interaction behaviors using a structured robotics architecture.

Students compare three control approaches and understand advantages of ROS 2 integration.

---

# Experiment 1 – Voice-Controlled Motion

Commands:

* "robot init"
* "robot hand shake"
* "robot give me five"

Pipeline:

```
Voice → Command Interpreter → Behavior Logic → Robot Motion
```

Students observe transition from direct URScript execution to MoveIt 2 execution.

---

# Experiment 2 – Face Identification Interaction

Workflow:

```
Camera → Face Recognition → Greeting Behavior
```

Example:

"Hello Manel"

Robot executes greeting motion.

---

# Experiment 3 – 3D Camera Pose Detection

Workflow:

```
Depth Camera → Hand Pose Detection → Behavior Selection
```

Examples:

* detected handshake pose → execute handshake
* detected raised hand → execute give‑me‑five

Students integrate perception with behavior control.

---

# Experiment 4 – ROS 2 Behavior Coordination

Execution using launch file:

```
ros2 launch ur5e_social_motion ur5e_social_interaction.launch.py
```

Nodes started:

* HRI node
* perception node
* behavior node
* motion node

Students visualize modular execution using:

```
ros2 node list
```

---

# Learning Outcomes

After this lab session students should:

* understand limitations of monolithic robotics scripts
* design modular Python robotics applications
* understand ROS 2 node-based architectures
* execute social interaction behaviors on UR5e
* integrate voice, perception, and motion subsystems

This progression mirrors real-world robotics software development practices.
