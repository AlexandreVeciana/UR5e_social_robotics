import os
import sys

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# RoboDK path (host-mounted or local install)
sys.path.append('/opt/RoboDK/Python')
sys.path.append('/RoboDK/Python')

# ─────────────────────────────────────────
# RoboDK import (safe optional mode)
# ─────────────────────────────────────────
ROBODK_AVAILABLE = False
RDK = None

try:
    from robodk.robolink import Robolink
    from robodk.robomath import *

    try:
        # If RoboDK is running locally or accessible
        RDK = Robolink()
        ROBODK_AVAILABLE = True
        print("[Robot] Connected to RoboDK")
    except Exception as e:
        print(f"[Robot] RoboDK not running: {e}")
        ROBODK_AVAILABLE = False

except ImportError:
    print("[Robot] RoboDK Python API not available")
    ROBODK_AVAILABLE = False

import tkinter as tk
from tkinter import messagebox


# ─────────────────────────────────────────
# RoboDK setup (only if available)
# ─────────────────────────────────────────
if ROBODK_AVAILABLE:
    try:
        relative_path = "src/roboDK/Pick&Place_UR5e_students.rdk"
        absolute_path = os.path.abspath(relative_path)

        RDK.AddFile(absolute_path)

        robot = RDK.Item("UR5e")
        base = RDK.Item("UR5e Base")
        tool = RDK.Item("2FG7")
        init_target = RDK.Item("Init")
        pick_target = RDK.Item("Pick")
        table = RDK.Item("Table")
        cube = RDK.Item("cube")

        cube.setVisible(False)

        robot.setPoseFrame(base)
        robot.setPoseTool(tool)
        robot.setSpeed(20)

        print("[Robot] RoboDK scene loaded")

    except Exception as e:
        print(f"[Robot] RoboDK setup failed: {e}")
        ROBODK_AVAILABLE = False
        robot = None
        init_target = None
        pick_target = None
        cube = None
else:
    robot = None
    init_target = None
    pick_target = None
    cube = None


# ─────────────────────────────────────────
# Robot primitives
# ─────────────────────────────────────────
def move_to_init():
    print("[Robot] move_to_init")
    if ROBODK_AVAILABLE and robot and init_target:
        robot.MoveL(init_target, True)
    else:
        print("[Mock] move_to_init")


def pick_cube():
    print("[Robot] pick_cube")
    if ROBODK_AVAILABLE and robot and pick_target and cube:
        robot.MoveL(pick_target, True)
        cube.setParentStatic(tool)
        print("[Robot] Pick done")
    else:
        print("[Mock] pick_cube")


def wave_response():
    print("[Robot] wave_response (mock or real)")


def stop_response():
    print("[Robot] stop_response")


def idle():
    print("[Robot] idle")


def bow_response():
    print("[Robot] bow_response")


# ─────────────────────────────────────────
# Command dispatcher
# ─────────────────────────────────────────
COMMAND_MAP = {
    "bow_response": bow_response,
    "wave_response": wave_response,
    "stop_response": stop_response,
    "idle": idle,
    "pick_cube": pick_cube,
    "move_to_init": move_to_init,
}


def execute_command(command: str):
    fn = COMMAND_MAP.get(command)
    if fn:
        print(f"[Robot] Executing: {command}")
        fn()
    else:
        print(f"[Robot] Unknown command: {command}")


# ─────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────
class RobotCommandNode(Node):
    def __init__(self):
        super().__init__('robot_command_node')

        self.subscription = self.create_subscription(
            String,
            'robot_command',
            self.callback,
            10
        )

        self.get_logger().info("RobotCommandNode ready, listening on /robot_command")

    def callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"Received: {command}")
        execute_command(command)


# ─────────────────────────────────────────
# Main ROS loop
# ─────────────────────────────────────────
def main():
    rclpy.init()

    node = RobotCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


# ─────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────
if __name__ == "__main__":
    main()