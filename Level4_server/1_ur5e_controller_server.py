#!/usr/bin/env python3

from robodk.robolink import *
from robodk.robomath import *

import socket
import threading
import yaml
import math
import time
import os


# ============================================================
# CONFIGURATION
# ============================================================

SERVER_IP = "0.0.0.0"
SERVER_PORT = 5000

ROBOT_IP = "192.168.1.4"
ROBOT_PORT = 30002

RDK_FILE = "src/roboDK/Social_UR5e.rdk"

MAX_YAML_SIZE_BYTES = 20000

robot_lock = threading.Lock()


# ============================================================
# ROBOT CONTROLLER
# ============================================================

class RobotController:

    def __init__(self):
        self.robot_socket = None
        self.real_robot_connected = False

        print("Loading RoboDK...")
        self.rdk = Robolink()
        time.sleep(2)

        self.rdk.AddFile(os.path.abspath(RDK_FILE))
        time.sleep(2)

        self.robot = self.rdk.Item("UR5e")
        self.base = self.rdk.Item("UR5e Base")
        self.tool = self.rdk.Item("Hand")

        self.robot.setPoseFrame(self.base)
        self.robot.setPoseTool(self.tool)

        self.connect_robot()
        self.set_tcp_from_robodk()

    def connect_robot(self):
        try:
            self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.robot_socket.settimeout(2.0)
            self.robot_socket.connect((ROBOT_IP, ROBOT_PORT))
            self.real_robot_connected = True
            print(f"Connected to UR5e at {ROBOT_IP}:{ROBOT_PORT}")
        except Exception as e:
            print(f"Robot connection failed: {e}")
            self.robot_socket = None
            self.real_robot_connected = False

    def send_script(self, script, wait_time=0.0):
        if not self.real_robot_connected:
            print("Robot not connected")
            return False

        try:
            self.robot_socket.sendall((script.strip() + "\n").encode("utf-8"))
            print("[URSCRIPT]", script)

            if wait_time > 0:
                time.sleep(wait_time)

            return True

        except Exception as e:
            print(f"Error sending URScript: {e}")
            self.real_robot_connected = False
            return False

    def set_tcp_from_robodk(self):
        x, y, z, rx, ry, rz = Pose_2_UR(self.robot.PoseTool())

        script = (
            f"set_tcp(p[{x/1000.0:.6f},{y/1000.0:.6f},{z/1000.0:.6f},"
            f"{rx:.6f},{ry:.6f},{rz:.6f}])"
        )

        self.send_script(script, wait_time=1.0)

    def movej(self, joints_deg, a=1.2, v=0.5, t=-1, r=0.0):
        joints_rad = [math.radians(q) for q in joints_deg]

        if t is not None and t >= 0:
            script = (
                f"movej([{joints_rad[0]:.6f},{joints_rad[1]:.6f},{joints_rad[2]:.6f},"
                f"{joints_rad[3]:.6f},{joints_rad[4]:.6f},{joints_rad[5]:.6f}], "
                f"a={a}, v={v}, t={t}, r={r})"
            )
            wait_time = t
        else:
            script = (
                f"movej([{joints_rad[0]:.6f},{joints_rad[1]:.6f},{joints_rad[2]:.6f},"
                f"{joints_rad[3]:.6f},{joints_rad[4]:.6f},{joints_rad[5]:.6f}], "
                f"a={a}, v={v}, r={r})"
            )
            wait_time = 0.0

        return self.send_script(script, wait_time=wait_time)

    def movel_pose(self, xyz_mm, rpy_deg, a=1.2, v=0.15, t=-1, r=0.0):
        x, y, z = xyz_mm
        roll, pitch, yaw = rpy_deg

        target_pose = transl(x, y, z) * rotx(math.radians(roll)) * roty(math.radians(pitch)) * rotz(math.radians(yaw))

        x, y, z, rx, ry, rz = Pose_2_UR(target_pose)

        if t is not None and t >= 0:
            script = (
                f"movel(p[{x/1000.0:.6f},{y/1000.0:.6f},{z/1000.0:.6f},"
                f"{rx:.6f},{ry:.6f},{rz:.6f}], "
                f"a={a}, v={v}, t={t}, r={r})"
            )
            wait_time = t
        else:
            script = (
                f"movel(p[{x/1000.0:.6f},{y/1000.0:.6f},{z/1000.0:.6f},"
                f"{rx:.6f},{ry:.6f},{rz:.6f}], "
                f"a={a}, v={v}, r={r})"
            )
            wait_time = 0.0

        return self.send_script(script, wait_time=wait_time)

    def execute_sequence(self, data):
        print("\nExecuting sequence:", data.get("sequence_name", "unnamed"))

        for step in data["steps"]:
            name = step.get("name", "unnamed_step")
            motion = step["motion"]

            a = step.get("acceleration", 1.2)
            v = step.get("velocity", 0.25)
            t = step.get("time", -1)
            r = step.get("blend", 0.0)

            print(f"\nStep: {name}")
            print(f"Motion: {motion}")

            if motion == "moveJ":
                self.movej(
                    joints_deg=step["joints_deg"],
                    a=a,
                    v=v,
                    t=t,
                    r=r
                )

            elif motion == "moveL":
                self.movel_pose(
                    xyz_mm=step["target_xyz_mm"],
                    rpy_deg=step["target_rpy_deg"],
                    a=a,
                    v=v,
                    t=t,
                    r=r
                )

            else:
                print(f"Unknown motion type: {motion}")

    def shutdown(self):
        if self.robot_socket:
            self.robot_socket.close()

        try:
            self.rdk.CloseRoboDK()
        except:
            pass

# ============================================================
# TCP SERVER
# ============================================================

def receive_all(conn):
    chunks = []
    total_size = 0

    while True:
        chunk = conn.recv(4096)

        if not chunk:
            break

        chunks.append(chunk)
        total_size += len(chunk)

        if total_size > MAX_YAML_SIZE_BYTES:
            raise RuntimeError("YAML file too large")

    return b"".join(chunks).decode("utf-8")


def handle_client(conn, addr, robot):
    print(f"\nConnection from {addr}")

    try:
        yaml_text = receive_all(conn)

        print("\nReceived YAML:")
        print(yaml_text)

        data = yaml.safe_load(yaml_text)

        if not robot_lock.acquire(blocking=False):
            response = "ERROR: Robot is busy. Try again later.\n"
        else:
            try:
                robot.execute_sequence(data)
                response = "OK: sequence executed\n"
            finally:
                robot_lock.release()

    except Exception as e:
        response = f"ERROR: {e}\n"
        print(response)

    conn.sendall(response.encode("utf-8"))
    conn.close()


def main():
    robot = RobotController()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(5)

    print(f"\nUR5e classroom server listening on {SERVER_IP}:{SERVER_PORT}")

    try:
        while True:
            conn, addr = server_socket.accept()

            thread = threading.Thread(
                target=handle_client,
                args=(conn, addr, robot),
                daemon=True
            )

            thread.start()

    except KeyboardInterrupt:
        print("\nStopping server...")

    finally:
        server_socket.close()
        robot.shutdown()


if __name__ == "__main__":
    main()