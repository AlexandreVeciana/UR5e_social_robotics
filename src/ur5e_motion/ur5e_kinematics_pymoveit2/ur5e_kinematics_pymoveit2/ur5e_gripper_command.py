#!/usr/bin/env python3

import socket
import rclpy
from rclpy.node import Node


class UR5eGripperCommand(Node):
    def __init__(self):
        super().__init__("ur5e_gripper_command")

        self.declare_parameter("command", "OPEN")
        self.declare_parameter("server_host", "0.0.0.0")
        self.declare_parameter("server_port", 50001)
        self.declare_parameter("accept_timeout_sec", 20.0)
        self.declare_parameter("reply_timeout_sec", 10.0)

        self.command = str(self.get_parameter("command").value).strip().upper()
        self.server_host = str(self.get_parameter("server_host").value)
        self.server_port = int(self.get_parameter("server_port").value)
        self.accept_timeout = float(self.get_parameter("accept_timeout_sec").value)
        self.reply_timeout = float(self.get_parameter("reply_timeout_sec").value)

        if self.command not in ["OPEN", "CLOSE"]:
            self.get_logger().error(
                f"Invalid command='{self.command}'. Use OPEN or CLOSE."
            )
            raise ValueError("Invalid gripper command")

        self.expected_reply = "DONE_OPEN" if self.command == "OPEN" else "DONE_CLOSE"

        self._done = False
        self.create_timer(0.1, self._run_once)

    def _recv_line(self, conn: socket.socket, timeout: float) -> str:
        conn.settimeout(timeout)
        data = b""
        while True:
            chunk = conn.recv(1024)
            if not chunk:
                break
            data += chunk
            if b"\n" in chunk:
                break
        return data.decode(errors="ignore").strip()

    def _run_once(self):
        if self._done:
            return
        self._done = True

        server = None
        conn = None

        try:
            self.get_logger().info(
                f"Starting TCP server on {self.server_host}:{self.server_port}"
            )

            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.server_host, self.server_port))
            server.listen(1)
            server.settimeout(self.accept_timeout)

            self.get_logger().info("Waiting for robot connection...")
            conn, addr = server.accept()
            self.get_logger().info(f"Robot connected from {addr}")

            hello = self._recv_line(conn, timeout=self.reply_timeout)
            if hello:
                self.get_logger().info(f"Robot says: {hello}")
            else:
                self.get_logger().warn("No initial message received from robot.")

            msg = self.command + "\n"
            self.get_logger().info(f"Sending command: {self.command}")
            conn.sendall(msg.encode())

            reply = self._recv_line(conn, timeout=self.reply_timeout)
            self.get_logger().info(f"Robot reply: {reply}")

            if reply != self.expected_reply:
                self.get_logger().error(
                    f"Unexpected robot reply. Expected '{self.expected_reply}', got '{reply}'"
                )
                raise RuntimeError("Unexpected robot reply")

            self.get_logger().info("Gripper command completed successfully.")
            rclpy.shutdown()

        except socket.timeout as e:
            self.get_logger().error(f"Socket timeout: {e}")
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")
            rclpy.shutdown()

        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass
            if server is not None:
                try:
                    server.close()
                except Exception:
                    pass


def main():
    rclpy.init()
    node = UR5eGripperCommand()
    rclpy.spin(node)


if __name__ == "__main__":
    main()