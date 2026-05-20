#!/usr/bin/env python3

import socket
import sys

PROFESSOR_PC_IP = "192.168.1.100"
PROFESSOR_PC_PORT = 5000


def main():
    if len(sys.argv) != 2:
        print("Usage:")
        print("python3 send_sequence.py handshake.yaml")
        return

    yaml_file = sys.argv[1]

    with open(yaml_file, "r", encoding="utf-8") as f:
        yaml_text = f.read()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((PROFESSOR_PC_IP, PROFESSOR_PC_PORT))

    sock.sendall(yaml_text.encode("utf-8"))
    sock.shutdown(socket.SHUT_WR)

    response = sock.recv(4096).decode("utf-8")
    print(response)

    sock.close()


if __name__ == "__main__":
    main()