import os
from robodk.robolink import *
from robodk.robomath import *

# ─────────────────────────────────────────
# Setup
# ─────────────────────────────────────────

absolute_path = os.path.expanduser("~/ros2_ws/src/roboDK/robot_gestures.rdk")

RDK = Robolink()
RDK.AddFile(absolute_path)

# Retrieve items
robot = RDK.Item("UR5e")
base  = RDK.Item("UR5e Base")
tool  = RDK.Item("2FG7")

# Retrieve your targets (add more as you create them in RoboDK)
init_target = RDK.Item("Init")
# Example targets — create these in RoboDK and name them accordingly:
# bow_target        = RDK.Item("Bow")
# wave_target       = RDK.Item("Wave")

# Robot configuration
robot.setPoseFrame(base)
robot.setPoseTool(tool)
robot.setSpeed(20)


# ─────────────────────────────────────────
# Shared helper
# ─────────────────────────────────────────

def move_to_init():
    """Return robot to home/init position."""
    print("[Robot] Returning to Init")
    robot.MoveL(init_target, True)
    print("[Robot] Init reached")


# ─────────────────────────────────────────
# Gesture responses
# ─────────────────────────────────────────

def bow_response():
    """
    Response to 'Bow' gesture.
    """
    print("[Robot] Executing: bow_response")
    move_to_init()



    print("[Robot] bow_response complete")


def cross_response():
    """
    Response to 'Cross' gesture.
    """
    print("[Robot] Executing: cross_response")
    move_to_init()


    print("[Robot] cross_response complete")


def golden_order_response():
    """
    Response to 'GoldenOrder' gesture.
    """
    print("[Robot] Executing: golden_order_response")
    move_to_init()


    print("[Robot] golden_order_response complete")


def half_sun_response():
    """
    Response to 'HalfSun' gesture.
    """
    print("[Robot] Executing: half_sun_response")
    move_to_init()


    print("[Robot] half_sun_response complete")


def handshake_response():
    """
    Response to 'Handshake' gesture.
    """
    print("[Robot] Executing: handshake_response")
    move_to_init()


    print("[Robot] handshake_response complete")


def idle():
    """
    Response to 'NoAction' — robot stays still or returns to init.
    """
    print("[Robot] NoAction detected — idle")



def point_down_response():
    """
    Response to 'PointDown' gesture.
    """
    print("[Robot] Executing: point_down_response")
    move_to_init()


    print("[Robot] point_down_response complete")


def praise_response():
    """
    Response to 'PraiseTheSun' gesture.
    """
    print("[Robot] Executing: praise_response")
    move_to_init()


    print("[Robot] praise_response complete")


def side_leg_response():
    """
    Response to 'SideLeg' gesture.
    """
    print("[Robot] Executing: side_leg_response")
    move_to_init()


    print("[Robot] side_leg_response complete")


def stop_response():
    """
    Response to 'Stop' gesture.
    """
    print("[Robot] Executing: stop_response")
    move_to_init()


    print("[Robot] stop_response complete")


def wave_response():
    """
    Response to 'Wave' gesture.
    """
    print("[Robot] Executing: wave_response")
    move_to_init()



    print("[Robot] wave_response complete")


# ─────────────────────────────────────────
# Dispatcher — called by robot_command_node
# ─────────────────────────────────────────

COMMAND_MAP = {
    'bow_response':          bow_response,
    'cross_response':        cross_response,
    'golden_order_response': golden_order_response,
    'half_sun_response':     half_sun_response,
    'handshake_response':    handshake_response,
    'idle':                  idle,
    'point_down_response':   point_down_response,
    'praise_response':       praise_response,
    'side_leg_response':     side_leg_response,
    'stop_response':         stop_response,
    'wave_response':         wave_response,
}

def execute_command(command: str):
    """
    Execute a robot command by name.
    Called from robot_command_node or directly for testing.
    """
    fn = COMMAND_MAP.get(command)
    if fn:
        fn()
    else:
        print(f"[Robot] Unknown command: '{command}' — ignoring")


# ─────────────────────────────────────────
# Manual test — run individual responses
# ─────────────────────────────────────────

if __name__ == "__main__":
    move_to_init()

    # Uncomment to test a specific response:
    # execute_command('wave_response')
    # execute_command('praise_response')
    # execute_command('stop_response')