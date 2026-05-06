"""
Convenience wrappers — one function per social gesture.

Each function resolves the YAML path from the installed package share
directory and delegates to run_motion_sequence().
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ur5e_social_motion.sequence_runner import run_motion_sequence

PACKAGE_SHARE_DIR = Path(get_package_share_directory("ur5e_social_motion"))
CONFIG_DIR = PACKAGE_SHARE_DIR / "config"


def _run(yaml_name: str, logger=None) -> int:
    return run_motion_sequence(CONFIG_DIR / yaml_name, logger=logger)


# ── Utility ───────────────────────────────────────────────────────────────────

def run_init_sequence(logger=None) -> int:
    """Move to the home/ready pose."""
    return _run("ur5e_social_init.yaml", logger)


def run_idle_sequence(logger=None) -> int:
    """Compact idle pose (used for NoAction)."""
    return _run("ur5e_social_idle.yaml", logger)


# ── Gestures ──────────────────────────────────────────────────────────────────

def run_bow_sequence(logger=None) -> int:
    return _run("ur5e_social_bow.yaml", logger)


def run_cross_sequence(logger=None) -> int:
    return _run("ur5e_social_cross.yaml", logger)


def run_golden_order_sequence(logger=None) -> int:
    return _run("ur5e_social_golden_order.yaml", logger)


def run_half_sun_sequence(logger=None) -> int:
    return _run("ur5e_social_half_sun.yaml", logger)


def run_handshake_sequence(logger=None) -> int:
    return _run("ur5e_social_handshake.yaml", logger)


def run_point_down_sequence(logger=None) -> int:
    return _run("ur5e_social_point_down.yaml", logger)


def run_praise_the_sun_sequence(logger=None) -> int:
    return _run("ur5e_social_praise_the_sun.yaml", logger)


def run_side_leg_sequence(logger=None) -> int:
    return _run("ur5e_social_side_leg.yaml", logger)


def run_stop_sequence(logger=None) -> int:
    return _run("ur5e_social_stop.yaml", logger)


def run_wave_sequence(logger=None) -> int:
    return _run("ur5e_social_wave.yaml", logger)
