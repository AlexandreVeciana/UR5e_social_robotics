from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from ur5e_social_motion.sequence_runner import run_motion_sequence


PACKAGE_SHARE_DIR = Path(get_package_share_directory("ur5e_social_motion"))
CONFIG_DIR = PACKAGE_SHARE_DIR / "config"


def run_init_sequence(logger=None):
    yaml_file = CONFIG_DIR / "ur5e_social_init.yaml"
    return run_motion_sequence(yaml_file, logger=logger)


def run_handshake_sequence(logger=None):
    yaml_file = CONFIG_DIR / "ur5e_social_handshake.yaml"
    return run_motion_sequence(yaml_file, logger=logger)


def run_highfive_sequence(logger=None):
    yaml_file = CONFIG_DIR / "ur5e_social_highfive.yaml"
    return run_motion_sequence(yaml_file, logger=logger)