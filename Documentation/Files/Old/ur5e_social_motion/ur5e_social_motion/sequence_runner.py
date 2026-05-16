from pathlib import Path
import time
import yaml

from ur5e_social_motion.ur5e_move_to_pose_exe import run_pose_motion


def load_sequence_yaml(yaml_file):
    yaml_file = Path(yaml_file)

    if not yaml_file.exists():
        raise FileNotFoundError(f"YAML file not found: {yaml_file}")

    with open(yaml_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None:
        raise ValueError(f"Empty YAML file: {yaml_file}")

    if "sequence" not in data:
        raise ValueError(f"YAML file must contain a top-level 'sequence' key: {yaml_file}")

    sequence = data["sequence"]

    if not isinstance(sequence, list):
        raise ValueError(f"'sequence' must be a list in YAML file: {yaml_file}")

    return sequence


def build_step_config(step):
    config = {
        "step_name": step.get("step_name", "unnamed_step"),
        "target_xyz_mm": step.get("target_xyz_mm", [350.0, 0.0, 450.0]),
        "target_rpy_deg": step.get("target_rpy_deg", [0.0, 0.0, 0.0]),
        "reference_frame": step.get("reference_frame", "table"),
        "table_frame_yaw_offset_deg": step.get("table_frame_yaw_offset_deg", 180.0),
        "group_name": step.get("group_name", "ur_manipulator"),
        "ik_link_name": step.get("ik_link_name", "tool0"),
        "seed_joints_deg": step.get(
            "seed_joints_deg",
            [0.0, -90.0, 90.0, -90.0, -90.0, 0.0],
        ),
        "use_seed_joints": step.get("use_seed_joints", True),
        "seed_from_joint_states": step.get("seed_from_joint_states", True),
        "ik_timeout_sec": step.get("ik_timeout_sec", 0.2),
        "max_velocity_scale": step.get("max_velocity_scale", 0.15),
        "max_acceleration_scale": step.get("max_acceleration_scale", 0.15),
        "execute": step.get("execute", True),
        "print_joints": step.get("print_joints", True),
    }

    return config


def run_motion_sequence(yaml_file, logger=None):
    sequence = load_sequence_yaml(yaml_file)

    if logger is not None:
        logger.info(f"Loaded motion sequence: {yaml_file}")
        logger.info(f"Number of steps: {len(sequence)}")

    for i, step in enumerate(sequence, start=1):
        step_name = step.get("step_name", f"step_{i}")

        if logger is not None:
            logger.info(f"Starting sequence step {i}: {step_name}")

        config = build_step_config(step)
        exit_code = run_pose_motion(config)

        if exit_code != 0:
            if logger is not None:
                logger.error(f"Step failed: {step_name} (exit code {exit_code})")
            return exit_code

        sleep_after = float(step.get("sleep_after", 0.0))
        if sleep_after > 0.0:
            if logger is not None:
                logger.info(f"Sleeping {sleep_after:.2f} s after step: {step_name}")
            time.sleep(sleep_after)

    if logger is not None:
        logger.info("Motion sequence finished successfully")

    return 0