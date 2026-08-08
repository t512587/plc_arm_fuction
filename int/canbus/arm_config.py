"""Stable constants: motor IDs, speeds, arm geometry, config loading."""
from __future__ import annotations

import json
from pathlib import Path

from motor_service import MotorConfig


CONFIG_PATH = Path(__file__).with_name("config.json")
POINT_CONFIG_PATH = Path(__file__).with_name("point_config.json")

# --- Motor definitions ---
MOTORS = {
    "ID 142": 2,
    "ID 143": 3,
    "ID 144": 4,
    "ID 145": 5,
}

# --- Speed constants ---
MAX_SPEED_DPS = 0x01F4
HOME_SPEED_DPS = 0x00C8  # slower speed for HOME (200 dps)
STEP_DEGREES = 1.0
MAX_MOVE_DEGREES = 180.0  # safety limit: block if any motor would move more than this

# --- Arm geometry (vacuum pump arm) ---
VACUUM_BIG_ARM_CM = 20.0    # big arm axis -> small arm axis (cm)
VACUUM_SMALL_ARM_CM = 17.5  # small arm axis -> suction nozzle (cm)
CAMERA_TO_NOZZLE_HEIGHT_CM = 60.0  # vertical distance camera -> nozzle (cm)

# --- Motor label shortcuts ---
VACUUM_BIG = "ID 142"
VACUUM_SMALL = "ID 143"
CAMERA_BIG = "ID 145"
CAMERA_SMALL = "ID 144"

# --- Motor angle safety limits (min, max) in degrees ---
# None means no limit. Values are absolute multi-turn angles.
# ID142 big arm: HOME(125.7) ± 90° = [35.7, 215.7] to avoid hitting frame
MOTOR_ANGLE_LIMITS: dict[str, tuple[float, float] | None] = {
    "ID 142": (-84, 96),     # vacuum big arm: HOME(6) ± 90°
    "ID 143": None,             # vacuum small arm: no limit
    "ID 144": None,             # camera small arm: no limit
    "ID 145": (-53.51, 126.49), # camera big arm: HOME(36.49) ± 90°
}


def load_point_config() -> dict[str, dict[str, float]]:
    """Load point_config.json and return {point_name: {motor_label: angle}}."""
    with POINT_CONFIG_PATH.open("r", encoding="utf-8") as file:
        raw = json.load(file)
    points = {}
    for point in raw.get("points", []):
        name = point["name"]
        # Convert keys like "ID142" -> "ID 142" to match MOTORS dict
        angles = {}
        for key, value in point["angles"].items():
            label = f"{key[:2]} {key[2:]}"  # "ID142" -> "ID 142"
            angles[label] = value
        points[name] = angles
    return points


def load_config() -> MotorConfig:
    with CONFIG_PATH.open("r", encoding="utf-8") as file:
        raw = json.load(file)
    return MotorConfig(
        can_interface=raw.get("can_interface", "waveshare_usbcana"),
        channel=raw.get("channel", "COM5"),
        bitrate=int(raw.get("bitrate", 1_000_000)),
        serial_baudrate=int(raw.get("serial_baudrate", 2_000_000)),
        motor_id=int(raw.get("motor_id", 1)),
        timeout_seconds=float(raw.get("timeout_seconds", 1.0)),
        canusb_path=raw.get("canusb_path", "./canusb"),
        canusb_use_sudo=bool(raw.get("canusb_use_sudo", True)),
        can_kwargs=dict(raw.get("can_kwargs", {})),
    )
