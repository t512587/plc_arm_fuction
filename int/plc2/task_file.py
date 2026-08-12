from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path


TASK_TYPES = {
    "control_settings",
    "pallet",
    "vision_transfer",
    "home",
    "wait",
    "confirm",
    "arm_pose",
    "amr_connect",
    "amr_setobs",
    "amr_read_map",
    "amr_goto",
    "amr_disconnect",
}
SLOTS = {"Y1", "Y2", "NONE"}
ACTIONS = {"suck", "push", "none"}
DIRECTIONS = {"Y1_TO_Y2", "Y2_TO_Y1"}
HOME_TARGETS = {"plc", "arm", "camera"}
ARM_POSES = {"HOME", "STANDBY"}
MAX_VISION_TRANSFER_REPEAT = 100


class TaskFileError(ValueError):
    pass


@dataclass(frozen=True)
class TaskStep:
    index: int
    type: str
    values: dict[str, str]

    def get(self, key: str, default: str | None = None) -> str | None:
        return self.values.get(key, default)


def parse_task_file(path: str | Path) -> list[TaskStep]:
    return parse_task_text(Path(path).read_text(encoding="utf-8-sig"))


def parse_task_text(text: str) -> list[TaskStep]:
    raw_steps: list[dict[str, str]] = []
    current: dict[str, str] | None = None

    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.casefold() == "step":
            if current is not None:
                raw_steps.append(current)
            current = {}
            continue
        if current is None:
            raise TaskFileError(f"line {line_number}: expected 'step' before key/value")
        if "=" not in line:
            raise TaskFileError(f"line {line_number}: expected key=value")
        key, value = line.split("=", 1)
        key = key.strip().casefold()
        value = value.strip()
        if not key:
            raise TaskFileError(f"line {line_number}: empty key")
        if key in current:
            raise TaskFileError(f"line {line_number}: duplicate key '{key}'")
        current[key] = value

    if current is not None:
        raw_steps.append(current)
    if not raw_steps:
        raise TaskFileError("task file has no steps")

    steps = [
        TaskStep(index=index, type=_require_type(raw, index), values=raw)
        for index, raw in enumerate(raw_steps, start=1)
    ]
    for step in steps:
        _validate_step(step)
    settings_steps = [step for step in steps if step.type == "control_settings"]
    if len(settings_steps) > 1:
        raise TaskFileError("control_settings may appear only once")
    if settings_steps and settings_steps[0].index != 1:
        raise TaskFileError("control_settings must be the first step")
    return steps


def _require_type(raw: dict[str, str], index: int) -> str:
    value = raw.get("type")
    if value is None:
        raise TaskFileError(f"step {index}: missing type")
    task_type = value.strip().casefold()
    if task_type not in TASK_TYPES:
        raise TaskFileError(
            f"step {index}: unsupported type '{value}'. "
            f"Use one of: {', '.join(sorted(TASK_TYPES))}"
        )
    raw["type"] = task_type
    return task_type


def _validate_step(step: TaskStep) -> None:
    if step.type == "control_settings":
        _validate_control_settings(step)
    elif step.type == "pallet":
        _validate_pallet(step)
    elif step.type == "vision_transfer":
        direction = _required(step, "transfer_direction").upper()
        if direction not in DIRECTIONS:
            raise TaskFileError(
                f"step {step.index}: transfer_direction must be Y1_TO_Y2 or Y2_TO_Y1"
            )
        step.values["transfer_direction"] = direction
        repeat = _positive_integer(
            step,
            "repeat",
            default=1,
            maximum=MAX_VISION_TRANSFER_REPEAT,
        )
        step.values["repeat"] = str(repeat)
    elif step.type == "home":
        target = _required(step, "target").casefold()
        if target not in HOME_TARGETS:
            raise TaskFileError(
                f"step {step.index}: home target must be plc, arm, or camera; "
                "use type=arm_pose with pose=HOME for synchronized four-axis HOME"
            )
        step.values["target"] = target
    elif step.type == "wait":
        seconds = _number(step, "seconds")
        if seconds < 0:
            raise TaskFileError(f"step {step.index}: seconds must be >= 0")
    elif step.type == "confirm":
        step.values.setdefault("message", f"Confirm step {step.index}")
    elif step.type == "arm_pose":
        pose = _required(step, "pose").upper()
        if pose not in ARM_POSES:
            raise TaskFileError(
                f"step {step.index}: arm_pose must be HOME or STANDBY"
            )
        step.values["pose"] = pose
    elif step.type in {"amr_connect", "amr_disconnect"}:
        _reject_unknown_keys(step, {"type"})
    elif step.type == "amr_setobs":
        _reject_unknown_keys(step, {"type", "precision_xy", "precision_yaw"})
        for key in ("precision_xy", "precision_yaw"):
            value = _number(step, key)
            if not math.isfinite(value) or value <= 0:
                raise TaskFileError(
                    f"step {step.index}: {key} must be greater than 0"
                )
            step.values[key] = f"{value:g}"
    elif step.type == "amr_read_map":
        _reject_unknown_keys(step, {"type", "map_name"})
        step.values["map_name"] = _required(step, "map_name")
    elif step.type == "amr_goto":
        _reject_unknown_keys(
            step,
            {
                "type", "position_id", "response_timeout_seconds",
                "arrival_xy", "hold_seconds", "poll_interval",
            },
        )
        step.values["position_id"] = _required(step, "position_id")
        timeout = _number(step, "response_timeout_seconds")
        if not math.isfinite(timeout) or timeout <= 0 or timeout > 3600:
            raise TaskFileError(
                f"step {step.index}: response_timeout_seconds must be between 0 and 3600"
            )
        step.values["response_timeout_seconds"] = f"{timeout:g}"
        step.values.setdefault("arrival_xy", "0.10")
        step.values.setdefault("hold_seconds", "0.8")
        step.values.setdefault("poll_interval", "0.35")
        arrival_xy = _number(step, "arrival_xy")
        if not math.isfinite(arrival_xy) or arrival_xy <= 0:
            raise TaskFileError(f"step {step.index}: arrival_xy must be greater than 0")
        hold_seconds = _number(step, "hold_seconds")
        if not math.isfinite(hold_seconds) or hold_seconds < 0:
            raise TaskFileError(f"step {step.index}: hold_seconds must be >= 0")
        poll_interval = _number(step, "poll_interval")
        if not math.isfinite(poll_interval) or poll_interval <= 0:
            raise TaskFileError(f"step {step.index}: poll_interval must be greater than 0")
        step.values["arrival_xy"] = f"{arrival_xy:g}"
        step.values["hold_seconds"] = f"{hold_seconds:g}"
        step.values["poll_interval"] = f"{poll_interval:g}"


def _reject_unknown_keys(step: TaskStep, allowed: set[str]) -> None:
    unknown = sorted(set(step.values) - allowed)
    if unknown:
        raise TaskFileError(
            f"step {step.index}: unsupported field(s): {', '.join(unknown)}"
        )


def _validate_control_settings(step: TaskStep) -> None:
    allowed = {"type", "x_speed", "y1_speed", "height_reference_depth_mm"}
    unknown = sorted(set(step.values) - allowed)
    if unknown:
        raise TaskFileError(
            f"step {step.index}: unsupported control setting(s): {', '.join(unknown)}"
        )
    for key in ("x_speed", "y1_speed"):
        value = _whole_number(step, key)
        if value < 1 or value > 32767:
            raise TaskFileError(f"step {step.index}: {key} must be between 1 and 32767")
        step.values[key] = str(value)
    reference = _number(step, "height_reference_depth_mm")
    if not math.isfinite(reference) or reference <= 0:
        raise TaskFileError(
            f"step {step.index}: height_reference_depth_mm must be greater than 0"
        )
    step.values["height_reference_depth_mm"] = f"{reference:g}"


def _validate_pallet(step: TaskStep) -> None:
    slot = _required(step, "slot").upper()
    action = _normalize_action(_required(step, "action"))
    if slot not in SLOTS:
        raise TaskFileError(f"step {step.index}: slot must be Y1, Y2, or none")
    if action not in ACTIONS:
        raise TaskFileError(f"step {step.index}: action must be suck, push, or none")
    step.values["slot"] = "none" if slot == "NONE" else slot
    step.values["action"] = action
    _number(step, "height_mm")
    if slot == "NONE" or action == "none":
        if slot != "NONE" or action != "none":
            raise TaskFileError(
                f"step {step.index}: slot and action must both be none to pause"
            )
        return
    _number(step, "forward_mm")


def _required(step: TaskStep, key: str) -> str:
    value = step.values.get(key)
    if value is None or value.strip() == "":
        raise TaskFileError(f"step {step.index}: missing {key}")
    return value.strip()


def _number(step: TaskStep, key: str) -> float:
    value = _required(step, key)
    try:
        return float(value)
    except ValueError as exc:
        raise TaskFileError(f"step {step.index}: {key} must be a number") from exc


def _positive_integer(
    step: TaskStep,
    key: str,
    *,
    default: int,
    maximum: int,
) -> int:
    raw_value = step.values.get(key)
    if raw_value is None or raw_value.strip() == "":
        return default
    try:
        value = int(raw_value.strip())
    except ValueError as exc:
        raise TaskFileError(
            f"step {step.index}: {key} must be a whole number"
        ) from exc
    if value < 1 or value > maximum:
        raise TaskFileError(
            f"step {step.index}: {key} must be between 1 and {maximum}"
        )
    return value


def _whole_number(step: TaskStep, key: str) -> int:
    raw_value = _required(step, key)
    try:
        return int(raw_value)
    except ValueError as exc:
        raise TaskFileError(f"step {step.index}: {key} must be a whole number") from exc


def _normalize_action(value: str) -> str:
    normalized = value.strip().casefold()
    aliases = {
        "吸": "suck",
        "推": "push",
        "無": "none",
        "跳過": "none",
    }
    return aliases.get(normalized, normalized)
