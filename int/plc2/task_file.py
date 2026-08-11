from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


TASK_TYPES = {"pallet", "vision_transfer", "home", "wait", "confirm", "arm_pose"}
SLOTS = {"Y1", "Y2", "NONE"}
ACTIONS = {"suck", "push", "none"}
DIRECTIONS = {"Y1_TO_Y2", "Y2_TO_Y1"}
HOME_TARGETS = {"plc", "arm", "camera", "all"}


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
    if step.type == "pallet":
        _validate_pallet(step)
    elif step.type == "vision_transfer":
        direction = _required(step, "transfer_direction").upper()
        if direction not in DIRECTIONS:
            raise TaskFileError(
                f"step {step.index}: transfer_direction must be Y1_TO_Y2 or Y2_TO_Y1"
            )
        step.values["transfer_direction"] = direction
    elif step.type == "home":
        target = _required(step, "target").casefold()
        if target not in HOME_TARGETS:
            raise TaskFileError(
                f"step {step.index}: home target must be plc, arm, camera, or all"
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
        if pose != "STANDBY":
            raise TaskFileError(f"step {step.index}: arm_pose currently supports only STANDBY")
        step.values["pose"] = pose


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


def _normalize_action(value: str) -> str:
    normalized = value.strip().casefold()
    aliases = {
        "吸": "suck",
        "推": "push",
        "無": "none",
        "跳過": "none",
    }
    return aliases.get(normalized, normalized)
