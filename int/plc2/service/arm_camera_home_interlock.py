from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum


# Physical collision envelope while any CAN arm/camera axis is not confirmed HOME.
# This is intentionally a code-level ceiling: configuration may tighten it, but
# must never raise it above 695 mm.
ARM_CAMERA_NOT_HOME_MAXIMUM_HEIGHT_MM = 695.0


class ArmCameraHomeState(str, Enum):
    UNKNOWN = "unknown"
    NOT_HOME = "not_home"
    HOME_CONFIRMED = "home_confirmed"


@dataclass(frozen=True)
class ArmCameraHomeSnapshot:
    state: ArmCameraHomeState
    reason: str

    @property
    def all_home_confirmed(self) -> bool:
        return self.state is ArmCameraHomeState.HOME_CONFIRMED

    def as_dict(self) -> dict[str, str | bool]:
        return {
            "state": self.state.value,
            "reason": self.reason,
            "all_home_confirmed": self.all_home_confirmed,
        }


class ArmCameraHomeInterlock:
    """Fail-safe shared state for the lift collision envelope.

    Process startup deliberately begins as UNKNOWN.  Heights above the
    arm/camera-not-home limit are allowed only after the arm subprocess has
    read back and confirmed every CAN motor at the configured HOME angles.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state = ArmCameraHomeState.UNKNOWN
        self._reason = "程式啟動後尚未確認手臂與相機皆在 HOME"

    @property
    def snapshot(self) -> ArmCameraHomeSnapshot:
        with self._lock:
            return ArmCameraHomeSnapshot(self._state, self._reason)

    @property
    def all_home_confirmed(self) -> bool:
        return self.snapshot.all_home_confirmed

    def mark_not_home(self, reason: str) -> None:
        self._set(ArmCameraHomeState.NOT_HOME, reason)

    def mark_home_confirmed(self, reason: str) -> None:
        self._set(ArmCameraHomeState.HOME_CONFIRMED, reason)

    def mark_unknown(self, reason: str) -> None:
        self._set(ArmCameraHomeState.UNKNOWN, reason)

    def _set(self, state: ArmCameraHomeState, reason: str) -> None:
        with self._lock:
            self._state = state
            self._reason = reason


ARM_CAMERA_HOME_INTERLOCK = ArmCameraHomeInterlock()
