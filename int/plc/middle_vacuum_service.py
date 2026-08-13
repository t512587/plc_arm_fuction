from __future__ import annotations

import threading
from enum import Enum
from typing import Any, Protocol

try:
    from service_plc import PLCConnectionError
except ModuleNotFoundError:  # Support importing as plc.middle_vacuum_service.
    from plc.service_plc import PLCConnectionError


class PointServiceProtocol(Protocol):
    def read_point(self, point_id: str) -> float | int | bool: ...
    def write_point(self, point_id: str, value: Any) -> None: ...


class MiddleVacuumMode(str, Enum):
    OFF = "off"
    VACUUM = "vacuum"
    BREAK_VACUUM = "break_vacuum"


class MiddleVacuumService:
    VACUUM_POINT = "M_VAC_ON"
    BREAK_VACUUM_POINT = "M_VAC_REL"

    def __init__(self, plc_service: PointServiceProtocol) -> None:
        self.plc_service = plc_service
        self._lock = threading.RLock()

    def read_state(self) -> dict[str, bool | str]:
        with self._lock:
            vacuum_on = bool(self.plc_service.read_point(self.VACUUM_POINT))
            break_vacuum_on = bool(self.plc_service.read_point(self.BREAK_VACUUM_POINT))
        if vacuum_on and break_vacuum_on:
            mode = "invalid"
        elif vacuum_on:
            mode = MiddleVacuumMode.VACUUM.value
        elif break_vacuum_on:
            mode = MiddleVacuumMode.BREAK_VACUUM.value
        else:
            mode = MiddleVacuumMode.OFF.value
        return {
            "mode": mode,
            "vacuum_on": vacuum_on,
            "break_vacuum_on": break_vacuum_on,
        }

    def set_mode(self, mode: MiddleVacuumMode | str) -> dict[str, bool | str]:
        try:
            selected = mode if isinstance(mode, MiddleVacuumMode) else MiddleVacuumMode(str(mode))
        except ValueError as exc:
            raise PLCConnectionError(f"不支援的中間真空模式: {mode}") from exc

        with self._lock:
            if selected is MiddleVacuumMode.VACUUM:
                self.plc_service.write_point(self.BREAK_VACUUM_POINT, False)
                self.plc_service.write_point(self.VACUUM_POINT, True)
            elif selected is MiddleVacuumMode.BREAK_VACUUM:
                self.plc_service.write_point(self.VACUUM_POINT, False)
                self.plc_service.write_point(self.BREAK_VACUUM_POINT, True)
            else:
                self.plc_service.write_point(self.VACUUM_POINT, False)
                self.plc_service.write_point(self.BREAK_VACUUM_POINT, False)

            state = self.read_state()
            if state["mode"] != selected.value:
                raise PLCConnectionError(
                    f"中間真空讀回不符：要求={selected.value}，讀回={state['mode']}"
                )
            return state
