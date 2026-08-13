from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from api import PLCConnectionError, PlcApi, PointDefinition


LOG_FILE = Path(__file__).resolve().parent / "service_plc.log"

logger = logging.getLogger("service_plc")
logger.setLevel(logging.INFO)
logger.propagate = False
if not logger.handlers:
    handler = logging.FileHandler(LOG_FILE, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    logger.addHandler(handler)


STATUS_IDLE = "idle"
STATUS_MOVING = "moving"
STATUS_SUCCESS = "success"
STATUS_TIMEOUT = "timeout"
STATUS_ERROR = "error"


@dataclass(slots=True)
class ActionResult:
    ok: bool
    status: str
    message: str
    started_at: float
    ended_at: float
    data: dict[str, Any] = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return self.ended_at - self.started_at


class ServicePLC:
    """PLC service with manual control functions and UI-friendly state."""

    monitor_points = ("Y1_CUR_POS", "Y2_CUR_POS", "X_CUR_POS")

    def __init__(self, api: PlcApi | None = None) -> None:
        self.api = api or PlcApi()
        self._lock = threading.RLock()
        self._status = STATUS_IDLE
        self._message = "未連線"
        self._logs: list[str] = []
        self._point_aliases = {"X_MOVE": "X_Move"}

    @property
    def status(self) -> str:
        return self._status

    @property
    def message(self) -> str:
        return self._message

    def connect(self, plc_name: str = "main_plc") -> str:
        with self._lock:
            result = self.api.connect(plc_name)
            self._set_status(STATUS_SUCCESS, result)
            self._log(f"connect plc={plc_name} result={result}")
            return result

    def disconnect(self, plc_name: str = "main_plc") -> str:
        with self._lock:
            result = self.api.disconnect(plc_name)
            self._set_status(STATUS_IDLE, result)
            self._log(f"disconnect plc={plc_name} result={result}")
            return result

    def list_groups(self) -> list[str]:
        return self.api.list_groups()

    def list_points(self, group: str | None = None) -> list[PointDefinition]:
        return self.api.list_points(group)

    def get_point(self, point_id: str) -> PointDefinition:
        return self.api.get_point(self._normalize_point_id(point_id))

    def read_point(self, point_id: str) -> int | float | bool:
        point_id = self._normalize_point_id(point_id)
        value = self.api.read_point(point_id)
        self._log(f"read_point point={point_id} value={value}")
        return value

    def write_point(self, point_id: str, value: Any) -> None:
        point_id = self._normalize_point_id(point_id)
        self.api.write_point(point_id, value)
        self._log(f"write_point point={point_id} value={value}")

    def read_monitor_snapshot(self) -> dict[str, Any]:
        snapshot: dict[str, Any] = {}
        for point_id in self.monitor_points:
            try:
                snapshot[point_id] = self.api.read_point(point_id)
            except Exception as exc:
                snapshot[point_id] = f"ERR: {exc}"
        snapshot["service_status"] = self.status
        snapshot["service_message"] = self.message
        snapshot["connected"] = self.api.is_connected("main_plc")
        return snapshot

    def toggle_move(self, point_id: str, enabled: bool) -> None:
        point_id = self._normalize_point_id(point_id)
        self.write_point(point_id, enabled)
        self._set_status(STATUS_SUCCESS, f"{point_id}={'ON' if enabled else 'OFF'}")

    def back_home(self, timeout: float = 60.0) -> ActionResult:
        start = time.monotonic()
        try:
            with self._lock:
                self._set_status(STATUS_MOVING, "backHOME moving")
                self._log("backHOME start")
                for point_id in ("Y1_HOME", "Y2_HOME", "X_HOME"):
                    self.write_point(point_id, True)
                time.sleep(0.5)
                for point_id in ("Y1_HOME", "Y2_HOME", "X_HOME"):
                    self.write_point(point_id, False)
                self._wait_until(
                    timeout=timeout,
                    interval=0.2,
                    predicate=lambda: (
                        self._as_number(self.read_point("Y1_CUR_POS")) == 0
                        and self._as_number(self.read_point("Y2_CUR_POS")) == 0
                        and self._as_number(self.read_point("X_CUR_POS")) == 0
                    ),
                    on_poll=self._check_negative_positions,
                )
                result = ActionResult(True, STATUS_SUCCESS, "success", start, time.monotonic())
                self._set_status(result.status, "backHOME success")
                self._log("backHOME success")
                return result
        except TimeoutError:
            result = ActionResult(False, STATUS_TIMEOUT, "timeout", start, time.monotonic())
            self._set_status(result.status, "backHOME timeout")
            self._log("backHOME timeout")
            return result
        except Exception as exc:
            result = ActionResult(False, STATUS_ERROR, str(exc), start, time.monotonic())
            self._set_status(result.status, f"backHOME error: {exc}")
            self._log(f"backHOME error={exc}")
            return result

    def move_to(self, position: str, action: str, height: int, depth: int, timeout: float = 60.0) -> ActionResult:
        start = time.monotonic()
        position = (position or "none").strip().lower()
        action = (action or "none").strip().lower()
        try:
            with self._lock:
                self._validate_height(height)
                self._validate_depth(depth)
                if position not in {"left", "right", "none"} or action not in {"pull", "push", "none"}:
                    raise ValueError("error command")
                if position == "none" and action == "none":
                    result = self.move_to_x(height, timeout=timeout)
                    return ActionResult(result.ok, result.status, result.message, start, time.monotonic(), result.data)
                if position == "left" and action == "push":
                    raise ValueError("error command")

                self._set_status(STATUS_MOVING, f"moveTo moving position={position} action={action}")
                self._log(f"moveTo start position={position} action={action} height={height} depth={depth}")
                x_result = self.move_to_x(height, timeout=timeout)
                if not x_result.ok:
                    return ActionResult(
                        False,
                        x_result.status,
                        x_result.message,
                        start,
                        time.monotonic(),
                        {"stage": "moveToX", **x_result.data},
                    )

                axis = "Y1" if position == "left" else "Y2"
                forward_point = f"{axis}_FWD_POS"
                move_point = f"{axis}_MOVE"
                current_point = f"{axis}_CUR_POS"
                vac_on = f"{axis}_VAC_ON"
                vac_off = f"{axis}_VAC_OFF"

                self.write_point(forward_point, depth)
                self.write_point(move_point, True)
                self._wait_until(
                    timeout=timeout,
                    interval=0.2,
                    predicate=lambda: self._as_number(self.read_point(current_point)) == depth,
                )
                self.write_point(move_point, False)

                if action == "none":
                    result = ActionResult(True, STATUS_SUCCESS, "success", start, time.monotonic(), {"axis": axis})
                    self._set_status(result.status, "moveTo success")
                    self._log(f"moveTo success axis={axis} action=none")
                    return result

                time.sleep(0.1)
                self.write_point(vac_on, True)
                time.sleep(1.0)
                self.write_point(forward_point, 0)
                self.write_point(move_point, True)
                self._wait_until(
                    timeout=timeout,
                    interval=0.2,
                    predicate=lambda: self._as_number(self.read_point(current_point)) == 0,
                )
                self.write_point(move_point, False)

                if action == "pull":
                    self.write_point(vac_on, False)
                    self.write_point(vac_off, True)
                    time.sleep(0.1)
                    self.write_point(vac_off, False)

                result = ActionResult(
                    True,
                    STATUS_SUCCESS,
                    "success",
                    start,
                    time.monotonic(),
                    {"axis": axis, "action": action},
                )
                self._set_status(result.status, f"moveTo success {axis} {action}")
                self._log(f"moveTo success axis={axis} action={action}")
                return result
        except TimeoutError:
            result = ActionResult(False, STATUS_TIMEOUT, "timeout", start, time.monotonic())
            self._set_status(result.status, "moveTo timeout")
            self._log("moveTo timeout")
            return result
        except Exception as exc:
            result = ActionResult(False, STATUS_ERROR, str(exc), start, time.monotonic())
            self._set_status(result.status, f"moveTo error: {exc}")
            self._log(f"moveTo error={exc}")
            return result

    def vacuum(self, action: bool, timeout: float = 60.0) -> ActionResult:
        start = time.monotonic()
        try:
            with self._lock:
                if action:
                    self.write_point("X_VAC_OFF", False)
                    self.write_point("X_VAC_ON", True)
                    self._set_status(STATUS_SUCCESS, "VAC success")
                    self._log("VAC action=True success")
                    return ActionResult(True, STATUS_SUCCESS, "success", start, time.monotonic())
                self.write_point("X_VAC_ON", False)
                self.write_point("X_VAC_OFF", True)
                time.sleep(0.1)
                self.write_point("X_VAC_OFF", False)
                self._set_status(STATUS_SUCCESS, "VAC success")
                self._log("VAC action=False success")
                return ActionResult(True, STATUS_SUCCESS, "success", start, time.monotonic())
        except Exception as exc:
            if time.monotonic() - start > timeout:
                result = ActionResult(False, STATUS_TIMEOUT, "timeout", start, time.monotonic())
                self._set_status(result.status, "VAC timeout")
                self._log("VAC timeout")
                return result
            result = ActionResult(False, STATUS_ERROR, str(exc), start, time.monotonic())
            self._set_status(result.status, f"VAC error: {exc}")
            self._log(f"VAC error={exc}")
            return result

    def move_to_x(self, height: int, timeout: float = 60.0) -> ActionResult:
        start = time.monotonic()
        try:
            with self._lock:
                self._validate_height(height)
                self.write_point("X_SPEED", 400)
                self.write_point("X_FWD_POS", height)
                self.write_point("X_Move", True)
                self._set_status(STATUS_MOVING, "moving X")
                self._log(f"moveToX start height={height}")
                self._wait_until(
                    timeout=timeout,
                    interval=0.2,
                    predicate=lambda: self._as_number(self.read_point("X_CUR_POS")) == height,
                )
                self.write_point("X_Move", False)
                result = ActionResult(True, STATUS_SUCCESS, "success X", start, time.monotonic(), {"height": height})
                self._set_status(result.status, "moveToX success")
                self._log(f"moveToX success height={height}")
                return result
        except TimeoutError:
            try:
                self.write_point("X_Move", False)
            except Exception:
                pass
            result = ActionResult(False, STATUS_TIMEOUT, "timeout", start, time.monotonic(), {"height": height})
            self._set_status(result.status, "moveToX timeout")
            self._log(f"moveToX timeout height={height}")
            return result
        except Exception as exc:
            try:
                self.write_point("X_Move", False)
            except Exception:
                pass
            result = ActionResult(False, STATUS_ERROR, str(exc), start, time.monotonic(), {"height": height})
            self._set_status(result.status, f"moveToX error: {exc}")
            self._log(f"moveToX error={exc}")
            return result

    def get_logs(self, limit: int = 500) -> list[str]:
        return self._logs[-limit:]

    def _validate_height(self, height: int) -> None:
        if height < 0 or height > 1300:
            raise ValueError("error height out of range")

    def _validate_depth(self, depth: int) -> None:
        if depth < 0 or depth > 400:
            raise ValueError("error depth out of range")

    def _wait_until(self, timeout: float, interval: float, predicate: Any, on_poll: Any | None = None) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() <= deadline:
            if on_poll is not None:
                on_poll()
            if predicate():
                return
            time.sleep(interval)
        raise TimeoutError("timeout")

    def _check_negative_positions(self) -> None:
        y1 = self._as_number(self.read_point("Y1_CUR_POS"))
        y2 = self._as_number(self.read_point("Y2_CUR_POS"))
        x = self._as_number(self.read_point("X_CUR_POS"))
        if y1 < 0 or y2 < 0 or x < 0:
            raise ValueError("error")

    def _normalize_point_id(self, point_id: str) -> str:
        return self._point_aliases.get(point_id, point_id)

    def _set_status(self, status: str, message: str) -> None:
        self._status = status
        self._message = message

    def _log(self, message: str) -> None:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        line = f"{stamp} {message}"
        self._logs.append(line)
        if len(self._logs) > 2000:
            self._logs = self._logs[-2000:]
        logger.info(message)

    @staticmethod
    def _as_number(value: Any) -> int:
        if isinstance(value, bool):
            return int(value)
        return int(round(float(value)))
