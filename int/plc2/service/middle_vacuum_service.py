from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import threading
import time
from typing import Any, Callable, Protocol

try:
    from config.loader import CONFIG_STORE, ConfigStore
    from lifecycle import LifecycleStatus, LifecycleTracked, tracked_operation
except ModuleNotFoundError:  # Support importing as plc2.service from repository root.
    from plc2.config.loader import CONFIG_STORE, ConfigStore
    from plc2.lifecycle import LifecycleStatus, LifecycleTracked, tracked_operation


class PointServiceProtocol(Protocol):
    def get_point(self, point_id: str): ...
    def read_point(self, point_id: str) -> float | int | bool: ...
    def write_point(self, point_id: str, value: Any) -> None: ...
    def register_write_guard(self, *, point_guard=None, bit_guard=None) -> None: ...


class MiddleVacuumMode(str, Enum):
    OFF = "off"
    VACUUM = "vacuum"
    BREAK_VACUUM = "break_vacuum"
    INVALID = "invalid"


class MiddleVacuumServiceError(RuntimeError):
    def __init__(self, operation: str, message: str, *, point_id: str | None = None) -> None:
        self.operation = operation
        self.point_id = point_id
        context = f" point={point_id}" if point_id else ""
        super().__init__(f"MiddleVacuumService.{operation}{context}: {message}")


@dataclass(frozen=True)
class MiddleVacuumServiceConfig:
    vacuum_point: str
    break_vacuum_point: str
    confirmation_point: str | None
    confirmation_active_value: bool
    stable_read_count: int
    confirmation_timeout_seconds: float
    poll_interval_seconds: float
    vacuum_settle_seconds: float
    release_settle_seconds: float

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "MiddleVacuumServiceConfig":
        raw = store.get_service("middle_vacuum")
        if raw is None:
            raise MiddleVacuumServiceError("config", "找不到 services.yml 的 middle_vacuum 設定")
        return cls(
            vacuum_point=str(raw["vacuum_point"]),
            break_vacuum_point=str(raw["break_vacuum_point"]),
            confirmation_point=(
                None
                if raw.get("confirmation_point") in {None, ""}
                else str(raw["confirmation_point"])
            ),
            confirmation_active_value=bool(
                raw.get("confirmation_active_value", True)
            ),
            stable_read_count=int(raw.get("stable_read_count", 3)),
            confirmation_timeout_seconds=float(
                raw.get("confirmation_timeout_seconds", 5.0)
            ),
            poll_interval_seconds=float(raw.get("poll_interval_seconds", 0.1)),
            vacuum_settle_seconds=float(raw.get("vacuum_settle_seconds", 2.0)),
            release_settle_seconds=float(raw.get("release_settle_seconds", 5.0)),
        )


class MiddleVacuumService(LifecycleTracked):
    """Controls M54/M56 as one interlocked middle-vacuum capability."""

    def __init__(
        self,
        plc_service: PointServiceProtocol,
        config: MiddleVacuumServiceConfig | None = None,
    ) -> None:
        self.plc_service = plc_service
        self.config = config or MiddleVacuumServiceConfig.load()
        self._guard_context = threading.local()
        self._operation_lock = threading.RLock()
        self._init_status_tracker("MiddleVacuumService")
        self._validate_config()
        self._validate_points()
        self._vacuum_address = int(self.plc_service.get_point(self.config.vacuum_point).address)
        self._break_vacuum_address = int(
            self.plc_service.get_point(self.config.break_vacuum_point).address
        )
        self.plc_service.register_write_guard(
            point_guard=self._guard_point_write,
            bit_guard=self._guard_bit_write,
        )

    def _validate_config(self) -> None:
        if self.config.stable_read_count <= 0:
            raise MiddleVacuumServiceError("config", "stable_read_count 必須大於 0")
        for name, value in (
            ("confirmation_timeout_seconds", self.config.confirmation_timeout_seconds),
            ("poll_interval_seconds", self.config.poll_interval_seconds),
            ("vacuum_settle_seconds", self.config.vacuum_settle_seconds),
            ("release_settle_seconds", self.config.release_settle_seconds),
        ):
            if value < 0:
                raise MiddleVacuumServiceError("config", f"{name} 不可小於 0")
        if self.config.confirmation_point and self.config.confirmation_timeout_seconds <= 0:
            raise MiddleVacuumServiceError(
                "config",
                "有設定 confirmation_point 時，confirmation_timeout_seconds 必須大於 0",
            )

    def _validate_points(self) -> None:
        for point_id in (self.config.vacuum_point, self.config.break_vacuum_point):
            point = self.plc_service.get_point(point_id)
            if str(point.device).upper() != "M" or not point.writable:
                raise MiddleVacuumServiceError(
                    "config",
                    "中間真空控制點必須是可寫入的 M 點",
                    point_id=point_id,
                )
        if self.config.confirmation_point:
            point = self.plc_service.get_point(self.config.confirmation_point)
            if str(point.type).lower() != "bit":
                raise MiddleVacuumServiceError(
                    "config",
                    "真空確認點必須是 bit 點",
                    point_id=self.config.confirmation_point,
                )

    @tracked_operation("read", "正在讀取中間真空狀態", "中間真空狀態讀取完成")
    def read_state(self) -> dict[str, bool | str]:
        with self._operation_lock:
            vacuum_on = bool(self.plc_service.read_point(self.config.vacuum_point))
            break_vacuum_on = bool(self.plc_service.read_point(self.config.break_vacuum_point))
        if vacuum_on and break_vacuum_on:
            mode = MiddleVacuumMode.INVALID
        elif vacuum_on:
            mode = MiddleVacuumMode.VACUUM
        elif break_vacuum_on:
            mode = MiddleVacuumMode.BREAK_VACUUM
        else:
            mode = MiddleVacuumMode.OFF
        return {
            "mode": mode.value,
            "vacuum_on": vacuum_on,
            "break_vacuum_on": break_vacuum_on,
        }

    @tracked_operation("set_mode", "正在切換中間真空", "中間真空切換完成")
    def set_mode(self, mode: MiddleVacuumMode | str) -> dict[str, bool | str]:
        try:
            selected = mode if isinstance(mode, MiddleVacuumMode) else MiddleVacuumMode(str(mode))
        except ValueError as exc:
            raise MiddleVacuumServiceError("set_mode", f"不支援的模式: {mode}") from exc
        if selected is MiddleVacuumMode.INVALID:
            raise MiddleVacuumServiceError("set_mode", "invalid 只用於回報異常，不可作為控制模式")

        with self._operation_lock:
            self._guard_context.bypass = True
            try:
                if selected is MiddleVacuumMode.VACUUM:
                    self.plc_service.write_point(self.config.break_vacuum_point, False)
                    self.plc_service.write_point(self.config.vacuum_point, True)
                elif selected is MiddleVacuumMode.BREAK_VACUUM:
                    self.plc_service.write_point(self.config.vacuum_point, False)
                    self.plc_service.write_point(self.config.break_vacuum_point, True)
                else:
                    self.plc_service.write_point(self.config.vacuum_point, False)
                    self.plc_service.write_point(self.config.break_vacuum_point, False)
            except Exception as exc:
                raise MiddleVacuumServiceError("set_mode", str(exc)) from exc
            finally:
                self._guard_context.bypass = False

            state = self.read_state()
        if state["mode"] != selected.value:
            raise MiddleVacuumServiceError(
                "set_mode",
                f"PLC 讀回模式不符，要求={selected.value}，讀回={state['mode']}",
            )
        return state

    def wait_transfer_ready(
        self,
        *,
        vacuum_expected: bool,
        cancel_event: threading.Event,
        progress_callback: Callable[[str], None] | None = None,
    ) -> dict[str, bool | str | float]:
        """Wait for adsorption/release confirmation, or a timer-only fallback."""

        point_id = self.config.confirmation_point
        if point_id is None:
            delay = (
                self.config.vacuum_settle_seconds
                if vacuum_expected
                else self.config.release_settle_seconds
            )
            action = "吸附" if vacuum_expected else "釋放"
            message = (
                f"沒有設定真空壓力確認點；僅定時等待 {delay:g} 秒，"
                f"不可視為{action}感測確認"
            )
            self._set_status(
                LifecycleStatus.WAITING_SIGNAL,
                "timer_only",
                message,
                confirmation="timer_only",
                vacuum_expected=vacuum_expected,
                delay_seconds=delay,
            )
            if progress_callback is not None:
                progress_callback(message)
            if cancel_event.wait(delay):
                raise MiddleVacuumServiceError("wait_transfer_ready", "流程已取消")
            return {
                "confirmation": "timer_only",
                "vacuum_expected": vacuum_expected,
                "delay_seconds": delay,
            }

        expected_value = (
            self.config.confirmation_active_value
            if vacuum_expected
            else not self.config.confirmation_active_value
        )
        action = "吸附成立" if vacuum_expected else "真空釋放"
        deadline = time.monotonic() + self.config.confirmation_timeout_seconds
        stable = 0
        while True:
            if cancel_event.is_set():
                raise MiddleVacuumServiceError("wait_transfer_ready", "流程已取消")
            actual = bool(self.plc_service.read_point(point_id))
            stable = stable + 1 if actual is expected_value else 0
            message = (
                f"等待{action}：{point_id}={actual}，"
                f"穩定 {stable}/{self.config.stable_read_count}"
            )
            self._set_status(
                LifecycleStatus.WAITING_SIGNAL,
                "sensor_confirmation",
                message,
                confirmation="sensor",
                confirmation_point=point_id,
                expected_value=expected_value,
                actual_value=actual,
                stable_reads=stable,
            )
            if progress_callback is not None:
                progress_callback(message)
            if stable >= self.config.stable_read_count:
                self._set_status(
                    LifecycleStatus.SUCCESS,
                    "sensor_confirmation",
                    f"{action}已由 {point_id} 確認",
                    confirmation="sensor",
                    confirmation_point=point_id,
                    vacuum_expected=vacuum_expected,
                )
                return {
                    "confirmation": "sensor",
                    "confirmation_point": point_id,
                    "vacuum_expected": vacuum_expected,
                }
            if time.monotonic() >= deadline:
                raise MiddleVacuumServiceError(
                    "wait_transfer_ready",
                    f"等待{action}超過 {self.config.confirmation_timeout_seconds:g} 秒",
                    point_id=point_id,
                )
            if cancel_event.wait(self.config.poll_interval_seconds):
                raise MiddleVacuumServiceError("wait_transfer_ready", "流程已取消")

    def _guard_bit_write(self, device: str, address: int, values: list[bool]) -> None:
        if device.upper() != "M" or bool(getattr(self._guard_context, "bypass", False)):
            return
        requested = {
            address + offset: bool(value)
            for offset, value in enumerate(values)
            if address + offset in {self._vacuum_address, self._break_vacuum_address}
        }
        if not requested:
            return
        vacuum_on = requested.get(
            self._vacuum_address,
            bool(self.plc_service.read_point(self.config.vacuum_point)),
        )
        break_vacuum_on = requested.get(
            self._break_vacuum_address,
            bool(self.plc_service.read_point(self.config.break_vacuum_point)),
        )
        if vacuum_on and break_vacuum_on:
            raise MiddleVacuumServiceError(
                "interlock",
                "原始 M 點寫入會讓 M54 與 M56 同時 ON，已阻擋",
            )

    def _guard_point_write(self, point_id: str, value: Any) -> None:
        if bool(getattr(self._guard_context, "bypass", False)) or not self._as_bool(value):
            return
        if point_id == self.config.vacuum_point:
            other = self.config.break_vacuum_point
        elif point_id == self.config.break_vacuum_point:
            other = self.config.vacuum_point
        else:
            return
        if bool(self.plc_service.read_point(other)):
            raise MiddleVacuumServiceError(
                "interlock",
                "M54 中間真空與 M56 中間破真空不可同時開啟；請使用中間真空快捷控制切換",
                point_id=point_id,
            )

    @staticmethod
    def _as_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in {"1", "true", "on", "yes", "是"}
