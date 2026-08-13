from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Protocol


LOGGER = logging.getLogger(__name__)


class PlcHomeService(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def get_point(self, point_id: str): ...

    def read_point(self, point_id: str) -> float | int | bool: ...

    def write_point(self, point_id: str, value: bool) -> None: ...


class HomeWorkflowState(str, Enum):
    PRECHECK = "precheck"
    COMMANDING = "commanding"
    WAITING = "waiting"
    SUCCESS = "success"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    ERROR = "error"


@dataclass(frozen=True)
class HomeWorkflowConfig:
    command_points: tuple[str, str, str] = ("X_HOME", "Y1_HOME", "Y2_HOME")
    position_points: tuple[str, str, str] = ("x_current_pos", "y1_current_pos", "y2_current_pos")
    axis_names: tuple[str, str, str] = ("X", "Y1", "Y2")
    motion_stop_points: tuple[str, ...] = (
        "X_UP",
        "X_DOWN",
        "LIFT_UP_POS",
        "LIFT_DN_POS",
        "L_FWD_POS",
        "R_FWD_POS",
    )
    pulse_seconds: float = 0.2
    poll_interval_seconds: float = 0.2
    timeout_seconds: float = 60.0
    position_tolerance: float = 0.0
    stable_read_count: int = 3
    read_error_limit: int = 3

    def validate(self) -> None:
        if not (len(self.command_points) == len(self.position_points) == len(self.axis_names)):
            raise ValueError("命令點位、位置點位與軸名稱數量必須一致")
        if self.pulse_seconds < 0 or self.poll_interval_seconds <= 0 or self.timeout_seconds <= 0:
            raise ValueError("流程時間設定必須大於 0")
        if self.position_tolerance < 0:
            raise ValueError("位置容許誤差不可小於 0")
        if self.stable_read_count <= 0 or self.read_error_limit <= 0:
            raise ValueError("穩定次數與讀取錯誤上限必須大於 0")


@dataclass(frozen=True)
class HomeWorkflowProgress:
    state: HomeWorkflowState
    message: str
    elapsed_seconds: float = 0.0
    positions: dict[str, float] = field(default_factory=dict)
    stable_reads: int = 0


@dataclass(frozen=True)
class HomeWorkflowResult:
    state: HomeWorkflowState
    message: str
    elapsed_seconds: float
    positions: dict[str, float] = field(default_factory=dict)

    @property
    def succeeded(self) -> bool:
        return self.state is HomeWorkflowState.SUCCESS

    @property
    def status(self) -> str:
        return self.state.value


class HomeWorkflow:
    def __init__(self, service: PlcHomeService, config: HomeWorkflowConfig | None = None) -> None:
        self.service = service
        self.config = config or HomeWorkflowConfig()
        self.config.validate()

    def run(
        self,
        cancel_event: threading.Event,
        progress_callback: Callable[[HomeWorkflowProgress], None] | None = None,
    ) -> HomeWorkflowResult:
        started = time.monotonic()
        positions: dict[str, float] = {}
        home_commands_active = False

        def elapsed() -> float:
            return time.monotonic() - started

        def report(
            state: HomeWorkflowState,
            message: str,
            *,
            stable_reads: int = 0,
        ) -> None:
            if progress_callback is not None:
                progress_callback(
                    HomeWorkflowProgress(state, message, elapsed(), dict(positions), stable_reads)
                )

        try:
            report(HomeWorkflowState.PRECHECK, "檢查 PLC 連線與歸零點位")
            self._precheck()
            positions = self._read_positions()
            LOGGER.info("home workflow started initial_positions=%s", positions)

            if cancel_event.is_set():
                return self._result(HomeWorkflowState.CANCELLED, "歸零流程已取消", elapsed(), positions)

            report(HomeWorkflowState.COMMANDING, "先停止 X、Y1、Y2 既有移動命令")
            self._stop_motion_commands()
            report(HomeWorkflowState.COMMANDING, "送出 X、Y1、Y2 歸零命令")
            home_commands_active = True
            self._set_home_commands(True)
            if cancel_event.wait(self.config.pulse_seconds):
                return self._result(HomeWorkflowState.CANCELLED, "歸零流程已取消", elapsed(), positions)
            self._set_home_commands(False)
            home_commands_active = False

            stable_reads = 0
            consecutive_read_errors = 0
            deadline = started + self.config.timeout_seconds
            while time.monotonic() < deadline:
                if cancel_event.is_set():
                    return self._result(HomeWorkflowState.CANCELLED, "歸零流程已取消", elapsed(), positions)

                try:
                    positions = self._read_positions()
                    consecutive_read_errors = 0
                except Exception as exc:
                    consecutive_read_errors += 1
                    LOGGER.warning(
                        "home position read failed attempt=%s/%s error=%s",
                        consecutive_read_errors,
                        self.config.read_error_limit,
                        exc,
                    )
                    if consecutive_read_errors >= self.config.read_error_limit:
                        raise RuntimeError(
                            f"連續 {consecutive_read_errors} 次讀取位置失敗: {exc}"
                        ) from exc
                    report(HomeWorkflowState.WAITING, f"位置讀取失敗，正在重試 ({consecutive_read_errors})")
                    cancel_event.wait(self.config.poll_interval_seconds)
                    continue

                all_home = all(
                    abs(value) <= self.config.position_tolerance for value in positions.values()
                )
                stable_reads = stable_reads + 1 if all_home else 0
                report(
                    HomeWorkflowState.WAITING,
                    self._position_message(positions, stable_reads),
                    stable_reads=stable_reads,
                )
                if stable_reads >= self.config.stable_read_count:
                    return self._result(
                        HomeWorkflowState.SUCCESS,
                        "X、Y1、Y2 已穩定歸零",
                        elapsed(),
                        positions,
                    )
                cancel_event.wait(self.config.poll_interval_seconds)

            return self._result(
                HomeWorkflowState.TIMEOUT,
                f"歸零超過 {self.config.timeout_seconds:g} 秒仍未完成",
                elapsed(),
                positions,
            )
        except Exception as exc:
            return self._result(HomeWorkflowState.ERROR, f"歸零流程失敗: {exc}", elapsed(), positions)
        finally:
            if home_commands_active:
                try:
                    self._set_home_commands(False)
                except Exception as exc:
                    LOGGER.error("failed to clear home commands: %s", exc)

    def _precheck(self) -> None:
        if not self.service.is_connected:
            raise RuntimeError("PLC 尚未連線")
        for point_id in self.config.command_points:
            point = self.service.get_point(point_id)
            if str(point.device).upper() != "M" or not point.writable:
                raise RuntimeError(f"歸零命令點位設定錯誤: {point_id}")
        for point_id in self.config.position_points:
            point = self.service.get_point(point_id)
            if str(point.device).upper() != "D":
                raise RuntimeError(f"位置回授點位設定錯誤: {point_id}")
        for point_id in self.config.motion_stop_points:
            point = self.service.get_point(point_id)
            if str(point.device).upper() != "M" or not point.writable:
                raise RuntimeError(f"移動停止點位設定錯誤: {point_id}")

    def _stop_motion_commands(self) -> None:
        errors: list[str] = []
        for point_id in self.config.motion_stop_points:
            try:
                self.service.write_point(point_id, False)
            except Exception as exc:
                errors.append(f"{point_id}: {exc}")
        if errors:
            raise RuntimeError("停止既有移動命令失敗: " + "; ".join(errors))

    def _set_home_commands(self, value: bool) -> None:
        written: list[str] = []
        try:
            for point_id in self.config.command_points:
                self.service.write_point(point_id, value)
                written.append(point_id)
        except Exception:
            if value:
                for point_id in written:
                    try:
                        self.service.write_point(point_id, False)
                    except Exception as cleanup_exc:
                        LOGGER.error("failed to clear partial command point=%s error=%s", point_id, cleanup_exc)
            raise

    def _read_positions(self) -> dict[str, float]:
        positions: dict[str, float] = {}
        for axis_name, point_id in zip(
            self.config.axis_names,
            self.config.position_points,
            strict=True,
        ):
            value = self.service.read_point(point_id)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise RuntimeError(f"{axis_name} 位置回傳值無效: {value!r}")
            positions[axis_name] = float(value)
        return positions

    def _position_message(self, positions: dict[str, float], stable_reads: int) -> str:
        values = "  ".join(f"{axis}={value:g}" for axis, value in positions.items())
        return f"等待歸零：{values}  穩定 {stable_reads}/{self.config.stable_read_count}"

    @staticmethod
    def _result(
        state: HomeWorkflowState,
        message: str,
        elapsed_seconds: float,
        positions: dict[str, float],
    ) -> HomeWorkflowResult:
        result = HomeWorkflowResult(state, message, elapsed_seconds, dict(positions))
        log = LOGGER.info if state is HomeWorkflowState.SUCCESS else LOGGER.warning
        log(
            "home workflow finished state=%s elapsed=%.3f positions=%s message=%s",
            state.value,
            elapsed_seconds,
            positions,
            message,
        )
        return result
