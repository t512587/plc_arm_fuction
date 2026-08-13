from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Protocol


LOGGER = logging.getLogger(__name__)


class PlcYAxisService(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def get_point(self, point_id: str): ...

    def read_point(self, point_id: str) -> float | int | bool: ...

    def write_point(self, point_id: str, value: float | bool) -> None: ...


class YAxisPositioningState(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    WAITING_SIGNAL = "waiting_signal"
    SUCCESS = "success"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    ERROR = "error"


@dataclass(frozen=True)
class YAxisSpec:
    axis: str
    target_point: str
    speed_point: str
    position_point: str
    command_point: str


AXIS_SPECS = {
    "Y1": YAxisSpec("Y1", "y1_forward_pos", "y1_forward_speed", "y1_current_pos", "L_FWD_POS"),
    "Y2": YAxisSpec("Y2", "y2_forward_pos", "y2_forward_speed", "y2_current_pos", "R_FWD_POS"),
}


@dataclass(frozen=True)
class YAxisPositioningConfig:
    tolerance: float = 2.0
    stable_read_count: int = 3
    poll_interval_seconds: float = 0.1
    trigger_reset_seconds: float = 0.15
    unchanged_timeout_seconds: float = 5.0
    timeout_seconds: float = 30.0
    minimum_target: float = 0.0
    maximum_target: float = 32767.0
    minimum_motion_delta: float = 1.0

    def validate(self) -> None:
        if self.tolerance < 0 or self.minimum_motion_delta <= 0:
            raise ValueError("Y 軸容許誤差與最小位移量設定無效")
        if self.stable_read_count <= 0:
            raise ValueError("Y 軸穩定讀取次數必須大於 0")
        if min(
            self.poll_interval_seconds,
            self.trigger_reset_seconds,
            self.unchanged_timeout_seconds,
            self.timeout_seconds,
        ) <= 0:
            raise ValueError("Y 軸流程時間設定必須大於 0")
        if not self.minimum_target < self.maximum_target:
            raise ValueError("Y 軸目標位置範圍設定無效")


@dataclass(frozen=True)
class YAxisPositioningProgress:
    state: YAxisPositioningState
    message: str
    axis: str
    elapsed_seconds: float
    target: float | None
    speed: float | None
    position: float | None
    command_on: bool | None


@dataclass(frozen=True)
class YAxisPositioningResult:
    state: YAxisPositioningState
    message: str
    axis: str
    elapsed_seconds: float
    target: float | None
    speed: float | None
    position: float | None
    command_on: bool | None
    movement_observed: bool

    @property
    def succeeded(self) -> bool:
        return self.state is YAxisPositioningState.SUCCESS

    @property
    def status(self) -> str:
        return self.state.value


class YAxisPositioningWorkflow:
    def __init__(
        self,
        service: PlcYAxisService,
        config: YAxisPositioningConfig | None = None,
    ) -> None:
        self.service = service
        self.config = config or YAxisPositioningConfig()
        self.config.validate()

    def run(
        self,
        axis: str,
        cancel_event: threading.Event,
        progress_callback: Callable[[YAxisPositioningProgress], None] | None = None,
    ) -> YAxisPositioningResult:
        normalized_axis = str(axis).strip().upper()
        started = time.monotonic()
        target: float | None = None
        speed: float | None = None
        position: float | None = None
        command_on: bool | None = None
        movement_observed = False
        command_touched = False

        def elapsed() -> float:
            return time.monotonic() - started

        def report(state: YAxisPositioningState, message: str) -> None:
            if progress_callback is not None:
                progress_callback(
                    YAxisPositioningProgress(
                        state,
                        message,
                        normalized_axis,
                        elapsed(),
                        target,
                        speed,
                        position,
                        command_on,
                    )
                )

        def result(state: YAxisPositioningState, message: str) -> YAxisPositioningResult:
            item = YAxisPositioningResult(
                state,
                message,
                normalized_axis,
                elapsed(),
                target,
                speed,
                position,
                command_on,
                movement_observed,
            )
            log = LOGGER.info if item.succeeded else LOGGER.warning
            log(
                "Y positioning finished axis=%s state=%s target=%s speed=%s position=%s "
                "command_on=%s moved=%s message=%s",
                normalized_axis,
                state.value,
                target,
                speed,
                position,
                command_on,
                movement_observed,
                message,
            )
            return item

        try:
            if normalized_axis not in AXIS_SPECS:
                raise RuntimeError(f"不支援的 Y 軸: {axis}")
            spec = AXIS_SPECS[normalized_axis]
            report(YAxisPositioningState.PENDING, f"{normalized_axis} 定位前置檢查中")
            self._precheck(spec)
            if cancel_event.is_set():
                return result(YAxisPositioningState.CANCELLED, f"{normalized_axis} 定位尚未開始即取消")

            target = self._read_number(spec.target_point)
            speed = self._read_number(spec.speed_point)
            position = self._read_number(spec.position_point)
            if not self.config.minimum_target <= target <= self.config.maximum_target:
                raise RuntimeError(
                    f"{normalized_axis} 目標 {target:g} 超出允許範圍 "
                    f"{self.config.minimum_target:g}～{self.config.maximum_target:g}"
                )
            if speed <= 0:
                raise RuntimeError(f"{normalized_axis} 定位速度必須大於 0，目前讀回 {speed:g}")

            report(
                YAxisPositioningState.RUNNING,
                f"{normalized_axis} 建立新定位觸發：目標 {target:g}、D240速度 {speed:g}、目前 {position:g}",
            )
            self.service.write_point(spec.command_point, False)
            command_touched = True
            if cancel_event.wait(self.config.trigger_reset_seconds):
                return result(YAxisPositioningState.CANCELLED, f"{normalized_axis} 定位已取消")
            command_on = bool(self.service.read_point(spec.command_point))
            if command_on:
                raise RuntimeError(f"{spec.command_point} 無法清除為 OFF，未送出新定位命令")

            initial_position = position
            last_position = position
            last_motion_at = time.monotonic()
            self.service.write_point(spec.command_point, True)
            command_on = bool(self.service.read_point(spec.command_point))
            report(
                YAxisPositioningState.WAITING_SIGNAL,
                f"{normalized_axis} 定位命令已送出：{spec.command_point}={'ON' if command_on else 'PLC已自行復歸'}，等待位置回授",
            )

            stable_reads = 0
            deadline = started + self.config.timeout_seconds
            while time.monotonic() < deadline:
                if cancel_event.is_set():
                    return result(YAxisPositioningState.CANCELLED, f"{normalized_axis} 定位已取消，命令已停止")

                position = self._read_number(spec.position_point)
                command_on = bool(self.service.read_point(spec.command_point))
                now = time.monotonic()
                if abs(position - last_position) >= self.config.minimum_motion_delta:
                    movement_observed = True
                    last_motion_at = now
                    last_position = position
                elif abs(position - initial_position) >= self.config.minimum_motion_delta:
                    movement_observed = True

                if abs(position - target) <= self.config.tolerance:
                    stable_reads += 1
                    report(
                        YAxisPositioningState.WAITING_SIGNAL,
                        f"{normalized_axis} 到位確認 {stable_reads}/{self.config.stable_read_count}：目前 {position:g}、目標 {target:g}",
                    )
                    if stable_reads >= self.config.stable_read_count:
                        return result(
                            YAxisPositioningState.SUCCESS,
                            f"{normalized_axis} 定位完成：{position:g}（目標 {target:g}、D240速度 {speed:g}）",
                        )
                else:
                    stable_reads = 0
                    report(
                        YAxisPositioningState.WAITING_SIGNAL,
                        f"{normalized_axis} 定位中：目前 {position:g}、目標 {target:g}、"
                        f"{spec.command_point}={'ON' if command_on else 'OFF'}、D240={speed:g}",
                    )

                if not movement_observed and now - last_motion_at >= self.config.unchanged_timeout_seconds:
                    command_text = "仍為 ON" if command_on else "已被 PLC 取消"
                    return result(
                        YAxisPositioningState.ERROR,
                        f"{normalized_axis} 命令{command_text}，但 {position:g} 的位置回授 "
                        f"{self.config.unchanged_timeout_seconds:g} 秒未變；這不等同機械卡住，請檢查 PLC 定位條件與速度來源",
                    )
                cancel_event.wait(self.config.poll_interval_seconds)

            command_text = "ON" if command_on else "OFF"
            return result(
                YAxisPositioningState.TIMEOUT,
                f"{normalized_axis} 定位超過 {self.config.timeout_seconds:g} 秒：目前 {position:g}、"
                f"目標 {target:g}、命令 {command_text}、D240={speed:g}",
            )
        except Exception as exc:
            LOGGER.exception("Y positioning failed axis=%s", normalized_axis)
            return result(YAxisPositioningState.ERROR, str(exc))
        finally:
            if command_touched and normalized_axis in AXIS_SPECS:
                try:
                    self.service.write_point(AXIS_SPECS[normalized_axis].command_point, False)
                    command_on = False
                except Exception:
                    LOGGER.exception("Failed to clear Y positioning command axis=%s", normalized_axis)

    def _precheck(self, spec: YAxisSpec) -> None:
        if not self.service.is_connected:
            raise RuntimeError("PLC 尚未連線")
        for point_id in (spec.target_point, spec.speed_point, spec.position_point):
            point = self.service.get_point(point_id)
            if str(point.device).upper() != "D":
                raise RuntimeError(f"Y 軸資料點位必須是 D: {point_id}")
        command = self.service.get_point(spec.command_point)
        if str(command.device).upper() != "M" or not command.writable:
            raise RuntimeError(f"Y 軸命令點位必須是可寫入 M: {spec.command_point}")

    def _read_number(self, point_id: str) -> float:
        value = self.service.read_point(point_id)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(f"{point_id} 回傳值無效: {value!r}")
        parsed = float(value)
        if not math.isfinite(parsed):
            raise RuntimeError(f"{point_id} 回傳值不是有限數值: {parsed!r}")
        return parsed
