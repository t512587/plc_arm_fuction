from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, fields
from enum import Enum
from pathlib import Path
from typing import Callable, Protocol

import yaml


LOGGER = logging.getLogger(__name__)
CONFIG_PATH = Path(__file__).resolve().parent / "config" / "vision_height_positioning.yml"


class PlcVisionHeightService(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def get_point(self, point_id: str): ...

    def read_point(self, point_id: str) -> float | int | bool: ...

    def write_point(self, point_id: str, value: float | bool) -> None: ...


class VisionHeightState(str, Enum):
    PRECHECK = "precheck"
    VACUUM = "vacuum"
    MOVING = "moving"
    FINE_TUNING = "fine_tuning"
    SETTLING = "settling"
    SUCCESS = "success"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    STALLED = "stalled"
    COMMAND_DROPPED = "command_dropped"
    POSITION_UNCHANGED = "position_unchanged"
    ERROR = "error"


@dataclass(frozen=True)
class SpeedProfile:
    # Compatibility with the existing UI; this workflow does not control speed.
    fast: int
    slow: int
    creep: int


@dataclass(frozen=True)
class VisionHeightConfig:
    target_height_mm: float = 560.0
    tolerance_mm: float = 1.0
    stable_read_count: int = 3
    poll_interval_seconds: float = 0.1
    vacuum_build_seconds: float = 0.5
    settle_seconds: float = 0.5
    timeout_seconds: float = 30.0
    minimum_height_mm: float = 0.0
    maximum_height_mm: float = 1450.0
    height_point: str = "x_current_pos"
    up_command_point: str = "LIFT_UP_POS"
    down_command_point: str = "LIFT_DN_POS"
    vacuum_points: tuple[str, str] = ("L_VAC_ON", "R_VAC_ON")
    vacuum_release_points: tuple[str, str] = ("L_VAC_REL", "R_VAC_REL")
    vacuum_feedback_points: tuple[str, ...] = ()
    vacuum_feedback_timeout_seconds: float = 3.0
    trigger_reset_seconds: float = 0.15
    unchanged_timeout_seconds: float = 5.0
    minimum_motion_delta_mm: float = 1.0

    def validate(self) -> None:
        if not self.minimum_height_mm < self.maximum_height_mm:
            raise ValueError("X 軸最低高度必須小於最高高度")
        if not self.minimum_height_mm <= self.target_height_mm <= self.maximum_height_mm:
            raise ValueError("目標高度超出 0～1450mm 軟體限制")
        if self.tolerance_mm < 0 or self.stable_read_count <= 0:
            raise ValueError("高度容許誤差與穩定讀取次數設定無效")
        if self.poll_interval_seconds <= 0 or self.timeout_seconds <= 0:
            raise ValueError("輪詢間隔與定位逾時必須大於 0")
        if self.vacuum_build_seconds < 0 or self.settle_seconds < 0:
            raise ValueError("等待時間不可小於 0")
        if self.vacuum_feedback_timeout_seconds <= 0:
            raise ValueError("真空回授逾時必須大於 0")
        if self.trigger_reset_seconds <= 0 or self.unchanged_timeout_seconds <= 0:
            raise ValueError("命令重置與位置未變化判斷時間必須大於 0")
        if self.minimum_motion_delta_mm <= 0:
            raise ValueError("最小高度變化量必須大於 0")


def load_vision_height_config(path: Path = CONFIG_PATH) -> VisionHeightConfig:
    if not path.exists():
        return VisionHeightConfig()
    with path.open("r", encoding="utf-8") as file:
        raw = yaml.safe_load(file) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"視覺高度設定格式錯誤: {path}")
    allowed = {field.name for field in fields(VisionHeightConfig)}
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"視覺高度設定含未知欄位: {', '.join(unknown)}")
    for key in ("vacuum_points", "vacuum_release_points", "vacuum_feedback_points"):
        if key in raw:
            raw[key] = tuple(raw[key] or ())
    config = VisionHeightConfig(**raw)
    config.validate()
    return config


@dataclass(frozen=True)
class VisionHeightProgress:
    state: VisionHeightState
    message: str
    elapsed_seconds: float
    height_mm: float | None
    active_speed: int | None
    speed_profile: SpeedProfile | None
    control_states: dict[str, bool]


@dataclass(frozen=True)
class VisionHeightResult:
    state: VisionHeightState
    message: str
    elapsed_seconds: float
    height_mm: float | None
    speed_profile: SpeedProfile | None = None

    @property
    def succeeded(self) -> bool:
        return self.state is VisionHeightState.SUCCESS

    @property
    def status(self) -> str:
        if self.state in {
            VisionHeightState.STALLED,
            VisionHeightState.COMMAND_DROPPED,
            VisionHeightState.POSITION_UNCHANGED,
        }:
            return "error"
        return self.state.value


class VisionHeightWorkflow:
    def __init__(self, service: PlcVisionHeightService, config: VisionHeightConfig | None = None) -> None:
        self.service = service
        self.config = config or VisionHeightConfig()
        self.config.validate()

    def run(
        self,
        cancel_event: threading.Event,
        progress_callback: Callable[[VisionHeightProgress], None] | None = None,
    ) -> VisionHeightResult:
        started = time.monotonic()
        height: float | None = None
        active_command: str | None = None
        control_states: dict[str, bool] = {}

        def elapsed() -> float:
            return time.monotonic() - started

        def result(state: VisionHeightState, message: str) -> VisionHeightResult:
            LOGGER.info(
                "preset vision height finished state=%s height=%s message=%s",
                state.value,
                height,
                message,
            )
            return VisionHeightResult(state, message, elapsed(), height)

        def report(state: VisionHeightState, message: str) -> None:
            if progress_callback is not None:
                progress_callback(
                    VisionHeightProgress(
                        state,
                        message,
                        elapsed(),
                        height,
                        None,
                        None,
                        dict(control_states),
                    )
                )

        try:
            report(VisionHeightState.PRECHECK, "檢查 PLC 指定位置點位...")
            self._precheck()
            height = self._read_height()

            self._stop_commands()

            for point_id in self.config.vacuum_release_points:
                self.service.write_point(point_id, False)
            for point_id in self.config.vacuum_points:
                self.service.write_point(point_id, True)
            if cancel_event.wait(self.config.vacuum_build_seconds):
                return result(VisionHeightState.CANCELLED, "已取消定位；左右真空保持開啟")
            vacuum_states = self._read_switches(self.config.vacuum_points)
            control_states.update(vacuum_states)
            if not all(vacuum_states.values()):
                failed = ", ".join(point for point, enabled in vacuum_states.items() if not enabled)
                raise RuntimeError(f"真空命令沒有保持 ON: {failed}")
            self._wait_for_vacuum_feedback(cancel_event)
            report(VisionHeightState.VACUUM, "左右真空已開啟，準備 PLC 指定位置移動")

            if abs(height - self.config.target_height_mm) > self.config.tolerance_mm:
                moving_up = height < self.config.target_height_mm
                # M376 is the PLC preset-position command for 560mm in both directions.
                active_command = self.config.up_command_point
                opposite_command = self.config.down_command_point
                direction_label = "上升" if moving_up else "下降"
                self.service.write_point(opposite_command, False)
                self.service.write_point(active_command, False)
                if cancel_event.wait(self.config.trigger_reset_seconds):
                    return result(VisionHeightState.CANCELLED, "已取消定位；左右真空保持開啟")
                if bool(self.service.read_point(active_command)):
                    raise RuntimeError(f"{active_command} 無法清除為 OFF，未送出新定位命令")
                self.service.write_point(active_command, True)
                control_states[opposite_command] = False
                control_states[active_command] = True
                LOGGER.info(
                    "PLC preset motion started direction=%s height=%s command=%s target=%s",
                    direction_label,
                    height,
                    active_command,
                    self.config.target_height_mm,
                )
            else:
                direction_label = "到位"

            stable_reads = 0
            last_height = height
            last_motion_at = time.monotonic()
            deadline = started + self.config.timeout_seconds
            while time.monotonic() < deadline:
                if cancel_event.is_set():
                    return result(VisionHeightState.CANCELLED, "已取消定位；升降命令已停止，左右真空保持開啟")
                height = self._read_height()
                if active_command is not None:
                    control_states[active_command] = bool(self.service.read_point(active_command))
                now = time.monotonic()
                if abs(height - last_height) >= self.config.minimum_motion_delta_mm:
                    last_height = height
                    last_motion_at = now
                distance = abs(height - self.config.target_height_mm)
                if distance <= self.config.tolerance_mm:
                    stable_reads += 1
                    report(
                        VisionHeightState.SETTLING,
                        f"高度穩定確認 {stable_reads}/{self.config.stable_read_count}：{height:g}mm",
                    )
                    if stable_reads >= self.config.stable_read_count:
                        self._stop_commands()
                        active_command = None
                        if cancel_event.wait(self.config.settle_seconds):
                            return result(VisionHeightState.CANCELLED, "已取消定位；左右真空保持開啟")
                        height = self._read_height()
                        if abs(height - self.config.target_height_mm) <= self.config.tolerance_mm:
                            return result(VisionHeightState.SUCCESS, f"視覺高度定位完成：{height:g}mm")
                        stable_reads = 0
                else:
                    stable_reads = 0
                    report(
                        VisionHeightState.MOVING,
                        f"PLC 指定位置{direction_label}中：目前 {height:g}mm，目標 {self.config.target_height_mm:g}mm，"
                        f"{active_command}={'ON' if control_states.get(active_command) else 'OFF'}",
                    )
                    if active_command is not None and now - last_motion_at >= self.config.unchanged_timeout_seconds:
                        if control_states.get(active_command):
                            return result(
                                VisionHeightState.POSITION_UNCHANGED,
                                f"{active_command} 仍為 ON，但高度回授 {self.config.unchanged_timeout_seconds:g} 秒未變；"
                                "這不等同機械卡住，請檢查 PLC 定位條件",
                            )
                        return result(
                            VisionHeightState.COMMAND_DROPPED,
                            f"{active_command} 已被 PLC 取消，且高度回授 {self.config.unchanged_timeout_seconds:g} 秒未變；"
                            "請檢查 PLC 定位允許條件",
                        )
                cancel_event.wait(self.config.poll_interval_seconds)

            return result(
                VisionHeightState.TIMEOUT,
                f"等待 PLC 指定位置超過 {self.config.timeout_seconds:g} 秒；升降命令已停止，真空保持開啟",
            )
        except Exception as exc:
            LOGGER.exception("PLC preset vision height failed")
            return result(VisionHeightState.ERROR, str(exc))
        finally:
            try:
                self._stop_commands()
            except Exception:
                LOGGER.exception("Failed to clear PLC preset command bits")

    def _precheck(self) -> None:
        if not self.service.is_connected:
            raise RuntimeError("PLC 尚未連線")
        readable = (self.config.height_point, *self.config.vacuum_feedback_points)
        writable = (
            self.config.up_command_point,
            self.config.down_command_point,
            *self.config.vacuum_points,
            *self.config.vacuum_release_points,
        )
        for point_id in readable:
            self.service.get_point(point_id)
        for point_id in writable:
            point = self.service.get_point(point_id)
            if not point.writable:
                raise RuntimeError(f"PLC 點位不可寫入: {point_id}")

    def _read_height(self) -> float:
        height = float(self.service.read_point(self.config.height_point))
        if not math.isfinite(height):
            raise RuntimeError(f"X 高度回授不是有效數值: {height!r}")
        if not self.config.minimum_height_mm <= height <= self.config.maximum_height_mm:
            raise RuntimeError(
                f"X 高度 {height:g}mm 超出軟體允許範圍 "
                f"{self.config.minimum_height_mm:g}～{self.config.maximum_height_mm:g}mm"
            )
        return height

    def _read_switches(self, point_ids: tuple[str, ...]) -> dict[str, bool]:
        return {point_id: bool(self.service.read_point(point_id)) for point_id in point_ids}

    def _stop_commands(self) -> None:
        self.service.write_point(self.config.up_command_point, False)
        self.service.write_point(self.config.down_command_point, False)

    def _wait_for_vacuum_feedback(self, cancel_event: threading.Event) -> None:
        if not self.config.vacuum_feedback_points:
            return
        deadline = time.monotonic() + self.config.vacuum_feedback_timeout_seconds
        while time.monotonic() < deadline:
            if cancel_event.is_set():
                return
            if all(self._read_switches(self.config.vacuum_feedback_points).values()):
                return
            cancel_event.wait(self.config.poll_interval_seconds)
        raise RuntimeError("真空回授逾時；未啟動升降定位")
