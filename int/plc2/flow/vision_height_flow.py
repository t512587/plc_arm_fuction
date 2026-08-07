from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, fields
from enum import Enum
from typing import Callable, Protocol

try:
    from config.loader import CONFIG_STORE, ConfigStore
    from lifecycle import LifecycleStatus, LifecycleTracked
except ModuleNotFoundError:
    from plc2.config.loader import CONFIG_STORE, ConfigStore
    from plc2.lifecycle import LifecycleStatus, LifecycleTracked


LOGGER = logging.getLogger(__name__)


class LiftServiceProtocol(Protocol):
    def precheck(self) -> None: ...
    def read_height(self) -> float: ...
    def configure_position(self, target_height_mm: float, speed: float | None = None) -> None: ...
    def set_vacuum(self, enabled: bool) -> None: ...
    def read_vacuum(self) -> dict[str, bool]: ...
    def start_up(self) -> None: ...
    def start_vision_positioning(self) -> None: ...
    def start_down(self) -> None: ...
    def read_motion_commands(self) -> dict[str, bool]: ...
    def stop(self) -> None: ...


class VisionHeightState(str, Enum):
    PRECHECK = "precheck"
    VACUUM = "vacuum"
    MOVING = "moving"
    SETTLING = "settling"
    SUCCESS = "success"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    ERROR = "error"


@dataclass(frozen=True)
class VisionHeightConfig:
    target_height_mm: float = 560.0
    tolerance_mm: float = 1.0
    stable_read_count: int = 3
    poll_interval_seconds: float = 0.1
    vacuum_build_seconds: float = 0.5
    settle_seconds: float = 0.5
    timeout_seconds: float = 30.0
    unchanged_timeout_seconds: float = 2.0
    retrigger_limit: int = 3

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "VisionHeightConfig":
        raw = store.get_flow("vision_height")
        if raw is None:
            raise ValueError("找不到 config/flows/vision_height.yml")
        allowed = {field.name for field in fields(cls)}
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise ValueError(f"vision height flow 未知設定: {', '.join(unknown)}")
        config = cls(**raw)
        config.validate()
        return config

    def validate(self) -> None:
        if self.target_height_mm < 0 or self.tolerance_mm < 0:
            raise ValueError("vision height 目標與容許誤差不可小於 0")
        if self.stable_read_count <= 0:
            raise ValueError("vision height 穩定次數必須大於 0")
        if self.poll_interval_seconds <= 0 or self.timeout_seconds <= 0:
            raise ValueError("vision height 輪詢與逾時必須大於 0")
        if self.vacuum_build_seconds < 0 or self.settle_seconds < 0:
            raise ValueError("vision height 等待時間不可小於 0")
        if self.unchanged_timeout_seconds <= 0:
            raise ValueError("vision height 高度未變逾時必須大於 0")
        if self.retrigger_limit < 0:
            raise ValueError("vision height 重新觸發次數不可小於 0")


@dataclass(frozen=True)
class VisionHeightProgress:
    status: LifecycleStatus
    state: VisionHeightState
    step: str
    message: str
    elapsed_seconds: float
    height_mm: float | None
    stable_reads: int
    vacuum_states: dict[str, bool]
    motion_states: dict[str, bool]


@dataclass(frozen=True)
class VisionHeightResult:
    status: LifecycleStatus
    state: VisionHeightState
    step: str
    message: str
    elapsed_seconds: float
    height_mm: float | None

    @property
    def succeeded(self) -> bool:
        return self.status is LifecycleStatus.SUCCESS


class VisionHeightFlow(LifecycleTracked):
    def __init__(
        self,
        service: LiftServiceProtocol,
        config: VisionHeightConfig | None = None,
    ) -> None:
        self.service = service
        self.config = config or VisionHeightConfig.load()
        self.config.validate()
        self._init_status_tracker("VisionHeightFlow")

    def run(
        self,
        cancel_event: threading.Event,
        progress_callback: Callable[[VisionHeightProgress], None] | None = None,
    ) -> VisionHeightResult:
        started = time.monotonic()
        step = "precheck"
        height: float | None = None
        vacuum_states: dict[str, bool] = {}
        motion_states: dict[str, bool] = {}
        lift_touched = False

        def lifecycle_status(state: VisionHeightState) -> LifecycleStatus:
            return {
                VisionHeightState.PRECHECK: LifecycleStatus.RUNNING,
                VisionHeightState.VACUUM: LifecycleStatus.RUNNING,
                VisionHeightState.MOVING: LifecycleStatus.WAITING_SIGNAL,
                VisionHeightState.SETTLING: LifecycleStatus.WAITING_SIGNAL,
                VisionHeightState.SUCCESS: LifecycleStatus.SUCCESS,
                VisionHeightState.CANCELLED: LifecycleStatus.CANCELLED,
                VisionHeightState.TIMEOUT: LifecycleStatus.TIMEOUT,
                VisionHeightState.ERROR: LifecycleStatus.ERROR,
            }[state]

        def elapsed() -> float:
            return time.monotonic() - started

        def report(state: VisionHeightState, message: str, stable_reads: int = 0) -> None:
            status = lifecycle_status(state)
            self._set_status(status, step, message, height_mm=height)
            if progress_callback is not None:
                progress_callback(
                    VisionHeightProgress(
                        status,
                        state,
                        step,
                        message,
                        elapsed(),
                        height,
                        stable_reads,
                        dict(vacuum_states),
                        dict(motion_states),
                    )
                )

        def result(state: VisionHeightState, message: str) -> VisionHeightResult:
            status = lifecycle_status(state)
            self._set_status(status, step, message, height_mm=height)
            item = VisionHeightResult(status, state, step, message, elapsed(), height)
            log = LOGGER.info if item.succeeded else LOGGER.warning
            log(
                "VisionHeightFlow finished status=%s state=%s step=%s elapsed=%.3f height=%s message=%s",
                status.value,
                state.value,
                step,
                item.elapsed_seconds,
                height,
                message,
            )
            return item

        try:
            report(VisionHeightState.PRECHECK, "檢查 LiftService")
            self.service.precheck()
            height = self.service.read_height()
            self.service.stop()
            lift_touched = True
            self.service.configure_position(self.config.target_height_mm)

            step = "vacuum"
            self.service.set_vacuum(True)
            if cancel_event.wait(self.config.vacuum_build_seconds):
                return result(VisionHeightState.CANCELLED, "視覺高度流程已取消，真空保持開啟")
            vacuum_states = self.service.read_vacuum()
            if not all(vacuum_states.values()):
                failed = ", ".join(side for side, enabled in vacuum_states.items() if not enabled)
                raise RuntimeError(f"LiftService 真空命令未保持 ON: {failed}")
            report(VisionHeightState.VACUUM, "左右真空已開啟")

            step = "move_to_height"
            if abs(height - self.config.target_height_mm) > self.config.tolerance_mm:
                direction = "定位"
                active_motion_key = "up"
                self.service.start_up()
            else:
                direction = "到位"
                active_motion_key = ""

            stable_reads = 0
            deadline = started + self.config.timeout_seconds
            last_height = height
            last_height_change_at = time.monotonic()
            retrigger_count = 0
            while time.monotonic() < deadline:
                if cancel_event.is_set():
                    return result(VisionHeightState.CANCELLED, "視覺高度流程已取消，真空保持開啟")
                height = self.service.read_height()
                motion_states = self.service.read_motion_commands()
                if last_height is None or abs(height - last_height) > self.config.tolerance_mm:
                    last_height = height
                    last_height_change_at = time.monotonic()
                if abs(height - self.config.target_height_mm) <= self.config.tolerance_mm:
                    stable_reads += 1
                    self.service.stop()
                    report(
                        VisionHeightState.SETTLING,
                        f"高度穩定 {stable_reads}/{self.config.stable_read_count}：{height:g}mm",
                        stable_reads,
                    )
                    if cancel_event.wait(self.config.settle_seconds):
                        return result(VisionHeightState.CANCELLED, "視覺高度流程已取消，真空保持開啟")
                    height = self.service.read_height()
                    if abs(height - self.config.target_height_mm) <= self.config.tolerance_mm:
                        return result(VisionHeightState.SUCCESS, f"視覺高度定位完成：{height:g}mm")
                    stable_reads = 0
                    last_height = height
                    last_height_change_at = time.monotonic()
                    if abs(height - self.config.target_height_mm) > self.config.tolerance_mm:
                        self.service.configure_position(self.config.target_height_mm)
                        self.service.start_up()
                else:
                    unchanged_seconds = time.monotonic() - last_height_change_at
                    if (
                        active_motion_key
                        and not motion_states.get(active_motion_key, False)
                        and unchanged_seconds >= self.config.unchanged_timeout_seconds
                    ):
                        if retrigger_count < self.config.retrigger_limit:
                            retrigger_count += 1
                            self.service.configure_position(self.config.target_height_mm)
                            self.service.start_up()
                            motion_states = self.service.read_motion_commands()
                            last_height_change_at = time.monotonic()
                            report(
                                VisionHeightState.MOVING,
                                (
                                    f"D52 尚未到 {self.config.target_height_mm:g}mm，"
                                    f"重新觸發 M376 {retrigger_count}/{self.config.retrigger_limit}："
                                    f"{height:g}mm → {self.config.target_height_mm:g}mm"
                                ),
                            )
                            cancel_event.wait(self.config.poll_interval_seconds)
                            continue
                        return result(
                            VisionHeightState.ERROR,
                            (
                                f"PLC {direction}定位命令已結束，且高度 {unchanged_seconds:g} 秒未變，"
                                f"目前停在 {height:g}mm，"
                                f"未到目標 {self.config.target_height_mm:g}mm"
                            ),
                        )
                    stable_reads = 0
                    report(
                        VisionHeightState.MOVING,
                        f"PLC 指定位置{direction}中：{height:g}mm → {self.config.target_height_mm:g}mm",
                    )
                cancel_event.wait(self.config.poll_interval_seconds)

            return result(
                VisionHeightState.TIMEOUT,
                f"等待視覺高度超過 {self.config.timeout_seconds:g} 秒，真空保持開啟",
            )
        except Exception as exc:
            return result(VisionHeightState.ERROR, f"VisionHeightFlow.{step}: {exc}")
        finally:
            if lift_touched:
                try:
                    self.service.stop()
                except Exception as exc:
                    LOGGER.exception("VisionHeightFlow cleanup failed: %s", exc)
