from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, fields
from enum import Enum
from typing import Any, Callable, Protocol

try:
    from config.loader import CONFIG_STORE, ConfigStore
    from lifecycle import LifecycleStatus, LifecycleTracked
except ModuleNotFoundError:
    from plc2.config.loader import CONFIG_STORE, ConfigStore
    from plc2.lifecycle import LifecycleStatus, LifecycleTracked


LOGGER = logging.getLogger(__name__)


class HomeServiceProtocol(Protocol):
    def precheck(self) -> None: ...
    def set_home_commands(self, enabled: bool) -> None: ...
    def read_positions(self) -> dict[str, float]: ...


class HomeFlowState(str, Enum):
    PRECHECK = "precheck"
    COMMANDING = "commanding"
    WAITING = "waiting"
    SUCCESS = "success"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    ERROR = "error"


@dataclass(frozen=True)
class HomeFlowConfig:
    pulse_seconds: float = 0.2
    poll_interval_seconds: float = 0.2
    timeout_seconds: float = 90.0
    position_tolerance_mm: float = 0.0
    stable_read_count: int = 3
    read_error_limit: int = 3

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "HomeFlowConfig":
        raw = store.get_flow("home")
        if raw is None:
            raise ValueError("找不到 config/flows/home.yml")
        allowed = {field.name for field in fields(cls)}
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise ValueError(f"home flow 未知設定: {', '.join(unknown)}")
        config = cls(**raw)
        config.validate()
        return config

    def validate(self) -> None:
        if self.pulse_seconds <= 0 or self.poll_interval_seconds <= 0 or self.timeout_seconds <= 0:
            raise ValueError("home flow 時間設定必須大於 0")
        if self.position_tolerance_mm < 0 or self.stable_read_count <= 0 or self.read_error_limit <= 0:
            raise ValueError("home flow 容許誤差與次數設定無效")


@dataclass(frozen=True)
class HomeFlowProgress:
    status: LifecycleStatus
    state: HomeFlowState
    step: str
    message: str
    elapsed_seconds: float
    positions: dict[str, float]
    stable_reads: int = 0


@dataclass(frozen=True)
class HomeFlowResult:
    status: LifecycleStatus
    state: HomeFlowState
    step: str
    message: str
    elapsed_seconds: float
    positions: dict[str, float]

    @property
    def succeeded(self) -> bool:
        return self.status is LifecycleStatus.SUCCESS


class HomeFlow(LifecycleTracked):
    def __init__(self, service: HomeServiceProtocol, config: HomeFlowConfig | None = None) -> None:
        self.service = service
        self.config = config or HomeFlowConfig.load()
        self.config.validate()
        self._init_status_tracker("HomeFlow")

    def run(
        self,
        cancel_event: threading.Event,
        progress_callback: Callable[[HomeFlowProgress], None] | None = None,
    ) -> HomeFlowResult:
        started = time.monotonic()
        positions: dict[str, float] = {}
        step = "precheck"
        commands_touched = False

        def lifecycle_status(state: HomeFlowState) -> LifecycleStatus:
            return {
                HomeFlowState.PRECHECK: LifecycleStatus.RUNNING,
                HomeFlowState.COMMANDING: LifecycleStatus.RUNNING,
                HomeFlowState.WAITING: LifecycleStatus.WAITING_SIGNAL,
                HomeFlowState.SUCCESS: LifecycleStatus.SUCCESS,
                HomeFlowState.CANCELLED: LifecycleStatus.CANCELLED,
                HomeFlowState.TIMEOUT: LifecycleStatus.TIMEOUT,
                HomeFlowState.ERROR: LifecycleStatus.ERROR,
            }[state]

        def elapsed() -> float:
            return time.monotonic() - started

        def report(state: HomeFlowState, message: str, stable_reads: int = 0) -> None:
            status = lifecycle_status(state)
            self._set_status(status, step, message, positions=dict(positions))
            if progress_callback is not None:
                progress_callback(
                    HomeFlowProgress(
                        status,
                        state,
                        step,
                        message,
                        elapsed(),
                        dict(positions),
                        stable_reads,
                    )
                )

        def result(state: HomeFlowState, message: str) -> HomeFlowResult:
            status = lifecycle_status(state)
            self._set_status(status, step, message, positions=dict(positions))
            item = HomeFlowResult(status, state, step, message, elapsed(), dict(positions))
            log = LOGGER.info if item.succeeded else LOGGER.warning
            log(
                "HomeFlow finished status=%s state=%s step=%s elapsed=%.3f positions=%s message=%s",
                status.value,
                state.value,
                step,
                item.elapsed_seconds,
                positions,
                message,
            )
            return item

        try:
            report(HomeFlowState.PRECHECK, "檢查 HomeService")
            self.service.precheck()
            positions = self.service.read_positions()
            if cancel_event.is_set():
                return result(HomeFlowState.CANCELLED, "一鍵回原點已取消")

            step = "command_home"
            report(HomeFlowState.COMMANDING, "送出 X、Y1、Y2 回原點命令")
            commands_touched = True
            self.service.set_home_commands(True)
            if cancel_event.wait(self.config.pulse_seconds):
                return result(HomeFlowState.CANCELLED, "一鍵回原點已取消")
            self.service.set_home_commands(False)

            step = "wait_zero"
            stable_reads = 0
            read_errors = 0
            deadline = started + self.config.timeout_seconds
            while time.monotonic() < deadline:
                if cancel_event.is_set():
                    return result(HomeFlowState.CANCELLED, "一鍵回原點已取消")
                try:
                    positions = self.service.read_positions()
                    read_errors = 0
                except Exception as exc:
                    read_errors += 1
                    report(HomeFlowState.WAITING, f"位置讀取失敗，重試 {read_errors}/{self.config.read_error_limit}")
                    if read_errors >= self.config.read_error_limit:
                        raise RuntimeError(f"HomeService 連續讀取失敗: {exc}") from exc
                    cancel_event.wait(self.config.poll_interval_seconds)
                    continue

                all_zero = all(
                    abs(value) <= self.config.position_tolerance_mm for value in positions.values()
                )
                stable_reads = stable_reads + 1 if all_zero else 0
                values = "  ".join(f"{axis}={value:g}" for axis, value in positions.items())
                report(
                    HomeFlowState.WAITING,
                    f"等待回原點：{values}，穩定 {stable_reads}/{self.config.stable_read_count}",
                    stable_reads,
                )
                if stable_reads >= self.config.stable_read_count:
                    return result(HomeFlowState.SUCCESS, "X、Y1、Y2 已穩定回原點")
                cancel_event.wait(self.config.poll_interval_seconds)

            return result(
                HomeFlowState.TIMEOUT,
                f"等待回原點超過 {self.config.timeout_seconds:g} 秒",
            )
        except Exception as exc:
            return result(HomeFlowState.ERROR, f"HomeFlow.{step}: {exc}")
        finally:
            if commands_touched:
                try:
                    self.service.set_home_commands(False)
                except Exception as exc:
                    LOGGER.exception("HomeFlow cleanup failed: %s", exc)
