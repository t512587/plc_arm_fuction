"""Headless LView left-to-right automatic transfer task.

This module owns the command-line task state.  It deliberately reuses the
existing PLC services and arm/vision workflow rather than duplicating D/M or
CAN commands in a new script.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any, Callable, Protocol

try:
    from config.loader import CONFIG_STORE
    from service.middle_vacuum_service import MiddleVacuumMode
    from service.pallet_transfer_service import PalletAction
except ModuleNotFoundError:
    from plc2.config.loader import CONFIG_STORE
    from plc2.service.middle_vacuum_service import MiddleVacuumMode
    from plc2.service.pallet_transfer_service import PalletAction


RUNTIME_PATH = Path(__file__).resolve().parents[1] / "runtime" / "auto_transfer_last.json"


class LiftProtocol(Protocol):
    def precheck(self) -> None: ...
    def read_height(self) -> float: ...
    def configure_position(self, target_height_mm: float, speed: float | None = None) -> None: ...
    def start_vision_positioning(self) -> None: ...
    def stop(self) -> None: ...


class PalletProtocol(Protocol):
    def precheck(self) -> None: ...
    def set_forward_position(self, slot: str, forward_mm: float) -> None: ...
    def start_forward(self, slot: str) -> None: ...
    def stop_forward(self, slot: str | None = None) -> None: ...
    def read_position(self, slot: str) -> float: ...
    def set_action_output(self, slot: str, action: PalletAction | str) -> dict: ...


class MiddleVacuumProtocol(Protocol):
    def set_mode(self, mode: MiddleVacuumMode | str) -> dict: ...
    def wait_transfer_ready(self, *, vacuum_expected: bool, cancel_event: threading.Event, progress_callback=None) -> dict: ...


class ArmProtocol(Protocol):
    def move_standby_pose(self, pose_name: str = "STANDBY") -> dict: ...
    def move_home_component(self, component: str) -> dict: ...
    def confirm_home(self, cancel_event: threading.Event | None = None) -> dict: ...
    def run_pick_and_place(self, cancel_event: threading.Event, *, pick_handoff, place_handoff, movement_handoff=None, view=None, progress_callback=None) -> dict: ...


@dataclass(frozen=True)
class AutoTransferConfig:
    lowest_height_mm: float = 0.0
    arm_safe_height_mm: float = 560.0
    cross_side_safe_height_mm: float = 460.0
    height_tolerance_mm: float = 3.0
    position_tolerance_mm: float = 1.0
    stable_reads: int = 2
    poll_interval_seconds: float = 0.1
    lift_timeout_seconds: float = 30.0
    pallet_timeout_seconds: float = 120.0
    pallet_action_delay_seconds: float = 3.0
    release_hold_seconds: float = 10.0
    release_retract_speed: float = 100.0

    @classmethod
    def load(cls) -> "AutoTransferConfig":
        raw = CONFIG_STORE.get_service("auto_transfer") or {}
        allowed = {item.name for item in fields(cls)}
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise ValueError("auto_transfer 未知設定: " + ", ".join(unknown))
        return cls(**raw)


class CheckpointStore:
    def __init__(self, path: Path = RUNTIME_PATH) -> None:
        self.path = path

    def save(self, data: dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(self.path)

    def load(self) -> dict[str, Any] | None:
        if not self.path.exists():
            return None
        return json.loads(self.path.read_text(encoding="utf-8"))


class AutoTransferTask:
    """Runs one LView left-to-right PLC + arm transfer without a GUI."""

    def __init__(
        self,
        lift: LiftProtocol,
        pallet: PalletProtocol,
        middle_vacuum: MiddleVacuumProtocol,
        arm: ArmProtocol,
        *,
        config: AutoTransferConfig | None = None,
        checkpoint_store: CheckpointStore | None = None,
        plc_snapshot_provider: Callable[[], dict[str, Any]] | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.lift = lift
        self.pallet = pallet
        self.middle_vacuum = middle_vacuum
        self.arm = arm
        self.config = config or AutoTransferConfig.load()
        self.store = checkpoint_store or CheckpointStore()
        self.clock = clock
        self.sleep = sleep
        self.plc_snapshot_provider = plc_snapshot_provider
        self.cancel_event = threading.Event()
        self.stage = "CREATED"
        self.arm_state: dict[str, Any] | None = None

    def cancel(self) -> None:
        self.cancel_event.set()

    def run(self, *, height_mm: float, forward_mm: float) -> dict[str, Any]:
        if height_mm < 0:
            raise ValueError("--height 必須大於或等於 0")
        if forward_mm < 0:
            raise ValueError("--forward-mm 必須大於或等於 0")
        started_at = time.time()
        try:
            self._stage("PRECHECK")
            self.lift.precheck()
            self.pallet.precheck()
            self._move_lift(self.config.lowest_height_mm)
            self._stage("STANDBY")
            self.arm_state = self.arm.move_standby_pose()

            self._stage("LOAD_BOTH")
            self._move_lift(height_mm)
            self._run_both(PalletAction.SUCK, forward_mm)
            self._move_lift(self.config.lowest_height_mm)

            self._stage("VISION_PREP")
            self._move_lift(self.config.arm_safe_height_mm)
            # Safe HOME ordering: small arms (ID143+ID144) retract first to
            # clear the pillar collision zone, then big arms (ID142+ID145).
            self.arm_state = self.arm.move_home_component("small")
            self.arm_state = self.arm.move_home_component("big")
            self.arm_state = self.arm.confirm_home(self.cancel_event)

            self._stage("LVIEW_TRANSFER")
            self.arm.run_pick_and_place(
                self.cancel_event,
                view="LView",
                movement_handoff=self._movement_gate,
                pick_handoff=self._pick_handoff,
                place_handoff=self._place_handoff,
            )
            # Persist a fresh four-axis readback after the subprocess claims it
            # has returned HOME.  This is the recovery reference after a stop.
            self.arm_state = self.arm.confirm_home(self.cancel_event)
            self._move_lift(self.config.lowest_height_mm)

            self._stage("PUSH_BOTH")
            self._move_lift(height_mm)
            self._run_both(PalletAction.PUSH, forward_mm)
            self._move_lift(self.config.lowest_height_mm)
            self._stage("COMPLETED", status="success", started_at=started_at)
            return self.store.load() or {}
        except Exception as exc:
            self._safe_stop()
            self._stage("CANCELLED" if self.cancel_event.is_set() else "FAILED", status="cancelled" if self.cancel_event.is_set() else "error", error=str(exc), started_at=started_at)
            raise

    def _pick_handoff(self, target_height_mm: float, _target: dict) -> None:
        self._stage("PICK_HANDOFF")
        self._move_lift(target_height_mm)
        self.middle_vacuum.set_mode(MiddleVacuumMode.VACUUM)
        self.middle_vacuum.wait_transfer_ready(vacuum_expected=True, cancel_event=self.cancel_event)
        self._move_lift(self.config.cross_side_safe_height_mm)

    def _place_handoff(self, target_height_mm: float, _target: dict) -> None:
        # --- PLACE_HOLD: 手臂保持放料姿態，平台到放料高度並確認穩定 ---
        self._stage("PLACE_HOLD")
        self._move_lift(target_height_mm)

        # --- RELEASE_START: M54 OFF → M56 ON，開始洩氣 ---
        self._stage("RELEASE_START")
        started = self.clock()
        self.middle_vacuum.set_mode(MiddleVacuumMode.BREAK_VACUUM)
        self.sleep(1.0)  # 洩氣等待：手臂定格、平台尚在放料高度

        # --- RELEASE_LOWER: 手臂定格，平台慢速回安全高度 560mm ---
        self._stage("RELEASE_LOWER")
        self._move_lift(self.config.arm_safe_height_mm, speed=self.config.release_retract_speed)
        self._confirm_lift(self.config.arm_safe_height_mm)

        # 補足總等待時間到 release_hold_seconds，期間支援取消
        remaining = self.config.release_hold_seconds - (self.clock() - started)
        if remaining > 0:
            self._wait_cancel(remaining)

        # --- RELEASE_CONFIRMED: 驗證 M56 OFF 讀回後才放行 ---
        state = self.middle_vacuum.set_mode(MiddleVacuumMode.OFF)
        if state.get("break_vacuum_on"):
            raise RuntimeError("RELEASE_CONFIRMED 失敗：M56 仍為 ON")
        self._stage("RELEASE_CONFIRMED")

    def _movement_gate(self, movement: str) -> None:
        # The arm subprocess is paused at every marker.  It may continue only
        # when the lift is stationary at the height safe for that segment.
        if "Move LGrap" in movement or "Move RGrap" in movement:
            target = self.config.arm_safe_height_mm
        elif "Move left target" in movement or "Move right target" in movement:
            target = self.config.cross_side_safe_height_mm
        elif "PICK DONE" in movement:
            target = self.config.cross_side_safe_height_mm
        else:
            target = self.config.arm_safe_height_mm
        self._confirm_lift(target)

    def _run_both(self, action: PalletAction, forward_mm: float) -> None:
        for slot in ("Y1", "Y2"):
            self.pallet.set_forward_position(slot, forward_mm)
            self.pallet.set_action_output(slot, action)
        for slot in ("Y1", "Y2"):
            self.pallet.start_forward(slot)
        self._wait_pallets(forward_mm)
        self._wait_cancel(self.config.pallet_action_delay_seconds)
        for slot in ("Y1", "Y2"):
            self.pallet.stop_forward(slot)
            self.pallet.set_forward_position(slot, 0.0)
            self.pallet.start_forward(slot)
        self._wait_pallets(0.0)
        self.pallet.stop_forward()
        if action is PalletAction.PUSH:
            for slot in ("Y1", "Y2"):
                self.pallet.set_action_output(slot, PalletAction.NONE)

    def _move_lift(self, target: float, *, speed: float | None = None) -> None:
        self._check_cancel()
        self.lift.configure_position(target, speed)
        self.lift.start_vision_positioning()
        self._wait_lift(target)
        self.lift.stop()
        self._confirm_lift(target)

    def _wait_lift(self, target: float) -> None:
        deadline = self.clock() + self.config.lift_timeout_seconds
        stable = 0
        while self.clock() < deadline:
            self._check_cancel()
            stable = stable + 1 if abs(self.lift.read_height() - target) <= self.config.height_tolerance_mm else 0
            if stable >= self.config.stable_reads:
                return
            self.sleep(self.config.poll_interval_seconds)
        raise TimeoutError(f"升降機未在 {self.config.lift_timeout_seconds:g} 秒內到達 {target:g}mm")

    def _confirm_lift(self, target: float) -> None:
        stable = 0
        max_retries = self.config.stable_reads * 3
        attempts = 0
        while stable < self.config.stable_reads:
            self._check_cancel()
            if abs(self.lift.read_height() - target) > self.config.height_tolerance_mm:
                attempts += 1
                if attempts >= max_retries:
                    raise RuntimeError(f"升降機停止後未在安全高度 {target:g}mm（連續 {max_retries} 次讀值超出容差 {self.config.height_tolerance_mm:g}mm）")
                stable = 0  # reset, retry
                self.sleep(self.config.poll_interval_seconds)
                continue
            stable += 1
            if stable < self.config.stable_reads:
                self.sleep(self.config.poll_interval_seconds)

    def _wait_pallets(self, target: float) -> None:
        deadline = self.clock() + self.config.pallet_timeout_seconds
        stable = 0
        while self.clock() < deadline:
            self._check_cancel()
            arrived = all(abs(self.pallet.read_position(slot) - target) <= self.config.position_tolerance_mm for slot in ("Y1", "Y2"))
            stable = stable + 1 if arrived else 0
            if stable >= self.config.stable_reads:
                return
            self.sleep(self.config.poll_interval_seconds)
        raise TimeoutError(f"Y1/Y2 未在 {self.config.pallet_timeout_seconds:g} 秒內到達 {target:g}mm")

    def _safe_stop(self) -> None:
        for stop in (self.lift.stop, self.pallet.stop_forward):
            try:
                stop()
            except Exception:
                pass

    def _check_cancel(self) -> None:
        if self.cancel_event.is_set():
            raise RuntimeError("自動任務已取消")

    def _wait_cancel(self, seconds: float) -> None:
        deadline = self.clock() + seconds
        while self.clock() < deadline:
            self._check_cancel()
            self.sleep(min(self.config.poll_interval_seconds, deadline - self.clock()))

    def _stage(self, stage: str, *, status: str = "running", error: str | None = None, started_at: float | None = None) -> None:
        self.stage = stage
        snapshot: dict[str, Any] = {"stage": stage, "status": status, "updated_at": time.time(), "config": asdict(self.config)}
        if started_at is not None:
            snapshot["started_at"] = started_at
        if error is not None:
            snapshot["error"] = error
        try:
            snapshot["plc"] = {"lift_height_mm": self.lift.read_height(), "y1_mm": self.pallet.read_position("Y1"), "y2_mm": self.pallet.read_position("Y2")}
        except Exception as exc:
            snapshot["snapshot_error"] = str(exc)
        if self.plc_snapshot_provider is not None:
            try:
                snapshot["plc_outputs"] = self.plc_snapshot_provider()
            except Exception as exc:
                snapshot["plc_outputs_error"] = str(exc)
        if self.arm_state is not None:
            snapshot["arm"] = self.arm_state
        self.store.save(snapshot)
