from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, fields, replace
from enum import Enum
from typing import Callable, Protocol

try:
    from config.loader import CONFIG_STORE, ConfigStore
    from lifecycle import LifecycleStatus, LifecycleTracked
    from service.arm_camera_home_interlock import (
        ARM_CAMERA_HOME_INTERLOCK,
        ARM_CAMERA_NOT_HOME_MAXIMUM_HEIGHT_MM,
        ArmCameraHomeInterlock,
    )
    from service.middle_vacuum_service import MiddleVacuumMode
    from service.pallet_transfer_service import PalletAction
except ModuleNotFoundError:
    from plc2.config.loader import CONFIG_STORE, ConfigStore
    from plc2.lifecycle import LifecycleStatus, LifecycleTracked
    from plc2.service.arm_camera_home_interlock import (
        ARM_CAMERA_HOME_INTERLOCK,
        ARM_CAMERA_NOT_HOME_MAXIMUM_HEIGHT_MM,
        ArmCameraHomeInterlock,
    )
    from plc2.service.middle_vacuum_service import MiddleVacuumMode
    from plc2.service.pallet_transfer_service import PalletAction


LOGGER = logging.getLogger(__name__)
SECOND_STEP_MAXIMUM_HEIGHT_MM = ARM_CAMERA_NOT_HOME_MAXIMUM_HEIGHT_MM
SECOND_STEP_ARM_MOTION_SAFE_HEIGHT_MM = 560.0
SECOND_STEP_ARM_PICK_APPROACH_HEIGHT_MM = 460.0


class LiftServiceProtocol(Protocol):
    def precheck(self) -> None: ...
    def read_height(self) -> float: ...
    def configure_position(self, target_height_mm: float, speed: float | None = None) -> None: ...
    def start_vision_positioning(self) -> None: ...
    def read_motion_commands(self) -> dict[str, bool]: ...
    def stop(self) -> None: ...


class PalletTransferServiceProtocol(Protocol):
    def precheck(self) -> None: ...
    def set_forward_position(self, slot: str, forward_mm: float) -> None: ...
    def set_forward_speed(self, slot: str, speed: float) -> None: ...
    def start_forward(self, slot: str) -> None: ...
    def stop_forward(self, slot: str | None = None) -> None: ...
    def read_forward_commands(self) -> dict[str, bool]: ...
    def read_position(self, slot: str) -> float: ...
    def set_action_output(self, slot: str, action: PalletAction | str) -> dict[str, bool | str]: ...


class SlotVacuumServiceProtocol(Protocol):
    def hold_for_main_cycle(self, side: str): ...
    def release_for_main_cycle(self, side: str): ...
    def shutdown_after_cycle(self): ...


class MiddleVacuumServiceProtocol(Protocol):
    def set_mode(self, mode: MiddleVacuumMode | str) -> dict[str, bool | str]: ...
    def wait_transfer_ready(
        self,
        *,
        vacuum_expected: bool,
        cancel_event: threading.Event,
        progress_callback: Callable[[str], None] | None = None,
    ) -> dict: ...


class VisionBridgeServiceProtocol(Protocol):
    def start(self) -> None: ...
    def notify(self, phase: str, **payload) -> None: ...
    def wait_signal(self, cancel_event: threading.Event, timeout_seconds: float | None = None) -> None: ...
    def wait_height(self, cancel_event: threading.Event, timeout_seconds: float | None = None) -> float: ...
    def wait_done(self, cancel_event: threading.Event, timeout_seconds: float | None = None) -> None: ...


class ArmVisionWorkflowServiceProtocol(Protocol):
    def run_pick_and_place(
        self,
        cancel_event: threading.Event,
        *,
        pick_handoff: Callable[[float, dict], None],
        place_handoff: Callable[[float, dict], None],
        movement_handoff: Callable[[str], None] | None = None,
        view: str | None = None,
        height_reference_depth_mm: float | None = None,
        progress_callback: Callable[[str], None] | None = None,
    ) -> dict: ...


class TransferDirection(str, Enum):
    Y1_TO_Y2 = "Y1_TO_Y2"
    Y2_TO_Y1 = "Y2_TO_Y1"

    @classmethod
    def parse(cls, value: "TransferDirection | str") -> "TransferDirection":
        if isinstance(value, cls):
            return value
        normalized = str(value).strip().upper().replace(" ", "")
        aliases = {
            "Y1_TO_Y2": cls.Y1_TO_Y2,
            "Y1→Y2": cls.Y1_TO_Y2,
            "Y1->Y2": cls.Y1_TO_Y2,
            "Y2_TO_Y1": cls.Y2_TO_Y1,
            "Y2→Y1": cls.Y2_TO_Y1,
            "Y2->Y1": cls.Y2_TO_Y1,
        }
        try:
            return aliases[normalized]
        except KeyError as exc:
            raise ValueError(f"不支援的第二步搬運方向：{value}") from exc

    @property
    def camera_view(self) -> str:
        # Camera team's side naming is opposite to the PLC Y-axis naming:
        # RView is the Y1 side, and LView is the Y2 side.
        return "RView" if self is self.Y1_TO_Y2 else "LView"

    @property
    def display_name(self) -> str:
        return "Y1 → Y2" if self is self.Y1_TO_Y2 else "Y2 → Y1"


class MainCycleState(str, Enum):
    IDLE = "idle"
    PRECHECK = "precheck"
    STEP1 = "step1"
    STEP2 = "step2"
    WAITING_FINAL_STEP1 = "waiting_final_step1"
    SUCCESS = "success"
    CANCELLED = "cancelled"
    STOP_UNCONFIRMED = "stop_unconfirmed"
    TIMEOUT = "timeout"
    ERROR = "error"


class MainCyclePhase(str, Enum):
    READY_FIRST_STEP = "ready_first_step"
    WAITING_STEP2 = "waiting_step2"
    WAITING_FINAL_STEP1 = "waiting_final_step1"
    COMPLETE = "complete"


@dataclass(frozen=True)
class Step1Command:
    slot: str
    action: str
    height_mm: float
    forward_mm: float | None = None
    x_speed: int | None = None
    y1_speed: int | None = None

    def normalized_slot(self) -> str:
        return self.slot.strip().upper()

    def normalized_action(self) -> PalletAction:
        return PalletAction(self.action.strip().lower())

    @property
    def is_pause(self) -> bool:
        return self.normalized_slot() == "NONE" or self.normalized_action() is PalletAction.NONE

    def validate(self) -> None:
        slot = self.normalized_slot()
        action = self.normalized_action()
        if slot not in {"Y1", "Y2", "NONE"}:
            raise ValueError(f"不支援的貨盤選擇: {self.slot}")
        if action is not PalletAction.NONE and slot == "NONE":
            raise ValueError("選擇 none 貨盤時，動作也必須是 none")
        if slot != "NONE" and action is PalletAction.NONE:
            raise ValueError("選擇 Y1/Y2 時，動作不可是 none")
        if not self.is_pause and self.forward_mm is None:
            raise ValueError("吸/推流程需要輸入 D510/D560 前進距離")
        for name, value in (("x_speed", self.x_speed), ("y1_speed", self.y1_speed)):
            if value is not None and not 1 <= value <= 32767:
                raise ValueError(f"{name} 必須介於 1～32767")


@dataclass(frozen=True)
class MainCycleConfig:
    vision_height_mm: float = 560.0
    cross_side_safe_height_mm: float = 460.0
    height_tolerance_mm: float = 1.0
    y_position_tolerance_mm: float = 1.0
    stable_read_count: int = 2
    poll_interval_seconds: float = 0.1
    action_delay_seconds: float = 3.0
    motion_timeout_seconds: float = 30.0
    y_motion_timeout_seconds: float = 120.0
    vision_signal_timeout_seconds: float = 30.0

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "MainCycleConfig":
        raw = store.get_flow("main_cycle")
        if raw is None:
            return cls()
        allowed = {field.name for field in fields(cls)}
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise ValueError(f"main cycle flow 未知設定: {', '.join(unknown)}")
        config = cls(**raw)
        config.validate()
        return config

    def validate(self) -> None:
        if self.vision_height_mm < 0 or self.cross_side_safe_height_mm < 0:
            raise ValueError("視覺高度與跨側安全高度不可小於 0")
        if self.height_tolerance_mm < 0 or self.y_position_tolerance_mm < 0:
            raise ValueError("容許誤差不可小於 0")
        if self.stable_read_count <= 0:
            raise ValueError("穩定讀取次數必須大於 0")
        for value in (
            self.poll_interval_seconds,
            self.action_delay_seconds,
            self.motion_timeout_seconds,
            self.y_motion_timeout_seconds,
            self.vision_signal_timeout_seconds,
        ):
            if value < 0:
                raise ValueError("時間設定不可小於 0")


@dataclass(frozen=True)
class MainCycleProgress:
    status: LifecycleStatus
    state: MainCycleState
    phase: MainCyclePhase
    step: str
    message: str
    elapsed_seconds: float
    height_mm: float | None
    vision_height_mm: float | None


@dataclass(frozen=True)
class MainCycleResult:
    status: LifecycleStatus
    state: MainCycleState
    phase: MainCyclePhase
    step: str
    message: str
    elapsed_seconds: float
    height_mm: float | None
    vision_height_mm: float | None
    transfer_direction: str | None = None

    @property
    def succeeded(self) -> bool:
        return self.status is LifecycleStatus.SUCCESS


class MainCycleFlow(LifecycleTracked):
    def __init__(
        self,
        lift_service: LiftServiceProtocol,
        pallet_service: PalletTransferServiceProtocol,
        middle_vacuum_service: MiddleVacuumServiceProtocol,
        vision_bridge_service: VisionBridgeServiceProtocol,
        config: MainCycleConfig | None = None,
        arm_vision_service: ArmVisionWorkflowServiceProtocol | None = None,
        home_interlock: ArmCameraHomeInterlock = ARM_CAMERA_HOME_INTERLOCK,
        slot_vacuum_service: SlotVacuumServiceProtocol | None = None,
    ) -> None:
        self.lift_service = lift_service
        self.pallet_service = pallet_service
        self.middle_vacuum_service = middle_vacuum_service
        self.vision_bridge_service = vision_bridge_service
        self.arm_vision_service = arm_vision_service
        self.home_interlock = home_interlock
        self.slot_vacuum_service = slot_vacuum_service
        self.config = config or MainCycleConfig.load()
        self.config.validate()
        self.phase = MainCyclePhase.READY_FIRST_STEP
        self._vision_height_1: float | None = None
        self._init_status_tracker("MainCycleFlow")

    def reset(self) -> None:
        self.phase = MainCyclePhase.READY_FIRST_STEP
        self._vision_height_1 = None
        self._set_status(LifecycleStatus.PENDING, "reset", "總流程已重置")

    def begin_independent_precheck(self, step_label: str) -> None:
        self._set_status(
            LifecycleStatus.RUNNING,
            "independent_home_precheck",
            f"{step_label}：正在確認四軸 HOME",
            phase=self.phase.value,
        )

    def finish_independent_precheck(
        self,
        status: LifecycleStatus,
        message: str,
    ) -> None:
        self._set_status(
            status,
            "independent_home_precheck",
            message,
            phase=self.phase.value,
        )

    def run_first_step(
        self,
        command: Step1Command,
        cancel_event: threading.Event,
        progress_callback: Callable[[MainCycleProgress], None] | None = None,
    ) -> MainCycleResult:
        if self.phase is not MainCyclePhase.READY_FIRST_STEP:
            return self._instant_error("first_step", f"目前階段是 {self.phase.value}，不能執行第一次第一步")
        result = self._run_step1(command, cancel_event, progress_callback, label="第一次第一步")
        if result.succeeded:
            self.phase = MainCyclePhase.WAITING_STEP2
            result = replace(result, phase=self.phase)
            self._set_status(
                result.status,
                result.step,
                result.message,
                phase=self.phase.value,
                height_mm=result.height_mm,
                vision_height_mm=result.vision_height_mm,
            )
        return result

    def run_second_step(
        self,
        cancel_event: threading.Event,
        progress_callback: Callable[[MainCycleProgress], None] | None = None,
        *,
        transfer_direction: TransferDirection | str = TransferDirection.Y1_TO_Y2,
        x_speed: int | None = None,
        height_reference_depth_mm: float | None = None,
    ) -> MainCycleResult:
        if self.phase is not MainCyclePhase.WAITING_STEP2:
            return self._instant_error("second_step", f"目前階段是 {self.phase.value}，不能執行第二步")
        try:
            direction = TransferDirection.parse(transfer_direction)
        except ValueError as exc:
            return self._instant_error("second_step_direction", str(exc))
        result = self._run_step2(
            cancel_event,
            progress_callback,
            direction,
            x_speed=x_speed,
            height_reference_depth_mm=height_reference_depth_mm,
        )
        if result.succeeded:
            self.phase = MainCyclePhase.WAITING_FINAL_STEP1
            result = replace(result, phase=self.phase)
            self._set_status(
                result.status,
                result.step,
                result.message,
                phase=self.phase.value,
                height_mm=result.height_mm,
                vision_height_mm=result.vision_height_mm,
                transfer_direction=direction.value,
                camera_view=direction.camera_view,
            )
        return result

    def run_final_step(
        self,
        command: Step1Command,
        cancel_event: threading.Event,
        progress_callback: Callable[[MainCycleProgress], None] | None = None,
    ) -> MainCycleResult:
        if self.phase is not MainCyclePhase.WAITING_FINAL_STEP1:
            return self._instant_error("final_step1", f"目前階段是 {self.phase.value}，不能執行第二次第一步")
        result = self._run_step1(command, cancel_event, progress_callback, label="第二次第一步")
        if result.succeeded:
            result = self._finish_third_step(result, completed_phase=MainCyclePhase.COMPLETE)
        return result

    def run_independent_first_step(
        self,
        command: Step1Command,
        cancel_event: threading.Event,
        progress_callback: Callable[[MainCycleProgress], None] | None = None,
    ) -> MainCycleResult:
        """Run the first block without advancing the guided three-step flow."""

        return self._run_step1(
            command,
            cancel_event,
            progress_callback,
            label="獨立第一步",
        )

    def run_independent_second_step(
        self,
        cancel_event: threading.Event,
        progress_callback: Callable[[MainCycleProgress], None] | None = None,
        *,
        transfer_direction: TransferDirection | str = TransferDirection.Y1_TO_Y2,
        x_speed: int | None = None,
        height_reference_depth_mm: float | None = None,
    ) -> MainCycleResult:
        """Run the second block without requiring or advancing a guided phase."""

        try:
            direction = TransferDirection.parse(transfer_direction)
        except ValueError as exc:
            return self._instant_error("independent_second_step_direction", str(exc))
        return self._run_step2(
            cancel_event,
            progress_callback,
            direction,
            x_speed=x_speed,
            height_reference_depth_mm=height_reference_depth_mm,
        )

    def prepare_independent_second_step_height(
        self,
        cancel_event: threading.Event,
        progress_callback: Callable[[MainCycleProgress], None] | None = None,
        *,
        x_speed: int | None = None,
    ) -> float:
        """Move to the fixed 560mm arm-safe height before the HOME precheck.

        Independent step 2 performs its HOME check at the API boundary.  The
        lift must therefore be positioned first, while no arm process is
        running, so pressing the block always raises or lowers the lift to the
        same safe height before any arm validation or motion.
        """

        step = "independent_second_step_to_safe_height"
        target_height_mm = SECOND_STEP_ARM_MOTION_SAFE_HEIGHT_MM

        def report(message: str) -> None:
            self._set_status(
                LifecycleStatus.RUNNING,
                step,
                f"獨立第二步：{message}",
                phase=self.phase.value,
                height_mm=None,
                vision_height_mm=self._vision_height_1,
            )
            if progress_callback is not None:
                progress_callback(
                    MainCycleProgress(
                        LifecycleStatus.RUNNING,
                        MainCycleState.STEP2,
                        self.phase,
                        step,
                        f"獨立第二步：{message}",
                        0.0,
                        None,
                        self._vision_height_1,
                    )
                )

        self._set_status(
            LifecycleStatus.RUNNING,
            step,
            f"獨立第二步：先將升降機定位到 {target_height_mm:g}mm",
            phase=self.phase.value,
            height_mm=None,
            vision_height_mm=self._vision_height_1,
        )
        try:
            self.lift_service.precheck()
            height = self._move_height(
                target_height_mm,
                cancel_event,
                report,
                speed=x_speed,
            )
            height = self._confirm_stopped_height(
                target_height_mm,
                cancel_event,
                report,
            )
        except Exception as exc:
            stop_errors = self._stop_and_confirm_motion(include_pallet=False)
            if stop_errors:
                raise RuntimeError(
                    "獨立第二步 560mm 定位失敗，且設備停止未確認："
                    + "；".join(stop_errors)
                ) from exc
            raise

        stop_errors = self._stop_and_confirm_motion(include_pallet=False)
        if stop_errors:
            raise RuntimeError(
                "獨立第二步已到 560mm，但設備停止未確認："
                + "；".join(stop_errors)
            )
        self._set_status(
            LifecycleStatus.RUNNING,
            step,
            f"獨立第二步：升降機已停止並確認在 {target_height_mm:g}±"
            f"{self.config.height_tolerance_mm:g}mm，接著確認四軸 HOME",
            phase=self.phase.value,
            height_mm=height,
            vision_height_mm=self._vision_height_1,
        )
        return height

    def run_independent_third_step(
        self,
        command: Step1Command,
        cancel_event: threading.Event,
        progress_callback: Callable[[MainCycleProgress], None] | None = None,
    ) -> MainCycleResult:
        """Run the third block independently and perform its vacuum shutdown."""

        result = self._run_step1(
            command,
            cancel_event,
            progress_callback,
            label="獨立第三步",
        )
        if result.succeeded:
            result = self._finish_third_step(result)
        return result

    def _finish_third_step(
        self,
        result: MainCycleResult,
        *,
        completed_phase: MainCyclePhase | None = None,
    ) -> MainCycleResult:
        try:
            if self.slot_vacuum_service is not None:
                self.slot_vacuum_service.shutdown_after_cycle()
            else:
                self.pallet_service.set_action_output("Y1", PalletAction.NONE)
                self.pallet_service.set_action_output("Y2", PalletAction.NONE)
            self.middle_vacuum_service.set_mode(MiddleVacuumMode.OFF)
        except Exception as exc:
            message = f"第三步真空收尾失敗：{exc}"
            result = replace(
                result,
                status=LifecycleStatus.ERROR,
                state=MainCycleState.ERROR,
                step="final_vacuum_shutdown",
                message=message,
            )
            self._set_status(
                result.status,
                result.step,
                result.message,
                phase=self.phase.value,
                height_mm=result.height_mm,
                vision_height_mm=result.vision_height_mm,
            )
            return result

        if completed_phase is not None:
            self.phase = completed_phase
        result = replace(
            result,
            phase=self.phase,
            message=f"{result.message}；Y1／Y2／M54／M56 已確認全部關閉",
        )
        self._set_status(
            result.status,
            result.step,
            result.message,
            phase=self.phase.value,
            height_mm=result.height_mm,
            vision_height_mm=result.vision_height_mm,
        )
        return result

    def _run_step1(
        self,
        command: Step1Command,
        cancel_event: threading.Event,
        progress_callback: Callable[[MainCycleProgress], None] | None,
        *,
        label: str,
    ) -> MainCycleResult:
        started = time.monotonic()
        state = MainCycleState.STEP1
        step = "step1_precheck"
        height: float | None = None
        vision_height = self._vision_height_1

        def elapsed() -> float:
            return time.monotonic() - started

        def report(message: str, status: LifecycleStatus = LifecycleStatus.RUNNING) -> None:
            self._set_status(status, step, message, phase=self.phase.value, height_mm=height, vision_height_mm=vision_height)
            if progress_callback is not None:
                progress_callback(MainCycleProgress(status, state, self.phase, step, message, elapsed(), height, vision_height))

        def result(result_state: MainCycleState, message: str, status: LifecycleStatus) -> MainCycleResult:
            self._set_status(
                status,
                step,
                message,
                state=result_state.value,
                phase=self.phase.value,
                height_mm=height,
                vision_height_mm=vision_height,
            )
            item = MainCycleResult(status, result_state, self.phase, step, message, elapsed(), height, vision_height)
            LOGGER.info("MainCycleFlow step1 finished status=%s state=%s step=%s message=%s", status.value, result_state.value, step, message)
            return item

        try:
            command.validate()
            step = "step1_precheck"
            report(f"{label}：檢查 service")
            self.lift_service.precheck()
            self.pallet_service.precheck()
            if command.is_pause:
                return result(MainCycleState.SUCCESS, f"{label} 已選 none，流程暫停/略過", LifecycleStatus.SUCCESS)

            step = "step1_move_height"
            height = self._move_height(
                command.height_mm,
                cancel_event,
                lambda message: report(f"{label}：{message}"),
                speed=command.x_speed,
            )

            step = "step1_pallet_action"
            slot = command.normalized_slot()
            action = command.normalized_action()
            forward_mm = float(command.forward_mm)
            report(f"{label}：寫入 {slot} 前進距離 {forward_mm:g}mm")
            self.pallet_service.set_forward_position(slot, forward_mm)
            if slot == "Y1" and command.y1_speed is not None:
                report(f"{label}：寫入並確認 Y1 前進速度 {command.y1_speed}")
                self.pallet_service.set_forward_speed(slot, command.y1_speed)
            self.pallet_service.start_forward(slot)
            output_slot = (
                "Y2" if slot == "Y1" else "Y1"
            ) if action is PalletAction.SUCK else slot
            if action is PalletAction.SUCK:
                selected_state = self.pallet_service.set_action_output(
                    slot,
                    PalletAction.SUCK,
                )
                if selected_state.get("mode") != PalletAction.SUCK.value:
                    raise RuntimeError(
                        f"{slot} 吸真空讀回失敗：{selected_state}"
                    )
                report(
                    f"{label}：選擇 {slot} 吸取，已關閉該側破真空並開啟吸真空；"
                    f"同時持續保持另一側 {output_slot} 真空"
                )
                if self.slot_vacuum_service is not None:
                    self.slot_vacuum_service.hold_for_main_cycle(output_slot)
                else:
                    self.pallet_service.set_action_output(
                        output_slot,
                        PalletAction.SUCK,
                    )
            elif action is PalletAction.PUSH:
                report(
                    f"{label}：確認 {slot} 放回貨架，關閉真空並開啟破真空"
                )
                if self.slot_vacuum_service is not None:
                    self.slot_vacuum_service.release_for_main_cycle(slot)
                else:
                    self.pallet_service.set_action_output(
                        slot,
                        PalletAction.PUSH,
                    )

            step = "step1_wait_y_forward_position"
            self._wait_pallet_position(
                slot,
                forward_mm,
                cancel_event,
                lambda message: report(f"{label}：{message}"),
                movement_name="前進",
            )

            step = "step1_action_delay"
            if cancel_event.wait(self.config.action_delay_seconds):
                return result(MainCycleState.CANCELLED, f"{label} 已取消", LifecycleStatus.CANCELLED)
            self.pallet_service.stop_forward(slot)
            self.pallet_service.set_forward_position(slot, 0.0)
            if slot == "Y1" and command.y1_speed is not None:
                self.pallet_service.set_forward_speed(slot, command.y1_speed)
            self.pallet_service.start_forward(slot)

            step = "step1_wait_y_position"
            self._wait_pallet_home(slot, cancel_event, lambda message: report(f"{label}：{message}"))
            self.pallet_service.stop_forward(slot)
            return result(MainCycleState.SUCCESS, f"{label} 完成", LifecycleStatus.SUCCESS)
        except TimeoutError as exc:
            if cancel_event.is_set():
                return result(
                    MainCycleState.CANCELLED,
                    f"{label} 已取消：{exc}",
                    LifecycleStatus.CANCELLED,
                )
            return result(MainCycleState.TIMEOUT, f"MainCycleFlow.{step}: {exc}", LifecycleStatus.TIMEOUT)
        except Exception as exc:
            if cancel_event.is_set():
                if self._is_stop_unconfirmed_error(exc):
                    return result(
                        MainCycleState.STOP_UNCONFIRMED,
                        f"{label} 已要求取消，但設備停止未確認：{exc}",
                        LifecycleStatus.ERROR,
                    )
                return result(
                    MainCycleState.CANCELLED,
                    f"{label} 已取消：{exc}",
                    LifecycleStatus.CANCELLED,
                )
            return result(MainCycleState.ERROR, f"MainCycleFlow.{step}: {exc}", LifecycleStatus.ERROR)
        finally:
            stop_errors = self._stop_and_confirm_motion(include_pallet=True)
            if stop_errors:
                step = "stop_unconfirmed"
                return result(
                    MainCycleState.STOP_UNCONFIRMED,
                    "設備停止未確認：" + "；".join(stop_errors),
                    LifecycleStatus.ERROR,
                )

    def _run_step2(
        self,
        cancel_event: threading.Event,
        progress_callback: Callable[[MainCycleProgress], None] | None,
        transfer_direction: TransferDirection,
        *,
        x_speed: int | None = None,
        height_reference_depth_mm: float | None = None,
    ) -> MainCycleResult:
        started = time.monotonic()
        state = MainCycleState.STEP2
        step = "step2_precheck"
        height: float | None = None
        pick_slot, place_slot = (
            ("Y1", "Y2")
            if transfer_direction is TransferDirection.Y1_TO_Y2
            else ("Y2", "Y1")
        )

        def elapsed() -> float:
            return time.monotonic() - started

        def report(message: str, status: LifecycleStatus = LifecycleStatus.RUNNING) -> None:
            self._set_status(
                status,
                step,
                message,
                phase=self.phase.value,
                height_mm=height,
                vision_height_mm=self._vision_height_1,
                transfer_direction=transfer_direction.value,
                camera_view=transfer_direction.camera_view,
            )
            if progress_callback is not None:
                progress_callback(MainCycleProgress(status, state, self.phase, step, message, elapsed(), height, self._vision_height_1))

        def result(result_state: MainCycleState, message: str, status: LifecycleStatus) -> MainCycleResult:
            self._set_status(
                status,
                step,
                message,
                state=result_state.value,
                phase=self.phase.value,
                height_mm=height,
                vision_height_mm=self._vision_height_1,
                transfer_direction=transfer_direction.value,
                camera_view=transfer_direction.camera_view,
            )
            item = MainCycleResult(
                status,
                result_state,
                self.phase,
                step,
                message,
                elapsed(),
                height,
                self._vision_height_1,
                transfer_direction.value,
            )
            LOGGER.info("MainCycleFlow step2 finished status=%s state=%s step=%s message=%s", status.value, result_state.value, step, message)
            return item

        def safe_height(requested_height_mm: float, source: str) -> float:
            requested = float(requested_height_mm)
            limited = min(requested, SECOND_STEP_MAXIMUM_HEIGHT_MM)
            if requested > limited:
                message = (
                    f"第二步安全限制：{source}要求 {requested:g}mm，"
                    f"已限制為 {limited:g}mm"
                )
                LOGGER.warning(message)
                report(message)
            return limited

        try:
            step = "step2_precheck"
            self.home_interlock.mark_not_home(
                "第二步已開始；必須在流程結束後重新確認手臂與 Camera HOME"
            )
            report(
                f"第二步：方向 {transfer_direction.display_name}，"
                f"使用 {transfer_direction.camera_view}，檢查 service"
            )
            self.lift_service.precheck()

            step = "step2_to_vision_height"
            vision_height_mm = safe_height(
                SECOND_STEP_ARM_MOTION_SAFE_HEIGHT_MM,
                "手臂固定安全高度",
            )
            cross_side_height_mm = safe_height(
                self.config.cross_side_safe_height_mm,
                "手臂跨側安全高度",
            )
            height = self._move_height(
                vision_height_mm,
                cancel_event,
                lambda message: report(f"第二步：{message}"),
                speed=x_speed,
            )

            if self.arm_vision_service is not None:
                step = "step2_arm_vision_pick_place"
                released_at_cross_side_height = False
                descended_to_pick_approach_height = False

                def movement_handoff(movement: str) -> None:
                    nonlocal height, step, descended_to_pick_approach_height
                    if "Lower to pick-approach height" in movement:
                        step = "step2_descend_to_pick_approach_height"
                        height = self._move_height(
                            SECOND_STEP_ARM_PICK_APPROACH_HEIGHT_MM,
                            cancel_event,
                            lambda message: report(
                                f"第二步拍照後下降至撿貨接近高度：{message}"
                            ),
                            speed=x_speed,
                        )
                        height = self._confirm_stopped_height(
                            SECOND_STEP_ARM_PICK_APPROACH_HEIGHT_MM,
                            cancel_event,
                            lambda message: report(
                                f"第二步撿貨接近高度確認：{message}"
                            ),
                        )
                        descended_to_pick_approach_height = True
                        report(
                            f"第二步：升降機已停止並確認在 "
                            f"{SECOND_STEP_ARM_PICK_APPROACH_HEIGHT_MM:g}"
                            f"±{self.config.height_tolerance_mm:g}mm，"
                            f"放行手臂前往撿貨位置"
                        )
                        return

                    step = "step2_confirm_safe_height_before_arm_motion"
                    required_height_mm = (
                        cross_side_height_mm
                        if (
                            released_at_cross_side_height
                            or "FAKE PLC PICK DONE" in movement
                            or "Move left target" in movement
                            or "Move right target" in movement
                        )
                        else (
                            SECOND_STEP_ARM_PICK_APPROACH_HEIGHT_MM
                            if descended_to_pick_approach_height
                            else vision_height_mm
                        )
                    )
                    height = self._confirm_stopped_height(
                        required_height_mm,
                        cancel_event,
                        lambda message: report(
                            f"第二步手臂移動前（{movement}）：{message}"
                        ),
                    )
                    report(
                        f"第二步：升降機已停止且高度確認為 "
                        f"{required_height_mm:g}±{self.config.height_tolerance_mm:g}mm，"
                        f"放行手臂動作：{movement}"
                    )

                def pick_handoff(target_height_mm: float, control_target: dict) -> None:
                    nonlocal height, step
                    self._vision_height_1 = safe_height(
                        target_height_mm,
                        "手臂取料高度",
                    )
                    step = "step2_pick_height"
                    height = self._move_height(
                        self._vision_height_1,
                        cancel_event,
                        lambda message: report(f"第二步取料：{message}"),
                        speed=x_speed,
                    )
                    self.middle_vacuum_service.set_mode(MiddleVacuumMode.VACUUM)
                    report(
                        f"第二步 {pick_slot} 取料："
                        f"D500={self._vision_height_1:g}mm 到位，M54 已開啟"
                    )
                    self.middle_vacuum_service.wait_transfer_ready(
                        vacuum_expected=True,
                        cancel_event=cancel_event,
                        progress_callback=lambda message: report(
                            f"第二步取料：{message}"
                        ),
                    )
                    step = "step2_safe_height_before_transfer"
                    height = self._move_height(
                        cross_side_height_mm,
                        cancel_event,
                        lambda message: report(f"第二步換邊前：{message}"),
                        speed=x_speed,
                    )
                    height = self._confirm_stopped_height(
                        cross_side_height_mm,
                        cancel_event,
                        lambda message: report(
                            f"第二步換邊放行前：{message}"
                        ),
                    )
                    report(
                        "第二步換邊前：貨物保持吸附，升降機已回到 "
                        f"{cross_side_height_mm:g}mm 安全高度且已停止，允許手臂換邊"
                    )
                    step = "step2_arm_transfer"
                    report(
                        f"第二步：{cross_side_height_mm:g}mm 跨側安全高度已確認，"
                        f"放行手臂由 {pick_slot} 移動到 {place_slot}"
                    )

                def place_handoff(target_height_mm: float, control_target: dict) -> None:
                    nonlocal height, step, released_at_cross_side_height
                    step = "step2_place_height"
                    height = self._confirm_stopped_height(
                        cross_side_height_mm,
                        cancel_event,
                        lambda message: report(
                            f"第二步放料前 {cross_side_height_mm:g}mm 確認：{message}"
                        ),
                    )
                    self.middle_vacuum_service.set_mode(MiddleVacuumMode.BREAK_VACUUM)
                    report(
                        f"第二步 {place_slot} 放料：升降機保持 "
                        f"D500={cross_side_height_mm:g}mm，M54 已關閉，M56 已開啟"
                    )
                    self.middle_vacuum_service.wait_transfer_ready(
                        vacuum_expected=False,
                        cancel_event=cancel_event,
                        progress_callback=lambda message: report(
                            f"第二步放料：{message}"
                        ),
                    )
                    self.middle_vacuum_service.set_mode(MiddleVacuumMode.OFF)
                    report("第二步放料：M56 破真空 5 秒完成，M54／M56 均已關閉")
                    released_at_cross_side_height = True
                    step = "step2_safe_height_before_home"
                    height = self._confirm_stopped_height(
                        cross_side_height_mm,
                        cancel_event,
                        lambda message: report(
                            f"第二步回 HOME 前 {cross_side_height_mm:g}mm 確認：{message}"
                        ),
                    )
                    report(
                        "第二步回 HOME 前：貨物已釋放，升降機維持在 "
                        f"{cross_side_height_mm:g}mm 且已停止，允許手臂回 HOME"
                    )
                    step = "step2_arm_return_home"
                    report(
                        f"第二步：{cross_side_height_mm:g}mm 跨側安全高度已確認，"
                        "放行手臂回 HOME"
                    )

                report("第二步：啟動 D435 / CANBus 手臂取放流程")
                arm_result = self.arm_vision_service.run_pick_and_place(
                    cancel_event,
                    pick_handoff=pick_handoff,
                    place_handoff=place_handoff,
                    movement_handoff=movement_handoff,
                    view=transfer_direction.camera_view,
                    height_reference_depth_mm=height_reference_depth_mm,
                    progress_callback=lambda message: report(f"第二步手臂：{message}"),
                )
                self._vision_height_1 = safe_height(
                    arm_result["plc_height_mm"],
                    "手臂流程回傳高度",
                )
                step = "step2_arm_home_confirmed"
                return result(
                    MainCycleState.SUCCESS,
                    f"第二步 {transfer_direction.display_name} 手臂取放完成，"
                    "請重新執行第二次第一步",
                    LifecycleStatus.SUCCESS,
                )

            step = "step2_start_bridge"
            self.vision_bridge_service.start()
            self.vision_bridge_service.notify("vision_height_ready", vision_height_mm=vision_height_mm)

            step = "step2_wait_height"
            report("第二步：等待 RealSense/CANBus 信號與高度", LifecycleStatus.WAITING_SIGNAL)
            self.vision_bridge_service.wait_signal(cancel_event, self.config.vision_signal_timeout_seconds)
            self._vision_height_1 = safe_height(
                self.vision_bridge_service.wait_height(
                    cancel_event,
                    self.config.vision_signal_timeout_seconds,
                ),
                "Vision Bridge 回傳高度",
            )

            step = "step2_m54_height"
            height = self._move_height(
                self._vision_height_1,
                cancel_event,
                lambda message: report(f"第二步：{message}"),
                speed=x_speed,
            )
            self.middle_vacuum_service.set_mode(MiddleVacuumMode.VACUUM)
            self.middle_vacuum_service.wait_transfer_ready(
                vacuum_expected=True,
                cancel_event=cancel_event,
                progress_callback=lambda message: report(f"第二步：{message}"),
            )
            self.vision_bridge_service.notify("m54_on_after_height", height_mm=self._vision_height_1)

            step = "step2_return_after_m54"
            height = self._move_height(
                cross_side_height_mm,
                cancel_event,
                lambda message: report(f"第二步：{message}"),
                speed=x_speed,
            )
            self.vision_bridge_service.notify(
                "returned_cross_side_height_after_m54",
                cross_side_height_mm=cross_side_height_mm,
            )

            step = "step2_m56_cross_side_height"
            height = self._confirm_stopped_height(
                cross_side_height_mm,
                cancel_event,
                lambda message: report(
                    f"第二步放料前 {cross_side_height_mm:g}mm 確認：{message}"
                ),
            )
            self.middle_vacuum_service.set_mode(MiddleVacuumMode.BREAK_VACUUM)
            self.middle_vacuum_service.wait_transfer_ready(
                vacuum_expected=False,
                cancel_event=cancel_event,
                progress_callback=lambda message: report(f"第二步：{message}"),
            )
            self.vision_bridge_service.notify(
                "m56_on_m54_off_at_cross_side_height",
                height_mm=cross_side_height_mm,
            )
            self.middle_vacuum_service.set_mode(MiddleVacuumMode.OFF)

            step = "step2_hold_cross_side_height_after_m56"
            height = self._confirm_stopped_height(
                cross_side_height_mm,
                cancel_event,
                lambda message: report(
                    f"第二步破真空後 {cross_side_height_mm:g}mm 確認：{message}"
                ),
            )
            self.vision_bridge_service.notify(
                "held_cross_side_height_after_m56",
                cross_side_height_mm=cross_side_height_mm,
            )

            step = "step2_wait_done"
            report("第二步：等待 RealSense/CANBus 完成信號", LifecycleStatus.WAITING_SIGNAL)
            self.vision_bridge_service.wait_done(cancel_event, self.config.vision_signal_timeout_seconds)
            return result(MainCycleState.SUCCESS, "第二步完成，請重新執行第二次第一步", LifecycleStatus.SUCCESS)
        except TimeoutError as exc:
            if cancel_event.is_set():
                return result(
                    MainCycleState.CANCELLED,
                    f"第二步已取消：{exc}",
                    LifecycleStatus.CANCELLED,
                )
            return result(MainCycleState.TIMEOUT, f"MainCycleFlow.{step}: {exc}", LifecycleStatus.TIMEOUT)
        except Exception as exc:
            if cancel_event.is_set():
                if self._is_stop_unconfirmed_error(exc):
                    return result(
                        MainCycleState.STOP_UNCONFIRMED,
                        f"第二步已要求取消，但設備停止未確認：{exc}",
                        LifecycleStatus.ERROR,
                    )
                return result(
                    MainCycleState.CANCELLED,
                    f"第二步已取消：{exc}",
                    LifecycleStatus.CANCELLED,
                )
            return result(MainCycleState.ERROR, f"MainCycleFlow.{step}: {exc}", LifecycleStatus.ERROR)
        finally:
            stop_errors = self._stop_and_confirm_motion(include_pallet=False)
            if stop_errors:
                step = "stop_unconfirmed"
                return result(
                    MainCycleState.STOP_UNCONFIRMED,
                    "設備停止未確認：" + "；".join(stop_errors),
                    LifecycleStatus.ERROR,
                )

    @staticmethod
    def _is_stop_unconfirmed_error(exc: Exception) -> bool:
        message = str(exc)
        markers = (
            "停止未確認",
            "停止失敗",
            "未收到停止確認",
            "沒有收到停止確認",
        )
        return any(marker in message for marker in markers)

    def _stop_and_confirm_motion(self, *, include_pallet: bool) -> list[str]:
        """Stop command outputs and verify their PLC readback is OFF."""

        errors: list[str] = []
        try:
            self.lift_service.stop()
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("MainCycleFlow lift stop command failed")
            errors.append(f"升降停止命令失敗：{exc}")
        try:
            lift_commands = self.lift_service.read_motion_commands()
            active_lift = sorted(name for name, active in lift_commands.items() if active)
            if active_lift:
                errors.append(f"升降命令仍為 ON：{', '.join(active_lift)}")
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("MainCycleFlow lift stop readback failed")
            errors.append(f"升降停止讀回失敗：{exc}")

        if not include_pallet:
            return errors

        try:
            self.pallet_service.stop_forward()
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("MainCycleFlow pallet stop command failed")
            errors.append(f"貨盤停止命令失敗：{exc}")
        try:
            pallet_commands = self.pallet_service.read_forward_commands()
            active_pallet = sorted(
                name for name, active in pallet_commands.items() if active
            )
            if active_pallet:
                errors.append(f"貨盤命令仍為 ON：{', '.join(active_pallet)}")
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("MainCycleFlow pallet stop readback failed")
            errors.append(f"貨盤停止讀回失敗：{exc}")
        return errors

    def _move_height(
        self,
        target_height_mm: float,
        cancel_event: threading.Event,
        report: Callable[[str], None],
        *,
        speed: int | None = None,
    ) -> float:
        self.lift_service.configure_position(target_height_mm, speed=speed)
        self.lift_service.start_vision_positioning()
        stable_reads = 0
        deadline = time.monotonic() + self.config.motion_timeout_seconds
        height = self.lift_service.read_height()
        while time.monotonic() < deadline:
            if cancel_event.is_set():
                raise RuntimeError("流程已取消")
            height = self.lift_service.read_height()
            if abs(height - target_height_mm) <= self.config.height_tolerance_mm:
                stable_reads += 1
                self.lift_service.stop()
                report(f"D500={target_height_mm:g}mm 已到位，M376 已關閉，穩定 {stable_reads}/{self.config.stable_read_count}")
                if stable_reads >= self.config.stable_read_count:
                    return height
            else:
                stable_reads = 0
                report(f"D500={target_height_mm:g}mm 移動中，目前 {height:g}mm")
            cancel_event.wait(self.config.poll_interval_seconds)
        raise TimeoutError(f"高度 {height:g}mm 未在 {self.config.motion_timeout_seconds:g} 秒內到 {target_height_mm:g}mm")

    def _confirm_stopped_height(
        self,
        target_height_mm: float,
        cancel_event: threading.Event,
        report: Callable[[str], None],
    ) -> float:
        """Keep the lift stopped and confirm its height before any arm motion."""

        self.lift_service.stop()
        height = target_height_mm
        for stable_read in range(1, self.config.stable_read_count + 1):
            if cancel_event.is_set():
                raise RuntimeError("流程已取消")
            height = self.lift_service.read_height()
            error_mm = abs(height - target_height_mm)
            if error_mm > self.config.height_tolerance_mm:
                raise RuntimeError(
                    "禁止手臂移動：升降機已停止，但實際高度 "
                    f"{height:g}mm 不在 {target_height_mm:g}±"
                    f"{self.config.height_tolerance_mm:g}mm"
                )
            report(
                f"升降機停止確認，實際高度 {height:g}mm，"
                f"穩定 {stable_read}/{self.config.stable_read_count}"
            )
            if stable_read < self.config.stable_read_count:
                if cancel_event.wait(self.config.poll_interval_seconds):
                    raise RuntimeError("流程已取消")
        return height

    def _wait_pallet_home(
        self,
        slot: str,
        cancel_event: threading.Event,
        report: Callable[[str], None],
    ) -> float:
        return self._wait_pallet_position(slot, 0.0, cancel_event, report, movement_name="回原點")

    def _wait_pallet_position(
        self,
        slot: str,
        target_mm: float,
        cancel_event: threading.Event,
        report: Callable[[str], None],
        *,
        movement_name: str,
    ) -> float:
        stable_reads = 0
        timeout_seconds = self.config.y_motion_timeout_seconds
        deadline = time.monotonic() + timeout_seconds
        position = 0.0
        while time.monotonic() < deadline:
            if cancel_event.is_set():
                raise RuntimeError("流程已取消")
            position = self.pallet_service.read_position(slot)
            if abs(position - target_mm) <= self.config.y_position_tolerance_mm:
                stable_reads += 1
            else:
                stable_reads = 0
            report(
                f"{slot} {movement_name}中，D62/D72={position:g}mm，"
                f"目標 {target_mm:g}mm，穩定 {stable_reads}/{self.config.stable_read_count}"
            )
            if stable_reads >= self.config.stable_read_count:
                return position
            cancel_event.wait(self.config.poll_interval_seconds)
        raise TimeoutError(f"{slot} 未在 {timeout_seconds:g} 秒內到達 {target_mm:g}mm")

    def _instant_error(self, step: str, message: str) -> MainCycleResult:
        self._set_status(LifecycleStatus.ERROR, step, message, phase=self.phase.value, vision_height_mm=self._vision_height_1)
        return MainCycleResult(
            LifecycleStatus.ERROR,
            MainCycleState.ERROR,
            self.phase,
            step,
            message,
            0.0,
            None,
            self._vision_height_1,
        )
