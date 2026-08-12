from __future__ import annotations

import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

try:
    from config.loader import CONFIG_STORE, ConfigStore
    from lifecycle import LifecycleStatus, LifecycleTracked
    from service.arm_camera_home_interlock import (
        ARM_CAMERA_HOME_INTERLOCK,
        ArmCameraHomeInterlock,
    )
except ModuleNotFoundError:
    from plc2.config.loader import CONFIG_STORE, ConfigStore
    from plc2.lifecycle import LifecycleStatus, LifecycleTracked
    from plc2.service.arm_camera_home_interlock import (
        ARM_CAMERA_HOME_INTERLOCK,
        ArmCameraHomeInterlock,
    )


ArmHandoff = Callable[[float, dict[str, Any]], None]
ArmMovementHandoff = Callable[[str], None]
ProgressCallback = Callable[[str], None]
TargetHeightValidator = Callable[[float], None]
ALL_HOME_CONFIRMED_MARKER = "[POSE_STATE] ALL_HOME_CONFIRMED"
ARM_HOME_CONFIRMED_MARKER = "[POSE_STATE] ARM_HOME_CONFIRMED"
CAMERA_HOME_CONFIRMED_MARKER = "[POSE_STATE] CAMERA_HOME_CONFIRMED"
ARM_STOP_CONFIRMED_MARKER = "[POSE_STATE] ARM_STOP_CONFIRMED"
ARM_STOP_UNCONFIRMED_MARKER = "[POSE_STATE] ARM_STOP_UNCONFIRMED"
NAMED_POSE_CONFIRMED_MARKER = "[POSE_STATE] NAMED_POSE_CONFIRMED"
POSE_RESULT_PREFIX = "[POSE_RESULT] "
ARM_VISION_MAXIMUM_PLC_HEIGHT_MM = 695.0


class ArmVisionWorkflowServiceError(RuntimeError):
    def __init__(self, operation: str, message: str) -> None:
        self.operation = operation
        super().__init__(f"ArmVisionWorkflowService.{operation}: {message}")


@dataclass(frozen=True)
class ArmVisionWorkflowConfig:
    enabled: bool = True
    python_path: str | None = None
    script_path: str = "d435_control.py"
    view: str = "LView"
    target_speed_dps: int = 200
    settle_seconds: float = 2.5
    target_wait_seconds: float = 2.0
    pick_wait_seconds: float = 2.0
    vision_height_mm: float = 560.0
    height_reference_depth_mm: float = 620.0
    height_formula_minimum_depth_m: float = 0.650
    startup_timeout_seconds: float = 30.0
    no_output_timeout_seconds: float = 90.0
    overall_timeout_seconds: float = 600.0
    poll_interval_seconds: float = 0.1
    home_tolerance_degrees: float = 1.0
    home_stable_reads: int = 3
    home_timeout_seconds: float = 15.0
    home_poll_interval_seconds: float = 0.2
    cancel_stop_timeout_seconds: float = 8.0

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "ArmVisionWorkflowConfig":
        raw = store.get_service("arm_vision_workflow")
        if raw is None:
            return cls()
        return cls(
            enabled=bool(raw.get("enabled", True)),
            python_path=None if raw.get("python_path") in {None, ""} else str(raw.get("python_path")),
            script_path=str(raw.get("script_path", cls.script_path)),
            view=str(raw.get("view", cls.view)),
            target_speed_dps=int(raw.get("target_speed_dps", cls.target_speed_dps)),
            settle_seconds=float(raw.get("settle_seconds", cls.settle_seconds)),
            target_wait_seconds=float(raw.get("target_wait_seconds", cls.target_wait_seconds)),
            pick_wait_seconds=float(raw.get("pick_wait_seconds", cls.pick_wait_seconds)),
            vision_height_mm=float(raw.get("vision_height_mm", cls.vision_height_mm)),
            height_reference_depth_mm=float(raw.get("height_reference_depth_mm", cls.height_reference_depth_mm)),
            height_formula_minimum_depth_m=float(
                raw.get(
                    "height_formula_minimum_depth_m",
                    cls.height_formula_minimum_depth_m,
                )
            ),
            startup_timeout_seconds=float(raw.get("startup_timeout_seconds", cls.startup_timeout_seconds)),
            no_output_timeout_seconds=float(raw.get("no_output_timeout_seconds", cls.no_output_timeout_seconds)),
            overall_timeout_seconds=float(raw.get("overall_timeout_seconds", cls.overall_timeout_seconds)),
            poll_interval_seconds=float(raw.get("poll_interval_seconds", cls.poll_interval_seconds)),
            home_tolerance_degrees=float(
                raw.get("home_tolerance_degrees", cls.home_tolerance_degrees)
            ),
            home_stable_reads=int(raw.get("home_stable_reads", cls.home_stable_reads)),
            home_timeout_seconds=float(
                raw.get("home_timeout_seconds", cls.home_timeout_seconds)
            ),
            home_poll_interval_seconds=float(
                raw.get(
                    "home_poll_interval_seconds",
                    cls.home_poll_interval_seconds,
                )
            ),
            cancel_stop_timeout_seconds=float(
                raw.get(
                    "cancel_stop_timeout_seconds",
                    cls.cancel_stop_timeout_seconds,
                )
            ),
        )

    def validate(self) -> None:
        if self.target_speed_dps <= 0:
            raise ArmVisionWorkflowServiceError("config", "target_speed_dps 必須大於 0")
        for name, value in (
            ("settle_seconds", self.settle_seconds),
            ("target_wait_seconds", self.target_wait_seconds),
            ("pick_wait_seconds", self.pick_wait_seconds),
            ("startup_timeout_seconds", self.startup_timeout_seconds),
            ("no_output_timeout_seconds", self.no_output_timeout_seconds),
            ("overall_timeout_seconds", self.overall_timeout_seconds),
            ("poll_interval_seconds", self.poll_interval_seconds),
            ("home_tolerance_degrees", self.home_tolerance_degrees),
            ("home_timeout_seconds", self.home_timeout_seconds),
            ("home_poll_interval_seconds", self.home_poll_interval_seconds),
            ("cancel_stop_timeout_seconds", self.cancel_stop_timeout_seconds),
        ):
            if value < 0:
                raise ArmVisionWorkflowServiceError("config", f"{name} 不可小於 0")
        if self.home_stable_reads <= 0:
            raise ArmVisionWorkflowServiceError("config", "home_stable_reads 必須大於 0")
        if not math.isfinite(self.height_formula_minimum_depth_m):
            raise ArmVisionWorkflowServiceError(
                "config",
                "height_formula_minimum_depth_m 必須是有限數值",
            )
        if self.height_formula_minimum_depth_m <= 0:
            raise ArmVisionWorkflowServiceError(
                "config",
                "height_formula_minimum_depth_m 必須大於 0",
            )
        for name, value in (
            ("home_timeout_seconds", self.home_timeout_seconds),
            ("home_poll_interval_seconds", self.home_poll_interval_seconds),
            ("cancel_stop_timeout_seconds", self.cancel_stop_timeout_seconds),
        ):
            if value <= 0:
                raise ArmVisionWorkflowServiceError("config", f"{name} 必須大於 0")


class ArmVisionWorkflowService(LifecycleTracked):
    def __init__(
        self,
        config: ArmVisionWorkflowConfig | None = None,
        *,
        base_dir: Path | None = None,
        home_interlock: ArmCameraHomeInterlock = ARM_CAMERA_HOME_INTERLOCK,
        target_height_validator: TargetHeightValidator | None = None,
    ) -> None:
        self.config = config or ArmVisionWorkflowConfig.load()
        self.config.validate()
        self.base_dir = base_dir or Path(__file__).resolve().parents[2]
        self.home_interlock = home_interlock
        self.target_height_validator = target_height_validator
        self._init_status_tracker("ArmVisionWorkflowService")

    @property
    def enabled(self) -> bool:
        return self.config.enabled

    def depth_m_to_plc_height_mm(
        self,
        depth_m: float,
        *,
        height_reference_depth_mm: float | None = None,
    ) -> float:
        depth_m = float(depth_m)
        if not math.isfinite(depth_m):
            raise ArmVisionWorkflowServiceError(
                "height",
                f"相機深度不是有限數值: {depth_m!r}",
            )
        if depth_m <= self.config.height_formula_minimum_depth_m:
            return min(depth_m * 1000.0, ARM_VISION_MAXIMUM_PLC_HEIGHT_MM)
        reference_mm = (
            self.config.height_reference_depth_mm
            if height_reference_depth_mm is None
            else float(height_reference_depth_mm)
        )
        if not math.isfinite(reference_mm) or reference_mm <= 0:
            raise ArmVisionWorkflowServiceError(
                "height",
                f"height_reference_depth_mm 必須是大於 0 的有限數值: {reference_mm!r}",
            )
        height_mm = (
            depth_m * 1000.0
            - reference_mm
            + self.config.vision_height_mm
        )
        if not math.isfinite(height_mm):
            raise ArmVisionWorkflowServiceError(
                "height",
                f"視覺換算高度不是有限數值: {height_mm!r}",
            )
        return min(height_mm, ARM_VISION_MAXIMUM_PLC_HEIGHT_MM)

    def _height_policy_text(self, depth_m: float, height_mm: float) -> str:
        threshold_m = self.config.height_formula_minimum_depth_m
        if depth_m <= threshold_m:
            return (
                f"相機 {depth_m:.3f}m ≤ {threshold_m:g}m，"
                f"直接採用實測高度 D500={height_mm:g}mm 放行"
            )
        return (
            f"相機 {depth_m:.3f}m > {threshold_m:g}m，"
            f"套用高度公式，D500={height_mm:g}mm"
        )

    def confirm_home(
        self,
        cancel_event: threading.Event | None = None,
    ) -> dict[str, Any]:
        """Read all four CAN axes and unlock the lift only after stable HOME."""

        if not self.enabled:
            raise ArmVisionWorkflowServiceError(
                "confirm_home",
                "手臂 HOME 確認目前未啟用",
            )

        command = self._home_confirmation_command()
        timeout_seconds = (
            self.config.startup_timeout_seconds
            + self.config.home_timeout_seconds
            + 5.0
        )
        self.home_interlock.mark_unknown("正在讀回四顆 CAN 馬達 HOME 角度")
        self._set_status(
            LifecycleStatus.RUNNING,
            "confirm_home",
            "正在確認 ID142～ID145 是否位於 HOME",
            command=" ".join(command),
        )

        process: subprocess.Popen[str] | None = None
        try:
            process = subprocess.Popen(
                command,
                cwd=self.base_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env={**os.environ, "PYTHONUNBUFFERED": "1"},
                start_new_session=os.name == "posix",
            )
            deadline = time.monotonic() + timeout_seconds
            output = ""
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    self._terminate_process(process)
                    output, _unused = process.communicate()
                    raise ArmVisionWorkflowServiceError(
                        "confirm_home_cancelled",
                        "HOME 確認已取消，獨立子流程不會啟動",
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._terminate_process(process)
                    output, _unused = process.communicate()
                    raise ArmVisionWorkflowServiceError(
                        "confirm_home_timeout",
                        f"四軸 HOME 確認超過 {timeout_seconds:g} 秒",
                    )
                try:
                    output, _unused = process.communicate(timeout=min(0.1, remaining))
                    break
                except subprocess.TimeoutExpired:
                    continue

            lines = [line.strip() for line in output.splitlines() if line.strip()]
            if process.returncode != 0:
                raise ArmVisionWorkflowServiceError(
                    "confirm_home",
                    f"HOME 確認程式結束碼 {process.returncode}；"
                    f"{self._last_output_text(lines)}",
                )
            if ALL_HOME_CONFIRMED_MARKER not in lines:
                raise ArmVisionWorkflowServiceError(
                    "confirm_home",
                    "沒有收到四軸 HOME 確認標記；"
                    f"{self._last_output_text(lines)}",
                )

            pose_data: dict[str, Any] = {}
            for line in reversed(lines):
                if line.startswith(POSE_RESULT_PREFIX):
                    value = json.loads(line.removeprefix(POSE_RESULT_PREFIX))
                    if isinstance(value, dict):
                        pose_data = value
                    break

            angles = pose_data.get("angles")
            if not isinstance(angles, dict) or not angles:
                raise ArmVisionWorkflowServiceError(
                    "confirm_home",
                    "HOME 已標記確認，但沒有收到四軸角度資料",
                )

            self.home_interlock.mark_home_confirmed(
                "ID142～ID145 已連續穩定讀回 HOME 角度"
            )
            result = {
                "confirmed": True,
                "state": self.home_interlock.snapshot.state.value,
                "angles": angles,
                "tolerance_degrees": self.config.home_tolerance_degrees,
                "stable_reads": self.config.home_stable_reads,
            }
            self._set_status(
                LifecycleStatus.SUCCESS,
                "confirm_home",
                "四顆 CAN 馬達 HOME 已確認，升降高度上限已恢復",
                **result,
            )
            return result
        except Exception as exc:
            self.home_interlock.mark_unknown(
                f"四軸 HOME 確認失敗：{type(exc).__name__}: {exc}"
            )
            operation = getattr(exc, "operation", "")
            if operation == "confirm_home_cancelled":
                lifecycle_status = LifecycleStatus.CANCELLED
                step = "confirm_home_cancelled"
            elif operation == "confirm_home_timeout":
                lifecycle_status = LifecycleStatus.TIMEOUT
                step = "confirm_home_timeout"
            else:
                lifecycle_status = LifecycleStatus.ERROR
                step = "confirm_home"
            self._set_status(lifecycle_status, step, str(exc))
            raise
        finally:
            if process is not None:
                self._terminate_process(process)
                self._close_process_streams(process)

    def move_home_component(self, component: str) -> dict[str, Any]:
        """Move and confirm only the arm pair or camera pair at HOME."""

        normalized = str(component).strip().lower()
        component_labels = {
            "arm": "手臂 ID142／ID143",
            "camera": "Camera ID144／ID145",
        }
        if normalized not in component_labels:
            raise ArmVisionWorkflowServiceError(
                "move_home_component",
                f"不支援的 HOME 元件：{component!r}",
            )
        if not self.enabled:
            raise ArmVisionWorkflowServiceError(
                "move_home_component",
                "手臂 HOME 控制目前未啟用",
            )

        command = self._home_component_command(normalized)
        timeout_seconds = (
            self.config.startup_timeout_seconds
            + self.config.home_timeout_seconds
            + self.config.settle_seconds
            + 5.0
        )
        label = component_labels[normalized]
        marker = (
            ARM_HOME_CONFIRMED_MARKER
            if normalized == "arm"
            else CAMERA_HOME_CONFIRMED_MARKER
        )
        self.home_interlock.mark_not_home(f"正在移動 {label} 回 HOME")
        self._set_status(
            LifecycleStatus.RUNNING,
            f"move_{normalized}_home",
            f"正在移動並確認 {label} HOME",
            command=" ".join(command),
        )

        try:
            completed = subprocess.run(
                command,
                cwd=self.base_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_seconds,
                check=False,
                env={**os.environ, "PYTHONUNBUFFERED": "1"},
            )
            lines = [
                line.strip()
                for line in completed.stdout.splitlines()
                if line.strip()
            ]
            if completed.returncode != 0:
                raise ArmVisionWorkflowServiceError(
                    "move_home_component",
                    f"{label} HOME 程式結束碼 {completed.returncode}；"
                    f"{self._last_output_text(lines)}",
                )
            if marker not in lines:
                raise ArmVisionWorkflowServiceError(
                    "move_home_component",
                    f"沒有收到 {label} HOME 確認標記；"
                    f"{self._last_output_text(lines)}",
                )

            pose_data: dict[str, Any] = {}
            for line in reversed(lines):
                if line.startswith(POSE_RESULT_PREFIX):
                    value = json.loads(line.removeprefix(POSE_RESULT_PREFIX))
                    if isinstance(value, dict):
                        pose_data = value
                    break
            angles = pose_data.get("angles")
            if not isinstance(angles, dict) or not angles:
                raise ArmVisionWorkflowServiceError(
                    "move_home_component",
                    f"{label} 已標記 HOME，但沒有收到角度資料",
                )

            self.home_interlock.mark_unknown(
                f"{label} 已回 HOME；仍需四軸 HOME 確認才能解鎖 1450mm"
            )
            result = {
                "component": normalized,
                "confirmed": True,
                "angles": angles,
                "all_home_confirmed": False,
            }
            self._set_status(
                LifecycleStatus.SUCCESS,
                f"move_{normalized}_home",
                f"{label} HOME 已確認",
                **result,
            )
            return result
        except subprocess.TimeoutExpired as exc:
            error = ArmVisionWorkflowServiceError(
                "move_home_component",
                f"{label} 回 HOME 超過 {timeout_seconds:g} 秒",
            )
            self.home_interlock.mark_unknown(str(error))
            self._set_status(
                LifecycleStatus.TIMEOUT,
                f"move_{normalized}_home",
                str(error),
            )
            raise error from exc
        except Exception as exc:
            self.home_interlock.mark_unknown(
                f"{label} HOME 失敗：{type(exc).__name__}: {exc}"
            )
            self._set_status(
                LifecycleStatus.ERROR,
                f"move_{normalized}_home",
                str(exc),
            )
            raise
            
    def move_named_pose(
        self,
        pose_name: str = "STANDBY",
    ) -> dict[str, Any]:
        """Move all four CAN axes to an allowed named pose and confirm it."""

        pose_name = str(pose_name).strip().upper()
        if pose_name not in {"HOME", "STANDBY"}:
            raise ArmVisionWorkflowServiceError(
                "move_named_pose",
                f"不支援的四軸姿態：{pose_name!r}",
            )

        if not self.enabled:
            raise ArmVisionWorkflowServiceError(
                "move_named_pose",
                "手臂四軸姿態控制目前未啟用",
            )

        command = self._named_pose_command(pose_name)
        timeout_seconds = (
            self.config.startup_timeout_seconds
            + self.config.home_timeout_seconds
            + self.config.settle_seconds
            + 5.0
        )

        self.home_interlock.mark_not_home(
            f"正在同步移動四軸到 {pose_name}"
        )
        self._set_status(
            LifecycleStatus.RUNNING,
            "move_named_pose",
            f"正在同步移動並確認四軸姿態 {pose_name}",
            command=" ".join(command),
        )

        try:
            completed = subprocess.run(
                command,
                cwd=self.base_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_seconds,
                check=False,
                env={
                    **os.environ,
                    "PYTHONUNBUFFERED": "1",
                },
            )

            lines = [
                line.strip()
                for line in completed.stdout.splitlines()
                if line.strip()
            ]

            if completed.returncode != 0:
                raise ArmVisionWorkflowServiceError(
                    "move_named_pose",
                    f"{pose_name} 程式結束碼 "
                    f"{completed.returncode}；"
                    f"{self._last_output_text(lines)}",
                )

            if NAMED_POSE_CONFIRMED_MARKER not in lines:
                raise ArmVisionWorkflowServiceError(
                    "move_named_pose",
                    f"沒有收到 {pose_name} 姿態確認標記；"
                    f"{self._last_output_text(lines)}",
                )

            pose_data: dict[str, Any] = {}

            for line in reversed(lines):
                if line.startswith(POSE_RESULT_PREFIX):
                    value = json.loads(
                        line.removeprefix(POSE_RESULT_PREFIX)
                    )
                    if isinstance(value, dict):
                        pose_data = value
                    break

            angles = pose_data.get("angles")
            if not isinstance(angles, dict) or not angles:
                raise ArmVisionWorkflowServiceError(
                    "move_named_pose",
                    f"{pose_name} 已確認，但沒有收到角度資料",
                )

            if pose_name == "HOME":
                self.home_interlock.mark_home_confirmed(
                    "ID142～ID145 已同步回到並穩定確認 HOME"
                )
            else:
                self.home_interlock.mark_not_home(
                    f"四軸已位於待機姿態 {pose_name}"
                )

            result = {
                "pose": pose_name,
                "confirmed": True,
                "angles": angles,
                "all_home_confirmed": pose_name == "HOME",
            }

            self._set_status(
                LifecycleStatus.SUCCESS,
                "move_named_pose",
                f"四軸姿態 {pose_name} 已確認",
                **result,
            )

            return result

        except subprocess.TimeoutExpired as exc:
            error = ArmVisionWorkflowServiceError(
                "move_named_pose",
                f"{pose_name} 移動超過 "
                f"{timeout_seconds:g} 秒",
            )
            self.home_interlock.mark_unknown(str(error))
            self._set_status(
                LifecycleStatus.TIMEOUT,
                "move_named_pose",
                str(error),
            )
            raise error from exc

        except Exception as exc:
            self.home_interlock.mark_unknown(
                f"{pose_name} 四軸姿態失敗："
                f"{type(exc).__name__}: {exc}"
            )
            self._set_status(
                LifecycleStatus.ERROR,
                "move_named_pose",
                str(exc),
            )
            raise

    def move_standby_pose(
        self,
        pose_name: str = "STANDBY",
    ) -> dict[str, Any]:
        """Backward-compatible wrapper for existing startup/manual callers."""

        return self.move_named_pose(pose_name)

    def run_pick_and_place(
        self,
        cancel_event: threading.Event,
        *,
        pick_handoff: ArmHandoff,
        place_handoff: ArmHandoff,
        movement_handoff: ArmMovementHandoff | None = None,
        view: str | None = None,
        height_reference_depth_mm: float | None = None,
        progress_callback: ProgressCallback | None = None,
    ) -> dict[str, Any]:
        if not self.enabled:
            raise ArmVisionWorkflowServiceError("run", "手臂視覺流程目前未啟用")

        if height_reference_depth_mm is not None:
            reference_mm = float(height_reference_depth_mm)
            if not math.isfinite(reference_mm) or reference_mm <= 0:
                raise ArmVisionWorkflowServiceError(
                    "run",
                    "height_reference_depth_mm 必須是大於 0 的有限數值",
                )

        selected_view = self.config.view if view is None else str(view)
        if selected_view not in {"LView", "RView"}:
            raise ArmVisionWorkflowServiceError(
                "run",
                f"不支援的 Camera 視角：{selected_view}",
            )
        pick_side, place_side = (
            ("left", "right")
            if selected_view == "LView"
            else ("right", "left")
        )
        pick_marker = f"[HANDOFF] Arm is at {pick_side} target suction position."
        place_marker = (
            f"[HANDOFF] Arm is at {place_side} outer-branch place position."
        )
        command = self._command(selected_view)
        target_file = self.base_dir / "control_target.json"
        started_at = time.time()
        lines: list[str] = []
        line_queue: queue.Queue[str | None] = queue.Queue()
        reader_done = threading.Event()
        pick_done = False
        place_done = False
        target_prechecked = False
        all_home_confirmed = False
        arm_stop_confirmed = False
        arm_stop_unconfirmed = False
        result: dict[str, Any] = {}

        def report(step: str, message: str, status: LifecycleStatus = LifecycleStatus.RUNNING, **data: Any) -> None:
            self._set_status(status, step, message, **data)
            if progress_callback is not None:
                progress_callback(message)

        report(
            "start",
            f"啟動 {selected_view} D435 / CANBus 手臂流程",
            command=" ".join(command),
            view=selected_view,
        )
        if cancel_event.is_set():
            self._set_status(LifecycleStatus.CANCELLED, "cancelled", "流程已取消")
            raise RuntimeError("流程已取消")
        if movement_handoff is not None:
            report(
                "waiting_safe_height",
                "手臂子程序尚未啟動，等待升降機停止並確認 560mm 安全高度",
                LifecycleStatus.WAITING_SIGNAL,
            )
            try:
                movement_handoff("啟動 Camera／手臂子程序")
            except Exception as exc:
                self.home_interlock.mark_unknown(
                    f"手臂未啟動：560mm 安全高度確認失敗：{exc}"
                )
                self._set_status(
                    LifecycleStatus.ERROR,
                    "safe_height_blocked",
                    str(exc),
                )
                raise
            report(
                "safe_height_confirmed",
                "升降機 560mm 安全高度已確認，允許啟動 Camera／手臂子程序",
            )
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        self.home_interlock.mark_not_home("手臂視覺子程序即將啟動，HOME 狀態暫停信任")
        try:
            process = subprocess.Popen(
                command,
                cwd=self.base_dir,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
                start_new_session=os.name == "posix",
            )
        except OSError as exc:
            self.home_interlock.mark_unknown(f"無法啟動手臂視覺子程序：{exc}")
            error = ArmVisionWorkflowServiceError("start", f"無法啟動手臂視覺子程序：{exc}")
            self._set_status(LifecycleStatus.ERROR, "start_error", str(error))
            raise error from exc

        def reader() -> None:
            assert process.stdout is not None
            try:
                for raw_line in process.stdout:
                    line_queue.put(raw_line.rstrip())
            finally:
                reader_done.set()
                line_queue.put(None)

        reader_thread = threading.Thread(target=reader, daemon=True, name="arm-vision-output")
        reader_thread.start()
        launched_at = time.monotonic()
        overall_deadline = launched_at + self.config.overall_timeout_seconds
        startup_deadline = launched_at + self.config.startup_timeout_seconds
        last_output_at: float | None = None
        cancel_requested_at: float | None = None
        cancel_deadline: float | None = None

        def request_cancel_and_collect_stop() -> tuple[bool, bool]:
            nonlocal cancel_requested_at, cancel_deadline, last_output_at
            cancel_requested_at = time.monotonic()
            cancel_deadline = (
                cancel_requested_at + self.config.cancel_stop_timeout_seconds
            )
            self.home_interlock.mark_unknown(
                "第二步取消中，等待四顆 CAN 馬達停止確認"
            )
            report(
                "cancelling",
                "已要求手臂子程序停止四顆 CAN 馬達",
                LifecycleStatus.WAITING_SIGNAL,
            )
            self._request_graceful_stop(process)

            stop_confirmed = False
            stop_unconfirmed = False
            while time.monotonic() <= cancel_deadline:
                try:
                    output_line = line_queue.get(
                        timeout=self.config.poll_interval_seconds
                    )
                except queue.Empty:
                    if process.poll() is not None and reader_done.is_set():
                        break
                    continue
                if output_line is None:
                    if process.poll() is not None and reader_done.is_set():
                        break
                    continue
                last_output_at = time.monotonic()
                if output_line:
                    lines.append(output_line)
                    del lines[:-80]
                    report("arm_output", output_line)
                if ARM_STOP_CONFIRMED_MARKER in output_line:
                    stop_confirmed = True
                    report(
                        "cancel_stop_confirmed",
                        "四顆 CAN 馬達已回覆停止命令",
                        LifecycleStatus.WAITING_SIGNAL,
                    )
                elif ARM_STOP_UNCONFIRMED_MARKER in output_line:
                    stop_unconfirmed = True
            return stop_confirmed, stop_unconfirmed

        try:
            while True:
                now = time.monotonic()
                if cancel_event.is_set() and cancel_requested_at is None:
                    cancel_requested_at = now
                    cancel_deadline = now + self.config.cancel_stop_timeout_seconds
                    self.home_interlock.mark_unknown(
                        "第二步取消中，等待四顆 CAN 馬達停止確認"
                    )
                    report(
                        "cancelling",
                        "已要求手臂子程序停止四顆 CAN 馬達",
                        LifecycleStatus.WAITING_SIGNAL,
                    )
                    self._request_graceful_stop(process)

                if cancel_deadline is not None and now > cancel_deadline:
                    raise RuntimeError(
                        "流程已取消，但未在 "
                        f"{self.config.cancel_stop_timeout_seconds:g} 秒內收到"
                        "四顆 CAN 馬達停止確認"
                    )

                if cancel_requested_at is None and now > overall_deadline:
                    raise TimeoutError(
                        f"ArmVisionWorkflowService.overall_timeout: 手臂視覺流程超過 "
                        f"{self.config.overall_timeout_seconds:g} 秒；{self._last_output_text(lines)}"
                    )
                if (
                    cancel_requested_at is None
                    and last_output_at is None
                    and now > startup_deadline
                ):
                    raise TimeoutError(
                        f"ArmVisionWorkflowService.startup_timeout: 子程序啟動後 "
                        f"{self.config.startup_timeout_seconds:g} 秒仍無任何輸出；"
                        f"{self._last_output_text(lines)}"
                    )
                if (
                    cancel_requested_at is None
                    and last_output_at is not None
                    and now - last_output_at > self.config.no_output_timeout_seconds
                ):
                    raise TimeoutError(
                        f"ArmVisionWorkflowService.no_output_timeout: 子程序已連續 "
                        f"{self.config.no_output_timeout_seconds:g} 秒無新輸出；"
                        f"{self._last_output_text(lines)}"
                    )

                try:
                    line = line_queue.get(timeout=self.config.poll_interval_seconds)
                except queue.Empty:
                    if reader_done.is_set():
                        break
                    continue

                if line is None:
                    if reader_done.is_set():
                        break
                    continue

                last_output_at = time.monotonic()
                if line:
                    lines.append(line)
                    lines = lines[-80:]
                    report("arm_output", line)

                if "[INFO] Saved control_target.json" in line and not target_prechecked:
                    control_target = self._load_control_target(target_file, started_at)
                    depth_m = float(control_target["depth_m"])
                    height_mm = self.depth_m_to_plc_height_mm(
                        depth_m,
                        height_reference_depth_mm=height_reference_depth_mm,
                    )
                    if self.target_height_validator is not None:
                        self.target_height_validator(height_mm)
                    target_prechecked = True
                    report(
                        "height_precheck",
                        f"視覺目標高度預檢通過：{self._height_policy_text(depth_m, height_mm)}",
                        depth_m=depth_m,
                        height_mm=height_mm,
                    )

                if ALL_HOME_CONFIRMED_MARKER in line:
                    all_home_confirmed = True
                if ARM_STOP_CONFIRMED_MARKER in line:
                    arm_stop_confirmed = True
                    report(
                        "cancel_stop_confirmed",
                        "四顆 CAN 馬達已回覆停止命令",
                        LifecycleStatus.WAITING_SIGNAL,
                    )
                elif ARM_STOP_UNCONFIRMED_MARKER in line:
                    arm_stop_unconfirmed = True

                if pick_marker in line and not pick_done:
                    control_target = self._load_control_target(target_file, started_at)
                    depth_m = float(control_target["depth_m"])
                    height_mm = self.depth_m_to_plc_height_mm(
                        depth_m,
                        height_reference_depth_mm=height_reference_depth_mm,
                    )
                    if self.target_height_validator is not None:
                        self.target_height_validator(height_mm)
                    report(
                        "pick_handoff",
                        f"{pick_side} 側取料交接：{self._height_policy_text(depth_m, height_mm)}",
                        LifecycleStatus.WAITING_SIGNAL,
                        depth_m=depth_m,
                        height_mm=height_mm,
                    )
                    pick_handoff(height_mm, control_target)
                    result = {
                        "depth_m": depth_m,
                        "plc_height_mm": height_mm,
                        "control_target": control_target,
                    }
                    pick_done = True

                if place_marker in line and not place_done:
                    control_target = result.get("control_target") or self._load_control_target(target_file, started_at)
                    depth_m = float(control_target["depth_m"])
                    height_mm = self.depth_m_to_plc_height_mm(
                        depth_m,
                        height_reference_depth_mm=height_reference_depth_mm,
                    )
                    report(
                        "place_handoff",
                        f"{place_side} 側放料交接：{self._height_policy_text(depth_m, height_mm)}",
                        LifecycleStatus.WAITING_SIGNAL,
                        depth_m=depth_m,
                        height_mm=height_mm,
                    )
                    place_handoff(height_mm, control_target)
                    result = {
                        "depth_m": depth_m,
                        "plc_height_mm": height_mm,
                        "control_target": control_target,
                    }
                    place_done = True

                if self._should_press_enter(line):
                    if (
                        movement_handoff is not None
                        and self._requires_motion_height_gate(line)
                    ):
                        movement = self._movement_description(line)
                        report(
                            "waiting_safe_height",
                            f"手臂保持暫停，等待升降機停止並確認此動作的安全高度：{movement}",
                            LifecycleStatus.WAITING_SIGNAL,
                        )
                        movement_handoff(movement)
                        report(
                            "safe_height_confirmed",
                            f"升降機安全高度已確認，送出手臂繼續訊號：{movement}",
                        )
                    self._press_enter(process)

            return_code = process.wait(timeout=2)
            if cancel_event.is_set():
                if arm_stop_confirmed:
                    raise RuntimeError("流程已取消，四顆 CAN 馬達已確認停止")
                detail = "子程序回報停止失敗" if arm_stop_unconfirmed else "沒有收到停止確認"
                raise RuntimeError(f"流程已取消，但{detail}")
            if return_code != 0:
                raise ArmVisionWorkflowServiceError(
                    "run",
                    "手臂視覺流程結束碼 "
                    f"{return_code}，最後輸出：{' | '.join(lines[-10:])}",
                )
            if not pick_done or not place_done:
                raise ArmVisionWorkflowServiceError(
                    "run",
                    f"手臂流程未完成交接 pick={pick_done} place={place_done}；"
                    f"{self._last_output_text(lines)}",
                )
            if not all_home_confirmed:
                raise ArmVisionWorkflowServiceError(
                    "run",
                    "手臂流程雖已完成交接，但沒有收到手臂與相機 HOME 角度確認",
                )
            self.home_interlock.mark_home_confirmed(
                "手臂子程序已讀回並確認手臂與相機全部位於 HOME"
            )
            report(
                "success",
                f"手臂視覺流程完成，D500 工作高度 {float(result['plc_height_mm']):g}mm",
                LifecycleStatus.SUCCESS,
                **{key: value for key, value in result.items() if key != "control_target"},
            )
            return result
        except Exception as exc:
            if cancel_event.is_set() and cancel_requested_at is None:
                arm_stop_confirmed, arm_stop_unconfirmed = (
                    request_cancel_and_collect_stop()
                )
                if arm_stop_confirmed:
                    exc = RuntimeError("流程已取消，四顆 CAN 馬達已確認停止")
                elif arm_stop_unconfirmed:
                    exc = RuntimeError("流程已取消，但子程序回報停止失敗")
                else:
                    exc = RuntimeError(
                        "流程已取消，但未在 "
                        f"{self.config.cancel_stop_timeout_seconds:g} 秒內收到"
                        "四顆 CAN 馬達停止確認"
                    )
            self.home_interlock.mark_unknown(
                f"手臂視覺流程未正常完成 HOME 確認：{type(exc).__name__}: {exc}"
            )
            if cancel_event.is_set():
                status = LifecycleStatus.CANCELLED
                error_step = "cancelled"
            elif isinstance(exc, TimeoutError):
                status = LifecycleStatus.TIMEOUT
                error_step = "timeout"
            else:
                status = LifecycleStatus.ERROR
                error_step = "error"
            self._set_status(status, error_step, str(exc), last_output=lines[-1] if lines else None)
            raise exc
        finally:
            self._terminate_process(process)
            reader_thread.join(timeout=1.0)
            self._close_process_streams(process)

    def _command(self, view: str | None = None) -> list[str]:
        python_path = self._resolve_path(self.config.python_path) if self.config.python_path else Path(sys.executable)
        if not python_path.exists():
            python_path = Path(sys.executable)
        script_path = self._resolve_path(self.config.script_path)
        if not script_path.exists():
            raise ArmVisionWorkflowServiceError("config", f"找不到手臂程式: {script_path}")
        return [
            str(python_path),
            "-u",
            str(script_path),
            "--view",
            self.config.view if view is None else view,
            "--execute",
            "--yes",
            "--target-speed",
            str(self.config.target_speed_dps),
            "--settle",
            str(self.config.settle_seconds),
            "--target-wait",
            str(self.config.target_wait_seconds),
            "--pick-wait",
            str(self.config.pick_wait_seconds),
            "--home-tolerance",
            str(self.config.home_tolerance_degrees),
            "--home-stable-reads",
            str(self.config.home_stable_reads),
            "--home-timeout",
            str(self.config.home_timeout_seconds),
            "--home-poll",
            str(self.config.home_poll_interval_seconds),
        ]

    def _home_confirmation_command(self) -> list[str]:
        python_path = (
            self._resolve_path(self.config.python_path)
            if self.config.python_path
            else Path(sys.executable)
        )
        if not python_path.exists():
            python_path = Path(sys.executable)
        script_path = self._resolve_path(self.config.script_path)
        if not script_path.exists():
            raise ArmVisionWorkflowServiceError(
                "config",
                f"找不到手臂程式: {script_path}",
            )
        return [
            str(python_path),
            "-u",
            str(script_path),
            "--confirm-home-only",
            "--home-tolerance",
            str(self.config.home_tolerance_degrees),
            "--home-stable-reads",
            str(self.config.home_stable_reads),
            "--home-timeout",
            str(self.config.home_timeout_seconds),
            "--home-poll",
            str(self.config.home_poll_interval_seconds),
        ]

    def _home_component_command(self, component: str) -> list[str]:
        command = self._home_confirmation_command()
        confirm_index = command.index("--confirm-home-only")
        command[confirm_index : confirm_index + 1] = [
            "--move-home-only",
            component,
            "--settle",
            str(self.config.settle_seconds),
        ]
        return command

    def _named_pose_command(
        self,
        pose_name: str,
    ) -> list[str]:
        command = self._home_confirmation_command()

        confirm_index = command.index(
            "--confirm-home-only"
        )

        command[confirm_index : confirm_index + 1] = [
            "--move-pose-only",
            pose_name,
            "--settle",
            str(self.config.settle_seconds),
        ]

        return command
        
    def _resolve_path(self, path: str | Path) -> Path:
        item = Path(path)
        if item.is_absolute():
            return item
        return self.base_dir / item

    def _load_control_target(self, path: Path, started_at: float) -> dict[str, Any]:
        if not path.exists():
            raise ArmVisionWorkflowServiceError("control_target", f"找不到 {path}")
        if path.stat().st_mtime < started_at - 1.0:
            raise ArmVisionWorkflowServiceError("control_target", f"{path} 不是本次流程產生的檔案")
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or data.get("depth_m") is None:
            raise ArmVisionWorkflowServiceError("control_target", "control_target.json 缺少 depth_m")
        return data

    @staticmethod
    def _should_press_enter(line: str) -> bool:
        if "[STEP]" not in line:
            return False
        return (
            "Move from " in line and " back to HOME" in line
            or "Move HOME" in line
            or "Move LGrap" in line
            or "Move RGrap" in line
            or "FAKE PLC PICK DONE" in line
            or "Move left target" in line
            or "Move right target" in line
            or "FAKE PLC PLACE DONE" in line
            or "Return right_outer" in line
            or "Return left_outer" in line
        )

    @staticmethod
    def _requires_motion_height_gate(line: str) -> bool:
        if "[STEP]" not in line:
            return False
        return any(
            marker in line
            for marker in (
                " back to HOME",
                "Move HOME",
                "Move LGrap",
                "Move RGrap",
                "FAKE PLC PICK DONE",
                "Move left target",
                "Move right target",
                "Return right_outer",
                "Return left_outer",
            )
        )

    @staticmethod
    def _movement_description(line: str) -> str:
        step_text = line.split("[STEP]", 1)[-1].strip()
        return step_text.split("Press Enter", 1)[0].strip() or "下一段手臂動作"

    @staticmethod
    def _press_enter(process: subprocess.Popen[str]) -> None:
        if process.stdin is None or process.poll() is not None:
            return
        process.stdin.write("\n")
        process.stdin.flush()

    @staticmethod
    def _request_graceful_stop(process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return
        if os.name == "posix":
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                return
            except (AttributeError, OSError, ProcessLookupError):
                pass
        process.terminate()

    @staticmethod
    def _terminate_process(process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return
        terminated_group = False
        if os.name == "posix":
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                terminated_group = True
            except (AttributeError, OSError, ProcessLookupError):
                pass
        if not terminated_group:
            process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            killed_group = False
            if os.name == "posix":
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    killed_group = True
                except (AttributeError, OSError, ProcessLookupError):
                    pass
            if not killed_group:
                process.kill()
            process.wait(timeout=3)

    @staticmethod
    def _close_process_streams(process: subprocess.Popen[str]) -> None:
        for stream in (process.stdin, process.stdout):
            if stream is None:
                continue
            try:
                stream.close()
            except (OSError, ValueError):
                pass

    @staticmethod
    def _last_output_text(lines: list[str]) -> str:
        if not lines:
            return "最後輸出：（尚無輸出）"
        return f"最後輸出：{lines[-1]}"
