from __future__ import annotations

import sys
import threading
import time
import unittest
import ast
import contextlib
import io
import tempfile
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


PLC2_DIR = Path(__file__).resolve().parents[1]
if str(PLC2_DIR) not in sys.path:
    sys.path.insert(0, str(PLC2_DIR))

from config.loader import CONFIG_STORE, ConfigStore
from flow.home_flow import HomeFlow, HomeFlowConfig, HomeFlowState
from flow.main_cycle_flow import MainCycleConfig, MainCycleFlow, MainCyclePhase, MainCycleState, Step1Command
from flow.vision_height_flow import VisionHeightConfig, VisionHeightFlow, VisionHeightState
from service.home_service import HomeService, HomeServiceError
from service.lift_service import LiftService, LiftServiceConfig, LiftServiceError
from service.plc_service import PlcService
from service.pallet_transfer_service import PalletAction, PalletTransferService
from service.arm_vision_workflow_service import (
    ArmVisionWorkflowConfig,
    ArmVisionWorkflowService,
)
from service.arm_camera_home_interlock import ArmCameraHomeInterlock
from service.y_axes_service import YAxesService
from service.slot_vacuum_service import (
    CargoPurpose,
    SlotOccupancy,
    SlotStateStore,
    SlotVacuumConfig,
    SlotVacuumMode,
    SlotVacuumService,
    SlotVacuumServiceError,
)
from service.middle_vacuum_service import MiddleVacuumMode, MiddleVacuumService
from service.middle_vacuum_service import (
    MiddleVacuumServiceConfig,
    MiddleVacuumServiceError,
)
from lifecycle import LifecycleStatus
from plc.client import PLCConnectionError
from api.main import (
    FLOW_LOCK,
    PLC_MANAGER,
    _request_flow_cancellation,
    _run_independent_main_cycle_step,
    app,
)
from service.plc_service import PLC_SERVICE
from ui.main import MainWindow
from d435_control import (
    ALL_HOME_CONFIRMED_MARKER,
    ARM_HOME_CONFIRMED_MARKER,
    CALIB_BY_VIEW,
    CAMERA_HOME_CONFIRMED_MARKER,
    PREDICTION_LIMITS_BY_VIEW,
    build_route_plan,
    compute_left_outer_branch_angles,
    confirm_home_pose,
    stop_all_motors_confirmed,
    wait_until_named_pose,
)

class FakeClient:
    def __init__(self, host: str, port: int, unit: int) -> None:
        self.host = host
        self.port = port
        self.unit = unit
        self.connected = False
        self.words: dict[int, int] = {}
        self.bits: dict[tuple[str, int], bool] = {}
        self.fail_writes = False
        self.write_attempts = 0

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.connected = False

    def read_d_register(self, address: int, count: int = 1) -> list[int]:
        return [self.words.get(address + offset, 0) for offset in range(count)]

    def write_d_register(self, address: int, values: list[int]) -> None:
        for offset, value in enumerate(values):
            self.words[address + offset] = value

    def read_bit_device(self, device: str, address: int, count: int = 1) -> list[bool]:
        return [self.bits.get((device, address + offset), False) for offset in range(count)]

    def write_bit_device(self, device: str, address: int, values: list[bool]) -> None:
        self.write_attempts += 1
        if self.fail_writes:
            raise PLCConnectionError("simulated disconnect")
        for offset, value in enumerate(values):
            self.bits[(device, address + offset)] = value


class FakePointService:
    def __init__(self) -> None:
        self.connected = True
        self.values: dict[str, float | bool] = {
            "X_CUR_POS": 400.0,
            "Y1_CUR_POS": 20.0,
            "Y2_CUR_POS": 30.0,
            "Y1_VAC_ON": False,
            "Y1_VAC_OFF": False,
            "Y2_VAC_ON": False,
            "Y2_VAC_OFF": False,
            "X_Move": False,
            "X_MOVE_DOWN": False,
            "X_UP": False,
            "X_DOWN": False,
            "X_VAC_ON": False,
            "X_VAC_OFF": False,
        }
        self.writes: list[tuple[str, float | bool]] = []
        self.fail_write_point: str | None = None

    def is_connected(self, plc_name: str = "main_plc") -> bool:  # noqa: ARG002
        return self.connected

    def get_point(self, point_id: str):
        point = CONFIG_STORE.get_point(point_id)
        if point is None:
            raise KeyError(point_id)
        return point

    def read_point(self, point_id: str) -> float | bool:
        return self.values.get(point_id, False)

    def write_point(self, point_id: str, value: float | bool) -> None:
        if point_id == self.fail_write_point:
            raise RuntimeError("simulated write failure")
        self.writes.append((point_id, value))
        self.values[point_id] = value

    def register_write_guard(self, **_kwargs) -> None:
        return


class FakeArmPoseController:
    def __init__(self, readings: list[dict[str, float]]) -> None:
        self.point_config = {
            "HOME": {
                "ID 142": 6.0,
                "ID 143": -22.93,
                "ID 144": 6.7,
                "ID 145": 36.49,
            }
        }
        self.readings = list(readings)
        self.last_reading = dict(readings[-1])
        self.stop_result = {
            label: "OK stopped raw=00"
            for label in ("ID 142", "ID 143", "ID 144", "ID 145")
        }

    def read_positions(self) -> dict:
        if self.readings:
            self.last_reading = dict(self.readings.pop(0))
        return {
            "_updates": {
                label: f"{angle:.2f}"
                for label, angle in self.last_reading.items()
            }
        }

    def stop_all(self) -> dict:
        return dict(self.stop_result)


class FakeHomeDomainService:
    def __init__(self, positions: list[dict[str, float]]) -> None:
        self.positions = positions
        self.index = 0
        self.commands: list[bool] = []

    def precheck(self) -> None:
        return

    def set_home_commands(self, enabled: bool) -> None:
        self.commands.append(enabled)

    def read_positions(self) -> dict[str, float]:
        value = self.positions[min(self.index, len(self.positions) - 1)]
        self.index += 1
        return dict(value)


class FakeLiftDomainService:
    def __init__(self, heights: list[float]) -> None:
        self.heights = heights
        self.index = 0
        self.vacuum = {"left": False, "right": False}
        self.motion = {"up": False, "down": False}
        self.calls: list[str] = []
        self.position_config: list[tuple[float, float | None]] = []

    def precheck(self) -> None:
        self.calls.append("precheck")

    def read_height(self) -> float:
        value = self.heights[min(self.index, len(self.heights) - 1)]
        self.index += 1
        return value

    def configure_position(self, target_height_mm: float, speed: float | None = None) -> None:
        self.calls.append(f"position={target_height_mm:g}")
        self.position_config.append((target_height_mm, speed))

    def set_vacuum(self, enabled: bool) -> None:
        self.calls.append(f"vacuum={enabled}")
        self.vacuum = {"left": enabled, "right": enabled}

    def read_vacuum(self) -> dict[str, bool]:
        return dict(self.vacuum)

    def start_up(self) -> None:
        self.calls.append("start_up")
        self.motion = {"up": True, "down": False}

    def start_vision_positioning(self) -> None:
        self.calls.append("start_vision_positioning")
        self.motion = {"up": True, "down": False}

    def start_down(self) -> None:
        self.calls.append("start_down")
        self.motion = {"up": False, "down": True}

    def read_motion_commands(self) -> dict[str, bool]:
        return dict(self.motion)

    def stop(self) -> None:
        self.calls.append("stop")
        self.motion = {"up": False, "down": False}


class FakePalletTransferDomainService:
    def __init__(self) -> None:
        self.calls: list[str] = []
        self.positions = {"Y1": 0.0, "Y2": 0.0}
        self.position_sequences: dict[str, list[float]] = {"Y1": [], "Y2": []}
        self.forward_commands = {"Y1": False, "Y2": False}

    def precheck(self) -> None:
        self.calls.append("precheck")

    def set_forward_position(self, slot: str, forward_mm: float) -> None:
        self.calls.append(f"{slot}:forward={forward_mm:g}")

    def start_forward(self, slot: str) -> None:
        self.calls.append(f"{slot}:start_forward")
        self.forward_commands[slot] = True

    def stop_forward(self, slot: str | None = None) -> None:
        self.calls.append(f"{slot or 'all'}:stop_forward")
        selected = self.forward_commands if slot is None else {slot: self.forward_commands[slot]}
        for side in selected:
            self.forward_commands[side] = False

    def read_forward_commands(self) -> dict[str, bool]:
        return dict(self.forward_commands)

    def read_position(self, slot: str) -> float:
        sequence = self.position_sequences.get(slot, [])
        if sequence:
            value = sequence.pop(0)
            self.positions[slot] = value
            return value
        return self.positions[slot]

    def set_action_output(self, slot: str, action: PalletAction | str) -> dict[str, bool | str]:
        selected = action.value if isinstance(action, PalletAction) else str(action)
        self.calls.append(f"{slot}:action={selected}")
        return {"slot": slot, "mode": selected}


class FakeSlotVacuumDomainService:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def hold_for_main_cycle(self, side: str) -> None:
        self.calls.append(f"{side}:hold")

    def release_for_main_cycle(self, side: str) -> None:
        self.calls.append(f"{side}:release")

    def shutdown_after_cycle(self) -> None:
        self.calls.append("all:shutdown")


class FakeMiddleVacuumDomainService:
    def __init__(self) -> None:
        self.modes: list[str] = []
        self.confirmations: list[str] = []

    def set_mode(self, mode) -> dict[str, bool | str]:
        value = mode.value if hasattr(mode, "value") else str(mode)
        self.modes.append(value)
        return {"mode": value}

    def wait_transfer_ready(
        self,
        *,
        vacuum_expected,
        cancel_event,
        progress_callback=None,
    ) -> dict:
        if cancel_event.is_set():
            raise RuntimeError("流程已取消")
        action = "vacuum" if vacuum_expected else "release"
        self.confirmations.append(action)
        if progress_callback is not None:
            progress_callback(f"fake {action} confirmed")
        return {"confirmation": "fake", "vacuum_expected": vacuum_expected}


class FakeVisionBridgeDomainService:
    def __init__(self, height: float) -> None:
        self.height = height
        self.events: list[tuple[str, dict]] = []

    def start(self) -> None:
        self.events.append(("start", {}))

    def notify(self, phase: str, **payload) -> None:
        self.events.append((phase, dict(payload)))

    def wait_signal(self, cancel_event, timeout_seconds: float | None = None) -> None:  # noqa: ARG002
        self.events.append(("wait_signal", {}))

    def wait_height(self, cancel_event, timeout_seconds: float | None = None) -> float:  # noqa: ARG002
        self.events.append(("wait_height", {}))
        return self.height

    def wait_done(self, cancel_event, timeout_seconds: float | None = None) -> None:  # noqa: ARG002
        self.events.append(("wait_done", {}))


class FakeArmVisionWorkflowDomainService:
    def __init__(self, height_mm: float) -> None:
        self.height_mm = height_mm
        self.calls: list[str] = []
        self.views: list[str | None] = []

    def run_pick_and_place(
        self,
        cancel_event,
        *,
        pick_handoff,
        place_handoff,
        movement_handoff=None,
        view=None,
        progress_callback=None,
    ) -> dict:
        if movement_handoff is not None:
            movement_handoff("fake arm start")
        self.views.append(view)
        self.calls.append("run_pick_and_place")
        if progress_callback is not None:
            progress_callback("fake arm started")
        pick_handoff(self.height_mm, {"depth_m": 0.78})
        self.calls.append("pick_handoff_done")
        place_handoff(self.height_mm, {"depth_m": 0.78})
        self.calls.append("place_handoff_done")
        return {"depth_m": 0.78, "plc_height_mm": self.height_mm, "control_target": {"depth_m": 0.78}}


class ConfigTests(unittest.TestCase):
    def test_latest_rview_calibration_and_reverse_route_are_enabled(self) -> None:
        self.assertEqual(
            (
                3.50379437,
                -0.77081708,
                -0.03201293,
                -0.09647447,
                -0.07305849,
                -48.91500422,
            ),
            CALIB_BY_VIEW["RView"]["ID 142"],
        )
        self.assertEqual(
            {"ID 142": (-100.0, 14.0), "ID 143": (15.0, 127.0)},
            PREDICTION_LIMITS_BY_VIEW["RView"],
        )
        route = build_route_plan("RView")
        self.assertEqual(["HOME", "RView", "capture_and_detect"], route[:3])
        self.assertIn("RGrap", route)
        self.assertIn("id142_left_mirror", route)

        class Controller:
            point_config = {
                "HOME": {
                    "ID 142": 6.0,
                    "ID 143": -22.93,
                }
            }

        mirrored = compute_left_outer_branch_angles(
            Controller(),
            {"ID 142": -50.0, "ID 143": 60.0},
        )
        self.assertEqual(62.0, mirrored["ID 142"])
        self.assertEqual(254.14, mirrored["ID 143"])

    def test_lifecycle_status_has_the_standard_seven_values(self) -> None:
        self.assertEqual(
            {
                "pending",
                "running",
                "waiting_signal",
                "success",
                "cancelled",
                "timeout",
                "error",
            },
            {item.value for item in LifecycleStatus},
        )

    def test_plc2_is_canonical_for_connection_points_services_and_flows(self) -> None:
        store = ConfigStore()
        store.load()

        self.assertEqual(5000, store.get_plc("main_plc").port)
        self.assertEqual(350, store.get_point("X_UP").address)
        self.assertEqual(351, store.get_point("X_DOWN").address)
        self.assertEqual(500, store.get_point("X_FWD_POS").address)
        self.assertEqual(210, store.get_point("X_SPEED").address)
        self.assertEqual("X_HOME", store.get_service("home")["command_points"]["X"])
        self.assertEqual("X_Move", store.get_service("lift")["up_point"])
        self.assertEqual(
            695.0,
            store.get_service("lift")["arm_camera_not_home_maximum_height_mm"],
        )
        self.assertEqual(
            620.0,
            store.get_service("arm_vision_workflow")["height_reference_depth_mm"],
        )
        self.assertEqual(
            0.650,
            store.get_service("arm_vision_workflow")["height_formula_minimum_depth_m"],
        )
        self.assertEqual(
            "Y1_VAC_ON",
            store.get_service("slot_vacuum")["slots"]["Y1"]["vacuum_point"],
        )
        self.assertEqual(54, store.get_point("X_VAC_ON").address)
        self.assertEqual(56, store.get_point("X_VAC_OFF").address)
        self.assertEqual(
            "X_VAC_ON",
            store.get_service("middle_vacuum")["vacuum_point"],
        )
        self.assertEqual(
            {"Y1": "Y1_MOVE", "Y2": "Y2_MOVE"},
            store.get_service("y_axes")["command_points"],
        )
        self.assertEqual(90.0, store.get_flow("home")["timeout_seconds"])
        self.assertEqual(
            15.0,
            store.get_service("arm_vision_workflow")["home_timeout_seconds"],
        )
        self.assertEqual(
            4.0,
            store.get_service("slot_vacuum")["release_pulse_seconds"],
        )
        self.assertEqual(
            5.0,
            store.get_service("middle_vacuum")["release_settle_seconds"],
        )
        self.assertEqual(
            460.0,
            store.get_flow("main_cycle")["cross_side_safe_height_mm"],
        )
        self.assertTrue(store.get_point("Y1_MOVE").writable)
        self.assertTrue(store.get_point("Y2_MOVE").writable)
        self.assertEqual(560.0, store.get_flow("vision_height")["target_height_mm"])

    def test_removed_control_sections_are_not_present_in_ui(self) -> None:
        source = (PLC2_DIR / "ui" / "main.py").read_text(encoding="utf-8")
        self.assertNotIn("快速位置設定", source)
        self.assertNotIn("QUICK_POSITION_POINTS", source)
        self.assertNotIn("Y1 / Y2 載貨狀態與真空守護", source)
        self.assertIn("真空／破真空快捷控制", source)

    def test_ui_background_workers_do_not_call_tk_after_directly(self) -> None:
        tree = ast.parse((PLC2_DIR / "ui" / "main.py").read_text(encoding="utf-8"))
        violations: list[int] = []
        worker_count = 0
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            if node.name != "worker":
                continue
            worker_count += 1
            for child in ast.walk(node):
                if (
                    isinstance(child, ast.Call)
                    and isinstance(child.func, ast.Attribute)
                    and child.func.attr == "after"
                    and isinstance(child.func.value, ast.Name)
                    and child.func.value.id == "self"
                ):
                    violations.append(child.lineno)

        self.assertGreater(worker_count, 0)
        self.assertEqual([], violations)

    def test_second_step_progress_translates_hardware_messages(self) -> None:
        self.assertEqual(
            "正在確認 CANBus 與四顆馬達連線",
            MainWindow._friendly_second_step_message(
                "第二步手臂：[INFO] Verifying CAN communication"
            ),
        )
        self.assertEqual(
            "D435i 相機暖機中",
            MainWindow._friendly_second_step_message("[INFO] Warming up camera"),
        )
        self.assertEqual(
            "檢查辨識高度與 695mm 安全上限",
            MainWindow._second_step_arm_stage("height_precheck", "fallback"),
        )

    def test_second_step_progress_shows_detected_and_lift_target_heights(self) -> None:
        detected, target = MainWindow._second_step_height_values(
            0.708214,
            648.214,
        )

        self.assertAlmostEqual(708.214, detected)
        self.assertAlmostEqual(648.214, target)
        self.assertEqual(
            (None, None),
            MainWindow._second_step_height_values("invalid", None),
        )

    def test_main_cycle_waiting_phase_only_opens_confirmation_gate(self) -> None:
        calls: list[tuple[str, str | None]] = []

        class FakeWindow:
            main_cycle_phase = "waiting_step2"

            def _show_main_cycle_continue_gate(self, phase: str) -> None:
                calls.append(("gate", phase))

            def _start_main_cycle_sequence(
                self,
                *,
                confirmed_phase: str | None = None,
            ) -> None:
                calls.append(("start", confirmed_phase))

        MainWindow.on_main_cycle_run_clicked(FakeWindow())

        self.assertEqual([("gate", "waiting_step2")], calls)

    def test_main_cycle_gate_button_passes_explicit_phase_confirmation(self) -> None:
        calls: list[tuple[str, str | None]] = []

        class FakeWindow:
            main_cycle_phase = "waiting_final_step1"

            def _close_main_cycle_continue_gate(self) -> None:
                calls.append(("close", None))

            def _start_main_cycle_sequence(
                self,
                *,
                confirmed_phase: str | None = None,
            ) -> None:
                calls.append(("start", confirmed_phase))

        MainWindow._continue_main_cycle_from_gate(
            FakeWindow(),
            "waiting_final_step1",
        )

        self.assertEqual(
            [
                ("close", None),
                ("start", "waiting_final_step1"),
            ],
            calls,
        )

    def test_successful_first_step_forces_waiting_step2_gate(self) -> None:
        calls: list[tuple[str, object]] = []

        class FakeWindow:
            _active_flow_label = "第一步"
            main_cycle_phase = "ready_first_step"

            def _finish_flow(self, *_args, **kwargs) -> bool:
                calls.append(("dialog", kwargs["show_result_dialog"]))
                return True

            def _refresh_main_cycle_phase_ui(self) -> None:
                calls.append(("phase", self.main_cycle_phase))

            def after(self, _delay: int, callback) -> None:
                callback()

            def _show_main_cycle_continue_gate(self, phase: str) -> None:
                calls.append(("gate", phase))

        window = FakeWindow()
        MainWindow._finish_main_cycle_sequence(
            window,
            {
                "ok": True,
                # Simulate the stale phase returned by the previous backend.
                "data": {"phase": "ready_first_step"},
            },
            None,
            "waiting_step2",
            expected_success_phase="waiting_step2",
        )

        self.assertEqual("waiting_step2", window.main_cycle_phase)
        self.assertEqual(
            [
                ("dialog", False),
                ("phase", "waiting_step2"),
                ("gate", "waiting_step2"),
            ],
            calls,
        )

    def test_home_postcheck_failure_keeps_physically_completed_phase(self) -> None:
        calls: list[tuple[str, object]] = []

        class FakeWindow:
            _active_flow_label = "第一步"
            main_cycle_phase = "ready_first_step"

            def _finish_flow(self, *_args, **_kwargs) -> bool:
                calls.append(("dialog", True))
                return True

            def _refresh_main_cycle_phase_ui(self) -> None:
                calls.append(("phase", self.main_cycle_phase))

        window = FakeWindow()
        MainWindow._finish_main_cycle_sequence(
            window,
            {
                "ok": False,
                "data": {
                    "phase": "waiting_step2",
                    "step": "post_step_home_check",
                    "step_result": {"status": "success"},
                },
            },
            None,
            "ready_first_step",
        )

        self.assertEqual("waiting_step2", window.main_cycle_phase)
        self.assertEqual(
            [("dialog", True), ("phase", "waiting_step2")],
            calls,
        )

    def test_api_uses_shared_plc_service_and_exposes_flow_routes(self) -> None:
        paths = {route.path for route in app.routes}

        self.assertIs(PLC_SERVICE, PLC_MANAGER)
        self.assertIn("/flows/home/run", paths)
        self.assertIn("/flows/home/cancel", paths)
        self.assertIn("/flows/vision-height/run", paths)
        self.assertIn("/flows/vision-height/cancel", paths)
        self.assertIn("/flows/main-cycle/first-step", paths)
        self.assertIn("/flows/main-cycle/second-step", paths)
        self.assertIn("/flows/main-cycle/final-step", paths)
        self.assertIn("/flows/main-cycle/independent/first-step", paths)
        self.assertIn("/flows/main-cycle/independent/second-step", paths)
        self.assertIn("/flows/main-cycle/independent/third-step", paths)
        self.assertIn("/arm-camera-home/confirm", paths)
        self.assertIn("/arm-camera-home/move-arm", paths)
        self.assertIn("/arm-camera-home/move-camera", paths)
        self.assertIn("/lifecycle/status", paths)
        self.assertIn("/slots", paths)
        self.assertIn("/slots/{side}", paths)
        self.assertIn("/slots/{side}/manual-release", paths)
        self.assertIn("/slots/enforce", paths)
        self.assertIn("/middle-vacuum", paths)

    def test_independent_api_home_precheck_and_runner_share_one_flow_lock(self) -> None:
        lock_states: list[tuple[str, bool]] = []

        def confirm_home(_cancel_event=None) -> dict:
            lock_states.append(("home", FLOW_LOCK.locked()))
            return {"angles": {"ID142": 0.0, "ID143": 0.0, "ID144": 0.0, "ID145": 0.0}}

        def runner():
            lock_states.append(("runner", FLOW_LOCK.locked()))
            return SimpleNamespace(
                succeeded=True,
                status=LifecycleStatus.SUCCESS,
                state=MainCycleState.SUCCESS,
                phase=MainCyclePhase.READY_FIRST_STEP,
                step="independent_test",
                message="done",
                elapsed_seconds=0.1,
                height_mm=560.0,
                vision_height_mm=None,
                transfer_direction=None,
            )

        with patch(
            "api.main.ARM_VISION_WORKFLOW_SERVICE.confirm_home",
            side_effect=confirm_home,
        ):
            response = _run_independent_main_cycle_step("獨立第一步", runner)

        self.assertTrue(response.ok)
        self.assertEqual(
            [("home", True), ("runner", True), ("home", True)],
            lock_states,
        )
        self.assertTrue(response.data["home_postcheck"]["confirmed"])
        self.assertTrue(response.data["home_postcheck"]["arm_home_confirmed"])
        self.assertTrue(response.data["home_postcheck"]["camera_home_confirmed"])
        self.assertFalse(FLOW_LOCK.locked())

    def test_independent_api_prepares_safe_height_before_home_under_same_lock(self) -> None:
        call_order: list[tuple[str, bool]] = []

        def prepare_height() -> float:
            call_order.append(("height", FLOW_LOCK.locked()))
            return 560.0

        def confirm_home(_cancel_event=None) -> dict:
            call_order.append(("home", FLOW_LOCK.locked()))
            return {"angles": {"ID142": 0.0, "ID143": 0.0, "ID144": 0.0, "ID145": 0.0}}

        def runner():
            call_order.append(("runner", FLOW_LOCK.locked()))
            return SimpleNamespace(
                succeeded=True,
                status=LifecycleStatus.SUCCESS,
                state=MainCycleState.SUCCESS,
                phase=MainCyclePhase.READY_FIRST_STEP,
                step="independent_test",
                message="done",
                elapsed_seconds=0.1,
                height_mm=560.0,
                vision_height_mm=None,
                transfer_direction=None,
            )

        with patch(
            "api.main.ARM_VISION_WORKFLOW_SERVICE.confirm_home",
            side_effect=confirm_home,
        ):
            response = _run_independent_main_cycle_step(
                "獨立第二步",
                runner,
                before_home_precheck=prepare_height,
            )

        self.assertTrue(response.ok)
        self.assertEqual(
            [
                ("height", True),
                ("home", True),
                ("runner", True),
                ("home", True),
            ],
            call_order,
        )
        self.assertEqual(560.0, response.data["safe_height_precheck_mm"])
        self.assertFalse(FLOW_LOCK.locked())

    def test_independent_api_safe_height_failure_never_checks_home_or_runs(self) -> None:
        home_called = False
        runner_called = False

        def confirm_home(_cancel_event=None) -> dict:
            nonlocal home_called
            home_called = True
            return {}

        def runner():
            nonlocal runner_called
            runner_called = True
            raise AssertionError("560mm 失敗後不可啟動子流程")

        with patch(
            "api.main.ARM_VISION_WORKFLOW_SERVICE.confirm_home",
            side_effect=confirm_home,
        ):
            response = _run_independent_main_cycle_step(
                "獨立第二步",
                runner,
                before_home_precheck=lambda: (_ for _ in ()).throw(
                    RuntimeError("升降機未到位")
                ),
            )

        self.assertFalse(response.ok)
        self.assertFalse(home_called)
        self.assertFalse(runner_called)
        self.assertEqual(
            "independent_second_step_to_safe_height",
            response.data["step"],
        )
        self.assertIn("升降機未到位", response.error)
        self.assertFalse(FLOW_LOCK.locked())

    def test_independent_api_postcheck_failure_blocks_success_response(self) -> None:
        confirm_calls = 0

        def confirm_home(_cancel_event=None) -> dict:
            nonlocal confirm_calls
            confirm_calls += 1
            if confirm_calls == 1:
                return {
                    "confirmed": True,
                    "angles": {
                        "ID142": 0.0,
                        "ID143": 0.0,
                        "ID144": 0.0,
                        "ID145": 0.0,
                    },
                }
            raise RuntimeError("ID145 未回 HOME")

        def runner():
            return SimpleNamespace(
                succeeded=True,
                status=LifecycleStatus.SUCCESS,
                state=MainCycleState.SUCCESS,
                phase=MainCyclePhase.READY_FIRST_STEP,
                step="independent_test",
                message="done",
                elapsed_seconds=0.1,
                height_mm=560.0,
                vision_height_mm=None,
                transfer_direction=None,
            )

        with patch(
            "api.main.ARM_VISION_WORKFLOW_SERVICE.confirm_home",
            side_effect=confirm_home,
        ):
            response = _run_independent_main_cycle_step("獨立第一步", runner)

        self.assertFalse(response.ok)
        self.assertEqual(2, confirm_calls)
        self.assertEqual("post_step_home_check", response.data["step"])
        self.assertFalse(response.data["home_postcheck"]["confirmed"])
        self.assertIn("ID145 未回 HOME", response.error)
        self.assertFalse(FLOW_LOCK.locked())

    def test_failed_independent_step_still_runs_home_postcheck(self) -> None:
        confirm_calls = 0

        def confirm_home(_cancel_event=None) -> dict:
            nonlocal confirm_calls
            confirm_calls += 1
            return {
                "confirmed": True,
                "angles": {
                    "ID142": 0.0,
                    "ID143": 0.0,
                    "ID144": 0.0,
                    "ID145": 0.0,
                },
            }

        def runner():
            return SimpleNamespace(
                succeeded=False,
                status=LifecycleStatus.ERROR,
                state=MainCycleState.ERROR,
                phase=MainCyclePhase.READY_FIRST_STEP,
                step="independent_test",
                message="simulated step failure",
                elapsed_seconds=0.1,
                height_mm=560.0,
                vision_height_mm=None,
                transfer_direction=None,
            )

        with patch(
            "api.main.ARM_VISION_WORKFLOW_SERVICE.confirm_home",
            side_effect=confirm_home,
        ):
            response = _run_independent_main_cycle_step("獨立第二步", runner)

        self.assertFalse(response.ok)
        self.assertEqual(2, confirm_calls)
        self.assertTrue(response.data["home_postcheck"]["confirmed"])
        self.assertIn("simulated step failure", response.error)
        self.assertFalse(FLOW_LOCK.locked())

    def test_independent_api_home_failure_never_starts_runner(self) -> None:
        runner_called = False

        def runner():
            nonlocal runner_called
            runner_called = True
            raise AssertionError("HOME 失敗後不可啟動子流程")

        with patch(
            "api.main.ARM_VISION_WORKFLOW_SERVICE.confirm_home",
            side_effect=RuntimeError("ID144 不在 HOME"),
        ):
            response = _run_independent_main_cycle_step("獨立第二步", runner)

        self.assertFalse(response.ok)
        self.assertFalse(runner_called)
        self.assertEqual("independent_home_precheck", response.data["step"])
        self.assertIn("ID144 不在 HOME", response.error)
        self.assertFalse(FLOW_LOCK.locked())

    def test_independent_api_cancel_after_home_never_starts_runner(self) -> None:
        runner_called = False

        def confirm_home(cancel_event) -> dict:
            cancel_event.set()
            return {"angles": {"ID142": 0.0, "ID143": 0.0, "ID144": 0.0, "ID145": 0.0}}

        def runner():
            nonlocal runner_called
            runner_called = True
            raise AssertionError("收到取消後不可啟動子流程")

        with patch(
            "api.main.ARM_VISION_WORKFLOW_SERVICE.confirm_home",
            side_effect=confirm_home,
        ):
            response = _run_independent_main_cycle_step("獨立第一步", runner)

        self.assertFalse(response.ok)
        self.assertFalse(runner_called)
        self.assertEqual("cancelled", response.data["state"])
        self.assertFalse(FLOW_LOCK.locked())

    def test_double_clicking_m52_off_uses_unified_y2_vacuum_control(self) -> None:
        calls: list[tuple[str, str]] = []

        class FakeTable:
            @staticmethod
            def identify_row(_y: int) -> str:
                return "Y2_VAC_ON"

            @staticmethod
            def identify_column(_x: int) -> str:
                return "#5"

        class FakeWindow:
            table = FakeTable()
            _values = {"Y2_VAC_ON": "True"}

            @staticmethod
            def _get_point(point_id: str):
                return CONFIG_STORE.get_point(point_id)

            @staticmethod
            def _is_bit(point) -> bool:
                return MainWindow._is_bit(point)

            @staticmethod
            def _set_side_vacuum(side: str, mode: str) -> None:
                calls.append((side, mode))

            @staticmethod
            def write_point_value(_point_id: str, _value: str) -> bool:
                raise AssertionError("M52 OFF 不應走一般點位寫入")

        event = type("Event", (), {"x": 0, "y": 0})()
        MainWindow.on_table_double_click(FakeWindow(), event)

        self.assertEqual([("Y2", "off")], calls)


class PlcServiceTests(unittest.TestCase):
    def test_arm_camera_interlock_blocks_every_lift_write_path_above_695(self) -> None:
        clients: list[FakeClient] = []

        def factory(host: str, port: int, unit: int) -> FakeClient:
            client = FakeClient(host, port, unit)
            clients.append(client)
            return client

        interlock = ArmCameraHomeInterlock()
        plc_service = PlcService(client_factory=factory)
        LiftService(plc_service, home_interlock=interlock)
        plc_service.connect()

        plc_service.write_point("X_FWD_POS", 695)
        with self.assertRaisesRegex(LiftServiceError, "695mm"):
            plc_service.write_point("X_FWD_POS", 696)
        with self.assertRaisesRegex(LiftServiceError, "695mm"):
            plc_service.write_d_register("main_plc", 500, [696])

        clients[0].words[500] = 696
        with self.assertRaisesRegex(LiftServiceError, "695mm"):
            plc_service.write_point("X_Move", True)
        with self.assertRaisesRegex(LiftServiceError, "695mm"):
            plc_service.write_bit_device("main_plc", "M", 376, [True])
        clients[0].words[500] = 695
        with self.assertRaisesRegex(LiftServiceError, "禁止使用連續手動上升"):
            plc_service.write_point("X_UP", True)
        with self.assertRaisesRegex(LiftServiceError, "禁止使用連續手動上升"):
            plc_service.write_bit_device("main_plc", "M", 350, [True])

        self.assertEqual(695, clients[0].words[500])
        self.assertNotIn(("M", 350), clients[0].bits)
        self.assertNotIn(("M", 376), clients[0].bits)

    def test_home_confirmation_restores_retracted_1450_height_envelope(self) -> None:
        clients: list[FakeClient] = []

        def factory(host: str, port: int, unit: int) -> FakeClient:
            client = FakeClient(host, port, unit)
            clients.append(client)
            return client

        interlock = ArmCameraHomeInterlock()
        interlock.mark_home_confirmed("test readback confirmed")
        plc_service = PlcService(client_factory=factory)
        LiftService(plc_service, home_interlock=interlock)
        plc_service.connect()

        plc_service.write_d_register("main_plc", 500, [739])
        plc_service.write_bit_device("main_plc", "M", 376, [True])
        plc_service.write_bit_device("main_plc", "M", 350, [True])

        self.assertEqual(739, clients[0].words[500])
        self.assertTrue(clients[0].bits[("M", 376)])
        self.assertTrue(clients[0].bits[("M", 350)])

    def test_middle_vacuum_service_switches_outputs_with_interlock(self) -> None:
        clients: list[FakeClient] = []

        def factory(host: str, port: int, unit: int) -> FakeClient:
            client = FakeClient(host, port, unit)
            clients.append(client)
            return client

        plc_service = PlcService(client_factory=factory)
        middle_service = MiddleVacuumService(plc_service)
        plc_service.connect()

        vacuum_state = middle_service.set_mode(MiddleVacuumMode.VACUUM)
        self.assertEqual("vacuum", vacuum_state["mode"])
        self.assertTrue(clients[0].bits[("M", 54)])
        self.assertFalse(clients[0].bits[("M", 56)])

        release_state = middle_service.set_mode(MiddleVacuumMode.BREAK_VACUUM)
        self.assertEqual("break_vacuum", release_state["mode"])
        self.assertFalse(clients[0].bits[("M", 54)])
        self.assertTrue(clients[0].bits[("M", 56)])

        off_state = middle_service.set_mode(MiddleVacuumMode.OFF)
        self.assertEqual("off", off_state["mode"])
        self.assertFalse(clients[0].bits[("M", 54)])
        self.assertFalse(clients[0].bits[("M", 56)])

    def test_middle_vacuum_interlock_blocks_point_and_raw_conflicts(self) -> None:
        clients: list[FakeClient] = []

        def factory(host: str, port: int, unit: int) -> FakeClient:
            client = FakeClient(host, port, unit)
            clients.append(client)
            return client

        plc_service = PlcService(client_factory=factory)
        MiddleVacuumService(plc_service)
        plc_service.connect()
        plc_service.write_point("X_VAC_ON", True)

        with self.assertRaisesRegex(RuntimeError, "不可同時開啟"):
            plc_service.write_point("X_VAC_OFF", True)
        with self.assertRaisesRegex(RuntimeError, "同時 ON"):
            plc_service.write_bit_device("main_plc", "M", 56, [True])

        plc_service.write_bit_device("main_plc", "M", 54, [False, False, True])
        self.assertFalse(clients[0].bits[("M", 54)])
        self.assertTrue(clients[0].bits[("M", 56)])

    def test_occupied_slot_blocks_all_vacuum_release_write_paths(self) -> None:
        clients: list[FakeClient] = []

        def factory(host: str, port: int, unit: int) -> FakeClient:
            client = FakeClient(host, port, unit)
            clients.append(client)
            return client

        plc_service = PlcService(client_factory=factory)
        plc_service.connect()
        with tempfile.TemporaryDirectory() as directory:
            slot_service = SlotVacuumService(
                plc_service,
                state_store=SlotStateStore(Path(directory) / "slot_states.json"),
            )
            slot_service.update_state("Y1", SlotOccupancy.OCCUPIED, cargo_id="BOX-LOCK")

            with self.assertRaisesRegex(RuntimeError, "禁止關閉真空"):
                plc_service.write_point("Y1_VAC_ON", False)
            with self.assertRaisesRegex(RuntimeError, "禁止啟動破真空"):
                plc_service.write_point("Y1_VAC_OFF", True)
            with self.assertRaisesRegex(RuntimeError, "M50=OFF"):
                plc_service.write_bit_device("main_plc", "M", 50, [False])

            empty = slot_service.update_state(
                "Y1",
                SlotOccupancy.EMPTY,
                confirm_release=True,
            )

        self.assertEqual(SlotOccupancy.EMPTY, empty.occupancy)
        self.assertFalse(clients[0].bits[("M", 50)])

    def test_point_service_reads_and_writes_by_point_id(self) -> None:
        clients: list[FakeClient] = []

        def factory(host: str, port: int, unit: int) -> FakeClient:
            client = FakeClient(host, port, unit)
            clients.append(client)
            return client

        service = PlcService(client_factory=factory)
        self.assertEqual(LifecycleStatus.PENDING, service.status)
        service.connect()
        self.assertEqual(LifecycleStatus.SUCCESS, service.status)

        service.write_point("X_SPEED", 500)
        service.write_point("X_UP", True)

        self.assertEqual(500, service.read_point("X_SPEED"))
        self.assertTrue(service.read_point("X_UP"))
        self.assertEqual(5000, clients[0].port)
        self.assertTrue(clients[0].bits[("M", 350)])

    def test_write_disconnect_is_not_automatically_retried(self) -> None:
        clients: list[FakeClient] = []

        def factory(host: str, port: int, unit: int) -> FakeClient:
            client = FakeClient(host, port, unit)
            clients.append(client)
            return client

        service = PlcService(client_factory=factory)
        service.connect()
        clients[0].fail_writes = True

        with self.assertRaisesRegex(RuntimeError, "寫入結果未知.*未自動重送"):
            service.write_point("X_UP", True)

        self.assertEqual(1, clients[0].write_attempts)
        self.assertEqual(1, len(clients))

    def test_raw_bit_writes_block_m389_and_m399_only(self) -> None:
        clients: list[FakeClient] = []

        def factory(host: str, port: int, unit: int) -> FakeClient:
            client = FakeClient(host, port, unit)
            clients.append(client)
            return client

        service = PlcService(client_factory=factory)
        service.connect()
        service.write_bit_device("main_plc", "M", 375, [True])
        service.write_bit_device("main_plc", "M", 374, [True])

        with self.assertRaisesRegex(RuntimeError, "M389"):
            service.write_bit_device("main_plc", "M", 389, [True])
        with self.assertRaisesRegex(RuntimeError, "M399"):
            service.write_bit_device("main_plc", "M", 399, [True])

        self.assertEqual(LifecycleStatus.ERROR, service.status)

        self.assertTrue(clients[0].bits[("M", 375)])
        self.assertTrue(clients[0].bits[("M", 374)])
        self.assertNotIn(("M", 389), clients[0].bits)
        self.assertNotIn(("M", 399), clients[0].bits)


class DomainServiceTests(unittest.TestCase):
    def test_middle_vacuum_timer_fallback_is_explicitly_not_sensor_confirmation(
        self,
    ) -> None:
        plc = FakePointService()
        config = replace(
            MiddleVacuumServiceConfig.load(),
            confirmation_point=None,
            vacuum_settle_seconds=0.0,
        )
        service = MiddleVacuumService(plc, config)
        messages: list[str] = []

        result = service.wait_transfer_ready(
            vacuum_expected=True,
            cancel_event=threading.Event(),
            progress_callback=messages.append,
        )

        self.assertEqual("timer_only", result["confirmation"])
        self.assertIn("不可視為吸附感測確認", messages[-1])

    def test_middle_break_vacuum_waits_five_seconds_before_arm_release(self) -> None:
        plc = FakePointService()
        service = MiddleVacuumService(plc, MiddleVacuumServiceConfig.load())
        waits: list[float] = []

        class RecordingEvent:
            @staticmethod
            def wait(delay: float) -> bool:
                waits.append(delay)
                return False

        result = service.wait_transfer_ready(
            vacuum_expected=False,
            cancel_event=RecordingEvent(),
        )

        self.assertEqual("timer_only", result["confirmation"])
        self.assertEqual(5.0, result["delay_seconds"])
        self.assertEqual([5.0], waits)

    def test_middle_vacuum_can_require_stable_sensor_confirmation(self) -> None:
        plc = FakePointService()
        plc.values["Y1_VAC_ON"] = True
        config = replace(
            MiddleVacuumServiceConfig.load(),
            confirmation_point="Y1_VAC_ON",
            stable_read_count=2,
            confirmation_timeout_seconds=0.1,
            poll_interval_seconds=0.001,
        )
        service = MiddleVacuumService(plc, config)

        result = service.wait_transfer_ready(
            vacuum_expected=True,
            cancel_event=threading.Event(),
        )

        self.assertEqual("sensor", result["confirmation"])
        self.assertEqual("Y1_VAC_ON", result["confirmation_point"])

    def test_middle_vacuum_sensor_timeout_blocks_arm_handoff(self) -> None:
        plc = FakePointService()
        plc.values["Y1_VAC_ON"] = False
        config = replace(
            MiddleVacuumServiceConfig.load(),
            confirmation_point="Y1_VAC_ON",
            stable_read_count=2,
            confirmation_timeout_seconds=0.003,
            poll_interval_seconds=0.001,
        )
        service = MiddleVacuumService(plc, config)

        with self.assertRaisesRegex(
            MiddleVacuumServiceError,
            "等待吸附成立超過",
        ):
            service.wait_transfer_ready(
                vacuum_expected=True,
                cancel_event=threading.Event(),
            )

    def test_occupied_slot_forces_vacuum_on_and_persists_state(self) -> None:
        plc = FakePointService()
        plc.values["Y1_VAC_OFF"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slot_states.json"
            service = SlotVacuumService(plc, state_store=SlotStateStore(path))

            state = service.update_state(
                "Y1",
                SlotOccupancy.OCCUPIED,
                cargo_id="BOX-001",
                purpose=CargoPurpose.TARGET_TO_PICK,
                shelf_id="A01",
                shelf_level=2,
            )

            self.assertEqual(SlotOccupancy.OCCUPIED, state.occupancy)
            self.assertEqual(LifecycleStatus.SUCCESS, service.status)
            self.assertIn(("Y1_VAC_OFF", False), plc.writes)
            self.assertIn(("Y1_VAC_ON", True), plc.writes)
            restored = SlotStateStore(path).get("Y1")
            self.assertEqual("BOX-001", restored.cargo_id)
            self.assertTrue(restored.vacuum_required)

    def test_empty_slot_requires_explicit_release_confirmation(self) -> None:
        plc = FakePointService()
        with tempfile.TemporaryDirectory() as directory:
            service = SlotVacuumService(
                plc,
                state_store=SlotStateStore(Path(directory) / "slot_states.json"),
            )
            service.update_state("Y2", SlotOccupancy.OCCUPIED, cargo_id="BOX-002")
            plc.writes.clear()

            with self.assertRaises(SlotVacuumServiceError):
                service.update_state("Y2", SlotOccupancy.EMPTY)

            self.assertEqual(SlotOccupancy.OCCUPIED, service.state_store.get("Y2").occupancy)
            self.assertNotIn(("Y2_VAC_ON", False), plc.writes)

            state = service.update_state(
                "Y2",
                SlotOccupancy.EMPTY,
                confirm_release=True,
            )
            self.assertEqual(SlotOccupancy.EMPTY, state.occupancy)
            self.assertIn(("Y2_VAC_ON", False), plc.writes)

    def test_side_vacuum_shortcuts_interlock_outputs_and_require_release_confirmation(self) -> None:
        plc = FakePointService()
        with tempfile.TemporaryDirectory() as directory:
            service = SlotVacuumService(
                plc,
                state_store=SlotStateStore(Path(directory) / "slot_states.json"),
            )

            vacuum_state = service.set_manual_mode("Y1", SlotVacuumMode.VACUUM)
            self.assertEqual("vacuum", vacuum_state["mode"])
            self.assertTrue(plc.values["Y1_VAC_ON"])
            self.assertFalse(plc.values["Y1_VAC_OFF"])

            with self.assertRaisesRegex(SlotVacuumServiceError, "必須明確確認"):
                service.set_manual_mode("Y1", SlotVacuumMode.BREAK_VACUUM)

            release_state = service.set_manual_mode(
                "Y1",
                SlotVacuumMode.BREAK_VACUUM,
                confirm_release=True,
            )
            self.assertEqual("break_vacuum", release_state["mode"])
            self.assertFalse(plc.values["Y1_VAC_ON"])
            self.assertTrue(plc.values["Y1_VAC_OFF"])
            self.assertEqual(SlotOccupancy.EMPTY, service.state_store.get("Y1").occupancy)
            self.assertFalse(service.state_store.get("Y1").vacuum_required)

            off_state = service.set_manual_mode(
                "Y1",
                SlotVacuumMode.OFF,
                confirm_release=True,
            )
            self.assertEqual("off", off_state["mode"])
            self.assertFalse(plc.values["Y1_VAC_ON"])
            self.assertFalse(plc.values["Y1_VAC_OFF"])

    def test_vacuum_enforcement_recovers_an_occupied_side(self) -> None:
        plc = FakePointService()
        with tempfile.TemporaryDirectory() as directory:
            service = SlotVacuumService(
                plc,
                state_store=SlotStateStore(Path(directory) / "slot_states.json"),
            )
            service.update_state("Y1", SlotOccupancy.OCCUPIED, cargo_id="BOX-003")
            plc.values["Y1_VAC_ON"] = False
            plc.writes.clear()

            enforced = service.enforce_required_vacuum()

            self.assertEqual({"Y1": True}, enforced)
            self.assertIn(("Y1_VAC_ON", True), plc.writes)

    def test_main_cycle_hold_persists_until_explicit_same_side_release(self) -> None:
        plc = FakePointService()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slot_states.json"
            service = SlotVacuumService(
                plc,
                state_store=SlotStateStore(path),
            )

            held = service.hold_for_main_cycle("Y2")

            self.assertEqual(SlotOccupancy.OCCUPIED, held.occupancy)
            self.assertTrue(held.vacuum_required)
            self.assertTrue(plc.values["Y2_VAC_ON"])
            self.assertFalse(plc.values["Y2_VAC_OFF"])
            self.assertTrue(SlotStateStore(path).get("Y2").vacuum_required)

            released = service.release_for_main_cycle("Y2")

            self.assertEqual(SlotOccupancy.EMPTY, released.occupancy)
            self.assertFalse(released.vacuum_required)
            self.assertFalse(plc.values["Y2_VAC_ON"])
            self.assertTrue(plc.values["Y2_VAC_OFF"])
            self.assertFalse(SlotStateStore(path).get("Y2").vacuum_required)

    def test_manual_release_turns_off_vacuum_and_clears_persistent_guard(self) -> None:
        plc = FakePointService()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slot_states.json"
            service = SlotVacuumService(
                plc,
                config=replace(SlotVacuumConfig.load(), release_pulse_seconds=0.0),
                state_store=SlotStateStore(path),
            )
            service.hold_for_main_cycle("Y1")
            plc.writes.clear()

            released = service.manual_release("Y1")

            self.assertEqual(SlotOccupancy.EMPTY, released.occupancy)
            self.assertFalse(released.vacuum_required)
            self.assertFalse(plc.values["Y1_VAC_ON"])
            self.assertFalse(plc.values["Y1_VAC_OFF"])
            self.assertEqual(
                [
                    ("Y1_VAC_ON", False),
                    ("Y1_VAC_OFF", True),
                    ("Y1_VAC_OFF", False),
                    ("Y1_VAC_ON", False),
                ],
                plc.writes,
            )
            restored = SlotStateStore(path).get("Y1")
            self.assertEqual(SlotOccupancy.EMPTY, restored.occupancy)
            self.assertFalse(restored.vacuum_required)
            self.assertEqual(LifecycleStatus.SUCCESS, service.status)

    def test_cycle_shutdown_turns_off_both_sides_and_clears_all_guards(self) -> None:
        plc = FakePointService()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slot_states.json"
            service = SlotVacuumService(
                plc,
                state_store=SlotStateStore(path),
            )
            service.hold_for_main_cycle("Y1")
            service.hold_for_main_cycle("Y2")

            released = service.shutdown_after_cycle()

            self.assertEqual({"Y1", "Y2"}, set(released))
            for side in ("Y1", "Y2"):
                self.assertEqual(SlotOccupancy.EMPTY, released[side].occupancy)
                self.assertFalse(released[side].vacuum_required)
                self.assertFalse(plc.values[f"{side}_VAC_ON"])
                self.assertFalse(plc.values[f"{side}_VAC_OFF"])
                self.assertFalse(SlotStateStore(path).get(side).vacuum_required)
            self.assertEqual(LifecycleStatus.SUCCESS, service.status)

    def test_cycle_shutdown_refuses_completion_when_vacuum_readback_stays_on(self) -> None:
        class StuckVacuumPointService(FakePointService):
            def write_point(self, point_id: str, value: float | bool) -> None:
                self.writes.append((point_id, value))
                if point_id == "Y2_VAC_ON" and value is False:
                    return
                self.values[point_id] = value

        plc = StuckVacuumPointService()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "slot_states.json"
            service = SlotVacuumService(
                plc,
                state_store=SlotStateStore(path),
            )
            service.hold_for_main_cycle("Y2")

            with self.assertRaisesRegex(RuntimeError, "Y2 真空輸出讀回仍為 ON"):
                service.shutdown_after_cycle()

            persisted = SlotStateStore(path).get("Y2")
            self.assertTrue(persisted.vacuum_required)
            self.assertEqual(LifecycleStatus.ERROR, service.status)

    def test_unknown_does_not_release_previous_occupied_vacuum_guard(self) -> None:
        plc = FakePointService()
        with tempfile.TemporaryDirectory() as directory:
            service = SlotVacuumService(
                plc,
                state_store=SlotStateStore(Path(directory) / "slot_states.json"),
            )
            service.update_state("Y1", SlotOccupancy.OCCUPIED, cargo_id="BOX-004")

            state = service.update_state("Y1", SlotOccupancy.UNKNOWN)

            self.assertEqual(SlotOccupancy.UNKNOWN, state.occupancy)
            self.assertTrue(state.vacuum_required)

    def test_home_service_uses_configured_points(self) -> None:
        plc = FakePointService()
        service = HomeService(plc)

        service.precheck()
        self.assertEqual(LifecycleStatus.SUCCESS, service.status)
        service.set_home_commands(True)
        self.assertEqual(LifecycleStatus.WAITING_SIGNAL, service.status)
        positions = service.read_positions()

        self.assertEqual({"X": 400.0, "Y1": 20.0, "Y2": 30.0}, positions)
        self.assertIn(("X_HOME", True), plc.writes)
        self.assertIn(("Y1_HOME", True), plc.writes)
        self.assertIn(("Y2_HOME", True), plc.writes)

    def test_y_axes_service_uses_excel_forward_points(self) -> None:
        plc = FakePointService()
        service = YAxesService(plc)

        service.precheck()
        service.start_both_positioning()
        self.assertEqual(LifecycleStatus.WAITING_SIGNAL, service.status)
        positions = service.read_positions()

        self.assertEqual({"Y1": 20.0, "Y2": 30.0}, positions)
        self.assertIn(("Y1_MOVE", True), plc.writes)
        self.assertIn(("Y2_MOVE", True), plc.writes)

    def test_pallet_transfer_service_uses_configured_y1_y2_points(self) -> None:
        plc = FakePointService()
        plc.values.update(
            {
                "Y1_VAC_OFF": False,
                "Y2_VAC_OFF": False,
                "Y1_FWD_POS": 0.0,
                "Y2_FWD_POS": 0.0,
            }
        )
        service = PalletTransferService(plc)

        service.precheck()
        service.set_forward_position("Y1", 123)
        service.start_forward("Y1")
        service.set_action_output("Y1", PalletAction.SUCK)
        service.stop_forward("Y1")

        self.assertIn(("Y1_FWD_POS", 123), plc.writes)
        self.assertIn(("Y1_MOVE", True), plc.writes)
        self.assertIn(("Y1_VAC_ON", True), plc.writes)
        self.assertIn(("Y1_MOVE", False), plc.writes)

    def test_lift_service_error_identifies_operation_and_point(self) -> None:
        plc = FakePointService()
        plc.fail_write_point = "Y2_VAC_ON"
        service = LiftService(plc)

        with self.assertRaises(LiftServiceError) as caught:
            service.set_vacuum(True)

        self.assertIn("LiftService.set_vacuum", str(caught.exception))
        self.assertIn("point=Y2_VAC_ON", str(caught.exception))

    def test_lift_service_uses_only_m376_m379_for_preset_motion(self) -> None:
        plc = FakePointService()
        service = LiftService(plc)

        service.configure_position(560.0)
        service.start_up()
        service.start_down()

        self.assertIn(("X_FWD_POS", 560.0), plc.writes)
        self.assertNotIn(("X_SPEED", 100.0), plc.writes)
        self.assertIn(("X_Move", True), plc.writes)
        self.assertIn(("X_MOVE_DOWN", True), plc.writes)
        self.assertNotIn(("X_UP", True), plc.writes)
        self.assertNotIn(("X_DOWN", True), plc.writes)

    def test_lift_service_can_write_configured_positioning_speed(self) -> None:
        plc = FakePointService()
        config = LiftServiceConfig.load()
        config = LiftServiceConfig(
            height_point=config.height_point,
            target_point=config.target_point,
            speed_point=config.speed_point,
            positioning_speed=200.0,
            up_point=config.up_point,
            down_point=config.down_point,
            manual_motion_points=config.manual_motion_points,
            manual_up_point=config.manual_up_point,
            vacuum_points=config.vacuum_points,
            minimum_height_mm=config.minimum_height_mm,
            maximum_height_mm=config.maximum_height_mm,
            arm_camera_not_home_maximum_height_mm=(
                config.arm_camera_not_home_maximum_height_mm
            ),
        )
        service = LiftService(plc, config=config)

        service.configure_position(560.0)

        self.assertIn(("X_FWD_POS", 560.0), plc.writes)
        self.assertIn(("X_SPEED", 200.0), plc.writes)

    def test_lift_blocks_above_695_until_arm_and_camera_home_is_confirmed(self) -> None:
        plc = FakePointService()
        interlock = ArmCameraHomeInterlock()
        service = LiftService(plc, home_interlock=interlock)

        service.configure_position(695.0)
        with self.assertRaisesRegex(LiftServiceError, "695mm"):
            service.configure_position(695.01)

        interlock.mark_home_confirmed("test readback confirmed")
        service.configure_position(739.015)

        self.assertIn(("X_FWD_POS", 695.0), plc.writes)
        self.assertIn(("X_FWD_POS", 739.015), plc.writes)

    def test_lift_allows_1450_only_while_arm_and_camera_home_are_confirmed(self) -> None:
        plc = FakePointService()
        interlock = ArmCameraHomeInterlock()
        service = LiftService(plc, home_interlock=interlock)

        with self.assertRaisesRegex(LiftServiceError, "695mm"):
            service.configure_position(1450.0)

        interlock.mark_home_confirmed("arm and camera readback confirmed")
        service.configure_position(1450.0)
        with self.assertRaisesRegex(LiftServiceError, "1450"):
            service.configure_position(1450.01)

        interlock.mark_not_home("second step started")
        with self.assertRaisesRegex(LiftServiceError, "695mm"):
            service.configure_position(1450.0)

        self.assertIn(("X_FWD_POS", 1450.0), plc.writes)

    def test_lift_code_ceiling_cannot_be_loosened_by_injected_config(self) -> None:
        plc = FakePointService()
        config = replace(
            LiftServiceConfig.load(),
            arm_camera_not_home_maximum_height_mm=1450.0,
        )
        service = LiftService(
            plc,
            config=config,
            home_interlock=ArmCameraHomeInterlock(),
        )

        service.configure_position(695.0)
        with self.assertRaisesRegex(LiftServiceError, "695mm"):
            service.configure_position(695.01)

    def test_lift_service_clears_manual_motion_before_positioning(self) -> None:
        plc = FakePointService()
        plc.values["X_UP"] = True
        service = LiftService(plc)

        service.start_up()

        self.assertIn(("X_UP", False), plc.writes)
        self.assertIn(("X_DOWN", False), plc.writes)
        self.assertIn(("X_Move", True), plc.writes)

    def test_vision_positioning_clears_m379_then_starts_m376(self) -> None:
        plc = FakePointService()
        service = LiftService(plc)

        service.start_vision_positioning()

        self.assertEqual(LifecycleStatus.WAITING_SIGNAL, service.status)

        self.assertEqual(
            [
                ("X_UP", False),
                ("X_DOWN", False),
                ("X_Move", False),
                ("X_MOVE_DOWN", False),
                ("X_Move", True),
            ],
            plc.writes,
        )


class ArmVisionWorkflowServiceTests(unittest.TestCase):
    @staticmethod
    def service_for_script(
        directory: Path,
        source: str,
        *,
        home_interlock: ArmCameraHomeInterlock | None = None,
        target_height_validator=None,
        **config_overrides,
    ) -> ArmVisionWorkflowService:
        script_path = directory / "dummy_arm_workflow.py"
        script_path.write_text(source, encoding="utf-8")
        values = {
            "python_path": sys.executable,
            "script_path": str(script_path),
            "startup_timeout_seconds": 1.0,
            "no_output_timeout_seconds": 1.0,
            "overall_timeout_seconds": 2.0,
            "poll_interval_seconds": 0.005,
        }
        values.update(config_overrides)
        return ArmVisionWorkflowService(
            ArmVisionWorkflowConfig(**values),
            base_dir=directory,
            home_interlock=home_interlock or ArmCameraHomeInterlock(),
            target_height_validator=target_height_validator,
        )

    @staticmethod
    def run_service(
        service: ArmVisionWorkflowService,
        cancel_event: threading.Event | None = None,
        progress_callback=None,
        movement_handoff=None,
        view=None,
    ):
        return service.run_pick_and_place(
            cancel_event or threading.Event(),
            pick_handoff=lambda _height, _target: None,
            place_handoff=lambda _height, _target: None,
            movement_handoff=movement_handoff,
            view=view,
            progress_callback=progress_callback,
        )

    def test_startup_timeout_reports_that_the_child_produced_no_output(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            service = self.service_for_script(
                Path(temp_dir),
                "import time\ntime.sleep(30)\n",
                startup_timeout_seconds=0.05,
            )

            with self.assertRaises(TimeoutError) as raised:
                self.run_service(service)

        self.assertIn("startup_timeout", str(raised.exception))
        self.assertIn("最後輸出：（尚無輸出）", str(raised.exception))
        self.assertEqual(LifecycleStatus.TIMEOUT, service.status)

    def test_standalone_home_confirmation_unlocks_interlock_with_four_angles(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            source = (
                "import json\n"
                "angles = {'ID 142': 6.0, 'ID 143': -22.93, "
                "'ID 144': 6.7, 'ID 145': 36.49}\n"
                "print('[POSE_STATE] ALL_HOME_CONFIRMED', flush=True)\n"
                "print('[POSE_RESULT] ' + json.dumps({'angles': angles}), flush=True)\n"
            )
            interlock = ArmCameraHomeInterlock()
            service = self.service_for_script(
                directory,
                source,
                home_interlock=interlock,
                home_timeout_seconds=0.1,
            )

            result = service.confirm_home()

        self.assertTrue(result["confirmed"])
        self.assertEqual(4, len(result["angles"]))
        self.assertTrue(interlock.all_home_confirmed)
        self.assertEqual(LifecycleStatus.SUCCESS, service.status)

    def test_standalone_home_confirmation_can_be_cancelled(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            service = self.service_for_script(
                Path(temp_dir),
                "import time\ntime.sleep(30)\n",
                startup_timeout_seconds=0.1,
                home_timeout_seconds=0.1,
            )
            cancel_event = threading.Event()
            timer = threading.Timer(0.05, cancel_event.set)
            started = time.monotonic()
            timer.start()
            try:
                with self.assertRaisesRegex(RuntimeError, "HOME 確認已取消"):
                    service.confirm_home(cancel_event)
            finally:
                timer.cancel()

        self.assertLess(time.monotonic() - started, 2.0)
        self.assertEqual(LifecycleStatus.CANCELLED, service.status)

    def test_component_home_confirmation_keeps_four_axis_interlock_locked(self) -> None:
        cases = (
            (
                "arm",
                ARM_HOME_CONFIRMED_MARKER,
                {"ID 142": 6.0, "ID 143": -22.93},
            ),
            (
                "camera",
                CAMERA_HOME_CONFIRMED_MARKER,
                {"ID 144": 6.7, "ID 145": 36.49},
            ),
        )
        for component, marker, angles in cases:
            with self.subTest(component=component), tempfile.TemporaryDirectory() as temp_dir:
                directory = Path(temp_dir)
                source = (
                    "import json\n"
                    f"print({marker!r}, flush=True)\n"
                    f"angles = {angles!r}\n"
                    "print('[POSE_RESULT] ' + json.dumps({'angles': angles}), flush=True)\n"
                )
                interlock = ArmCameraHomeInterlock()
                service = self.service_for_script(
                    directory,
                    source,
                    home_interlock=interlock,
                    home_timeout_seconds=0.1,
                    settle_seconds=0.0,
                )

                result = service.move_home_component(component)
                command = service._home_component_command(component)

                self.assertEqual(component, result["component"])
                self.assertEqual(angles, result["angles"])
                self.assertFalse(result["all_home_confirmed"])
                self.assertFalse(interlock.all_home_confirmed)
                self.assertEqual("unknown", interlock.snapshot.state.value)
                self.assertIn("--move-home-only", command)
                self.assertIn(component, command)
                self.assertNotIn("--confirm-home-only", command)
                self.assertEqual(LifecycleStatus.SUCCESS, service.status)

    def test_standalone_home_confirmation_keeps_interlock_locked_without_marker(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            interlock = ArmCameraHomeInterlock()
            service = self.service_for_script(
                directory,
                "print('HOME outside tolerance', flush=True)\n",
                home_interlock=interlock,
                home_timeout_seconds=0.1,
            )

            with self.assertRaisesRegex(
                RuntimeError,
                "沒有收到四軸 HOME 確認標記",
            ):
                service.confirm_home()

        self.assertFalse(interlock.all_home_confirmed)
        self.assertEqual("unknown", interlock.snapshot.state.value)

    def test_no_output_timeout_keeps_and_reports_the_last_child_output(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            service = self.service_for_script(
                Path(temp_dir),
                "import time\nprint('[INFO] VIEW = LView', flush=True)\ntime.sleep(30)\n",
                no_output_timeout_seconds=0.05,
            )

            with self.assertRaises(TimeoutError) as raised:
                self.run_service(service)

        self.assertIn("no_output_timeout", str(raised.exception))
        self.assertIn("最後輸出：[INFO] VIEW = LView", str(raised.exception))
        self.assertEqual("[INFO] VIEW = LView", service.status_snapshot.data["last_output"])

    def test_cancel_terminates_the_child_and_returns_without_waiting_for_its_sleep(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            service = self.service_for_script(
                Path(temp_dir),
                "import time\nprint('READY', flush=True)\ntime.sleep(30)\n",
                no_output_timeout_seconds=5.0,
                overall_timeout_seconds=10.0,
            )
            cancel_event = threading.Event()
            ready = threading.Event()
            errors: list[Exception] = []

            def worker() -> None:
                try:
                    self.run_service(
                        service,
                        cancel_event,
                        lambda message: ready.set() if message == "READY" else None,
                    )
                except Exception as exc:
                    errors.append(exc)

            thread = threading.Thread(target=worker)
            thread.start()
            self.assertTrue(ready.wait(1.0))
            cancel_event.set()
            thread.join(2.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(1, len(errors))
        self.assertIn("流程已取消", str(errors[0]))
        self.assertIn("沒有收到停止確認", str(errors[0]))
        self.assertEqual(LifecycleStatus.CANCELLED, service.status)

    def test_cancel_waits_for_child_motor_stop_confirmation(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            source = (
                "import signal, sys, time\n"
                "def stop(_signum, _frame):\n"
                "    print('[POSE_STATE] ARM_STOP_CONFIRMED', flush=True)\n"
                "    sys.exit(0)\n"
                "signal.signal(signal.SIGTERM, stop)\n"
                "print('READY', flush=True)\n"
                "while True: time.sleep(0.01)\n"
            )
            service = self.service_for_script(
                Path(temp_dir),
                source,
                no_output_timeout_seconds=5.0,
                overall_timeout_seconds=10.0,
                cancel_stop_timeout_seconds=1.0,
            )
            cancel_event = threading.Event()
            ready = threading.Event()
            errors: list[Exception] = []

            def worker() -> None:
                try:
                    self.run_service(
                        service,
                        cancel_event,
                        lambda message: ready.set() if message == "READY" else None,
                    )
                except Exception as exc:
                    errors.append(exc)

            thread = threading.Thread(target=worker)
            thread.start()
            self.assertTrue(ready.wait(1.0))
            cancel_event.set()
            thread.join(2.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(1, len(errors))
        self.assertIn("四顆 CAN 馬達已確認停止", str(errors[0]))
        self.assertEqual(LifecycleStatus.CANCELLED, service.status)

    def test_cancel_during_plc_handoff_still_waits_for_motor_stop(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            source = (
                "import json, signal, sys, time\n"
                "from pathlib import Path\n"
                "def stop(_signum, _frame):\n"
                "    print('[POSE_STATE] ARM_STOP_CONFIRMED', flush=True)\n"
                "    sys.exit(0)\n"
                "signal.signal(signal.SIGTERM, stop)\n"
                "Path('control_target.json').write_text("
                "json.dumps({'depth_m': 0.6}), encoding='utf-8')\n"
                "print('[INFO] Saved control_target.json', flush=True)\n"
                "print('[HANDOFF] Arm is at left target suction position.', flush=True)\n"
                "while True: time.sleep(0.01)\n"
            )
            service = self.service_for_script(
                Path(temp_dir),
                source,
                no_output_timeout_seconds=5.0,
                overall_timeout_seconds=10.0,
                cancel_stop_timeout_seconds=1.0,
            )
            cancel_event = threading.Event()

            def cancel_during_pick(_height, _target) -> None:
                cancel_event.set()
                raise RuntimeError("PLC handoff cancelled")

            with self.assertRaisesRegex(
                RuntimeError,
                "四顆 CAN 馬達已確認停止",
            ):
                service.run_pick_and_place(
                    cancel_event,
                    pick_handoff=cancel_during_pick,
                    place_handoff=lambda _height, _target: None,
                )

        self.assertEqual(LifecycleStatus.CANCELLED, service.status)

    def test_already_cancelled_request_does_not_start_the_child(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            marker = Path(temp_dir) / "started"
            service = self.service_for_script(
                Path(temp_dir),
                f"from pathlib import Path\nPath({str(marker)!r}).touch()\n",
            )
            cancel_event = threading.Event()
            cancel_event.set()

            with self.assertRaisesRegex(RuntimeError, "流程已取消"):
                self.run_service(service, cancel_event)

            self.assertFalse(marker.exists())
            self.assertEqual(LifecycleStatus.CANCELLED, service.status)

    def test_initial_height_gate_failure_does_not_start_the_child(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            marker = Path(temp_dir) / "started"
            service = self.service_for_script(
                Path(temp_dir),
                f"from pathlib import Path\nPath({str(marker)!r}).touch()\n",
            )

            def reject_height(_movement: str) -> None:
                raise RuntimeError("禁止手臂移動：目前高度不是 560mm")

            with self.assertRaisesRegex(RuntimeError, "不是 560mm"):
                self.run_service(service, movement_handoff=reject_height)

            self.assertFalse(marker.exists())
            self.assertEqual(LifecycleStatus.ERROR, service.status)

    def test_motion_step_waits_for_height_gate_before_pressing_enter(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            moved_marker = directory / "moved"
            source = (
                "from pathlib import Path\n"
                "print('[STEP] Move from LView back to HOME', flush=True)\n"
                "input()\n"
                f"Path({str(moved_marker)!r}).touch()\n"
            )
            service = self.service_for_script(directory, source)
            movements: list[str] = []

            def gate(movement: str) -> None:
                movements.append(movement)
                if "Move from LView back to HOME" in movement:
                    raise RuntimeError("禁止手臂移動：560mm 尚未確認")

            with self.assertRaisesRegex(RuntimeError, "560mm 尚未確認"):
                self.run_service(service, movement_handoff=gate)

            self.assertEqual("啟動 Camera／手臂子程序", movements[0])
            self.assertIn("Move from LView back to HOME", movements[1])
            self.assertFalse(moved_marker.exists())

    def test_pick_done_cannot_release_arm_until_560_gate_passes(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            moved_marker = directory / "transfer_started"
            source = (
                "import json\n"
                "from pathlib import Path\n"
                "Path('control_target.json').write_text("
                "json.dumps({'depth_m': 0.6}), encoding='utf-8')\n"
                "print('[HANDOFF] Arm is at left target suction position.', flush=True)\n"
                "print('[STEP] FAKE PLC PICK DONE: object attached and safe to transfer right', flush=True)\n"
                "input()\n"
                f"Path({str(moved_marker)!r}).touch()\n"
            )
            service = self.service_for_script(directory, source)
            movements: list[str] = []

            def gate(movement: str) -> None:
                movements.append(movement)
                if "FAKE PLC PICK DONE" in movement:
                    raise RuntimeError("禁止換邊：升降機尚未穩定在 560mm")

            with self.assertRaisesRegex(RuntimeError, "尚未穩定在 560mm"):
                self.run_service(service, movement_handoff=gate)

            self.assertTrue(any("FAKE PLC PICK DONE" in item for item in movements))
            self.assertFalse(moved_marker.exists())

    def test_success_requires_terminal_home_readback_marker(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            source = (
                "import json\n"
                "from pathlib import Path\n"
                "Path('control_target.json').write_text("
                "json.dumps({'depth_m': 0.6}), encoding='utf-8')\n"
                "print('[INFO] Saved control_target.json', flush=True)\n"
                "print('[HANDOFF] Arm is at left target suction position.', flush=True)\n"
                "print('[HANDOFF] Arm is at right outer-branch place position.', flush=True)\n"
                "print('[POSE_STATE] ALL_HOME_CONFIRMED', flush=True)\n"
            )
            interlock = ArmCameraHomeInterlock()
            service = self.service_for_script(
                directory,
                source,
                home_interlock=interlock,
            )

            result = self.run_service(service)

        self.assertEqual(600.0, result["plc_height_mm"])
        self.assertTrue(interlock.all_home_confirmed)

    def test_completed_handoffs_without_home_marker_remain_locked_to_695(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            source = (
                "import json\n"
                "from pathlib import Path\n"
                "Path('control_target.json').write_text("
                "json.dumps({'depth_m': 0.6}), encoding='utf-8')\n"
                "print('[INFO] Saved control_target.json', flush=True)\n"
                "print('[HANDOFF] Arm is at left target suction position.', flush=True)\n"
                "print('[HANDOFF] Arm is at right outer-branch place position.', flush=True)\n"
            )
            interlock = ArmCameraHomeInterlock()
            service = self.service_for_script(
                directory,
                source,
                home_interlock=interlock,
            )

            with self.assertRaisesRegex(
                RuntimeError,
                "沒有收到手臂與相機 HOME 角度確認",
            ):
                self.run_service(service)

        self.assertFalse(interlock.all_home_confirmed)
        self.assertEqual("unknown", interlock.snapshot.state.value)

    def test_height_conversion_clamps_to_695_before_pick_and_place(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            source = (
                "import json\n"
                "from pathlib import Path\n"
                "Path('control_target.json').write_text("
                "json.dumps({'depth_m': 0.779015}), encoding='utf-8')\n"
                "print('[INFO] Saved control_target.json', flush=True)\n"
                "print('[HANDOFF] Arm is at left target suction position.', flush=True)\n"
                "print('[HANDOFF] Arm is at right outer-branch place position.', flush=True)\n"
                "print('[POSE_STATE] ALL_HOME_CONFIRMED', flush=True)\n"
            )
            interlock = ArmCameraHomeInterlock()
            lift = LiftService(
                FakePointService(),
                home_interlock=interlock,
            )
            service = self.service_for_script(
                directory,
                source,
                home_interlock=interlock,
                target_height_validator=lift.validate_target_height,
                no_output_timeout_seconds=5.0,
                overall_timeout_seconds=10.0,
            )

            result = self.run_service(service)

        self.assertEqual(695.0, result["plc_height_mm"])
        self.assertTrue(interlock.all_home_confirmed)
        self.assertEqual(LifecycleStatus.SUCCESS, service.status)

    def test_rview_uses_right_pick_and_left_place_handoffs(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            source = (
                "import json\n"
                "from pathlib import Path\n"
                "Path('control_target.json').write_text("
                "json.dumps({'depth_m': 0.779015}), encoding='utf-8')\n"
                "print('[INFO] Saved control_target.json', flush=True)\n"
                "print('[HANDOFF] Arm is at right target suction position.', flush=True)\n"
                "print('[HANDOFF] Arm is at left outer-branch place position.', flush=True)\n"
                "print('[POSE_STATE] ALL_HOME_CONFIRMED', flush=True)\n"
            )
            service = self.service_for_script(directory, source)

            result = self.run_service(service, view="RView")

        self.assertEqual(695.0, result["plc_height_mm"])
        self.assertEqual(LifecycleStatus.SUCCESS, service.status)

    def test_timeout_settings_do_not_change_the_arm_script_command_sequence(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            service = self.service_for_script(Path(temp_dir), "")

            command = service._command()
            lview_command = service._command("LView")

        self.assertEqual(
            [
                "--view",
                "LView",
                "--execute",
                "--yes",
                "--target-speed",
                "200",
                "--settle",
                "2.5",
                "--target-wait",
                "2.0",
                "--pick-wait",
                "2.0",
                "--home-tolerance",
                "1.0",
                "--home-stable-reads",
                "3",
                "--home-timeout",
                "15.0",
                "--home-poll",
                "0.2",
            ],
            command[3:],
        )
        self.assertEqual("LView", lview_command[lview_command.index("--view") + 1])

    def test_config_uses_dual_direction_arm_script(self) -> None:
        service = ArmVisionWorkflowService()

        command = service._command()

        self.assertEqual(
            (PLC2_DIR.parent / "d435_control.py").resolve(),
            Path(command[2]).resolve(),
        )
        self.assertEqual("RView", command[command.index("--view") + 1])


class FlowTests(unittest.TestCase):
    @staticmethod
    def home_config(**overrides) -> HomeFlowConfig:
        values = {
            "pulse_seconds": 0.001,
            "poll_interval_seconds": 0.001,
            "timeout_seconds": 0.2,
            "position_tolerance_mm": 0.0,
            "stable_read_count": 2,
            "read_error_limit": 2,
        }
        values.update(overrides)
        return HomeFlowConfig(**values)

    @staticmethod
    def vision_config(**overrides) -> VisionHeightConfig:
        values = {
            "target_height_mm": 560.0,
            "tolerance_mm": 1.0,
            "stable_read_count": 2,
            "poll_interval_seconds": 0.001,
            "vacuum_build_seconds": 0.0,
            "settle_seconds": 0.0,
            "timeout_seconds": 0.2,
            "unchanged_timeout_seconds": 0.02,
            "retrigger_limit": 3,
        }
        values.update(overrides)
        return VisionHeightConfig(**values)

    @staticmethod
    def main_cycle_config(**overrides) -> MainCycleConfig:
        values = {
            "vision_height_mm": 560.0,
            "height_tolerance_mm": 1.0,
            "y_position_tolerance_mm": 1.0,
            "stable_read_count": 2,
            "poll_interval_seconds": 0.001,
            "action_delay_seconds": 0.0,
            "motion_timeout_seconds": 0.2,
            "y_motion_timeout_seconds": 0.2,
            "vision_signal_timeout_seconds": 0.2,
        }
        values.update(overrides)
        return MainCycleConfig(**values)

    def test_home_flow_pulses_service_and_waits_for_stable_zero(self) -> None:
        service = FakeHomeDomainService(
            [
                {"X": 5, "Y1": 5, "Y2": 5},
                {"X": 1, "Y1": 0, "Y2": 0},
                {"X": 0, "Y1": 0, "Y2": 0},
                {"X": 0, "Y1": 0, "Y2": 0},
            ]
        )

        statuses: list[LifecycleStatus] = []
        result = HomeFlow(service, self.home_config()).run(
            threading.Event(),
            lambda progress: statuses.append(progress.status),
        )

        self.assertEqual(HomeFlowState.SUCCESS, result.state)
        self.assertEqual(LifecycleStatus.SUCCESS, result.status)
        self.assertIn(LifecycleStatus.RUNNING, statuses)
        self.assertIn(LifecycleStatus.WAITING_SIGNAL, statuses)
        self.assertIn(True, service.commands)
        self.assertFalse(service.commands[-1])

    def test_vision_height_flow_calls_lift_service_in_order(self) -> None:
        service = FakeLiftDomainService([400, 500, 550, 560, 560, 560])

        result = VisionHeightFlow(service, self.vision_config()).run(threading.Event())

        self.assertEqual(VisionHeightState.SUCCESS, result.state)
        self.assertEqual("precheck", service.calls[0])
        self.assertIn("position=560", service.calls)
        self.assertIn("vacuum=True", service.calls)
        self.assertEqual([(560.0, None)], service.position_config)
        self.assertIn("start_up", service.calls)
        self.assertNotIn("vacuum=False", service.calls)

    def test_vision_height_above_target_still_uses_m376_positioning(self) -> None:
        service = FakeLiftDomainService([700, 650, 600, 560, 560, 560])

        result = VisionHeightFlow(service, self.vision_config()).run(threading.Event())

        self.assertEqual(VisionHeightState.SUCCESS, result.state)
        self.assertIn("start_up", service.calls)
        self.assertNotIn("start_down", service.calls)
        self.assertEqual({"up": False, "down": False}, service.motion)

    def test_vision_height_reports_stopped_short_of_target(self) -> None:
        class StoppedLiftService(FakeLiftDomainService):
            def read_motion_commands(self) -> dict[str, bool]:
                return {"up": False, "down": False}

        service = StoppedLiftService([0, 93, 100, 100, 100, 100, 100])

        result = VisionHeightFlow(service, self.vision_config()).run(threading.Event())

        self.assertEqual(VisionHeightState.ERROR, result.state)
        self.assertIn("停在 100mm", result.message)
        self.assertEqual(4, service.calls.count("start_up"))

    def test_vision_height_retriggers_until_d52_reaches_target(self) -> None:
        class OnceStoppedLiftService(FakeLiftDomainService):
            def __init__(self) -> None:
                super().__init__([0, 541, 541, 541, 541, 541, 560, 560])
                self.motion_reads = 0

            def read_motion_commands(self) -> dict[str, bool]:
                self.motion_reads += 1
                if self.motion_reads <= 2:
                    return {"up": False, "down": False}
                return dict(self.motion)

        service = OnceStoppedLiftService()

        result = VisionHeightFlow(service, self.vision_config(unchanged_timeout_seconds=0.0001)).run(threading.Event())

        self.assertEqual(VisionHeightState.SUCCESS, result.state)
        self.assertGreaterEqual(service.calls.count("start_up"), 2)

    def test_vision_height_cancel_stops_motion_and_keeps_vacuum(self) -> None:
        service = FakeLiftDomainService([400, 450, 500, 530])
        cancel_event = threading.Event()

        def cancel_on_move(progress) -> None:
            if progress.state is VisionHeightState.MOVING:
                cancel_event.set()

        result = VisionHeightFlow(service, self.vision_config()).run(cancel_event, cancel_on_move)

        self.assertEqual(VisionHeightState.CANCELLED, result.state)
        self.assertEqual(LifecycleStatus.CANCELLED, result.status)
        self.assertEqual({"up": False, "down": False}, service.motion)
        self.assertEqual({"left": True, "right": True}, service.vacuum)

    def test_flow_error_keeps_service_operation_and_point_context(self) -> None:
        class FailingLiftService(FakeLiftDomainService):
            def set_vacuum(self, enabled: bool) -> None:  # noqa: ARG002
                raise LiftServiceError(
                    "set_vacuum",
                    "simulated write failure",
                    point_id="Y2_VAC_ON",
                )

        service = FailingLiftService([400])

        result = VisionHeightFlow(service, self.vision_config()).run(threading.Event())

        self.assertEqual(VisionHeightState.ERROR, result.state)
        self.assertEqual(LifecycleStatus.ERROR, result.status)
        self.assertIn("VisionHeightFlow.vacuum", result.message)
        self.assertIn("LiftService.set_vacuum", result.message)
        self.assertIn("point=Y2_VAC_ON", result.message)

    def test_main_cycle_runs_first_step_second_step_and_final_step(self) -> None:
        heights = (
            [560, 560, 560]
            + [560, 560, 560, 620, 620, 620, 460, 460, 460, 620, 620, 620, 560, 560, 560]
            + [560, 560, 560]
        )
        lift = FakeLiftDomainService(heights)
        pallet = FakePalletTransferDomainService()
        pallet.position_sequences["Y1"] = [0.0, 111.0, 111.0, 30.0, 0.0, 0.0]
        pallet.position_sequences["Y2"] = [0.0, 222.0, 222.0, 40.0, 0.0, 0.0]
        middle = FakeMiddleVacuumDomainService()
        vision = FakeVisionBridgeDomainService(620.0)
        flow = MainCycleFlow(
            lift,
            pallet,
            middle,
            vision,
            self.main_cycle_config(),
        )
        cancel_event = threading.Event()

        first = flow.run_first_step(
            Step1Command("Y1", "suck", 560.0, 111.0),
            cancel_event,
        )
        second = flow.run_second_step(cancel_event)
        final = flow.run_final_step(
            Step1Command("Y2", "push", 560.0, 222.0),
            cancel_event,
        )

        self.assertEqual(MainCycleState.SUCCESS, first.state)
        self.assertEqual(MainCycleState.SUCCESS, second.state)
        self.assertEqual(MainCycleState.SUCCESS, final.state)
        self.assertEqual(MainCyclePhase.WAITING_STEP2, first.phase)
        self.assertEqual(MainCyclePhase.WAITING_FINAL_STEP1, second.phase)
        self.assertEqual(MainCyclePhase.COMPLETE, final.phase)
        self.assertEqual(MainCyclePhase.COMPLETE, flow.phase)
        self.assertIn("Y1:forward=111", pallet.calls)
        self.assertIn("Y2:action=suck", pallet.calls)
        self.assertIn("Y1:forward=0", pallet.calls)
        self.assertIn("Y2:forward=222", pallet.calls)
        self.assertIn("Y2:action=push", pallet.calls)
        self.assertIn("Y2:forward=0", pallet.calls)
        self.assertLess(
            pallet.calls.index("Y1:stop_forward"),
            pallet.calls.index("Y1:forward=0"),
        )
        self.assertLess(
            pallet.calls.index("Y2:stop_forward"),
            pallet.calls.index("Y2:forward=0"),
        )
        self.assertLess(
            pallet.calls.index("Y2:action=suck"),
            pallet.calls.index("Y1:forward=0"),
        )
        self.assertLess(
            pallet.calls.index("Y2:action=push"),
            pallet.calls.index("Y2:forward=0"),
        )
        self.assertGreaterEqual(pallet.calls.count("Y1:start_forward"), 2)
        self.assertGreaterEqual(pallet.calls.count("Y2:start_forward"), 2)
        self.assertEqual(["vacuum", "break_vacuum", "off", "off"], middle.modes)
        self.assertIn("Y1:action=none", pallet.calls)
        self.assertIn("Y2:action=none", pallet.calls)
        self.assertEqual(
            [560.0, 560.0, 620.0, 460.0, 620.0, 560.0, 560.0],
            [target for target, _speed in lift.position_config],
        )
        self.assertEqual(7, lift.calls.count("start_vision_positioning"))
        self.assertGreaterEqual(lift.calls.count("stop"), 7)

        event_names = [name for name, _payload in vision.events]
        self.assertEqual("start", event_names[0])
        self.assertIn("vision_height_ready", event_names)
        self.assertIn("m54_on_after_height", event_names)
        self.assertIn("returned_cross_side_height_after_m54", event_names)
        self.assertIn("m56_on_m54_off_after_same_height", event_names)
        self.assertIn("returned_vision_height_after_m56", event_names)
        m54_payload = dict(vision.events[event_names.index("m54_on_after_height")][1])
        m56_payload = dict(vision.events[event_names.index("m56_on_m54_off_after_same_height")][1])
        self.assertEqual(620.0, m54_payload["height_mm"])
        self.assertEqual(620.0, m56_payload["height_mm"])

    def test_independent_first_step_can_repeat_without_advancing_guided_phase(self) -> None:
        lift = FakeLiftDomainService([560.0] * 6)
        pallet = FakePalletTransferDomainService()
        pallet.position_sequences["Y1"] = [
            111.0,
            111.0,
            0.0,
            0.0,
            111.0,
            111.0,
            0.0,
            0.0,
        ]
        flow = MainCycleFlow(
            lift,
            pallet,
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(560.0),
            self.main_cycle_config(),
        )
        command = Step1Command("Y1", "suck", 560.0, 111.0)

        first = flow.run_independent_first_step(command, threading.Event())
        repeated = flow.run_independent_first_step(command, threading.Event())

        self.assertTrue(first.succeeded)
        self.assertTrue(repeated.succeeded)
        self.assertEqual(MainCyclePhase.READY_FIRST_STEP, flow.phase)
        self.assertEqual(2, pallet.calls.count("Y1:forward=111"))

    def test_independent_second_step_ignores_guided_phase(self) -> None:
        lift = FakeLiftDomainService(
            [560.0, 560.0, 560.0]
            + [620.0, 620.0, 620.0]
            + [460.0, 460.0, 460.0]
            + [620.0, 620.0, 620.0]
            + [560.0, 560.0, 560.0]
        )
        flow = MainCycleFlow(
            lift,
            FakePalletTransferDomainService(),
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(620.0),
            self.main_cycle_config(),
        )
        flow.phase = MainCyclePhase.COMPLETE

        result = flow.run_independent_second_step(threading.Event())

        self.assertTrue(result.succeeded)
        self.assertEqual(MainCyclePhase.COMPLETE, flow.phase)

    def test_independent_second_step_prepares_560_before_home_precheck(self) -> None:
        lift = FakeLiftDomainService(
            [700.0, 640.0, 580.0, 560.0, 560.0, 560.0, 560.0]
        )
        flow = MainCycleFlow(
            lift,
            FakePalletTransferDomainService(),
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(560.0),
            self.main_cycle_config(),
        )

        height = flow.prepare_independent_second_step_height(threading.Event())

        self.assertEqual(560.0, height)
        self.assertEqual([(560.0, None)], lift.position_config)
        self.assertIn("start_vision_positioning", lift.calls)
        self.assertFalse(any(lift.read_motion_commands().values()))
        self.assertEqual(
            "independent_second_step_to_safe_height",
            flow.status_snapshot.step,
        )

    def test_independent_third_step_shuts_down_vacuum_without_advancing_phase(self) -> None:
        slot_vacuum = FakeSlotVacuumDomainService()
        middle = FakeMiddleVacuumDomainService()
        flow = MainCycleFlow(
            FakeLiftDomainService([560.0]),
            FakePalletTransferDomainService(),
            middle,
            FakeVisionBridgeDomainService(560.0),
            self.main_cycle_config(),
            slot_vacuum_service=slot_vacuum,
        )

        result = flow.run_independent_third_step(
            Step1Command("none", "none", 560.0),
            threading.Event(),
        )

        self.assertTrue(result.succeeded)
        self.assertEqual(MainCyclePhase.READY_FIRST_STEP, flow.phase)
        self.assertEqual(["all:shutdown"], slot_vacuum.calls)
        self.assertEqual(["off"], middle.modes)

    def test_main_cycle_reports_stop_unconfirmed_when_lift_command_stays_on(self) -> None:
        class LiftWithFailedStopReadback(FakeLiftDomainService):
            def stop(self) -> None:
                self.calls.append("stop_failed_readback")
                self.motion = {"up": True, "down": False}

        lift = LiftWithFailedStopReadback([560.0])
        flow = MainCycleFlow(
            lift,
            FakePalletTransferDomainService(),
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(560.0),
            self.main_cycle_config(),
        )

        result = flow.run_independent_first_step(
            Step1Command("none", "none", 560.0),
            threading.Event(),
        )

        self.assertEqual(MainCycleState.STOP_UNCONFIRMED, result.state)
        self.assertEqual(LifecycleStatus.ERROR, result.status)
        self.assertEqual("stop_unconfirmed", result.step)
        self.assertIn("升降命令仍為 ON", result.message)
        self.assertEqual("stop_unconfirmed", flow.status_snapshot.data["state"])

    def test_main_cycle_reports_stop_unconfirmed_when_pallet_stop_write_fails(self) -> None:
        class PalletWithFailedStop(FakePalletTransferDomainService):
            def stop_forward(self, slot: str | None = None) -> None:
                raise RuntimeError("simulated M375/M374 write failure")

        flow = MainCycleFlow(
            FakeLiftDomainService([560.0]),
            PalletWithFailedStop(),
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(560.0),
            self.main_cycle_config(),
        )

        result = flow.run_independent_first_step(
            Step1Command("none", "none", 560.0),
            threading.Event(),
        )

        self.assertEqual(MainCycleState.STOP_UNCONFIRMED, result.state)
        self.assertIn("貨盤停止命令失敗", result.message)

    def test_main_cycle_suck_holds_opposite_side_until_selected_side_push(self) -> None:
        lift = FakeLiftDomainService([560.0] * 6)
        pallet = FakePalletTransferDomainService()
        pallet.position_sequences["Y1"] = [111.0, 111.0, 0.0, 0.0]
        pallet.position_sequences["Y2"] = [222.0, 222.0, 0.0, 0.0]
        slot_vacuum = FakeSlotVacuumDomainService()
        flow = MainCycleFlow(
            lift,
            pallet,
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(560.0),
            self.main_cycle_config(),
            slot_vacuum_service=slot_vacuum,
        )
        cancel_event = threading.Event()

        first = flow.run_first_step(
            Step1Command("Y1", "suck", 560.0, 111.0),
            cancel_event,
        )
        # The second-step mechanics are covered separately. Jump to its
        # successful phase so this test can focus on the explicit release.
        flow.phase = MainCyclePhase.WAITING_FINAL_STEP1
        final = flow.run_final_step(
            Step1Command("Y2", "push", 560.0, 222.0),
            cancel_event,
        )

        self.assertTrue(first.succeeded)
        self.assertTrue(final.succeeded)
        self.assertEqual(
            ["Y2:hold", "Y2:release", "all:shutdown"],
            slot_vacuum.calls,
        )
        self.assertIn("Y1:action=suck", pallet.calls)
        self.assertNotIn("Y2:action=push", pallet.calls)

    def test_main_cycle_requires_second_step_before_final_step(self) -> None:
        flow = MainCycleFlow(
            FakeLiftDomainService([560, 560, 560]),
            FakePalletTransferDomainService(),
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(620.0),
            self.main_cycle_config(),
        )

        result = flow.run_final_step(
            Step1Command("Y1", "suck", 560.0, 111.0),
            threading.Event(),
        )

        self.assertEqual(MainCycleState.ERROR, result.state)
        self.assertEqual(MainCyclePhase.READY_FIRST_STEP, result.phase)

    def test_cancelled_final_step_does_not_release_vacuum_guard(self) -> None:
        slot_vacuum = FakeSlotVacuumDomainService()
        middle = FakeMiddleVacuumDomainService()
        flow = MainCycleFlow(
            FakeLiftDomainService([560.0]),
            FakePalletTransferDomainService(),
            middle,
            FakeVisionBridgeDomainService(560.0),
            self.main_cycle_config(),
            slot_vacuum_service=slot_vacuum,
        )
        flow.phase = MainCyclePhase.WAITING_FINAL_STEP1
        cancel_event = threading.Event()
        cancel_event.set()

        result = flow.run_final_step(
            Step1Command("Y2", "push", 560.0, 222.0),
            cancel_event,
        )

        self.assertEqual(MainCycleState.CANCELLED, result.state)
        self.assertEqual(MainCyclePhase.WAITING_FINAL_STEP1, flow.phase)
        self.assertEqual([], slot_vacuum.calls)
        self.assertEqual([], middle.modes)

    def test_main_cycle_second_step_can_use_arm_vision_workflow(self) -> None:
        lift = FakeLiftDomainService(
            [560, 560, 560]
            + [560, 560, 560]
            + [675, 675, 675]
            + [460, 460, 460]
            + [460, 460]
            + [675, 675, 675]
            + [560, 560, 560]
        )
        pallet = FakePalletTransferDomainService()
        middle = FakeMiddleVacuumDomainService()
        vision = FakeVisionBridgeDomainService(620.0)

        class SafetyCheckingArm(FakeArmVisionWorkflowDomainService):
            def __init__(self, height_mm: float) -> None:
                super().__init__(height_mm)
                self.safe_height_after_handoff: list[float] = []

            def run_pick_and_place(
                self,
                cancel_event,
                *,
                pick_handoff,
                place_handoff,
                movement_handoff=None,
                view=None,
                progress_callback=None,
            ) -> dict:
                if movement_handoff is not None:
                    movement_handoff("fake arm start")
                self.views.append(view)
                self.calls.append("run_pick_and_place")
                pick_handoff(self.height_mm, {"depth_m": 0.78})
                self.safe_height_after_handoff.append(
                    lift.position_config[-1][0]
                )
                self.calls.append("pick_handoff_done")
                place_handoff(self.height_mm, {"depth_m": 0.78})
                self.safe_height_after_handoff.append(
                    lift.position_config[-1][0]
                )
                self.calls.append("place_handoff_done")
                return {
                    "depth_m": 0.78,
                    "plc_height_mm": self.height_mm,
                    "control_target": {"depth_m": 0.78},
                }

        arm = SafetyCheckingArm(675.0)
        flow = MainCycleFlow(
            lift,
            pallet,
            middle,
            vision,
            self.main_cycle_config(),
            arm_vision_service=arm,
        )
        cancel_event = threading.Event()

        flow.run_first_step(Step1Command("none", "none", 560.0), cancel_event)
        second = flow.run_second_step(cancel_event)

        self.assertEqual(MainCycleState.SUCCESS, second.state)
        self.assertEqual(MainCyclePhase.WAITING_FINAL_STEP1, flow.phase)
        self.assertEqual(["run_pick_and_place", "pick_handoff_done", "place_handoff_done"], arm.calls)
        self.assertEqual([460.0, 560.0], arm.safe_height_after_handoff)
        self.assertEqual(["vacuum", "break_vacuum", "off"], middle.modes)
        self.assertEqual(["vacuum", "release"], middle.confirmations)
        self.assertEqual(
            [560.0, 675.0, 460.0, 675.0, 560.0],
            [target for target, _speed in lift.position_config],
        )
        self.assertEqual(["RView"], arm.views)
        self.assertEqual([], vision.events)

    def test_main_cycle_y2_to_y1_selects_lview(self) -> None:
        lift = FakeLiftDomainService(
            [560.0] * 5
            + [675.0] * 3
            + [460.0] * 3
            + [460.0] * 2
            + [675.0] * 3
            + [560.0] * 3
        )
        arm = FakeArmVisionWorkflowDomainService(675.0)
        flow = MainCycleFlow(
            lift,
            FakePalletTransferDomainService(),
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(675.0),
            self.main_cycle_config(),
            arm_vision_service=arm,
        )
        cancel_event = threading.Event()
        flow.run_first_step(
            Step1Command("none", "none", 560.0),
            cancel_event,
        )

        second = flow.run_second_step(
            cancel_event,
            transfer_direction="Y2_TO_Y1",
        )

        self.assertEqual(MainCycleState.SUCCESS, second.state)
        self.assertEqual("Y2_TO_Y1", second.transfer_direction)
        self.assertEqual(["LView"], arm.views)
        self.assertIn("Y2 → Y1", second.message)

    def test_main_cycle_second_step_caps_every_bridge_height_at_695(self) -> None:
        lift = FakeLiftDomainService(
            [560.0] * 3
            + [695.0] * 3
            + [460.0] * 3
            + [695.0] * 3
            + [560.0] * 3
        )
        vision = FakeVisionBridgeDomainService(900.0)
        progress_messages: list[str] = []
        interlock = ArmCameraHomeInterlock()
        interlock.mark_home_confirmed("first step prerequisite confirmed")
        flow = MainCycleFlow(
            lift,
            FakePalletTransferDomainService(),
            FakeMiddleVacuumDomainService(),
            vision,
            self.main_cycle_config(vision_height_mm=900.0),
            home_interlock=interlock,
        )
        cancel_event = threading.Event()

        flow.run_first_step(
            Step1Command("none", "none", 1450.0),
            cancel_event,
        )
        second = flow.run_second_step(
            cancel_event,
            lambda progress: progress_messages.append(progress.message),
        )

        self.assertEqual(MainCycleState.SUCCESS, second.state)
        targets = [target for target, _speed in lift.position_config]
        self.assertEqual([560.0, 695.0, 460.0, 695.0, 560.0], targets)
        self.assertTrue(all(target <= 695.0 for target in targets))
        self.assertEqual(695.0, second.vision_height_mm)
        self.assertTrue(any("900mm" in message and "695mm" in message for message in progress_messages))
        event_payloads = {
            name: payload
            for name, payload in vision.events
            if name not in {"start", "wait_signal", "wait_height", "wait_done"}
        }
        self.assertEqual(560.0, event_payloads["vision_height_ready"]["vision_height_mm"])
        self.assertEqual(695.0, event_payloads["m54_on_after_height"]["height_mm"])
        self.assertEqual(460.0, event_payloads["returned_cross_side_height_after_m54"]["cross_side_height_mm"])
        self.assertEqual(695.0, event_payloads["m56_on_m54_off_after_same_height"]["height_mm"])
        self.assertFalse(interlock.all_home_confirmed)
        self.assertEqual("not_home", interlock.snapshot.state.value)

    def test_main_cycle_second_step_caps_arm_handoff_height_at_695(self) -> None:
        lift = FakeLiftDomainService(
            [560.0] * 6
            + [695.0] * 3
            + [460.0] * 3
            + [460.0] * 2
            + [695.0] * 3
            + [560.0] * 3
        )
        arm = FakeArmVisionWorkflowDomainService(900.0)
        flow = MainCycleFlow(
            lift,
            FakePalletTransferDomainService(),
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(900.0),
            self.main_cycle_config(vision_height_mm=900.0),
            arm_vision_service=arm,
        )
        cancel_event = threading.Event()

        flow.run_first_step(
            Step1Command("none", "none", 1450.0),
            cancel_event,
        )
        second = flow.run_second_step(cancel_event)

        self.assertEqual(MainCycleState.SUCCESS, second.state)
        targets = [target for target, _speed in lift.position_config]
        self.assertEqual([560.0, 695.0, 460.0, 695.0, 560.0], targets)
        self.assertTrue(all(target <= 695.0 for target in targets))
        self.assertEqual(695.0, second.vision_height_mm)

    def test_main_cycle_blocks_arm_start_if_height_is_not_560_at_motion_gate(self) -> None:
        lift = FakeLiftDomainService(
            [560.0, 560.0, 560.0, 562.0, 562.0]
        )
        arm = FakeArmVisionWorkflowDomainService(675.0)
        flow = MainCycleFlow(
            lift,
            FakePalletTransferDomainService(),
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(675.0),
            self.main_cycle_config(),
            arm_vision_service=arm,
        )
        cancel_event = threading.Event()

        flow.run_first_step(
            Step1Command("none", "none", 560.0),
            cancel_event,
        )
        second = flow.run_second_step(cancel_event)

        self.assertEqual(MainCycleState.ERROR, second.state)
        self.assertEqual([], arm.calls)
        self.assertIn("禁止手臂移動", second.message)
        self.assertIn("562mm", second.message)

    def test_main_cycle_second_step_preserves_cancelled_lifecycle_status(self) -> None:
        lift = FakeLiftDomainService([560, 560, 560])
        flow = MainCycleFlow(
            lift,
            FakePalletTransferDomainService(),
            FakeMiddleVacuumDomainService(),
            FakeVisionBridgeDomainService(560.0),
            self.main_cycle_config(),
            arm_vision_service=FakeArmVisionWorkflowDomainService(560.0),
        )
        cancel_event = threading.Event()
        flow.run_first_step(
            Step1Command("none", "none", 560.0),
            cancel_event,
        )
        cancel_event.set()

        second = flow.run_second_step(cancel_event)

        self.assertEqual(MainCycleState.CANCELLED, second.state)
        self.assertEqual(LifecycleStatus.CANCELLED, second.status)
        self.assertEqual(MainCyclePhase.WAITING_STEP2, flow.phase)

    def test_arm_vision_height_formula_only_applies_above_0650m(self) -> None:
        service = ArmVisionWorkflowService(
            ArmVisionWorkflowConfig(
                enabled=False,
                vision_height_mm=560.0,
                height_reference_depth_mm=620.0,
            )
        )

        self.assertEqual(649.0, service.depth_m_to_plc_height_mm(0.649))
        self.assertEqual(650.0, service.depth_m_to_plc_height_mm(0.650))
        self.assertAlmostEqual(590.001, service.depth_m_to_plc_height_mm(0.650001))
        self.assertEqual(603.0, service.depth_m_to_plc_height_mm(0.663))
        self.assertEqual(615.0, service.depth_m_to_plc_height_mm(0.675))
        self.assertEqual(640.0, service.depth_m_to_plc_height_mm(0.7))
        self.assertAlmostEqual(
            648.214282989502,
            service.depth_m_to_plc_height_mm(0.708214282989502),
        )
        self.assertEqual(655.0, service.depth_m_to_plc_height_mm(0.715))
        self.assertEqual(678.0, service.depth_m_to_plc_height_mm(0.738))
        self.assertAlmostEqual(678.01, service.depth_m_to_plc_height_mm(0.73801))
        self.assertEqual(680.0, service.depth_m_to_plc_height_mm(0.74))
        self.assertEqual(685.0, service.depth_m_to_plc_height_mm(0.745))
        self.assertEqual(695.0, service.depth_m_to_plc_height_mm(0.9))

    def test_home_pose_requires_consecutive_stable_four_motor_readbacks(self) -> None:
        home = {
            "ID 142": 6.0,
            "ID 143": -22.93,
            "ID 144": 6.7,
            "ID 145": 36.49,
        }
        outside = {**home, "ID 145": 39.0}
        controller = FakeArmPoseController([home, outside, home, home, home])

        with contextlib.redirect_stdout(io.StringIO()):
            result = wait_until_named_pose(
                controller,
                "HOME",
                tolerance_degrees=1.0,
                stable_reads=3,
                timeout_seconds=0.1,
                poll_interval_seconds=0.001,
            )

        self.assertEqual(home, result)
        self.assertEqual([], controller.readings)

    def test_home_pose_can_confirm_camera_pair_without_requiring_arm_pair(self) -> None:
        reading = {
            "ID 142": 100.0,
            "ID 143": 100.0,
            "ID 144": 6.7,
            "ID 145": 36.49,
        }
        controller = FakeArmPoseController([reading, reading, reading])

        with contextlib.redirect_stdout(io.StringIO()):
            result = wait_until_named_pose(
                controller,
                "HOME",
                tolerance_degrees=1.0,
                stable_reads=3,
                timeout_seconds=0.1,
                poll_interval_seconds=0.001,
                motor_labels=("ID 144", "ID 145"),
            )

        self.assertEqual({"ID 144": 6.7, "ID 145": 36.49}, result)

    def test_final_home_marker_is_emitted_only_after_readback_confirmation(self) -> None:
        home = {
            "ID 142": 6.0,
            "ID 143": -22.93,
            "ID 144": 6.7,
            "ID 145": 36.49,
        }
        controller = FakeArmPoseController([home, home, home])
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            confirm_home_pose(
                controller,
                tolerance_degrees=1.0,
                stable_reads=3,
                timeout_seconds=0.1,
                poll_interval_seconds=0.001,
                final_confirmation=True,
            )

        self.assertIn(ALL_HOME_CONFIRMED_MARKER, output.getvalue())

    def test_failed_home_readback_never_emits_final_confirmation_marker(self) -> None:
        outside = {
            "ID 142": 6.0,
            "ID 143": -22.93,
            "ID 144": 6.7,
            "ID 145": 40.0,
        }
        controller = FakeArmPoseController([outside])
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            with self.assertRaises(TimeoutError):
                confirm_home_pose(
                    controller,
                    tolerance_degrees=1.0,
                    stable_reads=3,
                    timeout_seconds=0.002,
                    poll_interval_seconds=0.001,
                    final_confirmation=True,
                )

        self.assertNotIn(ALL_HOME_CONFIRMED_MARKER, output.getvalue())

    def test_motor_stop_marker_requires_all_four_stop_responses(self) -> None:
        home = {
            "ID 142": 6.0,
            "ID 143": -22.93,
            "ID 144": 6.7,
            "ID 145": 36.49,
        }
        controller = FakeArmPoseController([home])
        controller.stop_result["ID 145"] = "ERROR no response"
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            confirmed = stop_all_motors_confirmed(controller)

        self.assertFalse(confirmed)
        self.assertIn("ARM_STOP_UNCONFIRMED", output.getvalue())

    def test_home_flow_timeout_has_timeout_lifecycle_status(self) -> None:
        service = FakeHomeDomainService(
            [{"X": 1, "Y1": 1, "Y2": 1}],
        )

        result = HomeFlow(
            service,
            self.home_config(timeout_seconds=0.01),
        ).run(threading.Event())

        self.assertEqual(HomeFlowState.TIMEOUT, result.state)
        self.assertEqual(LifecycleStatus.TIMEOUT, result.status)

    def test_cancel_request_reports_already_finished_without_setting_event(self) -> None:
        flow = HomeFlow(
            FakeHomeDomainService([{"X": 0, "Y1": 0, "Y2": 0}]),
            self.home_config(),
        )
        flow._set_status(LifecycleStatus.TIMEOUT, "wait_zero", "等待回原點逾時")
        cancel_event = threading.Event()

        response = _request_flow_cancellation(flow, cancel_event, "HomeFlow")

        self.assertTrue(cancel_event.is_set())
        self.assertTrue(response.data["already_finished"])
        self.assertTrue(response.data["cancel_requested"])
        self.assertEqual("timeout", response.data["flow"]["status"])

    def test_cancel_request_sets_event_while_flow_is_running(self) -> None:
        flow = HomeFlow(
            FakeHomeDomainService([{"X": 1, "Y1": 0, "Y2": 0}]),
            self.home_config(),
        )
        flow._set_status(LifecycleStatus.WAITING_SIGNAL, "wait_zero", "等待回原點")
        cancel_event = threading.Event()

        response = _request_flow_cancellation(flow, cancel_event, "HomeFlow")

        self.assertTrue(cancel_event.is_set())
        self.assertFalse(response.data["already_finished"])
        self.assertTrue(response.data["cancel_requested"])


if __name__ == "__main__":
    unittest.main()
