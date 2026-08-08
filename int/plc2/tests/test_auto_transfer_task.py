from __future__ import annotations

import threading
from pathlib import Path

from flow.auto_transfer_task import AutoTransferConfig, AutoTransferTask, CheckpointStore


class FakeLift:
    def __init__(self) -> None:
        self.height = 0.0
        self.moves: list[tuple[float, float | None]] = []

    def precheck(self) -> None:
        return None

    def read_height(self) -> float:
        return self.height

    def configure_position(self, target_height_mm: float, speed: float | None = None) -> None:
        self.height = target_height_mm
        self.moves.append((target_height_mm, speed))

    def start_vision_positioning(self) -> None:
        return None

    def stop(self) -> None:
        return None


class FakePallet:
    def __init__(self) -> None:
        self.position = {"Y1": 0.0, "Y2": 0.0}
        self.target = {"Y1": 0.0, "Y2": 0.0}
        self.actions: list[tuple[str, str]] = []

    def precheck(self) -> None:
        return None

    def set_forward_position(self, slot: str, forward_mm: float) -> None:
        self.target[slot] = forward_mm

    def start_forward(self, slot: str) -> None:
        self.position[slot] = self.target[slot]

    def stop_forward(self, slot: str | None = None) -> None:
        return None

    def read_position(self, slot: str) -> float:
        return self.position[slot]

    def set_action_output(self, slot: str, action) -> dict:
        self.actions.append((slot, action.value))
        return {"mode": action.value}


class FakeVacuum:
    def __init__(self) -> None:
        self.modes: list[str] = []

    def set_mode(self, mode) -> dict:
        self.modes.append(mode.value)
        return {"mode": mode.value}

    def wait_transfer_ready(self, *, vacuum_expected: bool, cancel_event: threading.Event, progress_callback=None) -> dict:
        return {"vacuum_expected": vacuum_expected}


class FakeArm:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def move_standby_pose(self, pose_name: str = "STANDBY") -> dict:
        self.calls.append(pose_name)
        return {}

    def move_home_component(self, component: str) -> dict:
        self.calls.append(f"home:{component}")
        return {}

    def confirm_home(self, cancel_event=None) -> dict:
        self.calls.append("confirm_home")
        return {}

    def run_pick_and_place(self, cancel_event, *, pick_handoff, place_handoff, movement_handoff=None, view=None, progress_callback=None) -> dict:
        self.calls.append(f"run:{view}")
        movement_handoff("Move from LView back to HOME before pick route")
        movement_handoff("Move HOME → LGrap")
        movement_handoff("Move LGrap → predicted left suction target")
        pick_handoff(350.0, {})
        movement_handoff("FAKE PLC PICK DONE")
        movement_handoff("Move left target")
        place_handoff(350.0, {})
        movement_handoff("FAKE PLC PLACE DONE")
        movement_handoff("Return right_outer")
        return {}


def test_lview_task_runs_both_pallets_and_returns_to_lowest(tmp_path: Path) -> None:
    lift = FakeLift()
    pallet = FakePallet()
    vacuum = FakeVacuum()
    arm = FakeArm()
    config = AutoTransferConfig(stable_reads=1, pallet_action_delay_seconds=0, release_hold_seconds=0)
    task = AutoTransferTask(
        lift,
        pallet,
        vacuum,
        arm,
        config=config,
        checkpoint_store=CheckpointStore(tmp_path / "last.json"),
        sleep=lambda _seconds: None,
    )

    result = task.run(height_mm=350, forward_mm=400)

    assert result["stage"] == "COMPLETED"
    assert arm.calls == ["STANDBY", "home:small", "home:big", "confirm_home", "run:LView", "confirm_home"]
    assert ("Y1", "suck") in pallet.actions and ("Y2", "suck") in pallet.actions
    assert ("Y1", "push") in pallet.actions and ("Y2", "push") in pallet.actions
    assert lift.moves[-1][0] == 0.0
    assert 460.0 in [move[0] for move in lift.moves]
    assert (560.0, 100.0) in lift.moves
    assert vacuum.modes == ["vacuum", "break_vacuum", "off"]
