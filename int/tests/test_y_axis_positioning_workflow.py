from __future__ import annotations

import threading
import unittest
from dataclasses import dataclass

from plc.y_axis_positioning_workflow import (
    YAxisPositioningConfig,
    YAxisPositioningState,
    YAxisPositioningWorkflow,
)


@dataclass
class FakePoint:
    device: str
    writable: bool


class FakeYAxisPlc:
    def __init__(self, axis: str, positions: list[float], *, command_holds: bool = True) -> None:
        self.is_connected = True
        self.axis = axis
        self.positions = positions
        self.position_index = 0
        self.command_holds = command_holds
        self.values: dict[str, float | bool] = {
            "y1_forward_pos": 100.0,
            "y2_forward_pos": 100.0,
            "y1_forward_speed": 2.0,
            "y2_forward_speed": 2.0,
            "L_FWD_POS": False,
            "R_FWD_POS": False,
        }
        self.writes: list[tuple[str, float | bool]] = []
        self.points = {
            "y1_forward_pos": FakePoint("D", True),
            "y2_forward_pos": FakePoint("D", True),
            "y1_forward_speed": FakePoint("D", True),
            "y2_forward_speed": FakePoint("D", True),
            "y1_current_pos": FakePoint("D", False),
            "y2_current_pos": FakePoint("D", False),
            "L_FWD_POS": FakePoint("M", True),
            "R_FWD_POS": FakePoint("M", True),
        }

    def get_point(self, point_id: str) -> FakePoint:
        return self.points[point_id]

    def read_point(self, point_id: str) -> float | bool:
        if point_id == ("y1_current_pos" if self.axis == "Y1" else "y2_current_pos"):
            value = self.positions[min(self.position_index, len(self.positions) - 1)]
            self.position_index += 1
            return value
        return self.values[point_id]

    def write_point(self, point_id: str, value: float | bool) -> None:
        self.writes.append((point_id, value))
        self.values[point_id] = bool(value) if point_id.endswith("FWD_POS") else value
        if point_id.endswith("FWD_POS") and value and not self.command_holds:
            self.values[point_id] = False


def fast_config(**overrides) -> YAxisPositioningConfig:
    values = {
        "stable_read_count": 2,
        "poll_interval_seconds": 0.001,
        "trigger_reset_seconds": 0.001,
        "unchanged_timeout_seconds": 0.02,
        "timeout_seconds": 0.2,
    }
    values.update(overrides)
    return YAxisPositioningConfig(**values)


class YAxisPositioningWorkflowTests(unittest.TestCase):
    def test_y1_creates_fresh_edge_and_waits_for_position(self) -> None:
        service = FakeYAxisPlc("Y1", [0, 20, 60, 99, 100, 100])

        result = YAxisPositioningWorkflow(service, fast_config()).run("Y1", threading.Event())

        self.assertEqual(YAxisPositioningState.SUCCESS, result.state)
        self.assertEqual(("L_FWD_POS", False), service.writes[0])
        self.assertIn(("L_FWD_POS", True), service.writes)
        self.assertEqual(("L_FWD_POS", False), service.writes[-1])

    def test_y2_accepts_plc_auto_reset_when_position_moves(self) -> None:
        service = FakeYAxisPlc("Y2", [0, 25, 75, 100, 100], command_holds=False)

        result = YAxisPositioningWorkflow(service, fast_config()).run("Y2", threading.Event())

        self.assertEqual(YAxisPositioningState.SUCCESS, result.state)
        self.assertTrue(result.movement_observed)

    def test_unchanged_feedback_reports_command_state_without_calling_it_stuck(self) -> None:
        service = FakeYAxisPlc("Y2", [0])

        result = YAxisPositioningWorkflow(service, fast_config()).run("Y2", threading.Event())

        self.assertEqual(YAxisPositioningState.ERROR, result.state)
        self.assertIn("位置回授", result.message)
        self.assertIn("不等同機械卡住", result.message)

    def test_zero_speed_is_rejected_before_command_on(self) -> None:
        service = FakeYAxisPlc("Y1", [0])
        service.values["y1_forward_speed"] = 0.0

        result = YAxisPositioningWorkflow(service, fast_config()).run("Y1", threading.Event())

        self.assertEqual(YAxisPositioningState.ERROR, result.state)
        self.assertNotIn(("L_FWD_POS", True), service.writes)


if __name__ == "__main__":
    unittest.main()
