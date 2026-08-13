from __future__ import annotations

import threading
import unittest
from dataclasses import dataclass

from plc.positioning_vision_height_workflow import (
    VisionHeightConfig,
    VisionHeightState,
    VisionHeightWorkflow,
    load_vision_height_config,
)


@dataclass
class FakePoint:
    device: str
    writable: bool


class FakePresetPlc:
    def __init__(self, heights: list[float]) -> None:
        self.is_connected = True
        self.heights = heights
        self.height_index = 0
        self.values: dict[str, float | bool] = {}
        self.writes: list[tuple[str, float | bool]] = []
        self.points = {
            "x_current_pos": FakePoint("D", False),
            "LIFT_UP_POS": FakePoint("M", True),
            "LIFT_DN_POS": FakePoint("M", True),
            "L_VAC_ON": FakePoint("M", True),
            "R_VAC_ON": FakePoint("M", True),
            "L_VAC_REL": FakePoint("M", True),
            "R_VAC_REL": FakePoint("M", True),
        }

    def get_point(self, point_id: str) -> FakePoint:
        return self.points[point_id]

    def read_point(self, point_id: str) -> float | bool:
        if point_id == "x_current_pos":
            value = self.heights[min(self.height_index, len(self.heights) - 1)]
            self.height_index += 1
            return value
        return self.values.get(point_id, False)

    def write_point(self, point_id: str, value: float | bool) -> None:
        self.writes.append((point_id, value))
        self.values[point_id] = value


def fast_config(**overrides) -> VisionHeightConfig:
    values = {
        "stable_read_count": 2,
        "poll_interval_seconds": 0.001,
        "vacuum_build_seconds": 0.0,
        "settle_seconds": 0.0,
        "timeout_seconds": 0.2,
        "trigger_reset_seconds": 0.001,
    }
    values.update(overrides)
    return VisionHeightConfig(**values)


class PresetVisionHeightWorkflowTests(unittest.TestCase):
    def test_workspace_config_uses_only_listed_lift_points(self) -> None:
        config = load_vision_height_config()

        self.assertEqual(560.0, config.target_height_mm)
        self.assertEqual("LIFT_UP_POS", config.up_command_point)
        self.assertEqual("LIFT_DN_POS", config.down_command_point)
        self.assertEqual((0.0, 1450.0), (config.minimum_height_mm, config.maximum_height_mm))

    def test_below_target_uses_only_m376(self) -> None:
        service = FakePresetPlc([400, 500, 550, 560, 560, 560])

        result = VisionHeightWorkflow(service, fast_config()).run(threading.Event())

        self.assertEqual(VisionHeightState.SUCCESS, result.state)
        self.assertIn(("LIFT_UP_POS", True), service.writes)
        self.assertNotIn(("LIFT_DN_POS", True), service.writes)
        self.assertIn(("L_VAC_ON", True), service.writes)
        self.assertIn(("R_VAC_ON", True), service.writes)
        self.assertLess(service.writes.index(("L_VAC_REL", False)), service.writes.index(("L_VAC_ON", True)))
        self.assertLess(service.writes.index(("R_VAC_REL", False)), service.writes.index(("R_VAC_ON", True)))

    def test_above_target_uses_m376_and_never_starts_m379(self) -> None:
        service = FakePresetPlc([700, 650, 600, 560, 560, 560])

        result = VisionHeightWorkflow(service, fast_config()).run(threading.Event())

        self.assertEqual(VisionHeightState.SUCCESS, result.state)
        self.assertIn(("LIFT_UP_POS", True), service.writes)
        self.assertNotIn(("LIFT_DN_POS", True), service.writes)

    def test_already_at_target_does_not_start_lift(self) -> None:
        service = FakePresetPlc([560, 560, 560, 560])

        result = VisionHeightWorkflow(service, fast_config()).run(threading.Event())

        self.assertEqual(VisionHeightState.SUCCESS, result.state)
        self.assertNotIn(("LIFT_UP_POS", True), service.writes)
        self.assertNotIn(("LIFT_DN_POS", True), service.writes)

    def test_cancel_stops_lift_and_keeps_vacuum(self) -> None:
        service = FakePresetPlc([400, 450, 500, 530, 550])
        cancel_event = threading.Event()

        def cancel_during_motion(progress) -> None:
            if progress.state is VisionHeightState.MOVING:
                cancel_event.set()

        result = VisionHeightWorkflow(service, fast_config()).run(cancel_event, cancel_during_motion)

        self.assertEqual(VisionHeightState.CANCELLED, result.state)
        self.assertFalse(service.values["LIFT_UP_POS"])
        self.assertFalse(service.values["LIFT_DN_POS"])
        self.assertTrue(service.values["L_VAC_ON"])
        self.assertTrue(service.values["R_VAC_ON"])

    def test_height_outside_zero_to_1450_is_rejected(self) -> None:
        service = FakePresetPlc([-1])

        result = VisionHeightWorkflow(service, fast_config()).run(threading.Event())

        self.assertEqual(VisionHeightState.ERROR, result.state)
        self.assertNotIn(("LIFT_UP_POS", True), service.writes)
        self.assertNotIn(("LIFT_DN_POS", True), service.writes)

    def test_unchanged_height_distinguishes_command_still_on(self) -> None:
        service = FakePresetPlc([400])

        result = VisionHeightWorkflow(
            service,
            fast_config(unchanged_timeout_seconds=0.01),
        ).run(threading.Event())

        self.assertEqual(VisionHeightState.POSITION_UNCHANGED, result.state)
        self.assertIn("仍為 ON", result.message)
        self.assertIn("不等同機械卡住", result.message)


if __name__ == "__main__":
    unittest.main()
