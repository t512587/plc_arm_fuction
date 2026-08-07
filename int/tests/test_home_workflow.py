from __future__ import annotations

import threading
import unittest
from dataclasses import dataclass

from plc.home_workflow import HomeWorkflow, HomeWorkflowConfig, HomeWorkflowState


@dataclass
class FakePoint:
    device: str
    writable: bool


class FakePlcService:
    def __init__(self, readings: list[dict[str, float]], *, connected: bool = True) -> None:
        self.is_connected = connected
        self.readings = readings
        self.read_index = 0
        self.read_axis_index = 0
        self.writes: list[tuple[str, bool]] = []
        self.points = {
            "X_HOME": FakePoint("M", True),
            "Y1_HOME": FakePoint("M", True),
            "Y2_HOME": FakePoint("M", True),
            "x_current_pos": FakePoint("D", False),
            "y1_current_pos": FakePoint("D", False),
            "y2_current_pos": FakePoint("D", False),
        }
        self.axis_by_point = {
            "x_current_pos": "X",
            "y1_current_pos": "Y1",
            "y2_current_pos": "Y2",
        }

    def get_point(self, point_id: str) -> FakePoint:
        return self.points[point_id]

    def read_point(self, point_id: str) -> float:
        reading = self.readings[min(self.read_index, len(self.readings) - 1)]
        value = reading[self.axis_by_point[point_id]]
        self.read_axis_index += 1
        if self.read_axis_index == 3:
            self.read_axis_index = 0
            self.read_index += 1
        return value

    def write_point(self, point_id: str, value: bool) -> None:
        self.writes.append((point_id, value))


class FailingReadPlcService(FakePlcService):
    def read_point(self, point_id: str) -> float:
        raise RuntimeError("PLC 通訊中斷")


def fast_config(**overrides) -> HomeWorkflowConfig:
    values = {
        "motion_stop_points": (),
        "pulse_seconds": 0.0,
        "poll_interval_seconds": 0.001,
        "timeout_seconds": 0.05,
        "stable_read_count": 3,
    }
    values.update(overrides)
    return HomeWorkflowConfig(**values)


class HomeWorkflowTests(unittest.TestCase):
    def test_stops_existing_motion_commands_before_home_pulse(self) -> None:
        service = FakePlcService([{"X": 0, "Y1": 0, "Y2": 0}])
        stop_points = ("LIFT_UP_POS", "L_FWD_POS", "R_FWD_POS")
        for point_id in stop_points:
            service.points[point_id] = FakePoint("M", True)

        result = HomeWorkflow(
            service,
            fast_config(motion_stop_points=stop_points),
        ).run(threading.Event())

        self.assertEqual(HomeWorkflowState.SUCCESS, result.state)
        self.assertEqual(
            [(point_id, False) for point_id in stop_points],
            service.writes[: len(stop_points)],
        )

    def test_success_requires_three_stable_reads_and_pulses_all_commands(self) -> None:
        service = FakePlcService(
            [
                {"X": 10, "Y1": 20, "Y2": 30},
                {"X": 0, "Y1": 0, "Y2": 0},
                {"X": 0, "Y1": 0, "Y2": 0},
                {"X": 0, "Y1": 0, "Y2": 0},
            ]
        )

        result = HomeWorkflow(service, fast_config(timeout_seconds=0.3)).run(threading.Event())

        self.assertEqual(HomeWorkflowState.SUCCESS, result.state)
        self.assertEqual({"X": 0.0, "Y1": 0.0, "Y2": 0.0}, result.positions)
        self.assertEqual(
            [
                ("X_HOME", True),
                ("Y1_HOME", True),
                ("Y2_HOME", True),
                ("X_HOME", False),
                ("Y1_HOME", False),
                ("Y2_HOME", False),
            ],
            service.writes,
        )

    def test_unstable_zero_resets_stable_counter(self) -> None:
        service = FakePlcService(
            [
                {"X": 5, "Y1": 5, "Y2": 5},
                {"X": 0, "Y1": 0, "Y2": 0},
                {"X": 1, "Y1": 0, "Y2": 0},
                {"X": 0, "Y1": 0, "Y2": 0},
                {"X": 0, "Y1": 0, "Y2": 0},
                {"X": 0, "Y1": 0, "Y2": 0},
            ]
        )

        result = HomeWorkflow(service, fast_config(timeout_seconds=0.3)).run(threading.Event())

        self.assertEqual(HomeWorkflowState.SUCCESS, result.state)
        self.assertGreaterEqual(service.read_index, 6)

    def test_position_read_error_is_not_treated_as_zero(self) -> None:
        service = FailingReadPlcService([{"X": 0, "Y1": 0, "Y2": 0}])

        result = HomeWorkflow(service, fast_config()).run(threading.Event())

        self.assertEqual(HomeWorkflowState.ERROR, result.state)
        self.assertIn("PLC 通訊中斷", result.message)
        self.assertEqual([], service.writes)

    def test_timeout_reports_last_positions(self) -> None:
        service = FakePlcService([{"X": 0, "Y1": 12, "Y2": 0}])

        result = HomeWorkflow(service, fast_config(timeout_seconds=0.005)).run(threading.Event())

        self.assertEqual(HomeWorkflowState.TIMEOUT, result.state)
        self.assertEqual(12.0, result.positions["Y1"])

    def test_cancelled_before_command_does_not_write(self) -> None:
        service = FakePlcService([{"X": 1, "Y1": 1, "Y2": 1}])
        cancel_event = threading.Event()
        cancel_event.set()

        result = HomeWorkflow(service, fast_config()).run(cancel_event)

        self.assertEqual(HomeWorkflowState.CANCELLED, result.state)
        self.assertEqual([], service.writes)

    def test_cancel_during_pulse_clears_all_home_commands(self) -> None:
        service = FakePlcService([{"X": 5, "Y1": 5, "Y2": 5}])
        cancel_event = threading.Event()

        def cancel_when_commanding(progress) -> None:
            if progress.state is HomeWorkflowState.COMMANDING:
                cancel_event.set()

        result = HomeWorkflow(service, fast_config(pulse_seconds=0.2)).run(
            cancel_event,
            cancel_when_commanding,
        )

        self.assertEqual(HomeWorkflowState.CANCELLED, result.state)
        self.assertEqual(
            [
                ("X_HOME", True),
                ("Y1_HOME", True),
                ("Y2_HOME", True),
                ("X_HOME", False),
                ("Y1_HOME", False),
                ("Y2_HOME", False),
            ],
            service.writes,
        )

    def test_disconnected_precheck_fails_without_writing(self) -> None:
        service = FakePlcService([{"X": 0, "Y1": 0, "Y2": 0}], connected=False)

        result = HomeWorkflow(service, fast_config()).run(threading.Event())

        self.assertEqual(HomeWorkflowState.ERROR, result.state)
        self.assertIn("PLC 尚未連線", result.message)
        self.assertEqual([], service.writes)


if __name__ == "__main__":
    unittest.main()
