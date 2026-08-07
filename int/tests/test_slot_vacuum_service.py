from __future__ import annotations

import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from plc.slot_vacuum_service import SlotOccupancy, SlotVacuumService


@dataclass
class FakePoint:
    device: str
    address: int


class FakeGuardedPlc:
    def __init__(self) -> None:
        self.values = {
            "L_VAC_ON": False,
            "L_VAC_REL": False,
            "R_VAC_ON": False,
            "R_VAC_REL": False,
        }
        self.points = {
            "L_VAC_ON": FakePoint("M", 50),
            "L_VAC_REL": FakePoint("M", 51),
            "R_VAC_ON": FakePoint("M", 52),
            "R_VAC_REL": FakePoint("M", 53),
        }
        self.point_guards: list[Callable[[str, Any], None]] = []

    def register_write_guard(self, **kwargs: Any) -> None:
        self.point_guards.append(kwargs["point_guard"])

    def get_point(self, point_id: str) -> FakePoint:
        return self.points[point_id]

    def read_point(self, point_id: str) -> bool:
        return self.values[point_id]

    def write_point(self, point_id: str, value: Any) -> None:
        for guard in self.point_guards:
            guard(point_id, value)
        self.values[point_id] = bool(value)


class SlotVacuumServiceTests(unittest.TestCase):
    def test_occupied_side_turns_on_and_protects_vacuum(self) -> None:
        plc = FakeGuardedPlc()
        with tempfile.TemporaryDirectory() as directory:
            service = SlotVacuumService(plc, Path(directory) / "states.json")
            states = service.set_occupancy("Y1", SlotOccupancy.OCCUPIED)

            self.assertEqual("occupied", states["Y1"])
            self.assertTrue(plc.values["L_VAC_ON"])
            with self.assertRaisesRegex(RuntimeError, "禁止關閉真空"):
                plc.write_point("L_VAC_ON", False)
            with self.assertRaisesRegex(RuntimeError, "禁止啟動破真空"):
                plc.write_point("L_VAC_REL", True)

    def test_leaving_occupied_requires_confirmation_and_persists(self) -> None:
        plc = FakeGuardedPlc()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "states.json"
            service = SlotVacuumService(plc, path)
            service.set_occupancy("Y2", "occupied")
            with self.assertRaisesRegex(RuntimeError, "必須確認"):
                service.set_occupancy("Y2", "empty")
            service.set_occupancy("Y2", "empty", confirm_release=True)

            reloaded = SlotVacuumService(FakeGuardedPlc(), path)
            self.assertEqual("empty", reloaded.list_states()["Y2"])


if __name__ == "__main__":
    unittest.main()
