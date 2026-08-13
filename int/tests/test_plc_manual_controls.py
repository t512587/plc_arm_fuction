from __future__ import annotations

import unittest

from plc.service_plc import PLCConnectionError, ServicePlc, load_points, validate_x_motion_request
from plc.middle_vacuum_service import MiddleVacuumMode, MiddleVacuumService


class FakePointService:
    def __init__(self) -> None:
        self.values = {"M_VAC_ON": False, "M_VAC_REL": False}
        self.writes: list[tuple[str, bool]] = []

    def read_point(self, point_id: str) -> bool:
        return self.values[point_id]

    def write_point(self, point_id: str, value: bool) -> None:
        self.writes.append((point_id, bool(value)))
        self.values[point_id] = bool(value)


class FailingWriteClient:
    def __init__(self) -> None:
        self.is_connected = True
        self.write_count = 0

    def write_bit_device(self, device: str, address: int, values: list[bool]) -> None:
        self.write_count += 1
        raise PLCConnectionError("simulated disconnect")

    def close(self) -> None:
        self.is_connected = False


class PlcManualControlTests(unittest.TestCase):
    def test_x_manual_controls_are_present_in_ui_point_source(self) -> None:
        points = {point.id: point for point in load_points()}

        self.assertEqual(("M", 350, True), (points["X_UP"].device, points["X_UP"].address, points["X_UP"].writable))
        self.assertEqual(("M", 351, True), (points["X_DOWN"].device, points["X_DOWN"].address, points["X_DOWN"].writable))

    def test_manual_controls_keep_zero_to_1450_software_limits(self) -> None:
        validate_x_motion_request("X_UP", True, 0)
        validate_x_motion_request("X_DOWN", True, 1450)
        with self.assertRaises(PLCConnectionError):
            validate_x_motion_request("X_UP", True, 1450)
        with self.assertRaises(PLCConnectionError):
            validate_x_motion_request("X_DOWN", True, 0)

    def test_y_positioning_uses_m388_m398_and_disables_m389_m399(self) -> None:
        points = {point.id: point for point in load_points()}

        self.assertEqual((388, True), (points["L_FWD_POS"].address, points["L_FWD_POS"].writable))
        self.assertEqual((398, True), (points["R_FWD_POS"].address, points["R_FWD_POS"].writable))
        self.assertEqual((389, False), (points["L_BWD_POS"].address, points["L_BWD_POS"].writable))
        self.assertEqual((399, False), (points["R_BWD_POS"].address, points["R_BWD_POS"].writable))

        service = ServicePlc()
        with self.assertRaisesRegex(PLCConnectionError, "M389"):
            service.write_m(389, True)
        with self.assertRaisesRegex(PLCConnectionError, "M399"):
            service.write_m(399, True)

    def test_middle_vacuum_points_and_safe_switching(self) -> None:
        points = {point.id: point for point in load_points()}
        self.assertEqual(("M", 54, True), (points["M_VAC_ON"].device, points["M_VAC_ON"].address, points["M_VAC_ON"].writable))
        self.assertEqual(("M", 55, True), (points["M_VAC_REL"].device, points["M_VAC_REL"].address, points["M_VAC_REL"].writable))

        plc = FakePointService()
        service = MiddleVacuumService(plc)
        vacuum = service.set_mode(MiddleVacuumMode.VACUUM)
        self.assertEqual("vacuum", vacuum["mode"])
        release = service.set_mode(MiddleVacuumMode.BREAK_VACUUM)
        self.assertEqual("break_vacuum", release["mode"])
        self.assertEqual(
            [("M_VAC_REL", False), ("M_VAC_ON", True), ("M_VAC_ON", False), ("M_VAC_REL", True)],
            plc.writes,
        )

    def test_middle_vacuum_direct_write_interlock(self) -> None:
        service = ServicePlc()
        service.read_m = lambda address: address == 55  # type: ignore[method-assign]
        with self.assertRaisesRegex(PLCConnectionError, "M54.*M55"):
            service.write_m(54, True)

    def test_side_vacuum_direct_write_interlock(self) -> None:
        service = ServicePlc()
        service.read_m = lambda address: address == 51  # type: ignore[method-assign]
        with self.assertRaisesRegex(PLCConnectionError, "M50.*M51"):
            service.write_m(50, True)

    def test_uncertain_motion_write_is_not_resent(self) -> None:
        service = ServicePlc()
        client = FailingWriteClient()
        service._client = client  # type: ignore[assignment]

        with self.assertRaisesRegex(PLCConnectionError, "未自動重送"):
            service.write_m(388, True)

        self.assertEqual(1, client.write_count)

    def test_configured_d_range_is_validated_before_write(self) -> None:
        service = ServicePlc()
        speed = service.get_point("y1_forward_speed")

        with self.assertRaisesRegex(PLCConnectionError, "下限"):
            service._validate_d_value(speed, 0)


if __name__ == "__main__":
    unittest.main()
