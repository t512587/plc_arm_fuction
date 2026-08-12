from __future__ import annotations

import sys
import unittest
from pathlib import Path


CANBUS_DIR = Path(__file__).resolve().parents[1] / "canbus"
if str(CANBUS_DIR) not in sys.path:
    sys.path.insert(0, str(CANBUS_DIR))

from arm_config import HOME_SPEED_DPS  # noqa: E402
from arm_controller import ArmController  # noqa: E402


HOME = {
    "ID 142": 6.0,
    "ID 143": -22.93,
    "ID 144": 6.7,
    "ID 145": 36.49,
}
STANDBY = {
    "ID 142": 96.0,
    "ID 143": -22.93,
    "ID 144": 6.7,
    "ID 145": 126.49,
}
RGRAP = {
    "ID 142": -65.25,
    "ID 143": 70.35,
    "ID 144": 6.7,
    "ID 145": 36.49,
}


class FakeMotorService:
    def __init__(self, fail_motor_id: int | None = None) -> None:
        self.fail_motor_id = fail_motor_id
        self.dispatches: list[tuple[int, float, int]] = []
        self.acknowledged: list[tuple[int, float, int]] = []
        self.stops: list[int] = []
        self.last_tx = ""

    def send_absolute_position(
        self,
        motor_id: int,
        target: float,
        speed: int,
    ) -> dict[str, object]:
        self.last_tx = f"motor={motor_id} target={target}"
        if motor_id == self.fail_motor_id:
            raise RuntimeError("simulated dispatch failure")
        self.dispatches.append((motor_id, target, speed))
        return {"tx": self.last_tx, "raw": "A4"}

    def absolute_position_control(
        self,
        motor_id: int,
        target: float,
        speed: int,
    ) -> dict[str, object]:
        self.last_tx = f"motor={motor_id} target={target}"
        if motor_id == self.fail_motor_id:
            raise RuntimeError("simulated acknowledged command failure")
        self.acknowledged.append((motor_id, target, speed))
        return {
            "tx": self.last_tx,
            "raw": "A4 ACK",
            "degree": target,
            "speed_dps": speed,
        }

    def stop_motor(self, motor_id: int) -> dict[str, str]:
        self.stops.append(motor_id)
        return {"raw": "81"}


def make_controller(service: FakeMotorService) -> ArmController:
    controller = ArmController.__new__(ArmController)
    controller.service = service
    controller.point_config = {
        "HOME": dict(HOME),
        "STANDBY": dict(STANDBY),
        "RGrap": dict(RGRAP),
    }
    controller.home_angles = dict(HOME)
    return controller


class ArmControllerSynchronizedPoseTests(unittest.TestCase):
    def test_home_quickly_acknowledges_each_axis_without_waiting_for_pose_completion(self) -> None:
        service = FakeMotorService()
        controller = make_controller(service)
        current = {
            "ID 142": 0.0,
            "ID 143": -20.0,
            "ID 144": 0.0,
            "ID 145": 30.0,
        }

        result = controller.go_to_point("HOME", current)

        self.assertEqual([], service.dispatches)
        self.assertEqual(
            [
                (2, 6.0, HOME_SPEED_DPS),
                (3, -22.93, HOME_SPEED_DPS),
                (4, 6.7, HOME_SPEED_DPS),
                (5, 36.49, HOME_SPEED_DPS),
            ],
            service.acknowledged,
        )
        self.assertEqual(set(HOME), set(result["_updates"]))
        self.assertEqual([], service.stops)

    def test_standby_quickly_acknowledges_each_axis_in_one_pose_stage(self) -> None:
        service = FakeMotorService()
        controller = make_controller(service)

        result = controller.go_to_point("STANDBY", dict(HOME))

        self.assertEqual([], service.dispatches)
        self.assertEqual(
            [
                (2, 96.0, 500),
                (3, -22.93, 500),
                (4, 6.7, 500),
                (5, 126.49, 500),
            ],
            service.acknowledged,
        )
        self.assertEqual(set(STANDBY), set(result["_updates"]))

    def test_rgrap_waits_for_each_motor_acknowledgement(self) -> None:
        service = FakeMotorService()
        controller = make_controller(service)

        result = controller.go_to_point("RGrap", dict(HOME))

        self.assertEqual([], service.dispatches)
        self.assertEqual(
            [
                (2, -65.25, 500),
                (3, 70.35, 500),
                (4, 6.7, 500),
                (5, 36.49, 500),
            ],
            service.acknowledged,
        )
        self.assertTrue(result["ID 142"].startswith("OK acknowledged RGrap"))
        self.assertTrue(result["ID 143"].startswith("OK acknowledged RGrap"))

    def test_safety_validation_blocks_entire_batch_before_dispatch(self) -> None:
        service = FakeMotorService()
        controller = make_controller(service)
        current = dict(HOME)
        current["ID 142"] = 400.0

        result = controller.go_to_point("HOME", current)

        self.assertIn("safety", result)
        self.assertEqual([], service.dispatches)
        self.assertEqual([], service.stops)

    def test_dispatch_failure_stops_all_axes_and_aborts_remaining_batch(self) -> None:
        service = FakeMotorService(fail_motor_id=3)
        controller = make_controller(service)

        result = controller.go_to_point("HOME", dict(HOME))

        self.assertEqual([], service.dispatches)
        self.assertEqual([(2, 6.0, HOME_SPEED_DPS)], service.acknowledged)
        self.assertTrue(result["ID 143"].startswith("ERROR"))
        self.assertEqual([2, 3, 4, 5], service.stops)
        self.assertTrue(result["ID 144"].startswith("SKIP"))
        self.assertTrue(result["ID 145"].startswith("SKIP"))


if __name__ == "__main__":
    unittest.main()
