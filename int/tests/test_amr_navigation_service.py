from __future__ import annotations

import unittest
from types import SimpleNamespace

from plc2.service.amr_navigation_service import (
    AmrNavigationError,
    AmrNavigationService,
)


class FakeAmr:
    def __init__(self, goto_result=None) -> None:
        self.config = SimpleNamespace(timeout_seconds=3.0)
        self.goto_result = goto_result or {"code": 0, "data": None, "msg": "success"}
        self.goto_calls = []
        self.current_waypoint = "Point-1"
        self.cancel_calls = 0
        self.disconnect_calls = 0
        self.confirm_calls = 0

    def connect(self):
        return {"success": True, "connected": True}

    def read_map(self, map_name):
        return {"success": True, "detail": {"map_name": map_name}}

    def parse_waypoints(self, _detail):
        return [
            SimpleNamespace(
                dp_uid="Profile-1",
                wp_uid=uid,
                name=str(index),
                x=float(index),
                y=2.0,
            )
            for index, uid in enumerate(
                ("Point-1", "Point-2", "Point-3"),
                start=1,
            )
        ]

    def go_to_waypoint(self, deploy_uid, waypoint_uid):
        self.current_waypoint = waypoint_uid
        self.goto_calls.append(
            (deploy_uid, waypoint_uid, self.config.timeout_seconds)
        )
        return self.goto_result

    def get_robot_data_once(self):
        index = int(self.current_waypoint.rsplit("-", 1)[-1])
        return {
            "fsm": "succeeded",
            "pose": {"position": {"x": float(index), "y": 2.0}},
        }

    def confirm_status(self):
        self.confirm_calls += 1
        return {"code": 0, "msg": "success"}

    def cancel_task(self):
        self.cancel_calls += 1
        return {"code": 0, "msg": "success"}

    def disconnect(self):
        self.disconnect_calls += 1
        return {"success": True, "connected": False}


class AmrNavigationServiceTests(unittest.TestCase):
    def test_three_waypoints_are_released_in_order(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)
        service.read_map("170-0810")

        results = [
            service.goto(position_id, 180, hold_seconds=0, poll_interval=0.01)
            for position_id in ("Point-1", "Point-2", "Point-3")
        ]

        self.assertTrue(all(result["arrived"] for result in results))
        self.assertEqual(
            [
                ("Profile-1", "Point-1", 180.0),
                ("Profile-1", "Point-2", 180.0),
                ("Profile-1", "Point-3", 180.0),
            ],
            fake.goto_calls,
        )

    def test_success_response_confirms_arrival(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)
        service.connect()
        service.read_map("170-0810")

        result = service.goto("Point-1", 180, hold_seconds=0, poll_interval=0.01)

        self.assertTrue(result["arrived"])
        self.assertEqual("Point-1", result["position_id"])
        self.assertEqual([("Profile-1", "Point-1", 180.0)], fake.goto_calls)
        self.assertEqual(3.0, fake.config.timeout_seconds)

    def test_non_success_response_blocks_next_step(self) -> None:
        for result in (
            {"code": 1, "msg": "failed"},
            {"code": 0, "msg": "failed"},
            {"code": 0},
            {"msg": "success"},
        ):
            with self.subTest(result=result):
                service = AmrNavigationService(FakeAmr(result))
                service.read_map("170-0810")
                with self.assertRaisesRegex(
                    AmrNavigationError,
                    "did not accept navigation",
                ):
                    service.goto("Point-1", 180)

    def test_success_response_does_not_bypass_physical_arrival(self) -> None:
        fake = FakeAmr()
        robot_reads = 0

        def robot_data():
            nonlocal robot_reads
            robot_reads += 1
            return {
                "fsm": "idle" if robot_reads == 1 else "moving",
                "pose": {"position": {"x": 99.0, "y": 99.0}},
            }

        fake.get_robot_data_once = robot_data
        service = AmrNavigationService(fake)
        service.read_map("170-0810")

        with self.assertRaisesRegex(AmrNavigationError, "arrival timeout"):
            service.goto(
                "Point-1",
                0.03,
                arrival_xy=0.10,
                hold_seconds=0,
                poll_interval=0.01,
            )

    def test_rejects_waypoint_outside_selected_map(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)
        service.read_map("170-0810")

        with self.assertRaisesRegex(AmrNavigationError, "not in map"):
            service.goto("Point-missing", 180)

        self.assertEqual([], fake.goto_calls)

    def test_cancel_always_attempts_disconnect(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)
        service.read_map("170-0810")

        result = service.cancel_and_disconnect()

        self.assertTrue(result["success"])
        self.assertEqual(1, fake.cancel_calls)
        self.assertEqual(1, fake.disconnect_calls)
        self.assertIsNone(service.map_name)


if __name__ == "__main__":
    unittest.main()
