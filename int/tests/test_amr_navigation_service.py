from __future__ import annotations

import unittest

from plc2.service.amr_navigation_service import (
    AmrNavigationError,
    AmrNavigationService,
)


class FakeAmr:
    """Minimal stand-in for ServiceAmr covering what amr_cmd.action calls."""

    def __init__(self, goto_result=None, fsm_sequence=None) -> None:
        self.goto_result = goto_result or {"code": 0, "data": None, "msg": "success"}
        # First read is the pre-navigation FSM check; later reads are the
        # goto_amr_position poll loop. Defaults to an immediate "succeeded".
        self._fsm_sequence = list(fsm_sequence or ["idle", "succeeded"])
        self._fsm_index = 0
        self.goto_calls: list[tuple[str, bool, int]] = []
        self.set_obs_calls: list[tuple[float, float]] = []
        self.cancel_calls = 0
        self.disconnect_calls = 0
        self.confirm_calls = 0

    def connect(self):
        return {"success": True, "connected": True}

    def read_map(self, map_name):
        return {"success": True, "detail": {"map_name": map_name}}

    def set_obs(self, precision_xy, precision_yaw):
        self.set_obs_calls.append((precision_xy, precision_yaw))
        return {
            "success": True,
            "arrival_tolerance_m": precision_xy,
            "angle_tolerance_rad": precision_yaw,
        }

    def goto_position(self, position_id, is_reverse, nav_type):
        self.goto_calls.append((position_id, is_reverse, nav_type))
        return self.goto_result

    def get_robot_data_once(self):
        index = min(self._fsm_index, len(self._fsm_sequence) - 1)
        self._fsm_index += 1
        return {"fsm": self._fsm_sequence[index]}

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
    def test_success_response_confirms_arrival(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)
        service.connect()
        service.read_map("170-0810")

        result = service.goto("Point-1", False, 2, poll_interval=0.01)

        self.assertTrue(result["arrived"])
        self.assertEqual("Point-1", result["position_id"])
        self.assertEqual([("Point-1", False, 2)], fake.goto_calls)

    def test_goto_forwards_is_reverse_and_nav_type(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)
        service.read_map("170-0810")

        service.goto("Point-2", True, 1, poll_interval=0.01)

        self.assertEqual([("Point-2", True, 1)], fake.goto_calls)

    def test_non_success_response_blocks_next_step(self) -> None:
        for result in (
            {"code": 1, "msg": "failed"},
            {"success": False, "message": "rejected"},
        ):
            with self.subTest(result=result):
                fake = FakeAmr(goto_result=result)
                service = AmrNavigationService(fake)
                service.read_map("170-0810")
                with self.assertRaises(AmrNavigationError):
                    service.goto("Point-1", False, 2, poll_interval=0.01)

    def test_fsm_failed_raises_and_blocks_next_step(self) -> None:
        fake = FakeAmr(fsm_sequence=["idle", "failed"])
        service = AmrNavigationService(fake)
        service.read_map("170-0810")

        with self.assertRaises(AmrNavigationError):
            service.goto("Point-1", False, 2, poll_interval=0.01)
        self.assertEqual(1, fake.confirm_calls)

    def test_goto_requires_map_read_first(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)

        with self.assertRaisesRegex(AmrNavigationError, "Read an AMR map"):
            service.goto("Point-1", False, 2)

        self.assertEqual([], fake.goto_calls)

    def test_set_navigation_precision_delegates_to_amr(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)

        result = service.set_navigation_precision(0.03, 0.087)

        self.assertTrue(result["success"])
        self.assertEqual([(0.03, 0.087)], fake.set_obs_calls)

    def test_cancel_task_does_not_disconnect(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)

        result = service.cancel_task()

        self.assertEqual({"code": 0, "msg": "success"}, result)
        self.assertEqual(1, fake.cancel_calls)
        self.assertEqual(0, fake.disconnect_calls)

    def test_confirm_status_delegates_to_amr(self) -> None:
        fake = FakeAmr()
        service = AmrNavigationService(fake)

        result = service.confirm_status()

        self.assertEqual({"code": 0, "msg": "success"}, result)
        self.assertEqual(1, fake.confirm_calls)

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
