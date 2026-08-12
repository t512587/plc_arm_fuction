from __future__ import annotations

import unittest
from unittest.mock import patch

import action_main

action_main.prepare_import_path()
from api import main as api_main  # noqa: E402


class FakeApiAmrService:
    def __init__(self) -> None:
        self.goto_calls = []
        self.set_precision_calls = []
        self.cleanup_calls = 0

    def connect(self):
        return {"success": True, "connected": True}

    def read_map(self, map_name):
        return {"success": True, "map_name": map_name, "waypoint_count": 3}

    def set_navigation_precision(self, precision_xy, precision_yaw):
        self.set_precision_calls.append((precision_xy, precision_yaw))
        return {
            "success": True,
            "precision_xy": precision_xy,
            "precision_yaw": precision_yaw,
        }

    def goto(self, position_id, response_timeout_seconds, **arrival_settings):
        self.goto_calls.append((position_id, response_timeout_seconds, arrival_settings))
        return {
            "success": True,
            "arrived": True,
            "position_id": position_id,
            "response": {"code": 0, "msg": "success"},
        }

    def disconnect(self):
        return {"success": True, "connected": False}

    def cancel_and_disconnect(self):
        self.cleanup_calls += 1
        return {"success": True, "cancelled": True, "errors": []}


class AmrApiTests(unittest.TestCase):
    def test_set_obs_endpoint_sets_navigation_precision(self) -> None:
        fake = FakeApiAmrService()
        body = api_main.AmrSetObsBody(
            precision_xy=0.03,
            precision_yaw=0.087,
        )

        with patch.object(api_main, "AMR_NAVIGATION_SERVICE", fake):
            response = api_main.set_amr_navigation_precision(body)

        self.assertTrue(response.ok)
        self.assertEqual([(0.03, 0.087)], fake.set_precision_calls)

    def test_goto_endpoint_returns_success_gate_data(self) -> None:
        fake = FakeApiAmrService()
        body = api_main.AmrGotoBody(
            position_id="Point-d6uj6p",
            response_timeout_seconds=180,
            arrival_xy=0.10,
            hold_seconds=0.8,
            poll_interval=0.35,
        )

        with (
            patch.object(api_main, "AMR_NAVIGATION_SERVICE", fake),
            patch.object(api_main.logger, "info") as log_info,
        ):
            response = api_main.goto_amr_position(body)

        self.assertTrue(response.ok)
        self.assertTrue(response.data["arrived"])
        self.assertEqual(
            [(
                "Point-d6uj6p",
                180.0,
                {"arrival_xy": 0.10, "hold_seconds": 0.8, "poll_interval": 0.35},
            )],
            fake.goto_calls,
        )
        log_info.assert_called_once_with(
            "AMR arrival confirmed position_id=%s code=%r msg=%r; "
            "continuing to next TXT step",
            "Point-d6uj6p",
            0,
            "success",
        )

    def test_cancel_endpoint_runs_cleanup(self) -> None:
        fake = FakeApiAmrService()

        with patch.object(api_main, "AMR_NAVIGATION_SERVICE", fake):
            response = api_main.cancel_amr_task()

        self.assertTrue(response.ok)
        self.assertEqual("cancelled", response.data["state"])
        self.assertEqual(1, fake.cleanup_calls)


if __name__ == "__main__":
    unittest.main()
