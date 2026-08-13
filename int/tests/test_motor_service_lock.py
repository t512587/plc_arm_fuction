from __future__ import annotations

import sys
import unittest
import uuid
from pathlib import Path


CANBUS_DIR = Path(__file__).resolve().parents[1] / "canbus"
if str(CANBUS_DIR) not in sys.path:
    sys.path.insert(0, str(CANBUS_DIR))

from motor_service import MotorConfig, MotorService  # noqa: E402


class MotorServiceChannelLockTests(unittest.TestCase):
    def test_same_channel_cannot_be_owned_by_two_services(self) -> None:
        channel = f"test-channel-{uuid.uuid4()}"
        first = MotorService(MotorConfig(channel=channel))
        second = MotorService(MotorConfig(channel=channel))

        try:
            first._acquire_channel_lock()
            with self.assertRaisesRegex(RuntimeError, "already in use"):
                second._acquire_channel_lock()
        finally:
            first._release_channel_lock()
            second._release_channel_lock()

    def test_channel_can_be_reacquired_after_release(self) -> None:
        channel = f"test-channel-{uuid.uuid4()}"
        first = MotorService(MotorConfig(channel=channel))
        second = MotorService(MotorConfig(channel=channel))

        first._acquire_channel_lock()
        first._release_channel_lock()
        try:
            second._acquire_channel_lock()
            self.assertIsNotNone(second._channel_lock_file)
        finally:
            second._release_channel_lock()


if __name__ == "__main__":
    unittest.main()
