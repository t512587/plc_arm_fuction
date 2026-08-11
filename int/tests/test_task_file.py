from __future__ import annotations

import unittest

from plc2.task_file import TaskFileError, parse_task_text


class TaskFileParserTests(unittest.TestCase):
    def test_parses_supported_step_types_in_order(self) -> None:
        steps = parse_task_text(
            """
            # sample
            step
            type=pallet
            slot=Y1
            action=suck
            height_mm=560
            forward_mm=120

            step
            type=vision_transfer
            transfer_direction=Y1_TO_Y2
            repeat=3

            step
            type=home
            target=all

            step
            type=wait
            seconds=1.5

            step
            type=confirm
            message=Check the part

            step
            type=arm_pose
            pose=STANDBY
            """
        )

        self.assertEqual(
            ["pallet", "vision_transfer", "home", "wait", "confirm", "arm_pose"],
            [step.type for step in steps],
        )
        self.assertEqual("Y1", steps[0].values["slot"])
        self.assertEqual("suck", steps[0].values["action"])
        self.assertEqual("Y1_TO_Y2", steps[1].values["transfer_direction"])
        self.assertEqual("3", steps[1].values["repeat"])
        self.assertEqual("all", steps[2].values["target"])
        self.assertEqual("STANDBY", steps[5].values["pose"])

    def test_rejects_key_value_before_step(self) -> None:
        with self.assertRaisesRegex(TaskFileError, "expected 'step'"):
            parse_task_text("type=pallet\n")

    def test_rejects_unknown_type(self) -> None:
        with self.assertRaisesRegex(TaskFileError, "unsupported type"):
            parse_task_text(
                """
                step
                type=unknown
                """
            )

    def test_rejects_pallet_without_forward_distance(self) -> None:
        with self.assertRaisesRegex(TaskFileError, "missing forward_mm"):
            parse_task_text(
                """
                step
                type=pallet
                slot=Y1
                action=suck
                height_mm=560
                """
            )

    def test_allows_pallet_pause_without_forward_distance(self) -> None:
        steps = parse_task_text(
            """
            step
            type=pallet
            slot=none
            action=none
            height_mm=560
            """
        )

        self.assertEqual("none", steps[0].values["slot"])
        self.assertEqual("none", steps[0].values["action"])

    def test_rejects_invalid_direction(self) -> None:
        with self.assertRaisesRegex(TaskFileError, "transfer_direction"):
            parse_task_text(
                """
                step
                type=vision_transfer
                transfer_direction=Y1_TO_Y3
                """
            )

    def test_vision_transfer_repeat_defaults_to_one(self) -> None:
        steps = parse_task_text(
            """
            step
            type=vision_transfer
            transfer_direction=Y2_TO_Y1
            """
        )

        self.assertEqual("1", steps[0].values["repeat"])

    def test_rejects_invalid_vision_transfer_repeat(self) -> None:
        for repeat in ("0", "-1", "1.5", "abc", "101"):
            with self.subTest(repeat=repeat), self.assertRaisesRegex(
                TaskFileError,
                "repeat",
            ):
                parse_task_text(
                    f"""
                    step
                    type=vision_transfer
                    transfer_direction=Y1_TO_Y2
                    repeat={repeat}
                    """
                )

    def test_rejects_negative_wait(self) -> None:
        with self.assertRaisesRegex(TaskFileError, "seconds must be >= 0"):
            parse_task_text(
                """
                step
                type=wait
                seconds=-1
                """
            )

    def test_rejects_unsupported_arm_pose(self) -> None:
        with self.assertRaisesRegex(TaskFileError, "only STANDBY"):
            parse_task_text(
                """
                step
                type=arm_pose
                pose=HOME
                """
            )


if __name__ == "__main__":
    unittest.main()
