from __future__ import annotations

import unittest

from plc2.task_file import TaskFileError, parse_task_text


class TaskFileParserTests(unittest.TestCase):
    def test_parses_supported_step_types_in_order(self) -> None:
        steps = parse_task_text(
            """
            # sample
            step
            type=control_settings
            x_speed=250
            y1_speed=180
            height_reference_depth_mm=615.5

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
            target=plc

            step
            type=wait
            seconds=1.5

            step
            type=confirm
            message=Check the part

            step
            type=arm_pose
            pose=home
            """
        )

        self.assertEqual(
            ["control_settings", "pallet", "vision_transfer", "home", "wait", "confirm", "arm_pose"],
            [step.type for step in steps],
        )
        self.assertEqual("250", steps[0].values["x_speed"])
        self.assertEqual("180", steps[0].values["y1_speed"])
        self.assertEqual("615.5", steps[0].values["height_reference_depth_mm"])
        self.assertEqual("Y1", steps[1].values["slot"])
        self.assertEqual("suck", steps[1].values["action"])
        self.assertEqual("Y1_TO_Y2", steps[2].values["transfer_direction"])
        self.assertEqual("3", steps[2].values["repeat"])
        self.assertEqual("plc", steps[3].values["target"])
        self.assertEqual("HOME", steps[6].values["pose"])

    def test_control_settings_must_be_first_and_unique(self) -> None:
        settings = """
            step
            type=control_settings
            x_speed=200
            y1_speed=200
            height_reference_depth_mm=620
        """
        with self.assertRaisesRegex(TaskFileError, "first step"):
            parse_task_text("step\ntype=wait\nseconds=0\n" + settings)
        with self.assertRaisesRegex(TaskFileError, "only once"):
            parse_task_text(settings + settings)

    def test_rejects_invalid_control_settings(self) -> None:
        cases = (
            ("x_speed=0\ny1_speed=200\nheight_reference_depth_mm=620", "x_speed"),
            ("x_speed=200\ny1_speed=1.5\nheight_reference_depth_mm=620", "y1_speed"),
            ("x_speed=200\ny1_speed=200\nheight_reference_depth_mm=0", "height_reference"),
            ("x_speed=200\ny1_speed=200\nheight_reference_depth_mm=nan", "height_reference"),
        )
        for body, message in cases:
            with self.subTest(body=body), self.assertRaisesRegex(TaskFileError, message):
                parse_task_text(f"step\ntype=control_settings\n{body}\n")

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

    def test_accepts_standby_arm_pose(self) -> None:
        steps = parse_task_text(
            """
            step
            type=arm_pose
            pose=STANDBY
            """
        )

        self.assertEqual("STANDBY", steps[0].values["pose"])

    def test_rejects_home_target_all_with_migration_message(self) -> None:
        with self.assertRaisesRegex(TaskFileError, "pose=HOME"):
            parse_task_text(
                """
                step
                type=home
                target=all
                """
            )

    def test_rejects_unsupported_arm_pose(self) -> None:
        with self.assertRaisesRegex(TaskFileError, "HOME or STANDBY"):
            parse_task_text(
                """
                step
                type=arm_pose
                pose=MOVE
                """
            )


if __name__ == "__main__":
    unittest.main()
