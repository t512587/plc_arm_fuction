"""Arm controller: business logic for reading, moving, saving positions."""
from __future__ import annotations

import json
import math

from arm_config import (
    CAMERA_BIG,
    HOME_SPEED_DPS,
    MAX_MOVE_DEGREES,
    MAX_SPEED_DPS,
    MOTOR_ANGLE_LIMITS,
    MOTORS,
    POINT_CONFIG_PATH,
    VACUUM_BIG,
    VACUUM_SMALL,
    VACUUM_SMALL_ARM_CM,
    load_config,
    load_point_config,
)
from motor_service import MotorService


class ArmController:
    """All motor control logic, independent of UI."""

    def __init__(self) -> None:
        self.config = load_config()
        self.service = MotorService(self.config)
        self.point_config = load_point_config()
        self.home_angles = self.point_config.get("HOME", {})

    @staticmethod
    def _check_angle_limits(angles: dict[str, float]) -> dict | None:
        """Check if target angles are within per-motor limits.
        Returns error dict if out of range, None if OK."""
        for label, target in angles.items():
            limits = MOTOR_ANGLE_LIMITS.get(label)
            if limits is None:
                continue
            lo, hi = limits
            if target < lo or target > hi:
                return {
                    "safety": (
                        f"BLOCKED. {label} target={target:.2f}° is outside "
                        f"safe range [{lo:.1f}°, {hi:.1f}°]."
                    )
                }
        return None

    def connect(self) -> str:
        return self.service.connect()

    def disconnect(self) -> str:
        return self.service.disconnect()

    @property
    def is_connected(self) -> bool:
        return self.service.is_connected

    def read_positions(self) -> dict:
        result = {}
        updates = {}
        for label, motor_id in MOTORS.items():
            try:
                response = self.service.read_multi_turn_angle(motor_id)
                result[f"{label} TX"] = response.get("tx", "")

                angle = float(response["angle_degrees"])
                result[label] = (
                    f"OK multi_turn_angle_degrees={angle:.2f} "
                    f"raw={response['raw']}"
                )
                updates[label] = f"{angle:.2f}"

            except Exception as exc:
                result[f"{label} TX"] = self.service.last_tx
                result[label] = f"ERROR {type(exc).__name__}: {exc}"

        result["_updates"] = updates
        result["_degree_positions"] = True
        return result

    def go_to_point(self, point_name: str, current_positions: dict[str, float] | None = None) -> dict:
        """Move all motors to a named point, then let the caller confirm it.

        Every motor command waits only for its immediate CAN acknowledgement;
        it does not wait for that motor to finish moving before sending the
        next command. This keeps one coordinated pose stage while avoiding the
        Waveshare serial adapter's unreliable back-to-back no-wait burst, which
        can route one target to the following motor.

        Args:
            point_name: Name of the point in point_config.json
            current_positions: Optional dict {label: current_angle} for safety check
        """
        angles = self.point_config.get(point_name)
        if not angles:
            return {"error": f"{point_name} position not found in point_config.json"}

        missing_targets = [label for label in MOTORS if label not in angles]
        if missing_targets:
            return {
                "error": (
                    f"{point_name} is missing motor targets: "
                    + ", ".join(missing_targets)
                )
            }

        # A named four-axis pose is all-or-none: read and validate every current
        # angle before dispatching any motion command.
        if current_positions is None:
            read_result = self.read_positions()
            current_positions = {
                label: float(value)
                for label, value in read_result.get("_updates", {}).items()
            }
        missing_positions = [
            label for label in MOTORS if label not in current_positions
        ]
        if missing_positions:
            return {
                "error": (
                    "Required motors not online: "
                    + ", ".join(missing_positions)
                )
            }

        # Safety check: verify no motor moves more than MAX_MOVE_DEGREES
        for label in MOTORS:
            current = current_positions[label]
            target = angles[label]
            diff = abs(target - current)
            if diff > MAX_MOVE_DEGREES:
                return {
                    "safety": (
                        f"BLOCKED. {label} would move {diff:.1f}° "
                        f"(current={current:.2f} -> target={target:.2f}). "
                        f"Max allowed is {MAX_MOVE_DEGREES}°."
                    )
                }

        # Safety check: verify target angles are within per-motor limits
        limit_error = self._check_angle_limits(angles)
        if limit_error:
            return limit_error

        speed = HOME_SPEED_DPS if point_name == "HOME" else MAX_SPEED_DPS
        result = {}
        updates = {}
        for label, motor_id in MOTORS.items():
            target = angles[label]
            try:
                response = self.service.absolute_position_control(
                    motor_id, target, speed
                )
                result[f"{label} TX"] = response.get("tx", "")
                result[label] = (
                    f"OK acknowledged {point_name} target={target:.2f} "
                    f"speed={speed} raw={response['raw']}"
                )
                updates[label] = f"{target:.2f}"
            except Exception as exc:
                result[f"{label} TX"] = self.service.last_tx
                result[label] = f"ERROR {point_name} target={target:.2f} {type(exc).__name__}: {exc}"
                result["_stop_after_error"] = self.stop_all()
                for pending_label in MOTORS:
                    if pending_label not in updates and pending_label != label:
                        result.setdefault(
                            pending_label,
                            "SKIP pose movement aborted after CAN error",
                        )
                break
        result["_updates"] = updates
        return result

    def run_targets(self, targets: dict[str, float]) -> dict:
        """Send absolute position commands to specific motors.

        Args:
            targets: dict {label: target_angle}
        """
        sent = {}
        for label, target in targets.items():
            # Per-motor angle limit check
            limits = MOTOR_ANGLE_LIMITS.get(label)
            if limits is not None:
                lo, hi = limits
                if target < lo or target > hi:
                    sent[label] = (
                        f"BLOCKED. target={target:.2f}° is outside limit "
                        f"[{lo:.1f}°, {hi:.1f}°]"
                    )
                    continue
            motor_id = MOTORS[label]
            try:
                response = self.service.absolute_position_control(motor_id, target, MAX_SPEED_DPS)
                sent[f"{label} TX"] = response.get("tx", "")
                sent[label] = (
                    f"OK target={target:.2f} degree={response['degree']} "
                    f"speed={response['speed_dps']} raw={response['raw']}"
                )
            except Exception as exc:
                sent[f"{label} TX"] = self.service.last_tx
                sent[label] = f"ERROR target={target:.2f} {type(exc).__name__}: {exc}"
        return sent

    def stop_all(self) -> dict:
        result = {}
        for label, motor_id in MOTORS.items():
            try:
                response = self.service.stop_motor(motor_id)
                result[f"{label} TX"] = response.get("tx", "")
                result[label] = f"OK stopped raw={response['raw']}"
            except Exception as exc:
                result[f"{label} TX"] = self.service.last_tx
                result[label] = f"ERROR {type(exc).__name__}: {exc}"
        return result

    def shutdown_all(self) -> dict:
        result = {}
        for label, motor_id in MOTORS.items():
            try:
                response = self.service.shutdown_motor(motor_id)
                result[f"{label} TX"] = response.get("tx", "")
                result[label] = f"OK shutdown raw={response['raw']}"
            except Exception as exc:
                result[f"{label} TX"] = self.service.last_tx
                result[label] = f"ERROR {type(exc).__name__}: {exc}"
        return result

    def save_position(self, point_name: str, angles: dict[str, float]) -> None:
        """Save angles to point_config.json and update in-memory config."""
        self.point_config[point_name] = angles
        if point_name == "HOME":
            self.home_angles = angles

        file_data = {"points": []}
        for name, point_angles in self.point_config.items():
            file_angles = {}
            for lbl, val in point_angles.items():
                file_key = lbl.replace(" ", "")  # "ID 142" -> "ID142"
                file_angles[file_key] = val
            file_data["points"].append({"name": name, "angles": file_angles})

        with POINT_CONFIG_PATH.open("w", encoding="utf-8") as f:
            json.dump(file_data, f, indent=2, ensure_ascii=False)
            f.write("\n")

    def calc_vacuum_angles(self, direction: str, offset_cm: float) -> dict:
        """Calculate vacuum arm angles based on direction and offset.

        Args:
            direction: "L" or "R"
            offset_cm: horizontal offset in cm (positive = further out)

        Returns:
            dict with big_arm_target, small_arm_target, or error info
        """
        max_offset = VACUUM_SMALL_ARM_CM
        if abs(offset_cm) > max_offset:
            return {
                "error": (
                    f"Offset {offset_cm:.1f}cm exceeds small arm reach "
                    f"({max_offset}cm). Clamped."
                ),
                "offset_cm": min(abs(offset_cm), max_offset),
            }

        view_point = "LView" if direction == "L" else "RView"
        view_angles = self.point_config.get(view_point)
        if not view_angles:
            return {"error": f"{view_point} not found in point_config.json"}

        home = self.point_config.get("HOME")
        if not home:
            return {"error": "HOME not found in point_config.json"}

        # Big arm: rotate same amount as camera big arm
        camera_big_home = home.get(CAMERA_BIG, 0)
        camera_big_view = view_angles.get(CAMERA_BIG, 0)
        camera_rotation = camera_big_view - camera_big_home

        vacuum_big_home = home.get(VACUUM_BIG, 0)
        vacuum_big_target = vacuum_big_home + camera_rotation

        # Small arm: compensate offset using arcsin
        vacuum_small_home = home.get(VACUUM_SMALL, 0)
        if abs(offset_cm) < 0.01:
            small_arm_delta = 0.0
        else:
            small_arm_delta = math.degrees(math.asin(offset_cm / VACUUM_SMALL_ARM_CM))

        vacuum_small_target = vacuum_small_home + small_arm_delta

        return {
            "direction": direction,
            "offset_cm": offset_cm,
            "camera_rotation": camera_rotation,
            "big_arm_home": vacuum_big_home,
            "big_arm_target": round(vacuum_big_target, 2),
            "small_arm_home": vacuum_small_home,
            "small_arm_delta": round(small_arm_delta, 2),
            "small_arm_target": round(vacuum_small_target, 2),
        }

    def move_vacuum_dynamic(
        self, direction: str, offset_cm: float, current_positions: dict[str, float]
    ) -> dict:
        """Calculate and execute vacuum arm dynamic movement.

        Returns:
            dict with results or error/safety info
        """
        calc = self.calc_vacuum_angles(direction, offset_cm)
        if "error" in calc:
            return calc

        big_target = calc["big_arm_target"]
        small_target = calc["small_arm_target"]

        big_current = current_positions.get(VACUUM_BIG, 0)
        small_current = current_positions.get(VACUUM_SMALL, 0)

        big_diff = abs(big_target - big_current)
        small_diff = abs(small_target - small_current)
        if big_diff > MAX_MOVE_DEGREES or small_diff > MAX_MOVE_DEGREES:
            return {
                "safety": (
                    f"BLOCKED. Move too large: "
                    f"big={big_diff:.1f}° small={small_diff:.1f}° "
                    f"(max={MAX_MOVE_DEGREES}°)"
                )
            }

        # Send commands
        targets = {VACUUM_BIG: big_target, VACUUM_SMALL: small_target}
        result = self.run_targets(targets)
        result["_calc"] = calc
        result["_updates"] = {VACUUM_BIG: f"{big_target:.2f}", VACUUM_SMALL: f"{small_target:.2f}"}
        return result
