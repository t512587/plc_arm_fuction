from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import time
from typing import Any

from amr_cmd.base.service_amr import ServiceAmr


logger = logging.getLogger("plc_backend")


class AmrNavigationError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class CachedWaypoint:
    deploy_uid: str
    waypoint_uid: str
    name: str
    x: float | None
    y: float | None


class AmrNavigationService:
    """AMR-only adapter used by the PLC2 TXT task API."""

    def __init__(self, service: ServiceAmr | Any | None = None) -> None:
        self.service = service or ServiceAmr()
        self.map_name: str | None = None
        self.waypoints: dict[str, CachedWaypoint] = {}

    def connect(self) -> dict[str, Any]:
        result = self.service.connect()
        if not isinstance(result, dict) or result.get("success") is not True:
            message = result.get("message") if isinstance(result, dict) else result
            raise AmrNavigationError(f"AMR connection failed: {message}")
        return result

    def set_navigation_precision(
        self,
        precision_xy: float,
        precision_yaw: float,
    ) -> dict[str, Any]:
        """Set arrival tolerances used by subsequent waypoint navigation."""
        xy = float(precision_xy)
        yaw = float(precision_yaw)
        if xy <= 0 or yaw <= 0:
            raise AmrNavigationError("AMR navigation precision must be greater than 0")
        self.service.config.precision_xy = xy
        self.service.config.precision_yaw = yaw
        return {
            "success": True,
            "precision_xy": xy,
            "precision_yaw": yaw,
        }

    def read_map(self, map_name: str) -> dict[str, Any]:
        result = self.service.read_map(map_name)
        detail = result.get("detail") if isinstance(result, dict) else None
        if not isinstance(detail, dict):
            raise AmrNavigationError("AMR map response has no detail object")
        parsed = self.service.parse_waypoints(detail)
        waypoints: dict[str, CachedWaypoint] = {}
        for waypoint in parsed:
            uid = str(waypoint.wp_uid or "").strip()
            if uid:
                waypoints[uid] = CachedWaypoint(
                    deploy_uid=str(waypoint.dp_uid or "").strip(),
                    waypoint_uid=uid,
                    name=str(waypoint.name or ""),
                    x=waypoint.x,
                    y=waypoint.y,
                )
        if not waypoints:
            raise AmrNavigationError(f"Map {map_name!r} has no waypoints")
        self.map_name = str(map_name).strip()
        self.waypoints = waypoints
        return {
            "success": True,
            "map_name": self.map_name,
            "waypoint_count": len(waypoints),
        }

    def goto(
        self,
        position_id: str,
        response_timeout_seconds: float,
        *,
        arrival_xy: float = 0.10,
        hold_seconds: float = 0.8,
        poll_interval: float = 0.35,
    ) -> dict[str, Any]:
        if self.map_name is None:
            raise AmrNavigationError("Read an AMR map before amr_goto")
        normalized = str(position_id).strip()
        waypoint = self.waypoints.get(normalized)
        if waypoint is None:
            raise AmrNavigationError(
                f"Waypoint {normalized!r} is not in map {self.map_name!r}"
            )
        if not waypoint.deploy_uid:
            raise AmrNavigationError(
                f"Waypoint {normalized!r} has no deployment UID"
            )
        timeout = float(response_timeout_seconds)
        arrival = float(arrival_xy)
        hold = float(hold_seconds)
        poll = float(poll_interval)
        if (
            not all(math.isfinite(value) for value in (timeout, arrival, hold, poll))
            or timeout <= 0
            or arrival <= 0
            or hold < 0
            or poll <= 0
        ):
            raise AmrNavigationError("Invalid AMR arrival wait settings")

        self._prepare_for_navigation()

        original_timeout = self.service.config.timeout_seconds
        self.service.config.timeout_seconds = timeout
        try:
            result = self.service.go_to_waypoint(
                waypoint.deploy_uid,
                waypoint.waypoint_uid,
            )
        finally:
            self.service.config.timeout_seconds = original_timeout

        if not isinstance(result, dict):
            raise AmrNavigationError(
                f"AMR goto returned a non-object response: {result!r}"
            )
        code = result.get("code")
        msg = result.get("msg")
        try:
            code_ok = int(code) == 0
        except (TypeError, ValueError):
            code_ok = False
        if not code_ok or msg != "success":
            raise AmrNavigationError(
                f"AMR did not accept navigation: code={code!r}, msg={msg!r}"
            )
        logger.info(
            "AMR navigation accepted position_id=%s code=%r msg=%r; waiting for arrival",
            normalized,
            code,
            msg,
        )

        arrival_result = self._wait_until_waypoint_arrived(
            waypoint,
            timeout_seconds=timeout,
            arrival_xy=arrival,
            hold_seconds=hold,
            poll_interval=poll,
        )
        return {
            "success": True,
            "arrived": True,
            "map_name": self.map_name,
            "position_id": normalized,
            "response": result,
            "arrival": arrival_result,
        }

    def _wait_until_waypoint_arrived(
        self,
        waypoint: CachedWaypoint,
        *,
        timeout_seconds: float,
        arrival_xy: float,
        hold_seconds: float,
        poll_interval: float,
    ) -> dict[str, Any]:
        strong_done_states = {
            "succeeded", "successed", "sucessed", "success",
            "finish", "finished", "complete", "completed",
        }
        weak_done_states = {"idle", ""}
        moving_states = {"moving", "running", "navigating", "navigation", "go_to", "goto"}
        failed_states = {
            "failed", "fail", "failure", "abort", "aborted",
            "cancel", "cancelled", "canceled",
        }
        deadline = time.monotonic() + timeout_seconds
        sent_at = time.monotonic()
        saw_motion = False
        stable_since: float | None = None
        last_log = 0.0
        last_fsm = ""
        last_distance: float | None = None

        while time.monotonic() < deadline:
            try:
                data = self.service.get_robot_data_once()
            except Exception as exc:  # noqa: BLE001
                raise AmrNavigationError(f"AMR robot_data read failed: {exc}") from exc
            if not isinstance(data, dict):
                raise AmrNavigationError(f"AMR robot_data returned non-object: {data!r}")

            fsm = self._normalize_fsm(data.get("fsm"))
            position = self._robot_position(data)
            distance: float | None = None
            if position is not None and waypoint.x is not None and waypoint.y is not None:
                distance = math.hypot(position[0] - waypoint.x, position[1] - waypoint.y)

            now = time.monotonic()
            if now - last_log >= 1.5:
                logger.info(
                    "AMR waiting position_id=%s fsm=%r distance_m=%s arrival_xy=%s",
                    waypoint.waypoint_uid,
                    fsm or "-",
                    "-" if distance is None else f"{distance:.3f}",
                    arrival_xy,
                )
                last_log = now

            if fsm in failed_states:
                raise AmrNavigationError(
                    f"AMR navigation failed: position_id={waypoint.waypoint_uid!r}, fsm={fsm!r}"
                )
            if fsm in moving_states:
                saw_motion = True
                stable_since = None

            close_enough = distance is not None and distance <= arrival_xy
            backend_done = fsm in strong_done_states and now - sent_at >= 0.5
            idle_after_motion = fsm in weak_done_states and saw_motion
            if backend_done or idle_after_motion or close_enough:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= max(0.2, hold_seconds):
                    reason = (
                        "backend_fsm"
                        if backend_done or idle_after_motion
                        else "position_distance"
                    )
                    logger.info(
                        "AMR physical arrival confirmed position_id=%s reason=%s "
                        "fsm=%r distance_m=%s hold_seconds=%s",
                        waypoint.waypoint_uid,
                        reason,
                        fsm or "-",
                        "-" if distance is None else f"{distance:.3f}",
                        hold_seconds,
                    )
                    return {
                        "confirmed": True,
                        "reason": reason,
                        "fsm": fsm,
                        "distance_m": distance,
                        "arrival_xy": arrival_xy,
                        "hold_seconds": hold_seconds,
                    }
            else:
                stable_since = None

            last_fsm = fsm
            last_distance = distance
            time.sleep(poll_interval)

        raise AmrNavigationError(
            "AMR arrival timeout: "
            f"position_id={waypoint.waypoint_uid!r}, fsm={last_fsm!r}, "
            f"distance_m={last_distance!r}, timeout_seconds={timeout_seconds}"
        )

    def _prepare_for_navigation(self) -> None:
        """Clear a terminal FSM left by the preceding navigation command."""
        try:
            data = self.service.get_robot_data_once()
        except Exception as exc:  # noqa: BLE001
            raise AmrNavigationError(
                f"AMR state read failed before navigation: {exc}"
            ) from exc
        if not isinstance(data, dict):
            raise AmrNavigationError(
                f"AMR state before navigation is not an object: {data!r}"
            )
        fsm = self._normalize_fsm(data.get("fsm"))
        if fsm in {"succeeded", "failed"}:
            result = self.service.confirm_status()
            logger.info("AMR cleared previous terminal state fsm=%r result=%r", fsm, result)
            time.sleep(0.4)
        elif fsm in {"moving", "running", "navigating", "navigation", "go_to", "goto"}:
            raise AmrNavigationError(
                f"AMR is already moving before navigation: fsm={fsm!r}"
            )

    @staticmethod
    def _normalize_fsm(value: Any) -> str:
        fsm = str(value or "").strip().lower()
        if fsm in {"successed", "sucessed", "succeed", "success"}:
            return "succeeded"
        if fsm in {"fail", "failure"}:
            return "failed"
        return fsm

    @staticmethod
    def _robot_position(data: dict[str, Any]) -> tuple[float, float] | None:
        pose = data.get("pose")
        if not isinstance(pose, dict):
            return None
        position = pose.get("position")
        if not isinstance(position, dict):
            return None
        try:
            return float(position["x"]), float(position["y"])
        except (KeyError, TypeError, ValueError):
            return None

    def disconnect(self) -> dict[str, Any]:
        result = self.service.disconnect()
        self.map_name = None
        self.waypoints = {}
        return result

    def cancel_and_disconnect(self) -> dict[str, Any]:
        errors: list[str] = []
        cancel_result: Any = None
        disconnect_result: Any = None
        try:
            cancel_result = self.service.cancel_task()
        except Exception as exc:  # noqa: BLE001
            errors.append(f"cancel_task: {exc}")
        try:
            disconnect_result = self.disconnect()
        except Exception as exc:  # noqa: BLE001
            errors.append(f"disconnect: {exc}")
        return {
            "success": not errors,
            "cancelled": True,
            "cancel_result": cancel_result,
            "disconnect_result": disconnect_result,
            "errors": errors,
        }
