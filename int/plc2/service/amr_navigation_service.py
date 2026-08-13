from __future__ import annotations

import logging
from typing import Any

from amr_cmd.action import build_action_registry, goto_amr_position
from amr_cmd.base.service_amr import ServiceAmr


logger = logging.getLogger("plc_backend")


class AmrNavigationError(RuntimeError):
    pass


class AmrNavigationService:
    """AMR-only adapter used by the PLC2 TXT task API.

    Delegates all AMR behavior to amr_cmd.action so this layer and the
    action.txt runner never drift into two different implementations of the
    same navigation/arrival logic.
    """

    def __init__(self, service: ServiceAmr | Any | None = None) -> None:
        self.service = service or ServiceAmr()
        self._actions = build_action_registry(self.service)
        self.map_name: str | None = None

    def connect(self) -> dict[str, Any]:
        result = self._actions["AMRconnect"]()
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
        return self._actions["AMRsetobs"](precision_xy, precision_yaw)

    def read_map(self, map_name: str) -> dict[str, Any]:
        result = self._actions["AMRread_map"](map_name)
        detail = result.get("detail") if isinstance(result, dict) else None
        if not isinstance(detail, dict):
            raise AmrNavigationError("AMR map response has no detail object")
        self.map_name = str(map_name).strip()
        return {
            "success": True,
            "map_name": self.map_name,
        }

    def goto(
        self,
        position_id: str,
        is_reverse: bool,
        nav_type: int,
        *,
        poll_interval: float = 0.35,
    ) -> dict[str, Any]:
        if self.map_name is None:
            raise AmrNavigationError("Read an AMR map before amr_goto")
        normalized = str(position_id).strip()

        result = goto_amr_position(
            self.service,
            normalized,
            is_reverse,
            nav_type,
            poll_interval=poll_interval,
        )

        if result.get("success") is not True:
            raise AmrNavigationError(str(result.get("message")))

        logger.info(
            "AMR navigation arrived position_id=%s fsm=%r",
            normalized,
            result.get("fsm"),
        )
        return {
            "success": True,
            "arrived": True,
            "map_name": self.map_name,
            "position_id": normalized,
            "response": result.get("navigation"),
            "fsm": result.get("fsm"),
            "robot_data": result.get("robot_data"),
        }

    def cancel_task(self) -> dict[str, Any]:
        """Cancel the AMR's currently running task, without disconnecting."""
        return self._actions["AMRcancel_task"]()

    def confirm_status(self) -> dict[str, Any]:
        """Acknowledge and clear a terminal succeeded/failed AMR FSM state."""
        return self._actions["AMRconfirm_status"]()

    def disconnect(self) -> dict[str, Any]:
        result = self._actions["AMRdisconnect"]()
        self.map_name = None
        return result

    def cancel_and_disconnect(self) -> dict[str, Any]:
        errors: list[str] = []
        cancel_result: Any = None
        disconnect_result: Any = None
        try:
            cancel_result = self._actions["AMRcancel_task"]()
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
