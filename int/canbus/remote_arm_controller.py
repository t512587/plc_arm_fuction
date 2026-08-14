"""Drop-in ArmController replacement that talks to canbus_daemon.py over HTTP.

canbus_daemon.py owns the one live CAN connection for the whole session.
RemoteArmController/RemoteMotorService present the same public interface as
ArmController/MotorService (arm_controller.py, motor_service.py) so existing
callers (d435_control.py) do not need to change their control logic — only
the object construction site changes.
"""
from __future__ import annotations

from typing import Any

import requests

from arm_config import load_point_config


class RemoteArmControllerError(RuntimeError):
    pass


class RemoteMotorService:
    def __init__(self, base_url: str, timeout_seconds: float) -> None:
        self._base_url = base_url
        self._timeout_seconds = timeout_seconds
        # `hasattr(controller.service, "_serial") and controller.service._serial`
        # is a truthiness check used by d435_control.py before calling
        # reset_input_buffer(); pointing it at self keeps that check working
        # unchanged while routing the actual call through the daemon.
        self._serial = self

    def _post(self, path: str, json_body: dict[str, Any] | None = None) -> dict[str, Any]:
        try:
            response = requests.post(
                f"{self._base_url}{path}", json=json_body, timeout=self._timeout_seconds
            )
            response.raise_for_status()
            return response.json()
        except requests.RequestException as exc:
            raise RemoteArmControllerError(
                f"找不到CAN常駐服務，請先啟動 canbus_daemon.py (url={self._base_url}): {exc}"
            ) from exc

    def absolute_position_control(
        self, motor_id: int, angle_degrees: float, max_speed_dps: int = 500
    ) -> dict[str, Any]:
        return self._post(
            "/absolute_position",
            {
                "motor_id": motor_id,
                "angle_degrees": angle_degrees,
                "max_speed_dps": max_speed_dps,
            },
        )

    def reset_input_buffer(self) -> None:
        self._post("/reset_input_buffer")


class RemoteArmController:
    """Same public surface as canbus.arm_controller.ArmController."""

    def __init__(self, base_url: str, timeout_seconds: float = 10.0) -> None:
        self._base_url = base_url.rstrip("/")
        self._timeout_seconds = timeout_seconds
        self.service = RemoteMotorService(self._base_url, timeout_seconds)
        # Points are static local data; reading them doesn't need the CAN
        # connection, so this stays a local file read instead of a round trip.
        self.point_config = load_point_config()
        self.home_angles = self.point_config.get("HOME", {})

    def connect(self) -> str:
        try:
            response = requests.get(f"{self._base_url}/health", timeout=self._timeout_seconds)
            response.raise_for_status()
            data = response.json()
        except requests.RequestException as exc:
            raise RemoteArmControllerError(
                f"找不到CAN常駐服務，請先啟動 canbus_daemon.py (url={self._base_url}): {exc}"
            ) from exc
        if not data.get("connected"):
            raise RemoteArmControllerError(
                f"CAN常駐服務尚未連上CAN bus (url={self._base_url})"
            )
        return f"Using persistent CAN connection via {self._base_url}."

    def disconnect(self) -> str:
        # The daemon owns the connection for the whole session; a single
        # caller finishing its work must not tear it down for everyone else.
        return "No-op: canbus_daemon.py keeps the CAN connection open."

    @property
    def is_connected(self) -> bool:
        try:
            response = requests.get(f"{self._base_url}/health", timeout=self._timeout_seconds)
            response.raise_for_status()
            return bool(response.json().get("connected"))
        except requests.RequestException:
            return False

    def read_positions(self) -> dict[str, Any]:
        try:
            response = requests.get(f"{self._base_url}/positions", timeout=self._timeout_seconds)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as exc:
            raise RemoteArmControllerError(
                f"找不到CAN常駐服務，請先啟動 canbus_daemon.py (url={self._base_url}): {exc}"
            ) from exc

    def go_to_point(
        self, point_name: str, current_positions: dict[str, float] | None = None
    ) -> dict[str, Any]:
        return self.service._post(
            "/go_to_point",
            {"point_name": point_name, "current_positions": current_positions},
        )

    def run_targets(self, targets: dict[str, float]) -> dict[str, Any]:
        return self.service._post("/run_targets", {"targets": targets})

    def stop_all(self) -> dict[str, Any]:
        return self.service._post("/stop_all")

    def shutdown_all(self) -> dict[str, Any]:
        return self.service._post("/shutdown_all")
