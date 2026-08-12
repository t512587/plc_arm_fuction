from __future__ import annotations

import json
import base64
import hashlib
import os
import socket
import struct
import tempfile
import time
import zlib
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlparse, urlencode, quote
from urllib import error, request

try:
    from .config_amr import DEFAULT_AMR_CONFIG, ServiceAmrConfig
except ImportError:  # pragma: no cover
    from config_amr import DEFAULT_AMR_CONFIG, ServiceAmrConfig

HttpGetJson = Callable[[str, dict[str, str], float], Any]
HttpPostJson = Callable[[str, dict[str, str], dict[str, Any], float], dict[str, Any]]
HeartbeatChecker = Callable[[str, int, float], bool]
RobotDataReceiver = Callable[[str, dict[str, str], float], dict[str, Any]]
BatteryDataReceiver = Callable[[str, int, float], dict[str, Any]]


@dataclass(slots=True)
class AmrHeartbeatStatus:
    connected: bool
    message: str


@dataclass(slots=True)
class AmrMapInfo:
    id: int
    name: str


@dataclass(slots=True)
class AmrParsedMap:
    height: int
    width: int
    pixels: list[list[int]]
    png_path: Path


@dataclass(slots=True)
class AmrWaypoint:
    dp_uid: str
    wp_uid: str
    name: str
    orientaion: Any
    rotation: Any
    x: float | None
    y: float | None


@dataclass(slots=True)
class AmrNetworkInterface:
    connection: str
    device: str
    state: str
    type: str


@dataclass(slots=True)
class AmrServiceSnapshot:
    heartbeat: AmrHeartbeatStatus
    robot_data: dict[str, Any]
    battery_data: dict[str, Any]
    maps: list[AmrMapInfo]
    error: str


class ServiceAmr:
    """AMR NEST API 資訊收集與控制服務。"""

    def __init__(
        self,
        config: ServiceAmrConfig | None = None,
        http_get_json: HttpGetJson | None = None,
        http_post_json: HttpPostJson | None = None,
        heartbeat_checker: HeartbeatChecker | None = None,
        robot_data_receiver: RobotDataReceiver | None = None,
        battery_data_receiver: BatteryDataReceiver | None = None,
        log_dir: Path | None = None,
    ) -> None:
        self.config = config or DEFAULT_AMR_CONFIG
        self._http_get_json_transport = http_get_json or self._default_http_get_json
        self._http_post_json_transport = http_post_json or self._default_http_post_json
        self._http_get_json = self._logged_http_get_json
        self._http_post_json = self._logged_http_post_json
        self._http_put_json = self._logged_http_put_json
        self._http_delete_json = self._logged_http_delete_json
        self._heartbeat_checker = heartbeat_checker or self._default_heartbeat_checker
        self._robot_data_receiver = robot_data_receiver or self._default_robot_data_receiver
        self._battery_data_receiver = battery_data_receiver or self._default_battery_data_receiver
        self.log_dir = log_dir or Path(__file__).with_name("log")
        self.last_error = ""
        self._connected = False

    def connect(self) -> dict[str, Any]:
        """Verify that the AMR control box is reachable."""
        heartbeat = self.check_heartbeat()
        self._connected = heartbeat.connected
        return {
            "success": heartbeat.connected,
            "connected": heartbeat.connected,
            "message": heartbeat.message,
        }

    def disconnect(self) -> dict[str, Any]:
        """Clear the action-side connection state (the API is stateless)."""
        self._connected = False
        return {"success": True, "connected": False, "message": "AMR 已斷線"}

    def read_map(self, map_name: str) -> dict[str, Any]:
        """Read map details by an exact map name."""
        normalized = str(map_name).strip()
        if not normalized:
            raise ValueError("map_name cannot be empty")
        maps = self.get_maps()
        match = next((item for item in maps if item.name == normalized), None)
        if match is None:
            available = ", ".join(item.name for item in maps) or "(無)"
            raise ValueError(f"找不到地圖名稱 {normalized!r}；可用地圖: {available}")
        return {
            "success": True,
            "map": {"id": match.id, "name": match.name},
            "detail": self.get_map_data_with_detail_ros(match.id),
        }

    def goto_position(self, position_id: str) -> dict[str, Any]:
        """Find a waypoint UID and submit a navigation request."""
        normalized = str(position_id).strip()
        if not normalized:
            raise ValueError("position_id cannot be empty")
        waypoint = self._find_waypoint_by_uid(normalized)
        if waypoint is None:
            raise ValueError(f"找不到位置 ID: {normalized}")
        return self.go_to_waypoint(waypoint.dp_uid, waypoint.wp_uid)

    @property
    def api_base_url(self) -> str:
        return f"http://{self.config.amr_IP}:{self.config.amr_api_port}"

    @property
    def robot_data_url(self) -> str:
        return f"ws://{self.config.amr_IP}:{self.config.robot_data_ws_port}/robot_data"

    @property
    def robot_control_base_url(self) -> str:
        return f"http://{self.config.amr_IP}:{self.config.robot_data_ws_port}"

    @property
    def path_plan_url(self) -> str:
        return f"ws://{self.config.amr_IP}:{self.config.robot_data_ws_port}/path_plan"

    @property
    def velocity_control_url(self) -> str:
        return f"ws://{self.config.amr_IP}:{self.config.robot_data_ws_port}/velocity_control"

    @property
    def map_stream_url(self) -> str:
        return f"ws://{self.config.amr_IP}:{self.config.robot_data_ws_port}/map"

    @property
    def scan_url(self) -> str:
        return f"ws://{self.config.amr_IP}:{self.config.robot_data_ws_port}/scan"

    @property
    def sound_base_url(self) -> str:
        return f"{self.api_base_url}/sound"

    @property
    def schedule_base_url(self) -> str:
        return f"{self.api_base_url}/schedule"

    @property
    def coverage_base_url(self) -> str:
        return f"http://{self.config.amr_IP}:{getattr(self.config, 'coverage_api_port', 1235)}"

    def headers(self) -> dict[str, str]:
        return {"x-api-key": self.config.api_key}

    def check_heartbeat(self) -> AmrHeartbeatStatus:
        try:
            connected = self._heartbeat_checker(
                self.config.amr_IP,
                self.config.amr_socket_port,
                self.config.timeout_seconds,
            )
            message = "控制盒心跳正常" if connected else "未收到控制盒心跳"
            return AmrHeartbeatStatus(connected=connected, message=message)
        except Exception as exc:
            self.last_error = str(exc)
            return AmrHeartbeatStatus(connected=False, message=f"控制盒心跳錯誤: {exc}")

    def get_robot_data_once(self) -> dict[str, Any]:
        try:
            return self._robot_data_receiver(self.robot_data_url, self.headers(), self.config.timeout_seconds)
        except Exception as exc:
            self.last_error = str(exc)
            raise RuntimeError(f"獲取機器人資料失敗: {exc}") from exc

    def get_battery_data_once(self) -> dict[str, Any]:
        try:
            return self._battery_data_receiver(
                self.config.amr_IP,
                self.config.amr_mqtt_port,
                self.config.timeout_seconds,
            )
        except Exception as exc:
            self.last_error = str(exc)
            raise RuntimeError(f"獲取電池資料失敗: {exc}") from exc

    def get_maps(self) -> list[AmrMapInfo]:
        errors: list[str] = []
        for endpoint in ("/deploy/getMaps", "/map/getMapList"):
            try:
                data = self._http_get_json(
                    f"{self.api_base_url}{endpoint}",
                    self.headers(),
                    self.config.timeout_seconds,
                )
                return self._parse_maps_response(data)
            except Exception as exc:
                errors.append(f"{endpoint}: {exc}")
        raise RuntimeError(f"獲取地圖列表失敗: {'; '.join(errors)}")

    def _parse_maps_response(self, data: Any) -> list[AmrMapInfo]:
        if isinstance(data, dict):
            if int(data.get("code", 0)) != 0:
                raise RuntimeError(str(data.get("message", data)))
            items = data.get("data", [])
        elif isinstance(data, list):
            items = data
        else:
            raise RuntimeError(f"回應格式錯誤: {data}")

        if not isinstance(items, list):
            raise RuntimeError(f"地圖列表格式錯誤: {items}")

        maps: list[AmrMapInfo] = []
        for item in items:
            if not isinstance(item, dict):
                continue
            maps.append(AmrMapInfo(id=int(item["id"]), name=str(item.get("name", ""))))
        return maps

    def get_map_data_with_detail_ros(self, map_id: int) -> dict[str, Any]:
        return self._http_get_json(
            f"{self.api_base_url}/deploy/getMapDataWithDetailRos/{int(map_id)}",
            self.headers(),
            self.config.timeout_seconds,
        )

    def confirm_status(self) -> dict[str, Any]:
        return self._http_get_json(
            f"{self.api_base_url}/deploy/confirmStatus",
            self.headers(),
            self.config.timeout_seconds,
        )

    def get_action_list(self) -> dict[str, Any]:
        return self._http_get_json(
            f"{self.api_base_url}/deploy/getAllActions",
            self.headers(),
            self.config.timeout_seconds,
        )

    def execute_action(self, action_id: int) -> dict[str, Any]:
        return self._http_get_json(
            f"{self.api_base_url}/deploy/executeAction/{int(action_id)}",
            self.headers(),
            self.config.timeout_seconds,
        )

    def get_Action_list(self) -> dict[str, Any]:
        return self.get_action_list()

    def exe_Action(self, action_id: int) -> dict[str, Any]:
        return self.execute_action(action_id)

    def list_network_interfaces(self) -> list[AmrNetworkInterface]:
        data = self._http_get_json(
            f"{self.api_base_url}/system/listNetworkInterfaces",
            self.headers(),
            self.config.timeout_seconds,
        )
        if int(data.get("code", 1)) != 0:
            raise RuntimeError(f"獲取網路介面清單失敗: {data.get('message', data)}")
        interfaces: list[AmrNetworkInterface] = []
        for item in data.get("data", []):
            interfaces.append(
                AmrNetworkInterface(
                    connection=str(item.get("CONNECTION", "")),
                    device=str(item.get("DEVICE", "")),
                    state=str(item.get("STATE", "")),
                    type=str(item.get("TYPE", "")),
                )
            )
        return interfaces

    def configure_network_interface(
        self,
        interface: str,
        ip_address: str,
        gateway: str,
        dns: str,
        subnet_mask: int,
    ) -> dict[str, Any]:
        payload = {
            "interface": interface,
            "ip_address": ip_address,
            "gateway": gateway,
            "dns": dns,
            "subnet_mask": int(subnet_mask),
        }
        return self._http_post_json(
            f"{self.api_base_url}/system/configureNetworkInterface",
            self.headers(),
            payload,
            self.config.timeout_seconds,
        )

    def parse_map_data(self, map_detail: dict[str, Any], output_path: Path | None = None) -> AmrParsedMap:
        map_payload = self._get_map_payload(map_detail)
        info = map_payload.get("info", {})
        height = int(info["height"])
        width = int(info["width"])
        raw_data = list(map_payload["data"])
        expected_size = height * width
        if len(raw_data) != expected_size:
            raise ValueError(f"地圖 data 長度錯誤: got {len(raw_data)}, expected {expected_size}")

        flat_pixels = [self._map_cell_to_pixel(value) for value in raw_data]
        source_rows = [flat_pixels[row_start:row_start + width] for row_start in range(0, expected_size, width)]
        pixels = list(reversed(source_rows))
        rendered_pixels = [pixel for row in pixels for pixel in row]
        target_path = output_path or Path(tempfile.gettempdir()) / "temp.png"
        target_path.parent.mkdir(parents=True, exist_ok=True)
        self._write_grayscale_png(target_path, width, height, rendered_pixels)
        return AmrParsedMap(height=height, width=width, pixels=pixels, png_path=target_path)

    def parse_waypoints(self, map_detail: dict[str, Any]) -> list[AmrWaypoint]:
        waypoints: list[AmrWaypoint] = []
        for item in self._iter_waypoint_items(map_detail):
            if not isinstance(item, dict):
                continue
            x_value = item.get("x", item.get("center", {}).get("x") if isinstance(item.get("center"), dict) else None)
            y_value = item.get("y", item.get("center", {}).get("y") if isinstance(item.get("center"), dict) else None)
            waypoints.append(
                AmrWaypoint(
                    dp_uid=str(item.get("dp_uid", item.get("uid", ""))),
                    wp_uid=str(item.get("wp_uid", item.get("uid", ""))),
                    name=str(item.get("name", "")),
                    orientaion=item.get("orientaion", item.get("orientation")),
                    rotation=item.get("rotation"),
                    x=self._optional_float(x_value),
                    y=self._optional_float(y_value),
                )
            )
        return waypoints

    def go_to_waypoint(
        self,
        deploy_uid: str,
        wp_uid: str,
        *,
        precision_xy: float | None = None,
        precision_yaw: float | None = None,
        is_reverse: bool = False,
        nav_type: int = 2,
    ) -> dict[str, Any]:
        payload = {
            "deploy_uid": deploy_uid,
            "wp_uid": wp_uid,
            "precision_xy": float(self.config.precision_xy if precision_xy is None else precision_xy),
            "precision_yaw": float(self.config.precision_yaw if precision_yaw is None else precision_yaw),
            "is_reverse": is_reverse,
            "nav_type": nav_type,
        }
        return self._http_post_json(
            f"{self.api_base_url}/deploy/goToWP",
            self.headers(),
            payload,
            self.config.timeout_seconds,
        )

    def relocateWithWP(self, deploy_uid: str, wp_uid: str) -> dict[str, Any]:
        payload = {
            "deploy_uid": deploy_uid,
            "wp_uid": wp_uid,
        }
        return self._http_post_json(
            f"{self.api_base_url}/deploy/relocateWithWP",
            self.headers(),
            payload,
            self.config.timeout_seconds,
        )

    def relocate_with_wp(self, deploy_uid: str, wp_uid: str) -> dict[str, Any]:
        return self.relocateWithWP(deploy_uid, wp_uid)

    def motion_control(self, direction: int, distance: float, speed: float) -> dict[str, Any]:
        payload = {
            "direction": int(direction),
            "distance": float(distance),
            "speed": float(speed),
        }
        return self._http_post_json(
            f"{self.robot_control_base_url}/motion_control",
            self.headers(),
            payload,
            self.config.timeout_seconds,
        )

    def cancel_task(self) -> dict[str, Any]:
        return self._http_get_json(
            f"{self.robot_control_base_url}/cancel_task",
            self.headers(),
            self.config.timeout_seconds,
        )

    def go_charge(
        self,
        *,
        action_id: int = 17,
        max_retries: int = 10,
        stuck_seconds: float = 5.0,
        poll_interval: float = 0.5,
        failed_delay_seconds: float = 5.0,
        confirm_delay_seconds: float = 3.0,
        idle_timeout_seconds: float = 30.0,
        operation_timeout_seconds: float = 120.0,
    ) -> dict[str, Any]:
        near_charge_point = str(getattr(self.config, "near_charge_point", "") or "").strip()
        if not near_charge_point:
            return {"go_charge": "failed", "message": "near_charge_point 未設定"}

        waypoint = self._find_waypoint(near_charge_point)
        if waypoint is None:
            return {"go_charge": "failed", "message": f"找不到充電點位: {near_charge_point}"}

        navigate_attempts = 0
        while navigate_attempts < max_retries:
            navigate_attempts += 1
            self.go_to_waypoint(waypoint.dp_uid, waypoint.wp_uid)
            nav_status = self._wait_for_charge_fsm(
                stuck_seconds=stuck_seconds,
                poll_interval=poll_interval,
                timeout_seconds=operation_timeout_seconds,
            )
            if nav_status == "succeeded":
                break
            if nav_status == "failed":
                self._sleep(failed_delay_seconds)
            self._recover_charge_flow(confirm_delay_seconds=confirm_delay_seconds, idle_timeout_seconds=idle_timeout_seconds, poll_interval=poll_interval)
        else:
            return {
                "go_charge": "failed",
                "message": "前往充電點失敗",
                "near_charge_point": near_charge_point,
                "navigate_attempts": navigate_attempts,
            }

        action_attempts = 0
        while action_attempts < max_retries:
            action_attempts += 1
            self.execute_action(action_id)
            action_status = self._wait_for_charge_fsm(
                stuck_seconds=stuck_seconds,
                poll_interval=poll_interval,
                timeout_seconds=operation_timeout_seconds,
            )
            if action_status == "succeeded":
                return {
                    "go_charge": "sucessed",
                    "message": "充電流程完成",
                    "near_charge_point": near_charge_point,
                    "navigate_attempts": navigate_attempts,
                    "action_attempts": action_attempts,
                }
            if action_status == "failed":
                self._sleep(failed_delay_seconds)
            self._recover_charge_flow(confirm_delay_seconds=confirm_delay_seconds, idle_timeout_seconds=idle_timeout_seconds, poll_interval=poll_interval)

        return {
            "go_charge": "failed",
            "message": "充電 action 失敗",
            "near_charge_point": near_charge_point,
            "navigate_attempts": navigate_attempts,
            "action_attempts": action_attempts,
        }

    def auto_cahrge_flow(
        self,
        *,
        action_id: int = 17,
        max_retries: int = 50,
        failed_retry_delay_seconds: float = 0.1,
        moving_poll_seconds: float = 1.0,
        status_callback: Callable[[dict[str, Any]], None] | None = None,
    ) -> dict[str, Any]:
        retry_count = 0
        self.execute_action(action_id)

        while True:
            robot_data = self.get_robot_data_once()
            fsm = self._normalized_fsm(robot_data.get("fsm"))
            status = {"retry_count": retry_count, "max_retries": max_retries, "fsm": fsm}
            if status_callback is not None:
                status_callback(status)

            if fsm == "succeeded":
                return {
                    "auto_cahrge_flow": "sucessed",
                    "message": "自動充電成功",
                    "retry_count": retry_count,
                    "fsm": fsm,
                }

            if fsm == "failed":
                self.confirm_status()
                self._sleep(failed_retry_delay_seconds)
                retry_count += 1
                if retry_count > max_retries:
                    return {
                        "auto_cahrge_flow": "failed",
                        "message": "自動充電失敗",
                        "retry_count": retry_count,
                        "fsm": fsm,
                    }
                self.execute_action(action_id)
                continue

            if fsm == "moving":
                self._sleep(moving_poll_seconds)
                continue

            self._sleep(moving_poll_seconds)

    def move_forward(self, distance: float = 0.2, speed: float = 0.2) -> dict[str, Any]:
        return self.motion_control(1, distance, speed)

    def move_backward(self, distance: float = 0.2, speed: float = 0.2) -> dict[str, Any]:
        return self.motion_control(2, distance, speed)

    def move_left(self, distance: float = 0.2, speed: float = 0.2) -> dict[str, Any]:
        return self.motion_control(3, distance, speed)

    def move_right(self, distance: float = 0.2, speed: float = 0.2) -> dict[str, Any]:
        return self.motion_control(4, distance, speed)

    def get_path_plan_once(self) -> Any:
        message = self._recv_websocket_text_once(
            self.path_plan_url,
            self.headers(),
            self.config.timeout_seconds,
        )
        try:
            return json.loads(message)
        except json.JSONDecodeError:
            return message

    def stream_velocity_control(
        self,
        linear: float | None,
        linear_y: float | None,
        angular: float | None,
        stop_event,
        *,
        rate_hz: float = 10.0,
    ) -> None:
        payload = {
            "linear": float(self.config.linear if linear is None else linear),
            "linear_y": float(self.config.linear_y if linear_y is None else linear_y),
            "angular": float(self.config.angular if angular is None else angular),
        }
        interval = 1.0 / max(float(rate_hz), 1.0)
        websocket = self._open_websocket_socket(self.velocity_control_url, self.headers(), self.config.timeout_seconds)
        try:
            while not stop_event.is_set():
                self._send_websocket_text_frame(websocket, json.dumps(payload, ensure_ascii=False))
                time.sleep(interval)
            self._send_websocket_text_frame(websocket, json.dumps({"linear": 0.0, "linear_y": 0.0, "angular": 0.0}))
        finally:
            websocket.close()

    def send_velocity_stop(self) -> None:
        websocket = self._open_websocket_socket(self.velocity_control_url, self.headers(), self.config.timeout_seconds)
        try:
            self._send_websocket_text_frame(websocket, json.dumps({"linear": 0.0, "linear_y": 0.0, "angular": 0.0}))
        finally:
            websocket.close()

    def set_task_id(self, task_uid: str = "") -> dict[str, Any]:
        return self._http_post_json(
            f"{self.robot_control_base_url}/set_task_id",
            self.headers(),
            {"task_uid": str(task_uid)},
            self.config.timeout_seconds,
        )

    def go_to_pose(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        *,
        use_pyr: bool = True,
        precision_xy: float | None = None,
        precision_yaw: float | None = None,
        is_reverse: bool = False,
        nav_type: str = "auto",
        task_id: str = "",
        deploy_id: str = "",
        inflation_radius: float = 1.0,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "pose": {
                "position": {"x": float(x), "y": float(y)},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "pyr": {"pitch": 0.0, "roll": 0.0, "yaw": float(yaw)},
            },
            "use_pyr": bool(use_pyr),
            "precision_xy": float(self.config.precision_xy if precision_xy is None else precision_xy),
            "precision_yaw": float(self.config.precision_yaw if precision_yaw is None else precision_yaw),
            "is_reverse": bool(is_reverse),
            "nav_type": str(nav_type),
            "inflation_radius": float(inflation_radius),
        }
        if task_id:
            payload["task_id"] = str(task_id)
        if deploy_id:
            payload["deploy_id"] = str(deploy_id)
        return self._http_post_json(f"{self.robot_control_base_url}/go_to", self.headers(), payload, self.config.timeout_seconds)

    def execute_task_control(self, task_uid: str, wps: list[dict[str, Any]], *, is_repeat: bool = False) -> dict[str, Any]:
        payload = {"task_uid": str(task_uid), "is_repeat": bool(is_repeat), "wps": wps}
        return self._http_post_json(f"{self.robot_control_base_url}/execute_task", self.headers(), payload, self.config.timeout_seconds)

    def set_lift_height(self, height_mm: int) -> dict[str, Any]:
        return self._http_post_json(f"{self.robot_control_base_url}/set_lift_height", self.headers(), {"height": int(height_mm)}, self.config.timeout_seconds)

    def set_pose(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        *,
        use_pyr: bool = True,
        use_absolute: bool = True,
    ) -> dict[str, Any]:
        payload = {
            "use_pyr": bool(use_pyr),
            "position": {"x": float(x), "y": float(y)},
            "orientation": {"z": 0.0, "w": 1.0},
            "pyr": {"yaw": float(yaw)},
            "use_absolute": bool(use_absolute),
        }
        return self._http_post_json(f"{self.robot_control_base_url}/set_pose", self.headers(), payload, self.config.timeout_seconds)

    def start_mapping(self, map_resolution: int = 5, robot_model: str = "diff") -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/mapping/{int(map_resolution)}/{quote(str(robot_model))}", self.headers(), self.config.timeout_seconds)

    def start_navigation(self, map_id: int, robot_model: str = "diff") -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/navigation/{int(map_id)}/{quote(str(robot_model))}", self.headers(), self.config.timeout_seconds)

    def stop_program(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/stopAndRemoveProgram", self.headers(), self.config.timeout_seconds)

    def is_program_running(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/isRunning", self.headers(), self.config.timeout_seconds)

    def get_current_map(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/deploy/getCurrentMap", self.headers(), self.config.timeout_seconds)

    def change_map(self, map_id: int) -> dict[str, Any]:
        return self._http_post_json(f"{self.api_base_url}/deploy/changeMap", self.headers(), {"map_id": int(map_id)}, self.config.timeout_seconds)

    def save_map(self, map_name: str) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/deploy/saveMap/{quote(str(map_name))}", self.headers(), self.config.timeout_seconds)

    def delete_map(self, map_id: int) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/deploy/deleteMap/{int(map_id)}", self.headers(), self.config.timeout_seconds)

    def change_map_name(self, map_id: int, new_name: str) -> dict[str, Any]:
        query = urlencode({"map_id": int(map_id), "new_name": str(new_name)})
        return self._http_get_json(f"{self.api_base_url}/map/changeMapName?{query}", self.headers(), self.config.timeout_seconds)

    def execute_deploy_task(self, uid: str, *, repeat: bool = False, do_reverse: bool = False, use_path_map: bool = False) -> dict[str, Any]:
        payload = {"uid": str(uid), "repeat": bool(repeat), "do_reverse": bool(do_reverse), "use_path_map": bool(use_path_map)}
        return self._http_post_json(f"{self.api_base_url}/deploy/executeTask", self.headers(), payload, self.config.timeout_seconds)

    def get_all_deployment_of_map(self, map_id: int) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/deploy/getAllDeploymentOfMap/{int(map_id)}", self.headers(), self.config.timeout_seconds)

    def save_deployment_profile(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._http_post_json(f"{self.api_base_url}/deploy/saveDeploymentProfile", self.headers(), payload, self.config.timeout_seconds)

    def delete_deployment(self, dp_uid: str) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/deploy/deleteDeployment/{quote(str(dp_uid))}", self.headers(), self.config.timeout_seconds)

    def get_network_ips(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/system/getNetworkIPs", self.headers(), self.config.timeout_seconds)

    def scan_wifi(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/system/scanWifi", self.headers(), self.config.timeout_seconds)

    def list_history_wifi(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/system/listHistoryWifi", self.headers(), self.config.timeout_seconds)

    def delete_history_wifi(self, ssid: str) -> dict[str, Any]:
        return self._http_post_json(f"{self.api_base_url}/system/deleteHistoryWifi", self.headers(), {"ssid": str(ssid)}, self.config.timeout_seconds)

    def connect_wifi_json(self, ssid: str, password: str) -> dict[str, Any]:
        return self._http_post_json(f"{self.api_base_url}/system/connectWifiJson", self.headers(), {"ssid": str(ssid), "password": str(password)}, self.config.timeout_seconds)

    def get_connected_wifi(self) -> Any:
        return self._http_get_json(f"{self.api_base_url}/system/getConnectedWifi", self.headers(), self.config.timeout_seconds)

    def set_ap_json(self, name: str, password_ap: str) -> dict[str, Any]:
        return self._http_post_json(f"{self.api_base_url}/system/setAPJson", self.headers(), {"name": str(name), "password_ap": str(password_ap)}, self.config.timeout_seconds)

    def poweroff(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.api_base_url}/system/poweroff", self.headers(), self.config.timeout_seconds)

    def set_collision_monitor(
        self,
        name: str,
        linear_x_limit: float,
        linear_y_limit: float,
        angular_limit: float,
        points: list[float],
    ) -> dict[str, Any]:
        payload = {
            "name": str(name),
            "linear_x_limit": float(linear_x_limit),
            "linear_y_limit": float(linear_y_limit),
            "angular_limit": float(angular_limit),
            "points": [float(point) for point in points],
        }
        return self._http_post_json(f"{self.api_base_url}/parameter/setCollisionMonitor/", self.headers(), payload, self.config.timeout_seconds)

    def enable_collision_monitor(self, enable: bool) -> dict[str, Any]:
        value = 1 if enable else 0
        return self._http_get_json(f"{self.api_base_url}/parameter/enableCollisionMonitor/{value}", self.headers(), self.config.timeout_seconds)

    def set_controller_max_speed(self, max_speed: float, max_reverse_speed: float) -> dict[str, Any]:
        payload = {"max_speed": float(max_speed), "max_reverse_speed": float(max_reverse_speed)}
        return self._http_post_json(f"{self.api_base_url}/parameter/setControllerMaxSpeed/", self.headers(), payload, self.config.timeout_seconds)

    def set_nav_controller_params(self, payload: dict[str, Any]) -> dict[str, Any]:
        """
        設定導航控制器進階參數 (包含前進與後退的 36 項參數，如 PID、前看距離、避障減速等)。
        """
        return self._http_post_json(
            f"{self.api_base_url}/parameter/setControllerParams",
            self.headers(),
            payload,
            self.config.timeout_seconds
        )

    def list_sound_files(self) -> Any:
        return self._http_get_json(f"{self.sound_base_url}/files", self.headers(), self.config.timeout_seconds)

    def play_sound_local(self, filename: str) -> dict[str, Any]:
        return self._http_get_json(f"{self.sound_base_url}/play_local/{quote(str(filename))}", self.headers(), self.config.timeout_seconds)

    def stop_sound_playback(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.sound_base_url}/stop_playback", self.headers(), self.config.timeout_seconds)

    def delete_sound_file(self, filename: str) -> dict[str, Any]:
        return self._http_delete_json(f"{self.sound_base_url}/delete/{quote(str(filename))}", self.headers(), self.config.timeout_seconds)

    def start_coverage_navigation(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._http_post_json(f"{self.coverage_base_url}/navigate_coverage", self.headers(), payload, self.config.timeout_seconds)

    def get_coverage_status(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.coverage_base_url}/status", self.headers(), self.config.timeout_seconds)

    def cancel_coverage_navigation(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.coverage_base_url}/cancel", self.headers(), self.config.timeout_seconds)

    def list_schedules(self) -> dict[str, Any]:
        return self._http_get_json(f"{self.schedule_base_url}/schedules", self.headers(), self.config.timeout_seconds)

    def create_schedule(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._http_post_json(f"{self.schedule_base_url}/schedules", self.headers(), payload, self.config.timeout_seconds)

    def get_schedule(self, schedule_id: int) -> dict[str, Any]:
        return self._http_get_json(f"{self.schedule_base_url}/schedules/{int(schedule_id)}", self.headers(), self.config.timeout_seconds)

    def update_schedule(self, schedule_id: int, payload: dict[str, Any]) -> dict[str, Any]:
        return self._http_put_json(f"{self.schedule_base_url}/schedules/{int(schedule_id)}", self.headers(), payload, self.config.timeout_seconds)

    def delete_schedule(self, schedule_id: int) -> dict[str, Any]:
        return self._http_delete_json(f"{self.schedule_base_url}/schedules/{int(schedule_id)}", self.headers(), self.config.timeout_seconds)

    def get_scan_once(self) -> Any:
        message = self._recv_websocket_text_once(self.scan_url, self.headers(), self.config.timeout_seconds)
        try:
            return json.loads(message)
        except json.JSONDecodeError:
            return message

    def get_map_stream_once(self) -> Any:
        message = self._recv_websocket_text_once(self.map_stream_url, self.headers(), self.config.timeout_seconds)
        try:
            return json.loads(message)
        except json.JSONDecodeError:
            return message

    def collect_snapshot(self) -> AmrServiceSnapshot:
        error_messages: list[str] = []
        heartbeat = self.check_heartbeat()
        robot_data: dict[str, Any] = {}
        battery_data: dict[str, Any] = {}
        maps: list[AmrMapInfo] = []

        try:
            robot_data = self.get_robot_data_once()
        except Exception as exc:
            error_messages.append(str(exc))

        try:
            battery_data = self.get_battery_data_once()
        except Exception as exc:
            error_messages.append(str(exc))

        try:
            maps = self.get_maps()
        except Exception as exc:
            error_messages.append(str(exc))

        error_text = "\n".join(error_messages)
        if error_text:
            self.last_error = error_text
        return AmrServiceSnapshot(
            heartbeat=heartbeat,
            robot_data=robot_data,
            battery_data=battery_data,
            maps=maps,
            error=error_text,
        )

    def _default_heartbeat_checker(self, ip: str, port: int, timeout: float) -> bool:
        with socket.create_connection((ip, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            data = sock.recv(1024)
            return bool(data)

    def _default_robot_data_receiver(self, url: str, headers: dict[str, str], timeout: float) -> dict[str, Any]:
        message = self._recv_websocket_text_once(url, headers, timeout)
        data = json.loads(str(message))
        if not isinstance(data, dict):
            raise RuntimeError(f"robot_data 回應格式錯誤: {message}")
        return data

    def _default_battery_data_receiver(self, host: str, port: int, timeout: float) -> dict[str, Any]:
        client_id = f"amr-service-{os.getpid()}"
        last_error: Exception | None = None
        for protocol_name, protocol_level in (("MQTT", 4), ("MQIsdp", 3)):
            try:
                with socket.create_connection((host, port), timeout=timeout) as sock:
                    sock.settimeout(timeout)
                    self._mqtt_send_connect(
                        sock,
                        client_id,
                        protocol_name=protocol_name,
                        protocol_level=protocol_level,
                        username=self.config.amr_mqtt_username,
                        password=self.config.amr_mqtt_password,
                    )
                    self._mqtt_expect_connack(sock)
                    self._mqtt_send_subscribe(sock, packet_id=1, topic="battery/data")
                    self._mqtt_expect_suback(sock, packet_id=1)
                    self._mqtt_send_publish(sock, topic="battery/sub", payload=b"0")
                    payload = self._mqtt_read_publish(sock, expected_topic="battery/data")
                break
            except Exception as exc:
                last_error = exc
        else:
            raise RuntimeError(f"MQTT 電池訂閱失敗: {last_error}") from last_error
        data = json.loads(payload.decode("utf-8"))
        if not isinstance(data, dict):
            raise RuntimeError(f"battery/data 回應格式錯誤: {payload!r}")
        return data

    def _recv_websocket_text_once(self, url: str, headers: dict[str, str], timeout: float) -> str:
        sock, initial_payload = self._open_websocket_socket(url, headers, timeout, return_initial_payload=True)
        try:
            return self._recv_websocket_frame(sock, initial_payload)
        finally:
            sock.close()

    def _open_websocket_socket(
        self,
        url: str,
        headers: dict[str, str],
        timeout: float,
        *,
        return_initial_payload: bool = False,
    ):
        parsed_url = urlparse(url)
        if parsed_url.scheme != "ws":
            raise RuntimeError(f"僅支援 ws:// websocket URL: {url}")

        host = parsed_url.hostname
        if not host:
            raise RuntimeError(f"websocket URL 缺少 host: {url}")
        port = parsed_url.port or 80
        path = parsed_url.path or "/"
        if parsed_url.query:
            path = f"{path}?{parsed_url.query}"

        websocket_key = base64.b64encode(os.urandom(16)).decode("ascii")
        header_lines = [
            f"GET {path} HTTP/1.1",
            f"Host: {host}:{port}",
            "Upgrade: websocket",
            "Connection: Upgrade",
            f"Sec-WebSocket-Key: {websocket_key}",
            "Sec-WebSocket-Version: 13",
        ]
        header_lines.extend(f"{key}: {value}" for key, value in headers.items())
        request_text = "\r\n".join(header_lines) + "\r\n\r\n"

        sock = socket.create_connection((host, port), timeout=timeout)
        try:
            sock.settimeout(timeout)
            sock.sendall(request_text.encode("ascii"))
            response_header, initial_payload = self._recv_until(sock, b"\r\n\r\n")
            if b" 101 " not in response_header.split(b"\r\n", 1)[0]:
                raise RuntimeError(response_header.decode("utf-8", errors="replace").strip())
            self._validate_websocket_accept(response_header, websocket_key)
            if return_initial_payload:
                return sock, initial_payload
            return sock
        except Exception:
            sock.close()
            raise

    def _send_websocket_text_frame(self, sock: socket.socket, text: str) -> None:
        payload = text.encode("utf-8")
        mask_key = os.urandom(4)
        frame = bytearray([0x81])
        payload_length = len(payload)
        if payload_length < 126:
            frame.append(0x80 | payload_length)
        elif payload_length <= 0xFFFF:
            frame.append(0x80 | 126)
            frame.extend(payload_length.to_bytes(2, "big"))
        else:
            frame.append(0x80 | 127)
            frame.extend(payload_length.to_bytes(8, "big"))
        frame.extend(mask_key)
        frame.extend(byte ^ mask_key[index % 4] for index, byte in enumerate(payload))
        sock.sendall(frame)

    def _recv_until(self, sock: socket.socket, marker: bytes) -> tuple[bytes, bytes]:
        data = b""
        while marker not in data:
            chunk = sock.recv(1024)
            if not chunk:
                break
            data += chunk
        header, separator, remainder = data.partition(marker)
        return header + separator, remainder

    def _validate_websocket_accept(self, response_header: bytes, websocket_key: str) -> None:
        expected_accept = base64.b64encode(
            hashlib.sha1((websocket_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()
        ).decode("ascii")
        header_values = self._parse_http_headers(response_header)
        actual_accept = header_values.get("sec-websocket-accept", "")
        if actual_accept.strip() != expected_accept:
            raise RuntimeError("robot_data websocket handshake 驗證失敗。")

    def _parse_http_headers(self, response_header: bytes) -> dict[str, str]:
        header_values: dict[str, str] = {}
        header_text = response_header.decode("utf-8", errors="replace")
        for line in header_text.splitlines()[1:]:
            if ":" not in line:
                continue
            name, value = line.split(":", 1)
            header_values[name.strip().lower()] = value.strip()
        return header_values

    def _recv_websocket_frame(self, sock: socket.socket, initial_buffer: bytes = b"") -> str:
        buffer = bytearray(initial_buffer)
        first_two = self._recv_exact(sock, 2, buffer)
        if len(first_two) < 2:
            raise RuntimeError("robot_data websocket 未收到資料。")

        first_byte, second_byte = first_two
        opcode = first_byte & 0x0F
        masked = bool(second_byte & 0x80)
        payload_length = second_byte & 0x7F
        if payload_length == 126:
            payload_length = int.from_bytes(self._recv_exact(sock, 2, buffer), "big")
        elif payload_length == 127:
            payload_length = int.from_bytes(self._recv_exact(sock, 8, buffer), "big")

        mask_key = self._recv_exact(sock, 4, buffer) if masked else b""
        payload = self._recv_exact(sock, payload_length, buffer)
        if masked:
            payload = bytes(byte ^ mask_key[index % 4] for index, byte in enumerate(payload))
        if opcode == 8:
            raise RuntimeError("robot_data websocket 已關閉。")
        if opcode != 1:
            raise RuntimeError(f"robot_data websocket 收到非文字 frame: opcode={opcode}")
        return payload.decode("utf-8")

    def _recv_exact(self, sock: socket.socket, size: int, buffer: bytearray | None = None) -> bytes:
        data = b""
        if buffer:
            take_size = min(size, len(buffer))
            data = bytes(buffer[:take_size])
            del buffer[:take_size]
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                break
            data += chunk
        return data

    def _mqtt_send_connect(
        self,
        sock: socket.socket,
        client_id: str,
        *,
        protocol_name: str,
        protocol_level: int,
        username: str = "",
        password: str = "",
    ) -> None:
        connect_flags = 0x02
        payload = self._mqtt_encode_utf8(client_id)
        if username:
            connect_flags |= 0x80
            payload += self._mqtt_encode_utf8(username)
            if password:
                connect_flags |= 0x40
                payload += self._mqtt_encode_utf8(password)
        variable_header = self._mqtt_encode_utf8(protocol_name) + bytes([protocol_level, connect_flags]) + (30).to_bytes(2, "big")
        self._mqtt_send_packet(sock, packet_type_flags=0x10, payload=variable_header + payload)

    def _mqtt_expect_connack(self, sock: socket.socket) -> None:
        packet_type, payload = self._mqtt_read_packet(sock)
        if packet_type != 0x20 or len(payload) < 2 or payload[1] != 0:
            return_code = payload[1] if len(payload) >= 2 else None
            message = self._mqtt_connack_error_message(return_code)
            raise RuntimeError(f"MQTT CONNACK 失敗: {message}; packet_type={packet_type:#x}, payload={payload!r}")

    def _mqtt_send_subscribe(self, sock: socket.socket, packet_id: int, topic: str) -> None:
        payload = packet_id.to_bytes(2, "big") + self._mqtt_encode_utf8(topic) + b"\x00"
        self._mqtt_send_packet(sock, packet_type_flags=0x82, payload=payload)

    def _mqtt_expect_suback(self, sock: socket.socket, packet_id: int) -> None:
        packet_type, payload = self._mqtt_read_packet(sock)
        if packet_type != 0x90 or len(payload) < 3:
            raise RuntimeError(f"MQTT SUBACK 失敗: packet_type={packet_type:#x}, payload={payload!r}")
        response_packet_id = int.from_bytes(payload[:2], "big")
        qos_response = payload[2]
        if response_packet_id != packet_id or qos_response == 0x80:
            raise RuntimeError(f"MQTT SUBACK 拒絕訂閱: packet_id={response_packet_id}, qos={qos_response:#x}")

    def _mqtt_send_publish(self, sock: socket.socket, topic: str, payload: bytes) -> None:
        self._mqtt_send_packet(sock, packet_type_flags=0x30, payload=self._mqtt_encode_utf8(topic) + payload)

    def _mqtt_read_publish(self, sock: socket.socket, expected_topic: str) -> bytes:
        while True:
            packet_type, payload = self._mqtt_read_packet(sock)
            packet_kind = packet_type & 0xF0
            if packet_kind == 0x90:
                continue
            if packet_kind != 0x30:
                continue
            topic_length = int.from_bytes(payload[:2], "big")
            topic = payload[2:2 + topic_length].decode("utf-8")
            message = payload[2 + topic_length:]
            if topic == expected_topic:
                return message

    def _mqtt_send_packet(self, sock: socket.socket, packet_type_flags: int, payload: bytes) -> None:
        sock.sendall(bytes([packet_type_flags]) + self._mqtt_encode_remaining_length(len(payload)) + payload)

    def _mqtt_read_packet(self, sock: socket.socket) -> tuple[int, bytes]:
        packet_type_bytes = self._recv_exact(sock, 1)
        if not packet_type_bytes:
            raise RuntimeError("MQTT 連線未收到資料。")
        remaining_length = self._mqtt_read_remaining_length(sock)
        return packet_type_bytes[0], self._recv_exact(sock, remaining_length)

    def _mqtt_read_remaining_length(self, sock: socket.socket) -> int:
        multiplier = 1
        value = 0
        while True:
            encoded_byte = self._recv_exact(sock, 1)
            if not encoded_byte:
                raise RuntimeError("MQTT remaining length 讀取失敗。")
            byte_value = encoded_byte[0]
            value += (byte_value & 127) * multiplier
            if (byte_value & 128) == 0:
                return value
            multiplier *= 128
            if multiplier > 128 * 128 * 128:
                raise RuntimeError("MQTT remaining length 格式錯誤。")

    def _mqtt_encode_remaining_length(self, value: int) -> bytes:
        encoded = bytearray()
        while True:
            encoded_byte = value % 128
            value //= 128
            if value > 0:
                encoded_byte |= 128
            encoded.append(encoded_byte)
            if value == 0:
                return bytes(encoded)

    def _mqtt_encode_utf8(self, value: str) -> bytes:
        encoded = value.encode("utf-8")
        return len(encoded).to_bytes(2, "big") + encoded

    def _mqtt_connack_error_message(self, return_code: int | None) -> str:
        messages = {
            0: "連線成功",
            1: "不可接受的協定版本",
            2: "識別碼被拒絕",
            3: "伺服器不可用",
            4: "使用者名稱或密碼錯誤",
            5: "未授權",
        }
        if return_code is None:
            return "CONNACK payload 格式錯誤"
        return messages.get(return_code, f"未知錯誤碼 {return_code}")

    def _logged_http_get_json(self, url: str, headers: dict[str, str], timeout: float) -> Any:
        sent_at = datetime.now()
        data = self._http_get_json_transport(url, headers, timeout)
        self._log_error_response_if_needed(
            sent_at=sent_at,
            api=url,
            request_data={},
            received_at=datetime.now(),
            response_data=data,
        )
        return data

    def _logged_http_post_json(
        self,
        url: str,
        headers: dict[str, str],
        payload: dict[str, Any],
        timeout: float,
    ) -> dict[str, Any]:
        sent_at = datetime.now()
        data = self._http_post_json_transport(url, headers, payload, timeout)
        self._log_error_response_if_needed(
            sent_at=sent_at,
            api=url,
            request_data=payload,
            received_at=datetime.now(),
            response_data=data,
        )
        return data

    def _log_error_response_if_needed(
        self,
        *,
        sent_at: datetime,
        api: str,
        request_data: Any,
        received_at: datetime,
        response_data: Any,
    ) -> None:
        if not isinstance(response_data, dict):
            return
        try:
            code = int(response_data.get("code", 0))
        except (TypeError, ValueError):
            return
        if code != 1:
            return
        record = {
            "sent": {
                "time": sent_at.isoformat(timespec="seconds"),
                "api": api,
                "data": request_data,
            },
            "received": {
                "time": received_at.isoformat(timespec="seconds"),
                "data": response_data,
            },
        }
        self._write_api_error_log(sent_at, record)

    def _write_api_error_log(self, sent_at: datetime, record: dict[str, Any]) -> None:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.log_dir / f"{sent_at.date().isoformat()}.log"
        with log_path.open("a", encoding="utf-8") as log_file:
            log_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _logged_http_put_json(
        self,
        url: str,
        headers: dict[str, str],
        payload: dict[str, Any],
        timeout: float,
    ) -> dict[str, Any]:
        sent_at = datetime.now()
        data = self._default_http_request_json(url, headers, "PUT", payload, timeout)
        self._log_error_response_if_needed(sent_at=sent_at, api=url, request_data=payload, received_at=datetime.now(), response_data=data)
        return data

    def _logged_http_delete_json(self, url: str, headers: dict[str, str], timeout: float) -> dict[str, Any]:
        sent_at = datetime.now()
        data = self._default_http_request_json(url, headers, "DELETE", None, timeout)
        self._log_error_response_if_needed(sent_at=sent_at, api=url, request_data={}, received_at=datetime.now(), response_data=data)
        return data

    def _default_http_request_json(
        self,
        url: str,
        headers: dict[str, str],
        method: str,
        payload: dict[str, Any] | None,
        timeout: float,
    ) -> Any:
        request_headers = {**headers, "Accept": "application/json"}
        body = None
        if payload is not None:
            request_headers["Content-Type"] = "application/json"
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        api_request = request.Request(url, data=body, headers=request_headers, method=method)
        try:
            with request.urlopen(api_request, timeout=timeout) as response:
                response_body = response.read().decode("utf-8")
        except error.HTTPError as exc:
            error_body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"AMR API HTTP {exc.code}: {error_body}") from exc
        except error.URLError as exc:
            raise RuntimeError(f"AMR API 連線失敗: {exc.reason}") from exc
        if not response_body.strip():
            return {"code": 0, "message": ""}
        try:
            data = json.loads(response_body)
        except json.JSONDecodeError:
            return {"code": 0, "message": response_body}
        if not isinstance(data, (dict, list, str)):
            raise RuntimeError(f"AMR API 回應格式錯誤: {response_body}")
        return data

    def _default_http_get_json(self, url: str, headers: dict[str, str], timeout: float) -> Any:
        api_request = request.Request(url, headers=headers, method="GET")
        try:
            with request.urlopen(api_request, timeout=timeout) as response:
                body = response.read().decode("utf-8")
        except error.HTTPError as exc:
            error_body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"AMR API HTTP {exc.code}: {error_body}") from exc
        except error.URLError as exc:
            raise RuntimeError(f"AMR API 連線失敗: {exc.reason}") from exc

        try:
            data = json.loads(body)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"AMR API 回應不是 JSON: {body}") from exc
        if not isinstance(data, (dict, list)):
            raise RuntimeError(f"AMR API 回應格式錯誤: {body}")
        return data

    def _default_http_post_json(
        self,
        url: str,
        headers: dict[str, str],
        payload: dict[str, Any],
        timeout: float,
    ) -> dict[str, Any]:
        data = self._default_http_request_json(url, headers, "POST", payload, timeout)
        if not isinstance(data, dict):
            raise RuntimeError(f"AMR API 回應格式錯誤: {data}")
        return data

    def _get_map_payload(self, map_detail: dict[str, Any]) -> dict[str, Any]:
        payload = map_detail.get("data", map_detail)
        if not isinstance(payload, dict):
            raise ValueError("地圖資料格式錯誤: data 不是物件")
        map_payload = payload.get("data", payload)
        if not isinstance(map_payload, dict):
            raise ValueError("地圖資料格式錯誤: data.data 不是物件")
        return map_payload

    def _iter_waypoint_items(self, value: Any):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == "waypoints" and isinstance(item, list):
                    yield from item
                else:
                    yield from self._iter_waypoint_items(item)
        elif isinstance(value, list):
            for item in value:
                yield from self._iter_waypoint_items(item)

    def _find_waypoint(self, waypoint_key: str) -> AmrWaypoint | None:
        normalized = waypoint_key.strip()
        if not normalized:
            return None
        for map_info in self.get_maps():
            try:
                map_detail = self.get_map_data_with_detail_ros(map_info.id)
                waypoints = self.parse_waypoints(map_detail)
            except Exception:
                continue
            for waypoint in waypoints:
                candidates = {
                    str(waypoint.name or "").strip(),
                    str(waypoint.wp_uid or "").strip(),
                    str(waypoint.dp_uid or "").strip(),
                }
                if normalized in candidates:
                    return waypoint
        return None

    def _find_waypoint_by_uid(self, wp_uid: str) -> AmrWaypoint | None:
        for map_info in self.get_maps():
            try:
                map_detail = self.get_map_data_with_detail_ros(map_info.id)
                waypoints = self.parse_waypoints(map_detail)
            except Exception:
                continue
            for waypoint in waypoints:
                if str(waypoint.wp_uid or "").strip() == wp_uid:
                    return waypoint
        return None

    def _wait_for_charge_fsm(self, *, stuck_seconds: float, poll_interval: float, timeout_seconds: float) -> str:
        last_position = self._charge_position_xy()
        last_changed_at = time.monotonic()
        deadline = time.monotonic() + max(timeout_seconds, 0.0)
        while True:
            if time.monotonic() > deadline:
                return "timeout"
            robot_data = self.get_robot_data_once()
            fsm = self._normalized_fsm(robot_data.get("fsm"))
            if fsm in {"succeeded", "failed"}:
                return fsm
            current_position = self._charge_position_xy(robot_data)
            if fsm == "moving":
                if current_position is not None and current_position != last_position:
                    last_position = current_position
                    last_changed_at = time.monotonic()
                elif current_position is not None and time.monotonic() - last_changed_at >= stuck_seconds:
                    return "stuck"
            self._sleep(poll_interval)

    def _recover_charge_flow(self, *, confirm_delay_seconds: float, idle_timeout_seconds: float, poll_interval: float) -> None:
        self.cancel_task()
        self._sleep(confirm_delay_seconds)
        self.confirm_status()
        self._wait_until_charge_idle(timeout_seconds=idle_timeout_seconds, poll_interval=poll_interval)

    def _wait_until_charge_idle(self, *, timeout_seconds: float, poll_interval: float) -> bool:
        deadline = time.monotonic() + max(timeout_seconds, 0.0)
        while time.monotonic() <= deadline:
            try:
                fsm = self._normalized_fsm(self.get_robot_data_once().get("fsm"))
            except Exception:
                fsm = ""
            if fsm == "idle":
                return True
            self._sleep(poll_interval)
        return False

    def _normalized_fsm(self, value: Any) -> str:
        fsm = str(value or "").strip().lower()
        if fsm in {"successed", "sucessed", "succeed", "success"}:
            return "succeeded"
        if fsm in {"fail", "failure"}:
            return "failed"
        return fsm

    def _charge_position_xy(self, robot_data: dict[str, Any] | None = None) -> tuple[float, float] | None:
        if robot_data is None:
            try:
                robot_data = self.get_robot_data_once()
            except Exception:
                return None
        pose = robot_data.get("pose") if isinstance(robot_data.get("pose"), dict) else {}
        position = pose.get("position") if isinstance(pose.get("position"), dict) else {}
        try:
            return (round(float(position["x"]), 3), round(float(position["y"]), 3))
        except (KeyError, TypeError, ValueError):
            return None

    def _sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)

    def _optional_float(self, value: Any) -> float | None:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _map_cell_to_pixel(self, value: Any) -> int:
        numeric_value = int(value)
        if numeric_value == 0:
            return 255
        if numeric_value == 100:
            return 0
        return 128

    def _write_grayscale_png(self, path: Path, width: int, height: int, flat_pixels: list[int]) -> None:
        raw_rows = []
        for row_index in range(height):
            start = row_index * width
            raw_rows.append(b"\x00" + bytes(flat_pixels[start:start + width]))
        compressed = zlib.compress(b"".join(raw_rows))

        def png_chunk(chunk_type: bytes, data: bytes) -> bytes:
            crc = zlib.crc32(chunk_type + data) & 0xFFFFFFFF
            return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", crc)

        png_data = b"".join(
            [
                b"\x89PNG\r\n\x1a\n",
                png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)),
                png_chunk(b"IDAT", compressed),
                png_chunk(b"IEND", b""),
            ]
        )
        path.write_bytes(png_data)
