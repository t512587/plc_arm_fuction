from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(slots=True)
class ServiceAmrConfig:
    """AMR NEST API 連線設定。"""

    amr_IP: str = "192.168.3.110"
    amr_socket_port: int = 9999
    amr_api_port: int = 5000
    api_key: str = os.getenv("AMR_API_KEY", "")
    robot_data_ws_port: int = 1234
    amr_mqtt_port: int = 1885
    coverage_api_port: int = 1235
    amr_mqtt_username: str = os.getenv("AMR_MQTT_USERNAME", "")
    amr_mqtt_password: str = os.getenv("AMR_MQTT_PASSWORD", "")
    timeout_seconds: float = 3.0
    near_charge_point : str = "H"
    precision_xy: float = 0.5
    precision_yaw: float = 0.5
    linear: float = 1.0
    linear_y: float = 1.0
    angular: float = 0.10


DEFAULT_AMR_CONFIG = ServiceAmrConfig()
