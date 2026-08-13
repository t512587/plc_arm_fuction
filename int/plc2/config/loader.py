"""載入 PLC / 點位設定檔的工具。

- 讀取 `config/plc_config.yml` 與 `config/points.yml`
- 提供 in-memory 查詢 point 資訊
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_DIR = BASE_DIR / "config"


@dataclass
class PLCDefinition:
    name: str
    host: str
    port: int
    protocol: str
    unit: int = 0


@dataclass
class PointDefinition:
    id: str
    name: str
    plc: str
    device: str
    address: int
    type: str
    group: str
    writable: bool = False
    min: Optional[float] = None
    max: Optional[float] = None
    # 進階型別設定（選用）：
    scale: Optional[float] = None
    pair_high: Optional[int] = None  # 32-bit / float 用的高位 D 地址


class ConfigStore:
    """載入並保存 PLC / 點位設定。"""

    def __init__(self) -> None:
        self._plcs: Dict[str, PLCDefinition] = {}
        self._points: Dict[str, PointDefinition] = {}
        self._services: Dict[str, Dict[str, Any]] = {}
        self._flows: Dict[str, Dict[str, Any]] = {}

    # ===== 載入 =====

    def load(self) -> None:
        self._plcs.clear()
        self._points.clear()
        self._services.clear()
        self._flows.clear()
        self._load_plc_config()
        self._load_points()
        self._load_services()
        self._load_flows()

    def _load_plc_config(self) -> None:
        path = CONFIG_DIR / "plc_config.yml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        for item in data.get("connections", []):
            plc = PLCDefinition(
                name=str(item["name"]),
                host=str(item["host"]),
                port=int(item["port"]),
                protocol=str(item.get("protocol", "mc")),
                unit=int(item.get("unit", 0)),
            )
            self._plcs[plc.name] = plc

    def _load_points(self) -> None:
        path = CONFIG_DIR / "points.yml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        for item in data or []:
            point = PointDefinition(
                id=str(item["id"]),
                name=str(item["name"]),
                plc=str(item["plc"]),
                device=str(item["device"]),
                address=int(item["address"]),
                type=str(item.get("type", "int")),
                group=str(item.get("group", "")),
                writable=bool(item.get("writable", False)),
                min=item.get("min"),
                max=item.get("max"),
                scale=float(item.get("scale")) if item.get("scale") is not None else None,
                pair_high=int(item["pair_high"]) if item.get("pair_high") is not None else None,
            )
            self._points[point.id] = point

    def _load_services(self) -> None:
        path = CONFIG_DIR / "services.yml"
        if not path.exists():
            return
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        services = data.get("services", {})
        if not isinstance(services, dict):
            raise ValueError(f"services.yml 的 services 必須是 mapping: {path}")
        for name, config in services.items():
            if not isinstance(config, dict):
                raise ValueError(f"service '{name}' 設定必須是 mapping")
            self._services[str(name)] = dict(config)

    def _load_flows(self) -> None:
        flow_dir = CONFIG_DIR / "flows"
        if not flow_dir.exists():
            return
        for path in sorted(flow_dir.glob("*.yml")):
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            if not isinstance(data, dict):
                raise ValueError(f"flow 設定必須是 mapping: {path}")
            self._flows[path.stem] = dict(data)

    # ===== 查詢 API =====

    def get_plc(self, name: str) -> Optional[PLCDefinition]:
        return self._plcs.get(name)

    def get_point(self, point_id: str) -> Optional[PointDefinition]:
        return self._points.get(point_id)

    def list_points(self, group: Optional[str] = None) -> List[PointDefinition]:
        if group is None:
            return list(self._points.values())
        return [p for p in self._points.values() if p.group == group]

    def get_service(self, name: str) -> Optional[Dict[str, Any]]:
        config = self._services.get(name)
        return dict(config) if config is not None else None

    def get_flow(self, name: str) -> Optional[Dict[str, Any]]:
        config = self._flows.get(name)
        return dict(config) if config is not None else None


# 全域單例（簡單做法，之後可換 DI）
CONFIG_STORE = ConfigStore()
CONFIG_STORE.load()
