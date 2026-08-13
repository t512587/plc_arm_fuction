from __future__ import annotations

import socket
import struct
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

try:
    import pymcprotocol
except ImportError:  # pragma: no cover
    pymcprotocol = None


BASE_DIR = Path(__file__).resolve().parent
POINTS_FILE = BASE_DIR / "points.yml"
PLC_CONFIG_FILE = BASE_DIR / "plc_config.yml"


class PLCError(Exception):
    """PLC service base error."""


class PLCConnectionError(PLCError):
    """PLC connect/read/write error."""


@dataclass(slots=True)
class PLCDefinition:
    name: str
    host: str
    port: int
    protocol: str = "mc"
    unit: int = 0


@dataclass(slots=True)
class PointDefinition:
    id: str
    name: str
    plc: str
    device: str
    address: int
    type: str
    group: str
    writable: bool = False
    min: float | None = None
    max: float | None = None
    scale: float | None = None
    pair_high: int | None = None


def load_plc_config(path: Path | None = None) -> dict[str, PLCDefinition]:
    config_path = path or PLC_CONFIG_FILE
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    plcs: dict[str, PLCDefinition] = {}
    for item in raw.get("connections", []):
        plc = PLCDefinition(
            name=str(item["name"]),
            host=str(item["host"]),
            port=int(item["port"]),
            protocol=str(item.get("protocol", "mc")),
            unit=int(item.get("unit", 0)),
        )
        plcs[plc.name] = plc
    return plcs


def load_points(path: Path | None = None) -> dict[str, PointDefinition]:
    points_path = path or POINTS_FILE
    raw = yaml.safe_load(points_path.read_text(encoding="utf-8")) or []
    points: dict[str, PointDefinition] = {}
    for item in raw:
        point = PointDefinition(
            id=str(item["id"]),
            name=str(item["name"]),
            plc=str(item["plc"]),
            device=str(item["device"]).upper(),
            address=int(item["address"]),
            type=str(item.get("type", "int")),
            group=str(item.get("group", "")),
            writable=bool(item.get("writable", False)),
            min=float(item["min"]) if item.get("min") is not None else None,
            max=float(item["max"]) if item.get("max") is not None else None,
            scale=float(item["scale"]) if item.get("scale") is not None else None,
            pair_high=int(item["pair_high"]) if item.get("pair_high") is not None else None,
        )
        points[point.id] = point
    return points


def _socket_broken(exc: Exception) -> bool:
    errno = getattr(exc, "errno", None)
    if errno in {32, 54, 10053, 10054, 104}:
        return True
    text = str(exc).lower()
    return any(
        marker in text
        for marker in (
            "broken pipe",
            "connection reset",
            "connection aborted",
            "10053",
            "10054",
            "errno 32",
            "errno 104",
        )
    )


class PlcApi:
    """Minimal MC protocol API used by service_plc.py."""

    def __init__(
        self,
        plc_config_path: Path | None = None,
        points_path: Path | None = None,
    ) -> None:
        self.plcs = load_plc_config(plc_config_path)
        self.points = load_points(points_path)
        self._clients: dict[str, Any] = {}
        self._desired: set[str] = set()
        self._lock = threading.RLock()

    def list_groups(self) -> list[str]:
        return sorted({point.group for point in self.points.values() if point.group})

    def list_points(self, group: str | None = None) -> list[PointDefinition]:
        points = list(self.points.values())
        if group is None:
            return sorted(points, key=lambda item: (item.group, item.address, item.id))
        return sorted(
            [point for point in points if point.group == group],
            key=lambda item: (item.address, item.id),
        )

    def get_point(self, point_id: str) -> PointDefinition:
        point = self.points.get(point_id)
        if point is None:
            raise KeyError(f"point not found: {point_id}")
        return point

    def connect(self, plc_name: str = "main_plc") -> str:
        with self._lock:
            plc = self.plcs.get(plc_name)
            if plc is None:
                raise PLCConnectionError(f"PLC not found: {plc_name}")
            self.disconnect(plc_name)
            client = self._build_client(plc)
            self._clients[plc_name] = client
            self._desired.add(plc_name)
            return f"PLC 已連線 {plc.name} {plc.host}:{plc.port}"

    def disconnect(self, plc_name: str = "main_plc") -> str:
        with self._lock:
            client = self._clients.pop(plc_name, None)
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
            self._desired.discard(plc_name)
            return f"PLC 已斷線 {plc_name}"

    def is_connected(self, plc_name: str = "main_plc") -> bool:
        return plc_name in self._clients

    def read_point(self, point_id: str) -> int | float | bool:
        point = self.get_point(point_id)
        return self._read_point(point)

    def write_point(self, point_id: str, value: Any) -> None:
        point = self.get_point(point_id)
        if not point.writable:
            raise PLCConnectionError(f"點位不可寫入: {point.id}")
        self._write_point(point, value)

    def read_register(self, device: str, address: int, count: int = 1) -> list[int] | list[bool]:
        device = device.upper()
        plc_name = "main_plc"
        if device == "D":
            return self._with_retry(plc_name, lambda client: self._read_d(client, address, count))
        return self._with_retry(plc_name, lambda client: self._read_bits(client, device, address, count))

    def write_register(self, device: str, address: int, values: list[Any]) -> None:
        device = device.upper()
        plc_name = "main_plc"
        if device == "D":
            encoded = [int(v) for v in values]
            self._with_retry(plc_name, lambda client: self._write_d(client, address, encoded))
            return
        encoded = [self._parse_bool(v) for v in values]
        self._with_retry(plc_name, lambda client: self._write_bits(client, device, address, encoded))

    def _build_client(self, plc: PLCDefinition) -> Any:
        if pymcprotocol is None:
            raise PLCConnectionError("pymcprotocol 未安裝，請先安裝: pip install pymcprotocol")
        try:
            client = pymcprotocol.Type3E()
            client.soc_timeout = 5
            client.connect(plc.host, plc.port)
            return client
        except ConnectionRefusedError as exc:
            raise PLCConnectionError(f"無法連線到 PLC {plc.host}:{plc.port}: TCP 連線被拒絕") from exc
        except (TimeoutError, socket.timeout) as exc:
            raise PLCConnectionError(f"無法連線到 PLC {plc.host}:{plc.port}: TCP 連線逾時") from exc
        except OSError as exc:
            raise PLCConnectionError(f"無法連線到 PLC {plc.host}:{plc.port}: {exc}") from exc
        except Exception as exc:
            raise PLCConnectionError(f"無法連線到 PLC {plc.host}:{plc.port}: {exc}") from exc

    def _require_client(self, plc_name: str) -> Any:
        client = self._clients.get(plc_name)
        if client is None and plc_name in self._desired:
            plc = self.plcs[plc_name]
            client = self._build_client(plc)
            self._clients[plc_name] = client
        if client is None:
            raise PLCConnectionError(f"PLC 尚未連線: {plc_name}")
        return client

    def _with_retry(self, plc_name: str, action: Any) -> Any:
        with self._lock:
            client = self._require_client(plc_name)
            try:
                return action(client)
            except Exception as exc:
                if not _socket_broken(exc):
                    raise
                self._drop_client(plc_name)
                client = self._require_client(plc_name)
                return action(client)

    def _drop_client(self, plc_name: str) -> None:
        client = self._clients.pop(plc_name, None)
        if client is not None:
            try:
                client.close()
            except Exception:
                pass

    def _read_d(self, client: Any, address: int, count: int) -> list[int]:
        try:
            return [int(value) for value in client.batchread_wordunits(f"D{int(address)}", int(count))]
        except Exception as exc:
            if _socket_broken(exc):
                self._drop_client("main_plc")
            raise PLCConnectionError(f"讀取 D{address} 失敗: {exc}") from exc

    def _write_d(self, client: Any, address: int, values: list[int]) -> None:
        try:
            client.batchwrite_wordunits(f"D{int(address)}", [int(v) for v in values])
        except Exception as exc:
            if _socket_broken(exc):
                self._drop_client("main_plc")
            raise PLCConnectionError(f"寫入 D{address} 失敗: {exc}") from exc

    def _read_bits(self, client: Any, device: str, address: int, count: int) -> list[bool]:
        try:
            return [bool(value) for value in client.batchread_bitunits(f"{device}{int(address)}", int(count))]
        except Exception as exc:
            if _socket_broken(exc):
                self._drop_client("main_plc")
            raise PLCConnectionError(f"讀取 {device}{address} 失敗: {exc}") from exc

    def _write_bits(self, client: Any, device: str, address: int, values: list[bool]) -> None:
        try:
            client.batchwrite_bitunits(f"{device}{int(address)}", [1 if v else 0 for v in values])
        except Exception as exc:
            if _socket_broken(exc):
                self._drop_client("main_plc")
            raise PLCConnectionError(f"寫入 {device}{address} 失敗: {exc}") from exc

    def _read_point(self, point: PointDefinition) -> int | float | bool:
        if point.device == "D":
            count = 2 if point.type.lower() in {"s32", "u32", "f32"} else 1
            raw = self._with_retry(point.plc, lambda client: self._read_d(client, point.address, count))
            return self._decode_d(point, raw)
        raw_bits = self._with_retry(point.plc, lambda client: self._read_bits(client, point.device, point.address, 1))
        return bool(raw_bits[0]) if raw_bits else False

    def _write_point(self, point: PointDefinition, value: Any) -> None:
        if point.device == "D":
            payload = self._encode_d(point, value)
            self._with_retry(point.plc, lambda client: self._write_d(client, point.address, payload))
            return
        parsed = self._parse_bool(value)
        self._with_retry(point.plc, lambda client: self._write_bits(client, point.device, point.address, [parsed]))

    def _decode_d(self, point: PointDefinition, raw_values: list[int]) -> int | float:
        point_type = point.type.lower()
        raw_lo = int(raw_values[0]) if raw_values else 0
        scale = point.scale if point.scale is not None else 1.0
        if point_type in {"int", "s16"}:
            value: int | float = self._s16(raw_lo)
        elif point_type == "u16":
            value = self._u16(raw_lo)
        elif point_type in {"s32", "u32", "f32"}:
            raw_hi = int(raw_values[1]) if len(raw_values) > 1 else 0
            if point_type == "s32":
                value = self._s32(raw_lo, raw_hi)
            elif point_type == "u32":
                value = self._u32(raw_lo, raw_hi)
            else:
                value = struct.unpack("<f", struct.pack("<HH", self._u16(raw_lo), self._u16(raw_hi)))[0]
        else:
            value = raw_lo
        return value * scale

    def _encode_d(self, point: PointDefinition, value: Any) -> list[int]:
        point_type = point.type.lower()
        scale = point.scale if point.scale is not None else 1.0
        raw_value = float(value) / scale
        if point_type in {"int", "s16", "u16", "bit"}:
            return [int(round(raw_value))]
        if point_type in {"s32", "u32"}:
            raw32 = int(round(raw_value))
            return [raw32 & 0xFFFF, (raw32 >> 16) & 0xFFFF]
        if point_type == "f32":
            low, high = struct.unpack("<HH", struct.pack("<f", float(raw_value)))
            return [int(low), int(high)]
        raise PLCConnectionError(f"不支援的 D type: {point.type}")

    @staticmethod
    def _parse_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        normalized = str(value).strip().lower()
        if normalized in {"1", "true", "on", "yes"}:
            return True
        if normalized in {"0", "false", "off", "no"}:
            return False
        raise PLCConnectionError(f"無法轉成 bool: {value}")

    @staticmethod
    def _u16(value: int) -> int:
        return int(value) & 0xFFFF

    @staticmethod
    def _s16(value: int) -> int:
        raw = int(value) & 0xFFFF
        return raw - 0x10000 if raw >= 0x8000 else raw

    @classmethod
    def _u32(cls, low: int, high: int) -> int:
        return cls._u16(low) | (cls._u16(high) << 16)

    @classmethod
    def _s32(cls, low: int, high: int) -> int:
        raw = cls._u32(low, high)
        return raw - 0x100000000 if raw >= 0x80000000 else raw
