from __future__ import annotations

import threading
import struct
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, TypeVar

import yaml

try:
    from plc.client import MitsubishiPLCClient, PLCConnectionConfig, PLCConnectionError
except ModuleNotFoundError as exc:  # Support importing as plc.service_plc from the repository root.
    if exc.name != "plc.client":
        raise
    from plc.plc.client import MitsubishiPLCClient, PLCConnectionConfig, PLCConnectionError


BASE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = BASE_DIR / "config" / "plc_config.yml"
POINTS_PATH = BASE_DIR / "config" / "points.yml"
T = TypeVar("T")
X_MINIMUM_HEIGHT_MM = 0.0
X_MAXIMUM_HEIGHT_MM = 1450.0
BLOCKED_M_ON_ADDRESSES = {389, 399}
VACUUM_INTERLOCK = {
    50: 51,
    51: 50,
    52: 53,
    53: 52,
    54: 55,
    55: 54,
}
ALLOWED_POINT_ALIASES = {
    ("D", 240): {"y1_forward_speed", "y2_forward_speed"},
}


def validate_x_motion_request(
    point_id: str,
    requested: bool,
    current_height_mm: float,
    *,
    minimum_height_mm: float = X_MINIMUM_HEIGHT_MM,
    maximum_height_mm: float = X_MAXIMUM_HEIGHT_MM,
) -> None:
    if not requested:
        return
    if point_id == "X_UP" and current_height_mm >= maximum_height_mm:
        raise PLCConnectionError(
            f"X 軸目前 {current_height_mm:g}mm，已達軟體上限 {maximum_height_mm:g}mm，禁止繼續上升"
        )
    if point_id == "X_DOWN" and current_height_mm <= minimum_height_mm:
        raise PLCConnectionError(
            f"X 軸目前 {current_height_mm:g}mm，已達軟體下限 {minimum_height_mm:g}mm，禁止繼續下降"
        )
@dataclass(frozen=True)
class PlcPoint:
    id: str
    name: str
    device: str
    address: int
    default: float | int | bool
    writable: bool = True
    type: str = "s16"
    scale: float = 1.0
    pair_high: int | None = None
    minimum: float | None = None
    maximum: float | None = None


@dataclass(frozen=True)
class PlcPointValue:
    point: PlcPoint
    value: float | int | bool | str
    error: str = ""
    raw: list[int] | list[bool] | None = None


def load_connections(path: Path = CONFIG_PATH) -> dict[str, PLCConnectionConfig]:
    raw = _load_yaml(path)
    connections: dict[str, PLCConnectionConfig] = {}
    for item in raw.get("connections", []):
        name = str(item["name"])
        connections[name] = PLCConnectionConfig(
            name=name,
            host=str(item["host"]),
            port=int(item["port"]),
            unit=int(item.get("unit", 0)),
        )
    return connections


def load_points(path: Path = POINTS_PATH) -> list[PlcPoint]:
    raw = _load_yaml(path)
    points: list[PlcPoint] = []
    for item in raw.get("points", []):
        device = str(item["device"]).upper()
        default: float | int | bool
        if device == "M":
            default = _parse_bool(item.get("default", 0))
        else:
            default = float(item.get("default", 0.0))
        point = PlcPoint(
                id=str(item["id"]),
                name=str(item["name"]),
                device=device,
                address=int(item["address"]),
                default=default,
                writable=bool(item.get("writable", True)),
                type=str(item.get("type", "bit" if device == "M" else "s16")),
                scale=float(item.get("scale", 1.0)),
                pair_high=int(item["pair_high"]) if item.get("pair_high") is not None else None,
                minimum=float(item["min"]) if item.get("min") is not None else None,
                maximum=float(item["max"]) if item.get("max") is not None else None,
            )
        if point.scale == 0:
            raise ValueError(f"PLC 點位 {point.id} scale 不可為 0")
        if point.minimum is not None and point.maximum is not None and point.minimum > point.maximum:
            raise ValueError(f"PLC 點位 {point.id} min 不可大於 max")
        points.append(point)
    _validate_point_identity(points, path)
    return points


def _validate_point_identity(points: list[PlcPoint], path: Path) -> None:
    ids: set[str] = set()
    by_address: dict[tuple[str, int], set[str]] = {}
    for point in points:
        if point.id in ids:
            raise ValueError(f"PLC 點位 ID 重複: {point.id} ({path})")
        ids.add(point.id)
        by_address.setdefault((point.device, point.address), set()).add(point.id)
    for key, point_ids in by_address.items():
        if len(point_ids) <= 1:
            continue
        if ALLOWED_POINT_ALIASES.get(key) == point_ids:
            continue
        device, address = key
        raise ValueError(
            f"PLC 實體位址 {device}{address} 被重複定義: {', '.join(sorted(point_ids))} ({path})"
        )


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"設定檔不存在: {path}")
    with path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    if not isinstance(data, dict):
        raise ValueError(f"設定檔格式錯誤: {path}")
    return data


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "on", "yes"}:
            return True
        if text in {"0", "false", "off", "no", ""}:
            return False
    raise ValueError(f"無法轉換為 switch 值: {value}")


class ServicePlc:
    """FastAPI-free PLC service for application use."""

    def __init__(
        self,
        plc_name: str = "main_plc",
        connections: dict[str, PLCConnectionConfig] | None = None,
        points: list[PlcPoint] | None = None,
    ) -> None:
        self.plc_name = plc_name
        self.connections = connections or load_connections()
        self.points = points or load_points()
        self.point_map = {point.id: point for point in self.points}
        self._client: MitsubishiPLCClient | None = None
        self._lock = threading.RLock()
        self._point_write_guards: list[Callable[[str, Any], None]] = []
        self._m_write_guards: list[Callable[[int, bool], None]] = []
        self.last_error = ""

    def register_write_guard(
        self,
        *,
        point_guard: Callable[[str, Any], None] | None = None,
        m_guard: Callable[[int, bool], None] | None = None,
    ) -> None:
        if point_guard is not None and point_guard not in self._point_write_guards:
            self._point_write_guards.append(point_guard)
        if m_guard is not None and m_guard not in self._m_write_guards:
            self._m_write_guards.append(m_guard)

    @property
    def is_connected(self) -> bool:
        return bool(self._client and self._client.is_connected)

    @property
    def connection(self) -> PLCConnectionConfig:
        try:
            return self.connections[self.plc_name]
        except KeyError as exc:
            raise PLCConnectionError(f"找不到 PLC 連線設定: {self.plc_name}") from exc

    def connect(self) -> str:
        with self._lock:
            self.disconnect()
            config = self.connection
            client = MitsubishiPLCClient(config.host, config.port, config.unit)
            client.connect()
            self._client = client
            return f"PLC 已連線 {config.name} {config.host}:{config.port}"

    def disconnect(self) -> str:
        with self._lock:
            if self._client is not None:
                self._client.close()
                self._client = None
            return "PLC 已斷線"

    def read_all_points(self) -> list[PlcPointValue]:
        values: list[PlcPointValue] = []
        for point in self.points:
            try:
                value, raw = self.read_point_with_raw(point.id)
                values.append(PlcPointValue(point, value, raw=raw))
            except Exception as exc:
                values.append(PlcPointValue(point, "-", str(exc)))
        return values

    def read_point(self, point_id: str) -> float | int | bool:
        value, _raw = self.read_point_with_raw(point_id)
        return value

    def read_point_with_raw(self, point_id: str) -> tuple[float | int | bool, list[int] | list[bool]]:
        point = self.get_point(point_id)
        if point.device == "D":
            return self._read_d_point_with_raw(point)
        if point.device == "M":
            raw = self._with_retry(lambda client: client.read_bit_device("M", point.address, 1))
            return (bool(raw[0]) if raw else False), raw
        raise PLCConnectionError(f"不支援裝置: {point.device}")

    def write_point(self, point_id: str, value: Any) -> None:
        point = self.get_point(point_id)
        for guard in self._point_write_guards:
            guard(point_id, value)
        if point_id in {"X_UP", "X_DOWN"}:
            requested = _parse_bool(value)
            if requested:
                current_height = float(self.read_point("x_current_pos"))
                validate_x_motion_request(point_id, requested, current_height)
        if not point.writable:
            raise PLCConnectionError(f"點位不可寫入: {point.name} ({point.id})")
        if point.device == "D":
            self._write_d_point(point, value)
            return
        if point.device == "M":
            self.write_m(point.address, _parse_bool(value))
            return
        raise PLCConnectionError(f"不支援裝置: {point.device}")

    def get_point(self, point_id: str) -> PlcPoint:
        try:
            return self.point_map[point_id]
        except KeyError as exc:
            raise KeyError(f"找不到點位 ID: {point_id}") from exc

    def read_d(self, address: int) -> float:
        values = self._with_retry(lambda client: client.read_d_register(address, 1))
        return float(values[0]) if values else 0.0

    def write_d(self, address: int, value: float | int) -> None:
        parsed = float(value)
        if not math.isfinite(parsed):
            raise PLCConnectionError(f"D{address} 寫入值必須是有限數值")
        encoded = int(round(parsed))
        if not -32768 <= encoded <= 65535:
            raise PLCConnectionError(f"D{address} 寫入值 {parsed:g} 超出單字範圍")
        self._write_once(lambda client: client.write_d_register(address, [encoded & 0xFFFF]), f"D{address}")

    def _read_d_point(self, point: PlcPoint) -> float:
        value, _raw = self._read_d_point_with_raw(point)
        return value

    def _read_d_point_with_raw(self, point: PlcPoint) -> tuple[float, list[int]]:
        word_count = 2 if point.type.lower() in {"s32", "u32", "f32"} else 1
        values = self._with_retry(lambda client: client.read_d_register(point.address, word_count))
        raw = self._decode_d(point, values)
        return float(raw) * point.scale, values

    def _write_d_point(self, point: PlcPoint, value: Any) -> None:
        parsed = float(value)
        self._validate_d_value(point, parsed)
        raw_value = parsed / point.scale
        values = self._encode_d(point, raw_value)
        self._write_once(
            lambda client: client.write_d_register(point.address, values),
            point.id,
        )

    def _validate_d_value(self, point: PlcPoint, value: float) -> None:
        if not math.isfinite(value):
            raise PLCConnectionError(f"{point.id} 寫入值必須是有限數值")
        if point.scale == 0:
            raise PLCConnectionError(f"{point.id} scale 不可為 0")
        if point.minimum is not None and value < point.minimum:
            raise PLCConnectionError(f"{point.id}={value:g} 低於允許下限 {point.minimum:g}")
        if point.maximum is not None and value > point.maximum:
            raise PLCConnectionError(f"{point.id}={value:g} 超過允許上限 {point.maximum:g}")

        raw = value / point.scale
        point_type = point.type.lower()
        limits = {
            "int": (-32768, 32767),
            "s16": (-32768, 32767),
            "u16": (0, 65535),
            "s32": (-2147483648, 2147483647),
            "u32": (0, 4294967295),
        }
        if point_type in limits:
            minimum, maximum = limits[point_type]
            if not minimum <= round(raw) <= maximum:
                raise PLCConnectionError(
                    f"{point.id}={value:g} 超出 {point.type} 可表示範圍"
                )

    def _decode_d(self, point: PlcPoint, values: list[int]) -> int | float:
        lo = int(values[0]) if values else 0
        hi = int(values[1]) if len(values) > 1 else 0
        point_type = point.type.lower()
        if point_type in {"int", "s16"}:
            return self._s16(lo)
        if point_type == "u16":
            return self._u16(lo)
        if point_type == "s32":
            return self._s32(lo, hi)
        if point_type == "u32":
            return self._u32(lo, hi)
        if point_type == "f32":
            return struct.unpack("<f", struct.pack("<HH", self._u16(lo), self._u16(hi)))[0]
        raise PLCConnectionError(f"不支援 D 型態: {point.type}")

    def _encode_d(self, point: PlcPoint, raw_value: float) -> list[int]:
        point_type = point.type.lower()
        if point_type in {"int", "s16", "u16"}:
            return [int(round(raw_value)) & 0xFFFF]
        if point_type in {"s32", "u32"}:
            raw = int(round(raw_value)) & 0xFFFFFFFF
            return [raw & 0xFFFF, (raw >> 16) & 0xFFFF]
        if point_type == "f32":
            return list(struct.unpack("<HH", struct.pack("<f", float(raw_value))))
        raise PLCConnectionError(f"不支援 D 型態: {point.type}")

    @staticmethod
    def _u16(value: int) -> int:
        return int(value) & 0xFFFF

    @classmethod
    def _s16(cls, value: int) -> int:
        value = cls._u16(value)
        return value - 0x10000 if value & 0x8000 else value

    @classmethod
    def _u32(cls, lo: int, hi: int) -> int:
        return (cls._u16(hi) << 16) | cls._u16(lo)

    @classmethod
    def _s32(cls, lo: int, hi: int) -> int:
        value = cls._u32(lo, hi)
        return value - 0x100000000 if value & 0x80000000 else value

    def read_m(self, address: int) -> bool:
        values = self._with_retry(lambda client: client.read_bit_device("M", address, 1))
        return bool(values[0]) if values else False

    def write_m(self, address: int, value: bool) -> None:
        address = int(address)
        requested = bool(value)
        for guard in self._m_write_guards:
            guard(address, requested)
        if address in BLOCKED_M_ON_ADDRESSES and requested:
            raise PLCConnectionError(f"安全設定禁止啟動 M{address}")
        if address in VACUUM_INTERLOCK and requested:
            other_address = VACUUM_INTERLOCK[address]
            if self.read_m(other_address):
                raise PLCConnectionError(
                    f"真空互鎖：M{address} 與 M{other_address} 不可同時 ON，請先關閉另一點位"
                )
        self._write_once(
            lambda client: client.write_bit_device("M", address, [requested]),
            f"M{address}",
        )

    def toggle_m_point(self, point_id: str) -> bool:
        point = self.get_point(point_id)
        if point.device != "M":
            raise PLCConnectionError(f"不是 M switch 點位: {point_id}")
        new_value = not self.read_m(point.address)
        self.write_m(point.address, new_value)
        return new_value

    def _require_client(self) -> MitsubishiPLCClient:
        if self._client is None or not self._client.is_connected:
            raise PLCConnectionError("PLC 尚未連線")
        return self._client

    def _with_retry(self, action: Callable[[MitsubishiPLCClient], T]) -> T:
        with self._lock:
            client = self._require_client()
            try:
                return action(client)
            except PLCConnectionError as first_exc:
                self.last_error = str(first_exc)
                self.disconnect()
                self.connect()
                client = self._require_client()
                try:
                    return action(client)
                except PLCConnectionError as second_exc:
                    self.last_error = str(second_exc)
                    raise second_exc from first_exc

    def _write_once(
        self,
        action: Callable[[MitsubishiPLCClient], None],
        point_label: str,
    ) -> None:
        with self._lock:
            client = self._require_client()
            try:
                action(client)
            except PLCConnectionError as exc:
                self.last_error = str(exc)
                self.disconnect()
                raise PLCConnectionError(
                    f"{point_label} 通訊中斷，寫入結果未知；為避免重複動作，未自動重送: {exc}"
                ) from exc


def main() -> None:
    service = ServicePlc()
    print(service.connect())
    for item in service.read_all_points():
        print(item.point.id, item.point.name, item.point.device, item.point.address, item.value, item.error)
    print(service.disconnect())


if __name__ == "__main__":
    main()
