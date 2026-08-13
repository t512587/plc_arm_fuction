from __future__ import annotations

import struct
import threading
from typing import Any, Callable, Protocol, TypeVar

try:
    from config.loader import CONFIG_STORE, ConfigStore, PointDefinition
    from lifecycle import LifecycleTracked, tracked_operation
    from plc.client import MitsubishiPLCClient, PLCConnectionError
except ModuleNotFoundError:  # Support importing as plc2.service from repository root.
    from plc2.config.loader import CONFIG_STORE, ConfigStore, PointDefinition
    from plc2.lifecycle import LifecycleTracked, tracked_operation
    from plc2.plc.client import MitsubishiPLCClient, PLCConnectionError


T = TypeVar("T")
BLOCKED_M_ON_ADDRESSES = {389, 399}


class PlcClientProtocol(Protocol):
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def read_d_register(self, address: int, count: int = 1) -> list[int]: ...
    def write_d_register(self, address: int, values: list[int]) -> None: ...
    def read_bit_device(self, device: str, address: int, count: int = 1) -> list[bool]: ...
    def write_bit_device(self, device: str, address: int, values: list[bool]) -> None: ...


class PlcServiceError(RuntimeError):
    def __init__(self, operation: str, message: str, *, point_id: str | None = None) -> None:
        self.operation = operation
        self.point_id = point_id
        context = f" point={point_id}" if point_id else ""
        super().__init__(f"PlcService.{operation}{context}: {message}")


class PlcPointValidationError(PlcServiceError):
    """A point write was rejected before any PLC command was sent."""


class PlcService(LifecycleTracked):
    """Point-oriented PLC service. D/M addresses stay inside points.yml."""

    def __init__(
        self,
        config_store: ConfigStore = CONFIG_STORE,
        client_factory: Callable[[str, int, int], PlcClientProtocol] = MitsubishiPLCClient,
    ) -> None:
        self.config_store = config_store
        self.client_factory = client_factory
        self._clients: dict[str, PlcClientProtocol] = {}
        self._lock = threading.RLock()
        self._point_write_guards: list[Callable[[str, Any], None]] = []
        self._bit_write_guards: list[
            Callable[[str, int, list[bool]], None]
        ] = []
        self._d_write_guards: list[
            Callable[[str, int, list[int]], None]
        ] = []
        self._init_status_tracker("PlcService")

    def register_write_guard(
        self,
        *,
        point_guard: Callable[[str, Any], None] | None = None,
        bit_guard: Callable[[str, int, list[bool]], None] | None = None,
        d_guard: Callable[[str, int, list[int]], None] | None = None,
    ) -> None:
        if point_guard is not None and point_guard not in self._point_write_guards:
            self._point_write_guards.append(point_guard)
        if bit_guard is not None and bit_guard not in self._bit_write_guards:
            self._bit_write_guards.append(bit_guard)
        if d_guard is not None and d_guard not in self._d_write_guards:
            self._d_write_guards.append(d_guard)

    def is_connected(self, plc_name: str = "main_plc") -> bool:
        return plc_name in self._clients

    @tracked_operation("connect", "正在連線 PLC", "PLC 連線完成")
    def connect(self, plc_name: str = "main_plc") -> None:
        with self._lock:
            plc = self.config_store.get_plc(plc_name)
            if plc is None:
                raise PlcServiceError("connect", f"找不到 PLC 設定: {plc_name}")
            self.disconnect(plc_name)
            try:
                client = self.client_factory(plc.host, plc.port, plc.unit)
                client.connect()
            except Exception as exc:
                raise PlcServiceError("connect", f"{plc.host}:{plc.port} 連線失敗: {exc}") from exc
            self._clients[plc_name] = client

    @tracked_operation("disconnect", "正在中斷 PLC 連線", "PLC 已中斷連線")
    def disconnect(self, plc_name: str = "main_plc") -> None:
        with self._lock:
            client = self._clients.pop(plc_name, None)
            if client is not None:
                client.close()

    def get_point(self, point_id: str) -> PointDefinition:
        point = self.config_store.get_point(point_id)
        if point is None:
            raise PlcServiceError("get_point", "找不到點位設定", point_id=point_id)
        return point

    @tracked_operation("read_point", "正在讀取 PLC 點位", "PLC 點位讀取完成")
    def read_point(self, point_id: str) -> float | int | bool:
        point = self.get_point(point_id)
        self._require_client(point.plc)
        try:
            if point.device.upper() == "D":
                count = 2 if point.type.lower() in {"s32", "u32", "f32"} else 1
                raw = self._with_retry(point.plc, lambda item: item.read_d_register(point.address, count))
                return self._decode_d(point, raw)
            raw_bits = self._with_retry(
                point.plc,
                lambda item: item.read_bit_device(point.device.upper(), point.address, 1),
            )
            return bool(raw_bits[0]) if raw_bits else False
        except PlcServiceError:
            raise
        except Exception as exc:
            raise PlcServiceError("read_point", str(exc), point_id=point_id) from exc

    @tracked_operation("write_point", "正在寫入 PLC 點位", "PLC 點位寫入完成")
    def write_point(self, point_id: str, value: Any) -> None:
        for guard in self._point_write_guards:
            guard(point_id, value)
        point = self.get_point(point_id)
        if not point.writable:
            raise PlcPointValidationError("write_point", "點位不可寫入", point_id=point_id)
        self._validate_range(point, value)
        try:
            if point.device.upper() == "D":
                encoded = self._encode_d(point, value)
                self._require_client(point.plc)
                self._write_once(
                    point.plc,
                    lambda item: item.write_d_register(point.address, encoded),
                    point_id,
                )
                return
            parsed = self._parse_bool(value)
            self._require_client(point.plc)
            self._write_once(
                point.plc,
                lambda item: item.write_bit_device(point.device.upper(), point.address, [parsed]),
                point_id,
            )
        except PlcPointValidationError:
            raise
        except (TypeError, ValueError) as exc:
            raise PlcPointValidationError(
                "write_point",
                f"點位值格式錯誤: {value!r}",
                point_id=point_id,
            ) from exc
        except PlcServiceError:
            raise
        except Exception as exc:
            raise PlcServiceError("write_point", str(exc), point_id=point_id) from exc

    @tracked_operation("read_d_register", "正在讀取 D 暫存器", "D 暫存器讀取完成")
    def read_d_register(self, plc_name: str, address: int, count: int = 1) -> list[int]:
        return self._with_retry(
            plc_name,
            lambda client: client.read_d_register(address, count),
        )

    @tracked_operation("write_d_register", "正在寫入 D 暫存器", "D 暫存器寫入完成")
    def write_d_register(self, plc_name: str, address: int, values: list[int]) -> None:
        parsed_values = [int(value) for value in values]
        for guard in self._d_write_guards:
            guard(plc_name, address, parsed_values)
        self._write_once(
            plc_name,
            lambda client: client.write_d_register(address, parsed_values),
            f"D{address}",
        )

    @tracked_operation("read_bit_device", "正在讀取 M 點位", "M 點位讀取完成")
    def read_bit_device(
        self,
        plc_name: str,
        device: str,
        address: int,
        count: int = 1,
    ) -> list[bool]:
        return self._with_retry(
            plc_name,
            lambda client: client.read_bit_device(device, address, count),
        )

    @tracked_operation("write_bit_device", "正在寫入 M 點位", "M 點位寫入完成")
    def write_bit_device(
        self,
        plc_name: str,
        device: str,
        address: int,
        values: list[bool],
    ) -> None:
        parsed_values = [bool(value) for value in values]
        for guard in self._bit_write_guards:
            guard(device.upper(), address, parsed_values)
        if device.upper() == "M":
            blocked = [
                address + offset
                for offset, value in enumerate(parsed_values)
                if bool(value) and address + offset in BLOCKED_M_ON_ADDRESSES
            ]
            if blocked:
                points = ", ".join(f"M{item}" for item in blocked)
                raise PlcServiceError("write_bit_device", f"安全設定禁止啟動 {points}")
        self._write_once(
            plc_name,
            lambda client: client.write_bit_device(device, address, parsed_values),
            f"{device}{address}",
        )

    def _require_client(self, plc_name: str) -> PlcClientProtocol:
        client = self._clients.get(plc_name)
        if client is None:
            raise PlcServiceError("connection", f"PLC 尚未連線: {plc_name}")
        return client

    def _with_retry(self, plc_name: str, action: Callable[[PlcClientProtocol], T]) -> T:
        with self._lock:
            client = self._require_client(plc_name)
            try:
                return action(client)
            except PLCConnectionError as first_exc:
                self.disconnect(plc_name)
                try:
                    self.connect(plc_name)
                    return action(self._require_client(plc_name))
                except Exception as second_exc:
                    raise PlcServiceError(
                        "retry",
                        f"重連重試失敗；第一次={first_exc}；第二次={second_exc}",
                    ) from second_exc

    def _write_once(
        self,
        plc_name: str,
        action: Callable[[PlcClientProtocol], None],
        point_id: str,
    ) -> None:
        with self._lock:
            client = self._require_client(plc_name)
            try:
                action(client)
            except PLCConnectionError as exc:
                self.disconnect(plc_name)
                raise PlcServiceError(
                    "write_point",
                    f"通訊中斷，寫入結果未知，為避免重複動作未自動重送: {exc}",
                    point_id=point_id,
                ) from exc

    @staticmethod
    def _validate_range(point: PointDefinition, value: Any) -> None:
        if point.device.upper() != "D":
            return
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exc:
            raise PlcPointValidationError(
                "write_point",
                f"點位值格式錯誤: {value!r}",
                point_id=point.id,
            ) from exc
        if point.min is not None and numeric < float(point.min):
            raise PlcPointValidationError(
                "write_point",
                f"數值 {numeric:g} 低於下限 {point.min}",
                point_id=point.id,
            )
        if point.max is not None and numeric > float(point.max):
            raise PlcPointValidationError(
                "write_point",
                f"數值 {numeric:g} 高於上限 {point.max}",
                point_id=point.id,
            )

    @classmethod
    def _decode_d(cls, point: PointDefinition, values: list[int]) -> float | int:
        lo = int(values[0]) if values else 0
        hi = int(values[1]) if len(values) > 1 else 0
        point_type = point.type.lower()
        if point_type in {"int", "s16"}:
            raw: float | int = cls._s16(lo)
        elif point_type == "u16":
            raw = cls._u16(lo)
        elif point_type == "s32":
            raw = cls._s32(lo, hi)
        elif point_type == "u32":
            raw = cls._u32(lo, hi)
        elif point_type == "f32":
            raw = struct.unpack("<f", struct.pack("<HH", cls._u16(lo), cls._u16(hi)))[0]
        else:
            raise PlcServiceError("decode", f"不支援的 D type: {point.type}", point_id=point.id)
        scale = point.scale if point.scale is not None else 1.0
        value = raw * scale
        return int(value) if isinstance(raw, int) and float(scale) == 1.0 else float(value)

    @classmethod
    def _encode_d(cls, point: PointDefinition, value: Any) -> list[int]:
        scale = point.scale if point.scale is not None else 1.0
        raw_value = float(value) / scale
        point_type = point.type.lower()
        if point_type in {"int", "s16", "u16"}:
            return [int(round(raw_value))]
        if point_type in {"s32", "u32"}:
            raw32 = int(round(raw_value))
            return [raw32 & 0xFFFF, (raw32 >> 16) & 0xFFFF]
        if point_type == "f32":
            return list(struct.unpack("<HH", struct.pack("<f", float(raw_value))))
        raise PlcServiceError("encode", f"不支援的 D type: {point.type}", point_id=point.id)

    @staticmethod
    def _parse_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        normalized = str(value).strip().lower()
        if normalized in {"1", "true", "on", "yes", "是"}:
            return True
        if normalized in {"0", "false", "off", "no", "否", ""}:
            return False
        raise ValueError(f"無法解析 bit 值: {value!r}")

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


PLC_SERVICE = PlcService()
