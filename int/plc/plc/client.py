from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import Any

try:
    import pymcprotocol
except ImportError:  # pragma: no cover
    pymcprotocol = None


class PLCConnectionError(Exception):
    """PLC connection or communication error."""


def _is_socket_disconnect(exc: Exception) -> bool:
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


@dataclass(frozen=True)
class PLCConnectionConfig:
    name: str
    host: str
    port: int
    unit: int = 0


class MitsubishiPLCClient:
    """Mitsubishi MC Protocol client wrapper."""

    def __init__(
        self,
        host: str,
        port: int,
        unit: int = 0,
        timeout_seconds: float = 5.0,
        plc_type: str = "Q",
    ) -> None:
        if pymcprotocol is None:
            raise PLCConnectionError("pymcprotocol is not installed.")
        self._host = host
        self._port = port
        self._unit = unit
        self._timeout_seconds = timeout_seconds
        self._plc_type = plc_type
        self._client: Any | None = None

    @property
    def is_connected(self) -> bool:
        return self._client is not None

    def connect(self) -> None:
        try:
            self._client = pymcprotocol.Type3E(plctype=self._plc_type)
            self._client.soc_timeout = self._timeout_seconds
            self._client.setaccessopt(timer_sec=int(self._timeout_seconds))
            self._client.connect(self._host, self._port)
        except ConnectionRefusedError as exc:
            raise PLCConnectionError(f"PLC {self._host}:{self._port} refused TCP connection.") from exc
        except (TimeoutError, socket.timeout) as exc:
            raise PLCConnectionError(f"PLC {self._host}:{self._port} TCP connection timed out.") from exc
        except OSError as exc:
            raise PLCConnectionError(f"PLC {self._host}:{self._port} connection failed: {exc}") from exc
        except Exception as exc:
            raise PLCConnectionError(f"PLC {self._host}:{self._port} connection failed: {exc}") from exc

    def close(self) -> None:
        if self._client is None:
            return
        try:
            self._client.close()
        except Exception:
            pass
        finally:
            self._client = None

    def read_d_register(self, address: int, count: int = 1) -> list[int]:
        client = self._require_client()
        try:
            data = client.batchread_wordunits(f"D{int(address)}", int(count))
            return [int(value) for value in data]
        except Exception as exc:
            if _is_socket_disconnect(exc):
                self.close()
            raise PLCConnectionError(f"讀取 D{address} 失敗: {exc}") from exc

    def write_d_register(self, address: int, values: list[int]) -> None:
        client = self._require_client()
        try:
            client.batchwrite_wordunits(f"D{int(address)}", [int(value) for value in values])
        except Exception as exc:
            if _is_socket_disconnect(exc):
                self.close()
            raise PLCConnectionError(f"寫入 D{address} 失敗: {exc}") from exc

    def read_bit_device(self, device: str, address: int, count: int = 1) -> list[bool]:
        client = self._require_client()
        device = device.upper()
        try:
            data = client.batchread_bitunits(f"{device}{int(address)}", int(count))
            return [bool(value) for value in data]
        except Exception as exc:
            if _is_socket_disconnect(exc):
                self.close()
            raise PLCConnectionError(f"讀取 {device}{address} 失敗: {exc}") from exc

    def write_bit_device(self, device: str, address: int, values: list[bool]) -> None:
        client = self._require_client()
        device = device.upper()
        try:
            client.batchwrite_bitunits(f"{device}{int(address)}", [1 if value else 0 for value in values])
        except Exception as exc:
            if _is_socket_disconnect(exc):
                self.close()
            raise PLCConnectionError(f"寫入 {device}{address} 失敗: {exc}") from exc

    def _require_client(self) -> Any:
        if self._client is None:
            raise PLCConnectionError("PLC 尚未連線")
        return self._client
