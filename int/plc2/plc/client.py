"""PLC 通訊層：封裝三菱 PLC 讀寫邏輯（MC Protocol）。

使用 pymcprotocol 與三菱 PLC 通訊。
目前使用 batchread_/batchwrite_ 介面，並依你的 pymcprotocol 版本採用
「頭地址字串 + 長度」的呼叫方式（例如 "D300", size）。
"""

from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import List, Optional, Any

try:
    import pymcprotocol
except ImportError:  # pragma: no cover - 若尚未安裝 pymcprotocol
    pymcprotocol = None


class PLCConnectionError(Exception):
    """PLC 連線 / 通訊相關錯誤。"""


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


@dataclass
class PLCConnectionConfig:
    name: str
    host: str
    port: int
    unit: int = 0


class MitsubishiPLCClient:
    """三菱 PLC Client，透過 MC Protocol 通訊。"""

    def __init__(self, host: str, port: int, unit: int = 0) -> None:
        if pymcprotocol is None:
            raise PLCConnectionError(
                "pymcprotocol 未安裝，請先安裝: pip install pymcprotocol"
            )

        self._host = host
        self._port = port
        self._unit = unit
        # _client 實際型別是 pymcprotocol.Type3E，但這裡用 Any 避免型別註解造成語法問題
        self._client: Optional[Any] = None

    def connect(self) -> None:
        """建立與 PLC 的連線。"""

        try:
            self._client = pymcprotocol.Type3E()
            self._client.soc_timeout = 5
            # 使用預設 access 設定
            self._client.connect(self._host, self._port)
        except ConnectionRefusedError as exc:
            raise PLCConnectionError(
                f"無法連線到 PLC {self._host}:{self._port}: TCP 連線被拒絕。"
                "請確認 PLC/轉接器已啟用 MC Protocol TCP server、port 設定正確，"
                "且沒有只開 UDP 或限制來源 IP。"
            ) from exc
        except TimeoutError as exc:
            raise PLCConnectionError(
                f"無法連線到 PLC {self._host}:{self._port}: TCP 連線逾時。"
                "請確認 PLC IP、網路路由、防火牆與設備電源狀態。"
            ) from exc
        except OSError as exc:
            if exc.errno == 113:
                detail = "沒有到目標主機的路由"
            elif exc.errno == 101:
                detail = "網路不可達"
            else:
                detail = str(exc)
            raise PLCConnectionError(
                f"無法連線到 PLC {self._host}:{self._port}: {detail}"
            ) from exc
        except socket.timeout as exc:
            raise PLCConnectionError(
                f"無法連線到 PLC {self._host}:{self._port}: TCP 連線逾時。"
                "請確認 PLC IP、網路路由、防火牆與設備電源狀態。"
            ) from exc
        except Exception as exc:  # pylint: disable=broad-except
            raise PLCConnectionError(
                f"無法連線到 PLC {self._host}:{self._port}: {exc}"
            ) from exc

    def close(self) -> None:
        """關閉連線。"""

        if self._client is not None:
            try:
                self._client.close()
            except Exception:  # pragma: no cover - 關閉失敗通常可以忽略
                pass
            finally:
                self._client = None

    # ===== D 暫存器 =====

    def read_d_register(self, address: int, count: int = 1) -> List[int]:
        """讀取 D 暫存器。"""

        if self._client is None:
            raise PLCConnectionError("PLC 尚未連線")

        try:
            # 你的 pymcprotocol 版本：batchread_wordunits(headdevice, size)
            head = f"D{address}"
            data = self._client.batchread_wordunits(head, count)
            return [int(v) for v in data]
        except Exception as exc:  # pylint: disable=broad-except
            if _is_socket_disconnect(exc):
                self.close()
            raise PLCConnectionError(f"讀取 D{address} 失敗: {exc}") from exc

    def write_d_register(self, address: int, values: List[int]) -> None:
        """寫入 D 暫存器。"""

        if self._client is None:
            raise PLCConnectionError("PLC 尚未連線")

        try:
            head = f"D{address}"
            # batchwrite_wordunits(headdevice, values)
            self._client.batchwrite_wordunits(head, values)
        except Exception as exc:  # pylint: disable=broad-except
            if _is_socket_disconnect(exc):
                self.close()
            raise PLCConnectionError(f"寫入 D{address} 失敗: {exc}") from exc

    # ===== Bit 類裝置 (M/X/Y...) =====

    def read_bit_device(self, device: str, address: int, count: int = 1) -> List[bool]:
        """讀取 bit 類型裝置（如 M/X/Y）。"""

        if self._client is None:
            raise PLCConnectionError("PLC 尚未連線")

        try:
            # 你的 pymcprotocol 版本：batchread_bitunits(headdevice, size)
            head = f"{device}{address}"
            raw = self._client.batchread_bitunits(head, count)
            return [bool(v) for v in raw]
        except Exception as exc:  # pylint: disable=broad-except
            if _is_socket_disconnect(exc):
                self.close()
            raise PLCConnectionError(f"讀取 {device}{address} 失敗: {exc}") from exc

    def write_bit_device(
        self,
        device: str,
        address: int,
        values: List[bool],
    ) -> None:
        """寫入 bit 類型裝置（如 M/X/Y）。"""

        if self._client is None:
            raise PLCConnectionError("PLC 尚未連線")

        try:
            int_values = [1 if v else 0 for v in values]
            head = f"{device}{address}"
            # batchwrite_bitunits(headdevice, values)
            self._client.batchwrite_bitunits(head, int_values)
        except Exception as exc:  # pylint: disable=broad-except
            if _is_socket_disconnect(exc):
                self.close()
            raise PLCConnectionError(f"寫入 {device}{address} 失敗: {exc}") from exc
