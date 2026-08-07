from __future__ import annotations

import json
import logging
import threading
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Iterator, Protocol


STATE_PATH = Path(__file__).resolve().parent / "config" / "slot_states.json"
LOGGER = logging.getLogger(__name__)


class PointServiceProtocol(Protocol):
    @property
    def is_connected(self) -> bool: ...

    def get_point(self, point_id: str): ...

    def read_point(self, point_id: str) -> float | int | bool: ...

    def write_point(self, point_id: str, value: Any) -> None: ...

    def register_write_guard(self, **kwargs: Any) -> None: ...


class SlotOccupancy(str, Enum):
    UNKNOWN = "unknown"
    OCCUPIED = "occupied"
    EMPTY = "empty"


@dataclass(frozen=True)
class SlotDefinition:
    side: str
    vacuum_point: str
    break_vacuum_point: str


SLOTS = {
    "Y1": SlotDefinition("Y1", "L_VAC_ON", "L_VAC_REL"),
    "Y2": SlotDefinition("Y2", "R_VAC_ON", "R_VAC_REL"),
}


class SlotVacuumService:
    def __init__(
        self,
        plc_service: PointServiceProtocol,
        state_path: Path = STATE_PATH,
    ) -> None:
        self.plc_service = plc_service
        self.state_path = state_path
        self._lock = threading.RLock()
        self._override = threading.local()
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._states = self._load_states()
        self.plc_service.register_write_guard(
            point_guard=self._guard_point_write,
            m_guard=self._guard_m_write,
        )

    def list_states(self) -> dict[str, str]:
        with self._lock:
            return {side: state.value for side, state in self._states.items()}

    def set_occupancy(
        self,
        side: str,
        occupancy: SlotOccupancy | str,
        *,
        confirm_release: bool = False,
    ) -> dict[str, str]:
        normalized = self._normalize_side(side)
        selected = occupancy if isinstance(occupancy, SlotOccupancy) else SlotOccupancy(str(occupancy))
        with self._lock:
            previous = self._states[normalized]
            if previous is SlotOccupancy.OCCUPIED and selected is not SlotOccupancy.OCCUPIED:
                if not confirm_release:
                    raise RuntimeError(f"{normalized} 目前有貨；必須確認貨物已移除才能解除真空保護")
            if selected is SlotOccupancy.OCCUPIED:
                self._ensure_vacuum(normalized)
            self._states[normalized] = selected
            self._save_states()
            return self.list_states()

    def enforce_required_vacuums(self) -> dict[str, str]:
        with self._lock:
            for side, state in self._states.items():
                if state is SlotOccupancy.OCCUPIED:
                    self._ensure_vacuum(side)
            return self.list_states()

    def start_watchdog(self, interval_seconds: float = 1.0) -> None:
        if interval_seconds <= 0:
            raise ValueError("真空守護週期必須大於 0")
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            return
        self._watchdog_stop.clear()

        def worker() -> None:
            while not self._watchdog_stop.wait(interval_seconds):
                if not self.plc_service.is_connected:
                    continue
                try:
                    self.enforce_required_vacuums()
                except Exception:
                    LOGGER.exception("載貨側真空守護檢查失敗")

        self._watchdog_thread = threading.Thread(
            target=worker,
            daemon=True,
            name="plc-slot-vacuum-watchdog",
        )
        self._watchdog_thread.start()

    def stop_watchdog(self) -> None:
        self._watchdog_stop.set()
        thread = self._watchdog_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._watchdog_thread = None

    def _ensure_vacuum(self, side: str) -> None:
        slot = SLOTS[side]
        if bool(self.plc_service.read_point(slot.break_vacuum_point)):
            self.plc_service.write_point(slot.break_vacuum_point, False)
        if not bool(self.plc_service.read_point(slot.vacuum_point)):
            self.plc_service.write_point(slot.vacuum_point, True)
        if not bool(self.plc_service.read_point(slot.vacuum_point)):
            raise RuntimeError(f"{side} 真空命令讀回仍為 OFF，載貨狀態未更新")

    def _guard_point_write(self, point_id: str, value: Any) -> None:
        if getattr(self._override, "enabled", False):
            return
        point = self.plc_service.get_point(point_id)
        if str(point.device).upper() != "M":
            return
        self._guard_m_write(int(point.address), self._parse_bool(value))

    def _guard_m_write(self, address: int, requested: bool) -> None:
        if getattr(self._override, "enabled", False):
            return
        for side, state in self._states.items():
            if state is not SlotOccupancy.OCCUPIED:
                continue
            slot = SLOTS[side]
            vacuum_address = int(self.plc_service.get_point(slot.vacuum_point).address)
            break_address = int(self.plc_service.get_point(slot.break_vacuum_point).address)
            if address == vacuum_address and not requested:
                raise RuntimeError(f"{side} 有貨，禁止關閉真空 M{address}")
            if address == break_address and requested:
                raise RuntimeError(f"{side} 有貨，禁止啟動破真空 M{address}")

    def _load_states(self) -> dict[str, SlotOccupancy]:
        defaults = {side: SlotOccupancy.UNKNOWN for side in SLOTS}
        try:
            raw = json.loads(self.state_path.read_text(encoding="utf-8"))
            for side in SLOTS:
                defaults[side] = SlotOccupancy(str(raw.get(side, SlotOccupancy.UNKNOWN.value)))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass
        return defaults

    def _save_states(self) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {side: state.value for side, state in self._states.items()}
        temporary = self.state_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(self.state_path)

    @staticmethod
    def _normalize_side(side: str) -> str:
        normalized = str(side).strip().upper()
        if normalized not in SLOTS:
            raise ValueError(f"不支援的載貨側別: {side}")
        return normalized

    @staticmethod
    def _parse_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in {"1", "true", "on", "yes"}

    @contextmanager
    def confirmed_release_override(self) -> Iterator[None]:
        previous = getattr(self._override, "enabled", False)
        self._override.enabled = True
        try:
            yield
        finally:
            self._override.enabled = previous
