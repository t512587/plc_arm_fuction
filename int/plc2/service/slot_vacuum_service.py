from __future__ import annotations

import json
import threading
import time
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Any, Protocol

try:
    from config.loader import CONFIG_STORE, ConfigStore
    from lifecycle import LifecycleStatus, LifecycleTracked
except ModuleNotFoundError:
    from plc2.config.loader import CONFIG_STORE, ConfigStore
    from plc2.lifecycle import LifecycleStatus, LifecycleTracked


BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_STATE_PATH = BASE_DIR / "runtime" / "slot_states.json"


class PointServiceProtocol(Protocol):
    def is_connected(self, plc_name: str = "main_plc") -> bool: ...
    def get_point(self, point_id: str): ...
    def read_point(self, point_id: str) -> float | int | bool: ...
    def write_point(self, point_id: str, value: Any) -> None: ...
    def register_write_guard(self, **kwargs: Any) -> None: ...


class SlotOccupancy(str, Enum):
    UNKNOWN = "unknown"
    EMPTY = "empty"
    OCCUPIED = "occupied"


class SlotVacuumMode(str, Enum):
    OFF = "off"
    VACUUM = "vacuum"
    BREAK_VACUUM = "break_vacuum"


class CargoPurpose(str, Enum):
    UNKNOWN = "unknown"
    TARGET_TO_PICK = "target_to_pick"
    WAITING_ROBOT = "waiting_robot"
    PICKED_RETURN = "picked_return"
    UNRELATED = "unrelated"


class SlotVacuumServiceError(RuntimeError):
    def __init__(self, operation: str, message: str, *, side: str | None = None) -> None:
        context = f" side={side}" if side else ""
        super().__init__(f"SlotVacuumService.{operation}{context}: {message}")


@dataclass(frozen=True)
class SlotState:
    side: str
    occupancy: SlotOccupancy = SlotOccupancy.UNKNOWN
    cargo_id: str | None = None
    purpose: CargoPurpose = CargoPurpose.UNKNOWN
    shelf_id: str | None = None
    shelf_level: int | None = None
    vacuum_required: bool = False
    updated_at: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        item = asdict(self)
        item["occupancy"] = self.occupancy.value
        item["purpose"] = self.purpose.value
        return item


@dataclass(frozen=True)
class SlotVacuumConfig:
    slots: dict[str, dict[str, str]]
    enforcement_interval_seconds: float = 1.0
    release_pulse_seconds: float = 4.0

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "SlotVacuumConfig":
        raw = store.get_service("slot_vacuum")
        if raw is None:
            raise SlotVacuumServiceError("config", "找不到 services.yml 的 slot_vacuum 設定")
        slots = raw.get("slots")
        if not isinstance(slots, dict) or set(slots) != {"Y1", "Y2"}:
            raise SlotVacuumServiceError("config", "slots 必須包含 Y1 與 Y2")
        normalized: dict[str, dict[str, str]] = {}
        for side, item in slots.items():
            if not isinstance(item, dict) or not {
                "vacuum_point",
                "break_vacuum_point",
            }.issubset(item):
                raise SlotVacuumServiceError("config", "真空點位設定不完整", side=str(side))
            normalized[str(side)] = {
                "vacuum_point": str(item["vacuum_point"]),
                "break_vacuum_point": str(item["break_vacuum_point"]),
            }
        interval = float(raw.get("enforcement_interval_seconds", 1.0))
        if interval <= 0:
            raise SlotVacuumServiceError("config", "enforcement_interval_seconds 必須大於 0")
        release_pulse = float(raw.get("release_pulse_seconds", 4.0))
        if release_pulse < 0:
            raise SlotVacuumServiceError("config", "release_pulse_seconds 不可小於 0")
        return cls(normalized, interval, release_pulse)


class SlotStateStore:
    def __init__(self, path: Path = DEFAULT_STATE_PATH) -> None:
        self.path = path
        self._lock = threading.RLock()
        self._states = self._load()

    def get(self, side: str) -> SlotState:
        with self._lock:
            return self._states[side]

    def all(self) -> dict[str, SlotState]:
        with self._lock:
            return dict(self._states)

    def save(self, state: SlotState) -> None:
        with self._lock:
            self._states[state.side] = state
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(".tmp")
            temporary.write_text(
                json.dumps(
                    {side: item.as_dict() for side, item in self._states.items()},
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )
            temporary.replace(self.path)

    def _load(self) -> dict[str, SlotState]:
        defaults = {side: SlotState(side=side) for side in ("Y1", "Y2")}
        if not self.path.exists():
            return defaults
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
            for side in defaults:
                item = raw.get(side, {})
                occupancy = SlotOccupancy(str(item.get("occupancy", "unknown")))
                defaults[side] = SlotState(
                    side=side,
                    occupancy=occupancy,
                    cargo_id=item.get("cargo_id"),
                    purpose=CargoPurpose(str(item.get("purpose", "unknown"))),
                    shelf_id=item.get("shelf_id"),
                    shelf_level=item.get("shelf_level"),
                    vacuum_required=(
                        occupancy is SlotOccupancy.OCCUPIED
                        or bool(item.get("vacuum_required", False))
                    ),
                    updated_at=float(item.get("updated_at", 0.0)),
                )
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return defaults
        return defaults


class SlotVacuumService(LifecycleTracked):
    def __init__(
        self,
        plc_service: PointServiceProtocol,
        config: SlotVacuumConfig | None = None,
        state_store: SlotStateStore | None = None,
    ) -> None:
        self.plc_service = plc_service
        self.config = config or SlotVacuumConfig.load()
        self.state_store = state_store or SlotStateStore()
        self._stop_event = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        self._release_override = threading.local()
        self._init_status_tracker("SlotVacuumService")
        register_guard = getattr(self.plc_service, "register_write_guard", None)
        if callable(register_guard):
            register_guard(
                point_guard=self._guard_point_write,
                bit_guard=self._guard_bit_write,
            )

    def list_states(self, *, include_vacuum: bool = True) -> dict[str, dict[str, Any]]:
        return {
            side: self.read_side_state(side, include_vacuum=include_vacuum)
            for side in self.state_store.all()
        }

    def read_side_state(
        self,
        side: str,
        *,
        include_vacuum: bool = True,
    ) -> dict[str, Any]:
        side = self._normalize_side(side)
        item = self.state_store.get(side).as_dict()
        item["vacuum_on"] = None
        item["break_vacuum_on"] = None
        if include_vacuum and self.plc_service.is_connected():
            points = self.config.slots[side]
            item["vacuum_on"] = bool(self.plc_service.read_point(points["vacuum_point"]))
            item["break_vacuum_on"] = bool(
                self.plc_service.read_point(points["break_vacuum_point"])
            )
        return item

    def set_manual_mode(
        self,
        side: str,
        mode: SlotVacuumMode | str,
        *,
        confirm_release: bool = False,
    ) -> dict[str, Any]:
        """Set one Y-side vacuum mode from an operator control."""

        side = self._normalize_side(side)
        selected = mode if isinstance(mode, SlotVacuumMode) else SlotVacuumMode(str(mode))
        self._set_status(
            LifecycleStatus.RUNNING,
            "set_manual_mode",
            f"操作員切換 {side} 真空模式為 {selected.value}",
        )
        try:
            if not self.plc_service.is_connected():
                raise SlotVacuumServiceError(
                    "set_manual_mode",
                    "PLC 尚未連線",
                    side=side,
                )

            previous = self.state_store.get(side)
            if selected is SlotVacuumMode.VACUUM:
                self._set_side_outputs(side, vacuum=True, break_vacuum=False)
            else:
                if not confirm_release:
                    raise SlotVacuumServiceError(
                        "set_manual_mode",
                        "關閉真空或啟動破真空必須明確確認貨物可安全釋放",
                        side=side,
                    )
                released_state = replace(
                    previous,
                    occupancy=SlotOccupancy.EMPTY,
                    cargo_id=None,
                    purpose=CargoPurpose.UNKNOWN,
                    shelf_id=None,
                    shelf_level=None,
                    vacuum_required=False,
                    updated_at=time.time(),
                )
                # Clear the persistent hold before changing outputs so the
                # watchdog cannot re-enable suction after operator confirmation.
                self.state_store.save(released_state)
                with self._allow_confirmed_release():
                    self._set_side_outputs(
                        side,
                        vacuum=False,
                        break_vacuum=(selected is SlotVacuumMode.BREAK_VACUUM),
                    )

            state = self.read_side_state(side, include_vacuum=True)
            vacuum_on = bool(state.get("vacuum_on"))
            break_vacuum_on = bool(state.get("break_vacuum_on"))
            state["mode"] = (
                "invalid"
                if vacuum_on and break_vacuum_on
                else SlotVacuumMode.VACUUM.value
                if vacuum_on
                else SlotVacuumMode.BREAK_VACUUM.value
                if break_vacuum_on
                else SlotVacuumMode.OFF.value
            )
        except Exception as exc:
            self._set_status(
                LifecycleStatus.ERROR,
                "set_manual_mode",
                str(exc),
                side=side,
            )
            raise
        self._set_status(
            LifecycleStatus.SUCCESS,
            "set_manual_mode",
            f"{side} 真空模式已切換為 {state['mode']}",
            side=side,
            mode=state["mode"],
        )
        return state

    def update_state(
        self,
        side: str,
        occupancy: SlotOccupancy,
        *,
        cargo_id: str | None = None,
        purpose: CargoPurpose = CargoPurpose.UNKNOWN,
        shelf_id: str | None = None,
        shelf_level: int | None = None,
        confirm_release: bool = False,
    ) -> SlotState:
        side = self._normalize_side(side)
        self._set_status(LifecycleStatus.RUNNING, "update_state", f"更新 {side} 載貨狀態")
        try:
            if not self.plc_service.is_connected():
                raise SlotVacuumServiceError("update_state", "PLC 尚未連線", side=side)
            previous = self.state_store.get(side)
            vacuum_required = (
                occupancy is SlotOccupancy.OCCUPIED
                or (
                    occupancy is SlotOccupancy.UNKNOWN
                    and previous.vacuum_required
                )
            )
            if vacuum_required:
                self._ensure_side_vacuum(side)
            elif occupancy is SlotOccupancy.EMPTY:
                if not confirm_release:
                    raise SlotVacuumServiceError(
                        "update_state",
                        "標記 empty 會關閉該側真空，必須明確確認釋放",
                        side=side,
                    )
                with self._allow_confirmed_release():
                    self._set_side_outputs(side, vacuum=False, break_vacuum=False)
                cargo_id = None
                purpose = CargoPurpose.UNKNOWN
                shelf_id = None
                shelf_level = None

            state = SlotState(
                side=side,
                occupancy=occupancy,
                cargo_id=(cargo_id or "").strip() or None,
                purpose=purpose,
                shelf_id=(shelf_id or "").strip() or None,
                shelf_level=shelf_level,
                vacuum_required=vacuum_required,
                updated_at=time.time(),
            )
            self.state_store.save(state)
        except Exception as exc:
            self._set_status(LifecycleStatus.ERROR, "update_state", str(exc), side=side)
            raise
        self._set_status(
            LifecycleStatus.SUCCESS,
            "update_state",
            f"{side} 已更新為 {occupancy.value}",
            side=side,
        )
        return state

    def enforce_required_vacuum(self) -> dict[str, bool]:
        enforced: dict[str, bool] = {}
        if not self.plc_service.is_connected():
            return enforced
        self._set_status(LifecycleStatus.RUNNING, "enforce", "檢查載貨側真空")
        try:
            for side, state in self.state_store.all().items():
                if state.vacuum_required:
                    self._ensure_side_vacuum(side)
                    enforced[side] = True
        except Exception as exc:
            self._set_status(LifecycleStatus.ERROR, "enforce", str(exc))
            raise
        self._set_status(
            LifecycleStatus.SUCCESS,
            "enforce",
            "載貨側真空檢查完成",
            enforced=enforced,
        )
        return enforced

    def hold_for_main_cycle(self, side: str) -> SlotState:
        """Latch one side's vacuum until a later explicit shelf-release action."""

        side = self._normalize_side(side)
        self._set_status(
            LifecycleStatus.RUNNING,
            "main_cycle_hold",
            f"總流程要求 {side} 持續真空",
        )
        try:
            if not self.plc_service.is_connected():
                raise SlotVacuumServiceError(
                    "main_cycle_hold",
                    "PLC 尚未連線",
                    side=side,
                )
            previous = self.state_store.get(side)
            self._ensure_side_vacuum(side)
            state = replace(
                previous,
                occupancy=SlotOccupancy.OCCUPIED,
                vacuum_required=True,
                updated_at=time.time(),
            )
            self.state_store.save(state)
        except Exception as exc:
            self._set_status(
                LifecycleStatus.ERROR,
                "main_cycle_hold",
                str(exc),
                side=side,
            )
            raise
        self._set_status(
            LifecycleStatus.SUCCESS,
            "main_cycle_hold",
            f"{side} 真空已鎖定保持，等待明確放回貨架",
            side=side,
        )
        return state

    def release_for_main_cycle(self, side: str) -> SlotState:
        """Explicitly release a held side when the user selects shelf return."""

        side = self._normalize_side(side)
        self._set_status(
            LifecycleStatus.RUNNING,
            "main_cycle_release",
            f"總流程確認 {side} 放回貨架，準備破真空",
        )
        try:
            if not self.plc_service.is_connected():
                raise SlotVacuumServiceError(
                    "main_cycle_release",
                    "PLC 尚未連線",
                    side=side,
                )
            previous = self.state_store.get(side)
            with self._allow_confirmed_release():
                self._set_side_outputs(
                    side,
                    vacuum=False,
                    break_vacuum=True,
                )
            state = replace(
                previous,
                occupancy=SlotOccupancy.EMPTY,
                cargo_id=None,
                purpose=CargoPurpose.UNKNOWN,
                shelf_id=None,
                shelf_level=None,
                vacuum_required=False,
                updated_at=time.time(),
            )
            self.state_store.save(state)
        except Exception as exc:
            self._set_status(
                LifecycleStatus.ERROR,
                "main_cycle_release",
                str(exc),
                side=side,
            )
            raise
        self._set_status(
            LifecycleStatus.SUCCESS,
            "main_cycle_release",
            f"{side} 真空已關閉並開啟破真空",
            side=side,
        )
        return state

    def manual_release(self, side: str) -> SlotState:
        """Release one side with a timed break-vacuum pulse after confirmation."""

        side = self._normalize_side(side)
        self._set_status(
            LifecycleStatus.RUNNING,
            "manual_release",
            f"操作員確認手動關閉 {side} 真空與破真空",
        )
        try:
            if not self.plc_service.is_connected():
                raise SlotVacuumServiceError(
                    "manual_release",
                    "PLC 尚未連線",
                    side=side,
                )
            previous = self.state_store.get(side)
            state = replace(
                previous,
                occupancy=SlotOccupancy.EMPTY,
                cargo_id=None,
                purpose=CargoPurpose.UNKNOWN,
                shelf_id=None,
                shelf_level=None,
                vacuum_required=False,
                updated_at=time.time(),
            )
            # Clear the persistent hold before pulsing, otherwise the watchdog
            # can turn suction back on during the release window.
            self.state_store.save(state)
            with self._allow_confirmed_release():
                self._set_side_outputs(
                    side,
                    vacuum=False,
                    break_vacuum=True,
                )
                if self.config.release_pulse_seconds > 0:
                    time.sleep(self.config.release_pulse_seconds)
                self._set_side_outputs(
                    side,
                    vacuum=False,
                    break_vacuum=False,
                )
        except Exception as exc:
            self._set_status(
                LifecycleStatus.ERROR,
                "manual_release",
                str(exc),
                side=side,
            )
            raise
        self._set_status(
            LifecycleStatus.SUCCESS,
            "manual_release",
            f"{side} 已由操作員手動破真空並關閉全部真空輸出",
            side=side,
        )
        return state

    def shutdown_after_cycle(self) -> dict[str, SlotState]:
        """Turn every Y-side vacuum output off after a successful full cycle."""

        self._set_status(
            LifecycleStatus.RUNNING,
            "cycle_shutdown",
            "總流程完成，關閉 Y1／Y2 真空與破真空",
        )
        released: dict[str, SlotState] = {}
        try:
            if not self.plc_service.is_connected():
                raise SlotVacuumServiceError(
                    "cycle_shutdown",
                    "PLC 尚未連線",
                )
            for side in self.config.slots:
                previous = self.state_store.get(side)
                with self._allow_confirmed_release():
                    self._set_side_outputs(
                        side,
                        vacuum=False,
                        break_vacuum=False,
                    )
                vacuum_on = bool(
                    self.plc_service.read_point(
                        self.config.slots[side]["vacuum_point"],
                    )
                )
                break_vacuum_on = bool(
                    self.plc_service.read_point(
                        self.config.slots[side]["break_vacuum_point"],
                    )
                )
                if vacuum_on or break_vacuum_on:
                    active_outputs = []
                    if vacuum_on:
                        active_outputs.append("真空")
                    if break_vacuum_on:
                        active_outputs.append("破真空")
                    raise SlotVacuumServiceError(
                        "cycle_shutdown",
                        f"{side} {'／'.join(active_outputs)}輸出讀回仍為 ON，流程不得標示完成",
                    )
                state = replace(
                    previous,
                    occupancy=SlotOccupancy.EMPTY,
                    cargo_id=None,
                    purpose=CargoPurpose.UNKNOWN,
                    shelf_id=None,
                    shelf_level=None,
                    vacuum_required=False,
                    updated_at=time.time(),
                )
                self.state_store.save(state)
                released[side] = state
        except Exception as exc:
            self._set_status(
                LifecycleStatus.ERROR,
                "cycle_shutdown",
                str(exc),
            )
            raise
        self._set_status(
            LifecycleStatus.SUCCESS,
            "cycle_shutdown",
            "總流程完成，Y1／Y2 真空與破真空均已關閉",
            sides=tuple(released),
        )
        return released

    def start_watchdog(self) -> None:
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            return
        self._stop_event.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            daemon=True,
            name="slot-vacuum-watchdog",
        )
        self._watchdog_thread.start()

    def stop_watchdog(self) -> None:
        self._stop_event.set()

    def _watchdog_loop(self) -> None:
        while not self._stop_event.wait(self.config.enforcement_interval_seconds):
            try:
                self.enforce_required_vacuum()
            except Exception:
                continue

    def _ensure_side_vacuum(self, side: str) -> None:
        points = self.config.slots[side]
        break_point = points["break_vacuum_point"]
        vacuum_point = points["vacuum_point"]
        if bool(self.plc_service.read_point(break_point)):
            self.plc_service.write_point(break_point, False)
        if not bool(self.plc_service.read_point(vacuum_point)):
            self.plc_service.write_point(vacuum_point, True)
        if not bool(self.plc_service.read_point(vacuum_point)):
            raise SlotVacuumServiceError("ensure_vacuum", "真空命令讀回仍為 OFF", side=side)

    def _set_side_outputs(self, side: str, *, vacuum: bool, break_vacuum: bool) -> None:
        points = self.config.slots[side]
        if vacuum and break_vacuum:
            raise SlotVacuumServiceError("set_outputs", "真空與破真空不可同時開啟", side=side)
        if break_vacuum:
            # Release safely: stop suction before energising break vacuum.
            self.plc_service.write_point(points["vacuum_point"], False)
            self.plc_service.write_point(points["break_vacuum_point"], True)
        else:
            # Suction safely: close break vacuum before enabling suction.
            self.plc_service.write_point(points["break_vacuum_point"], False)
            self.plc_service.write_point(points["vacuum_point"], vacuum)

    def _normalize_side(self, side: str) -> str:
        normalized = side.strip().upper()
        if normalized not in self.config.slots:
            raise SlotVacuumServiceError("side", f"不支援的側別：{side!r}")
        return normalized

    def _guard_point_write(self, point_id: str, value: Any) -> None:
        if getattr(self._release_override, "enabled", False):
            return
        requested = self._parse_bool(value)
        point = self.plc_service.get_point(point_id)
        if str(point.device).upper() != "M":
            return
        requested_address = int(point.address)
        for side, points in self.config.slots.items():
            if not self.state_store.get(side).vacuum_required:
                continue
            vacuum_address = int(self.plc_service.get_point(points["vacuum_point"]).address)
            break_address = int(self.plc_service.get_point(points["break_vacuum_point"]).address)
            if requested_address == vacuum_address and not requested:
                raise SlotVacuumServiceError(
                    "write_guard",
                    "載貨側禁止關閉真空",
                    side=side,
                )
            if requested_address == break_address and requested:
                raise SlotVacuumServiceError(
                    "write_guard",
                    "載貨側禁止啟動破真空",
                    side=side,
                )

    def _guard_bit_write(self, device: str, address: int, values: list[bool]) -> None:
        if getattr(self._release_override, "enabled", False) or device != "M":
            return
        for offset, requested in enumerate(values):
            current_address = address + offset
            for side, points in self.config.slots.items():
                if not self.state_store.get(side).vacuum_required:
                    continue
                vacuum_address = self.plc_service.get_point(points["vacuum_point"]).address
                break_address = self.plc_service.get_point(points["break_vacuum_point"]).address
                if current_address == vacuum_address and not requested:
                    raise SlotVacuumServiceError(
                        "write_guard",
                        f"載貨側禁止寫入 M{current_address}=OFF",
                        side=side,
                    )
                if current_address == break_address and requested:
                    raise SlotVacuumServiceError(
                        "write_guard",
                        f"載貨側禁止寫入 M{current_address}=ON",
                        side=side,
                    )

    @contextmanager
    def _allow_confirmed_release(self):
        previous = getattr(self._release_override, "enabled", False)
        self._release_override.enabled = True
        try:
            yield
        finally:
            self._release_override.enabled = previous

    @staticmethod
    def _parse_bool(value: Any) -> bool:
        if isinstance(value, str):
            return value.strip().lower() not in {"", "0", "false", "off", "no"}
        return bool(value)
