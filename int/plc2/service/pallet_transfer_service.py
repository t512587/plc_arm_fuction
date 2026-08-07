from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Protocol

try:
    from config.loader import CONFIG_STORE, ConfigStore
    from lifecycle import LifecycleStatus, LifecycleTracked, tracked_operation
except ModuleNotFoundError:
    from plc2.config.loader import CONFIG_STORE, ConfigStore
    from plc2.lifecycle import LifecycleStatus, LifecycleTracked, tracked_operation


class PointServiceProtocol(Protocol):
    def is_connected(self, plc_name: str = "main_plc") -> bool: ...
    def get_point(self, point_id: str): ...
    def read_point(self, point_id: str) -> float | int | bool: ...
    def write_point(self, point_id: str, value: Any) -> None: ...


class PalletAction(str, Enum):
    NONE = "none"
    SUCK = "suck"
    PUSH = "push"


class PalletTransferServiceError(RuntimeError):
    def __init__(self, operation: str, message: str, *, point_id: str | None = None) -> None:
        self.operation = operation
        self.point_id = point_id
        context = f" point={point_id}" if point_id else ""
        super().__init__(f"PalletTransferService.{operation}{context}: {message}")


@dataclass(frozen=True)
class PalletSlotConfig:
    forward_target_point: str
    forward_command_point: str
    position_point: str
    vacuum_point: str
    break_vacuum_point: str


@dataclass(frozen=True)
class PalletTransferServiceConfig:
    slots: dict[str, PalletSlotConfig]
    minimum_forward_mm: float
    maximum_forward_mm: float

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "PalletTransferServiceConfig":
        raw = store.get_service("pallet_transfer")
        if raw is None:
            raise PalletTransferServiceError("config", "找不到 services.yml 的 pallet_transfer 設定")
        slots = raw.get("slots")
        if not isinstance(slots, dict) or set(slots) != {"Y1", "Y2"}:
            raise PalletTransferServiceError("config", "slots 必須包含 Y1 與 Y2")
        return cls(
            slots={
                str(slot): PalletSlotConfig(
                    forward_target_point=str(item["forward_target_point"]),
                    forward_command_point=str(item["forward_command_point"]),
                    position_point=str(item["position_point"]),
                    vacuum_point=str(item["vacuum_point"]),
                    break_vacuum_point=str(item["break_vacuum_point"]),
                )
                for slot, item in slots.items()
            },
            minimum_forward_mm=float(raw.get("minimum_forward_mm", -32768.0)),
            maximum_forward_mm=float(raw.get("maximum_forward_mm", 32767.0)),
        )


class PalletTransferService(LifecycleTracked):
    """Small point-oriented service for the Y1/Y2 pallet suck/push sequence."""

    def __init__(
        self,
        plc_service: PointServiceProtocol,
        config: PalletTransferServiceConfig | None = None,
    ) -> None:
        self.plc_service = plc_service
        self.config = config or PalletTransferServiceConfig.load()
        self._init_status_tracker("PalletTransferService")

    @tracked_operation("precheck", "檢查貨盤吸推點位", "貨盤吸推點位檢查完成")
    def precheck(self) -> None:
        if not self.plc_service.is_connected():
            raise PalletTransferServiceError("precheck", "PLC 尚未連線")
        for slot, item in self.config.slots.items():
            for point_id in (
                item.forward_target_point,
                item.forward_command_point,
                item.position_point,
                item.vacuum_point,
                item.break_vacuum_point,
            ):
                point = self.plc_service.get_point(point_id)
                if point is None:
                    raise PalletTransferServiceError("precheck", f"{slot} 找不到點位", point_id=point_id)
            target = self.plc_service.get_point(item.forward_target_point)
            if str(target.device).upper() != "D" or not target.writable:
                raise PalletTransferServiceError("precheck", "前進距離必須是可寫 D 點", point_id=item.forward_target_point)
            position = self.plc_service.get_point(item.position_point)
            if str(position.device).upper() != "D":
                raise PalletTransferServiceError("precheck", "位置回授必須是 D 點", point_id=item.position_point)
            for point_id in (
                item.forward_command_point,
                item.vacuum_point,
                item.break_vacuum_point,
            ):
                point = self.plc_service.get_point(point_id)
                if str(point.device).upper() != "M" or not point.writable:
                    raise PalletTransferServiceError("precheck", "控制點必須是可寫 M 點", point_id=point_id)

    @tracked_operation("set_forward_position", "寫入貨盤前進距離", "貨盤前進距離寫入完成")
    def set_forward_position(self, slot: str, forward_mm: float) -> None:
        item = self._slot(slot)
        if not self.config.minimum_forward_mm <= forward_mm <= self.config.maximum_forward_mm:
            raise PalletTransferServiceError(
                "set_forward_position",
                f"前進距離 {forward_mm:g}mm 超出 {self.config.minimum_forward_mm:g}～{self.config.maximum_forward_mm:g}mm",
                point_id=item.forward_target_point,
            )
        self.plc_service.write_point(item.forward_target_point, forward_mm)

    @tracked_operation(
        "start_forward",
        "啟動貨盤定位",
        "貨盤定位已啟動，等待 PLC 回授",
        success_status=LifecycleStatus.WAITING_SIGNAL,
    )
    def start_forward(self, slot: str) -> None:
        item = self._slot(slot)
        self.stop_forward(slot)
        self.plc_service.write_point(item.forward_command_point, True)

    @tracked_operation("stop_forward", "停止貨盤定位命令", "貨盤定位命令已停止")
    def stop_forward(self, slot: str | None = None) -> None:
        errors: list[str] = []
        slots = self.config.slots.values() if slot is None else (self._slot(slot),)
        for item in slots:
            try:
                self.plc_service.write_point(item.forward_command_point, False)
            except Exception as exc:
                errors.append(f"{item.forward_command_point}: {exc}")
        if errors:
            raise PalletTransferServiceError("stop_forward", "; ".join(errors))

    @tracked_operation(
        "read_forward_commands",
        "讀取貨盤定位命令",
        "貨盤定位命令讀取完成",
    )
    def read_forward_commands(self) -> dict[str, bool]:
        try:
            return {
                slot: bool(self.plc_service.read_point(item.forward_command_point))
                for slot, item in self.config.slots.items()
            }
        except Exception as exc:
            raise PalletTransferServiceError(
                "read_forward_commands",
                str(exc),
            ) from exc

    @tracked_operation("read_position", "讀取貨盤位置", "貨盤位置讀取完成")
    def read_position(self, slot: str) -> float:
        value = self.plc_service.read_point(self._slot(slot).position_point)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise PalletTransferServiceError("read_position", f"位置回傳值無效: {value!r}")
        return float(value)

    @tracked_operation("set_action_output", "切換貨盤吸推輸出", "貨盤吸推輸出切換完成")
    def set_action_output(self, slot: str, action: PalletAction | str) -> dict[str, bool | str]:
        selected = action if isinstance(action, PalletAction) else PalletAction(str(action))
        item = self._slot(slot)
        if selected is PalletAction.SUCK:
            self.plc_service.write_point(item.break_vacuum_point, False)
            self.plc_service.write_point(item.vacuum_point, True)
        elif selected is PalletAction.PUSH:
            self.plc_service.write_point(item.vacuum_point, False)
            self.plc_service.write_point(item.break_vacuum_point, True)
        else:
            self.plc_service.write_point(item.vacuum_point, False)
            self.plc_service.write_point(item.break_vacuum_point, False)
        state = self.read_action_output(slot)
        if state["mode"] != selected.value:
            raise PalletTransferServiceError(
                "set_action_output",
                f"輸出讀回模式不符，要求={selected.value}，讀回={state['mode']}",
                point_id=(
                    item.vacuum_point
                    if selected is PalletAction.SUCK
                    else item.break_vacuum_point
                ),
            )
        return state

    @tracked_operation("read_action_output", "讀取貨盤吸推輸出", "貨盤吸推輸出讀取完成")
    def read_action_output(self, slot: str) -> dict[str, bool | str]:
        item = self._slot(slot)
        vacuum_on = bool(self.plc_service.read_point(item.vacuum_point))
        break_vacuum_on = bool(self.plc_service.read_point(item.break_vacuum_point))
        if vacuum_on and break_vacuum_on:
            mode = "invalid"
        elif vacuum_on:
            mode = PalletAction.SUCK.value
        elif break_vacuum_on:
            mode = PalletAction.PUSH.value
        else:
            mode = PalletAction.NONE.value
        return {"slot": slot.upper(), "mode": mode, "vacuum_on": vacuum_on, "break_vacuum_on": break_vacuum_on}

    def _slot(self, slot: str) -> PalletSlotConfig:
        key = str(slot).upper()
        try:
            return self.config.slots[key]
        except KeyError as exc:
            raise PalletTransferServiceError("slot", f"不支援的貨盤: {slot}") from exc
