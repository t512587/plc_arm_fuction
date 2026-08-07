from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Protocol

try:
    from config.loader import CONFIG_STORE, ConfigStore
    from lifecycle import LifecycleStatus, LifecycleTracked, tracked_operation
    from service.arm_camera_home_interlock import (
        ARM_CAMERA_NOT_HOME_MAXIMUM_HEIGHT_MM,
        ARM_CAMERA_HOME_INTERLOCK,
        ArmCameraHomeInterlock,
    )
except ModuleNotFoundError:
    from plc2.config.loader import CONFIG_STORE, ConfigStore
    from plc2.lifecycle import LifecycleStatus, LifecycleTracked, tracked_operation
    from plc2.service.arm_camera_home_interlock import (
        ARM_CAMERA_NOT_HOME_MAXIMUM_HEIGHT_MM,
        ARM_CAMERA_HOME_INTERLOCK,
        ArmCameraHomeInterlock,
    )


class PointServiceProtocol(Protocol):
    def is_connected(self, plc_name: str = "main_plc") -> bool: ...
    def get_point(self, point_id: str): ...
    def read_point(self, point_id: str) -> float | int | bool: ...
    def write_point(self, point_id: str, value: Any) -> None: ...


class LiftServiceError(RuntimeError):
    def __init__(self, operation: str, message: str, *, point_id: str | None = None) -> None:
        self.operation = operation
        self.point_id = point_id
        context = f" point={point_id}" if point_id else ""
        super().__init__(f"LiftService.{operation}{context}: {message}")


@dataclass(frozen=True)
class LiftServiceConfig:
    height_point: str
    target_point: str
    speed_point: str
    positioning_speed: float | None
    up_point: str
    down_point: str
    manual_motion_points: tuple[str, ...]
    manual_up_point: str
    vacuum_points: dict[str, str]
    minimum_height_mm: float
    maximum_height_mm: float
    arm_camera_not_home_maximum_height_mm: float

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "LiftServiceConfig":
        raw = store.get_service("lift")
        if raw is None:
            raise LiftServiceError("config", "找不到 services.yml 的 lift 設定")
        vacuums = raw.get("vacuum_points")
        if not isinstance(vacuums, dict) or set(vacuums) != {"left", "right"}:
            raise LiftServiceError("config", "vacuum_points 必須包含 left/right")
        raw_speed = raw.get("positioning_speed")
        configured_not_home_maximum = float(
            raw.get(
                "arm_camera_not_home_maximum_height_mm",
                ARM_CAMERA_NOT_HOME_MAXIMUM_HEIGHT_MM,
            )
        )
        return cls(
            height_point=str(raw["height_point"]),
            target_point=str(raw["target_point"]),
            speed_point=str(raw["speed_point"]),
            positioning_speed=None if raw_speed is None else float(raw_speed),
            up_point=str(raw["up_point"]),
            down_point=str(raw["down_point"]),
            manual_motion_points=tuple(str(point_id) for point_id in raw.get("manual_motion_points", ())),
            manual_up_point=str(raw.get("manual_up_point", "X_UP")),
            vacuum_points={str(side): str(point) for side, point in vacuums.items()},
            minimum_height_mm=float(raw.get("minimum_height_mm", 0.0)),
            maximum_height_mm=float(raw.get("maximum_height_mm", 1450.0)),
            arm_camera_not_home_maximum_height_mm=min(
                configured_not_home_maximum,
                ARM_CAMERA_NOT_HOME_MAXIMUM_HEIGHT_MM,
            ),
        )


class LiftService(LifecycleTracked):
    def __init__(
        self,
        plc_service: PointServiceProtocol,
        config: LiftServiceConfig | None = None,
        *,
        home_interlock: ArmCameraHomeInterlock = ARM_CAMERA_HOME_INTERLOCK,
    ) -> None:
        self.plc_service = plc_service
        self.config = config or LiftServiceConfig.load()
        self.home_interlock = home_interlock
        if not (
            self.config.minimum_height_mm
            <= self.arm_camera_not_home_maximum_height_mm
            <= self.config.maximum_height_mm
        ):
            raise LiftServiceError(
                "config",
                "arm_camera_not_home_maximum_height_mm 必須位於升降機最小/最大高度內",
            )
        self._init_status_tracker("LiftService")
        register_guard = getattr(self.plc_service, "register_write_guard", None)
        if callable(register_guard):
            register_guard(
                point_guard=self._guard_point_write,
                bit_guard=self._guard_bit_write,
                d_guard=self._guard_d_write,
            )

    @property
    def arm_camera_not_home_maximum_height_mm(self) -> float:
        """Effective ceiling; callers/configuration cannot loosen 695 mm."""
        return min(
            self.config.arm_camera_not_home_maximum_height_mm,
            ARM_CAMERA_NOT_HOME_MAXIMUM_HEIGHT_MM,
        )

    @property
    def is_connected(self) -> bool:
        return self.plc_service.is_connected()

    @tracked_operation("precheck", "檢查升降與真空點位", "升降與真空點位檢查完成")
    def precheck(self) -> None:
        if not self.is_connected:
            raise LiftServiceError("precheck", "PLC 尚未連線")
        height = self.plc_service.get_point(self.config.height_point)
        if str(height.device).upper() != "D":
            raise LiftServiceError("precheck", "高度回授必須是 D 點", point_id=self.config.height_point)
        for point_id in (self.config.target_point, self.config.speed_point):
            point = self.plc_service.get_point(point_id)
            if str(point.device).upper() != "D" or not point.writable:
                raise LiftServiceError("precheck", "定位參數必須是可寫 D 點", point_id=point_id)
        for point_id in (
            self.config.up_point,
            self.config.down_point,
            *self.config.manual_motion_points,
            *self.config.vacuum_points.values(),
        ):
            point = self.plc_service.get_point(point_id)
            if str(point.device).upper() != "M" or not point.writable:
                raise LiftServiceError("precheck", "控制點必須是可寫 M 點", point_id=point_id)

    @tracked_operation("read_height", "讀取目前高度", "目前高度讀取完成")
    def read_height(self) -> float:
        try:
            value = self.plc_service.read_point(self.config.height_point)
        except Exception as exc:
            raise LiftServiceError("read_height", str(exc), point_id=self.config.height_point) from exc
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise LiftServiceError("read_height", f"回傳值無效: {value!r}", point_id=self.config.height_point)
        height = float(value)
        if not math.isfinite(height):
            raise LiftServiceError("read_height", f"回傳值不是有限數值: {height!r}", point_id=self.config.height_point)
        if not self.config.minimum_height_mm <= height <= self.config.maximum_height_mm:
            raise LiftServiceError(
                "read_height",
                f"高度 {height:g}mm 超出 {self.config.minimum_height_mm:g}～{self.config.maximum_height_mm:g}mm",
                point_id=self.config.height_point,
            )
        return height

    @tracked_operation("configure_position", "寫入升降定位參數", "升降定位參數寫入完成")
    def configure_position(self, target_height_mm: float, speed: float | None = None) -> None:
        self.validate_target_height(target_height_mm, operation="configure_position")
        selected_speed = self.config.positioning_speed if speed is None else speed
        if selected_speed is not None and (not math.isfinite(selected_speed) or selected_speed <= 0):
            raise LiftServiceError(
                "configure_position",
                f"定位速度必須大於 0: {selected_speed!r}",
                point_id=self.config.speed_point,
            )
        try:
            self.plc_service.write_point(self.config.target_point, target_height_mm)
            if selected_speed is not None:
                self.plc_service.write_point(self.config.speed_point, selected_speed)
        except Exception as exc:
            raise LiftServiceError("configure_position", str(exc), point_id=self.config.target_point) from exc

    def validate_target_height(
        self,
        target_height_mm: float,
        *,
        operation: str = "validate_target_height",
    ) -> None:
        target = float(target_height_mm)
        if not math.isfinite(target):
            raise LiftServiceError(
                operation,
                f"目標高度不是有限數值: {target!r}",
                point_id=self.config.target_point,
            )
        if not self.config.minimum_height_mm <= target <= self.config.maximum_height_mm:
            raise LiftServiceError(
                operation,
                f"目標高度 {target:g}mm 超出 "
                f"{self.config.minimum_height_mm:g}～{self.config.maximum_height_mm:g}mm",
                point_id=self.config.target_point,
            )
        snapshot = self.home_interlock.snapshot
        restricted_max = self.arm_camera_not_home_maximum_height_mm
        if not snapshot.all_home_confirmed and target > restricted_max:
            raise LiftServiceError(
                operation,
                f"手臂或相機尚未確認回 HOME（state={snapshot.state.value}；"
                f"reason={snapshot.reason}），目標高度 {target:g}mm 超過安全上限 "
                f"{restricted_max:g}mm",
                point_id=self.config.target_point,
            )

    @tracked_operation("set_vacuum", "寫入左右真空命令", "左右真空命令寫入完成")
    def set_vacuum(self, enabled: bool) -> None:
        for point_id in self.config.vacuum_points.values():
            try:
                self.plc_service.write_point(point_id, enabled)
            except Exception as exc:
                raise LiftServiceError("set_vacuum", str(exc), point_id=point_id) from exc

    @tracked_operation("read_vacuum", "讀取左右真空狀態", "左右真空狀態讀取完成")
    def read_vacuum(self) -> dict[str, bool]:
        states: dict[str, bool] = {}
        for side, point_id in self.config.vacuum_points.items():
            try:
                states[side] = bool(self.plc_service.read_point(point_id))
            except Exception as exc:
                raise LiftServiceError("read_vacuum", str(exc), point_id=point_id) from exc
        return states

    @tracked_operation(
        "start_up",
        "啟動升降定位",
        "升降定位已啟動，等待 PLC 回授",
        success_status=LifecycleStatus.WAITING_SIGNAL,
    )
    def start_up(self) -> None:
        self._start(self.config.up_point, self.config.down_point, "start_up")

    @tracked_operation(
        "start_vision_positioning",
        "啟動視覺高度定位",
        "視覺高度定位已啟動，等待 PLC 回授",
        success_status=LifecycleStatus.WAITING_SIGNAL,
    )
    def start_vision_positioning(self) -> None:
        self._start(
            self.config.up_point,
            self.config.down_point,
            "start_vision_positioning",
        )

    @tracked_operation(
        "start_down",
        "啟動下降定位",
        "下降定位已啟動，等待 PLC 回授",
        success_status=LifecycleStatus.WAITING_SIGNAL,
    )
    def start_down(self) -> None:
        self._start(self.config.down_point, self.config.up_point, "start_down")

    def _start(self, active_point: str, opposite_point: str, operation: str) -> None:
        try:
            for point_id in self.config.manual_motion_points:
                self.plc_service.write_point(point_id, False)
            self.plc_service.write_point(active_point, False)
            self.plc_service.write_point(opposite_point, False)
            self.plc_service.write_point(active_point, True)
        except Exception as exc:
            try:
                self.stop()
            except Exception:
                pass
            raise LiftServiceError(operation, str(exc), point_id=active_point) from exc

    @tracked_operation("read_motion_commands", "讀取升降命令", "升降命令讀取完成")
    def read_motion_commands(self) -> dict[str, bool]:
        try:
            commands = {
                "up": bool(self.plc_service.read_point(self.config.up_point)),
                "down": bool(self.plc_service.read_point(self.config.down_point)),
            }
            for point_id in self.config.manual_motion_points:
                commands[f"manual:{point_id}"] = bool(
                    self.plc_service.read_point(point_id)
                )
            return commands
        except Exception as exc:
            raise LiftServiceError("read_motion_commands", str(exc)) from exc

    @tracked_operation("stop", "停止升降命令", "升降命令已停止")
    def stop(self) -> None:
        errors: list[str] = []
        for point_id in (self.config.up_point, self.config.down_point, *self.config.manual_motion_points):
            try:
                self.plc_service.write_point(point_id, False)
            except Exception as exc:
                errors.append(f"{point_id}: {exc}")
        if errors:
            raise LiftServiceError("stop", "; ".join(errors))

    def _guard_point_write(self, point_id: str, value: Any) -> None:
        point = self.plc_service.get_point(point_id)
        target = self.plc_service.get_point(self.config.target_point)
        if (
            str(point.device).upper() == "D"
            and point.plc == target.plc
            and int(point.address) == int(target.address)
        ):
            self.validate_target_height(float(value), operation="write_guard")
            return
        if str(point.device).upper() != "M" or not self._parse_bool(value):
            return
        self._guard_motion_start(int(point.address), operation="write_guard")

    def _guard_bit_write(self, device: str, address: int, values: list[bool]) -> None:
        if device != "M":
            return
        for offset, requested in enumerate(values):
            if requested:
                self._guard_motion_start(address + offset, operation="raw_bit_write_guard")

    def _guard_d_write(self, plc_name: str, address: int, values: list[int]) -> None:
        target = self.plc_service.get_point(self.config.target_point)
        if plc_name != target.plc:
            return
        offset = int(target.address) - int(address)
        if offset < 0 or offset >= len(values):
            return
        raw = int(values[offset])
        point_type = str(target.type).lower()
        if point_type in {"int", "s16"}:
            raw &= 0xFFFF
            engineering_value = raw - 0x10000 if raw & 0x8000 else raw
        elif point_type == "u16":
            engineering_value = raw & 0xFFFF
        else:
            engineering_value = raw
        scale = 1.0 if target.scale is None else float(target.scale)
        self.validate_target_height(
            float(engineering_value) * scale,
            operation="raw_d_write_guard",
        )

    def _guard_motion_start(self, address: int, *, operation: str) -> None:
        snapshot = self.home_interlock.snapshot
        if snapshot.all_home_confirmed:
            return
        manual_up = self.plc_service.get_point(self.config.manual_up_point)
        if address == int(manual_up.address):
            raise LiftServiceError(
                operation,
                f"手臂或相機尚未確認回 HOME（state={snapshot.state.value}；"
                f"reason={snapshot.reason}），禁止使用連續手動上升；"
                f"請改用不超過 {self.arm_camera_not_home_maximum_height_mm:g}mm 的定位命令",
                point_id=self.config.manual_up_point,
            )
        positioning = self.plc_service.get_point(self.config.up_point)
        if address == int(positioning.address):
            target = float(self.plc_service.read_point(self.config.target_point))
            self.validate_target_height(target, operation=operation)

    @staticmethod
    def _parse_bool(value: Any) -> bool:
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "1", "on", "yes"}:
                return True
            if normalized in {"false", "0", "off", "no", ""}:
                return False
        return bool(value)
