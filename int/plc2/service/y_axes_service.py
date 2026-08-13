from __future__ import annotations

from dataclasses import dataclass
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


class YAxesServiceError(RuntimeError):
    def __init__(self, operation: str, message: str, *, point_id: str | None = None) -> None:
        context = f" point={point_id}" if point_id else ""
        super().__init__(f"YAxesService.{operation}{context}: {message}")


@dataclass(frozen=True)
class YAxesServiceConfig:
    command_points: dict[str, str]
    position_points: dict[str, str]

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "YAxesServiceConfig":
        raw = store.get_service("y_axes")
        if raw is None:
            raise YAxesServiceError("config", "找不到 services.yml 的 y_axes 設定")
        commands = raw.get("command_points")
        positions = raw.get("position_points")
        if not isinstance(commands, dict) or set(commands) != {"Y1", "Y2"}:
            raise YAxesServiceError("config", "command_points 必須包含 Y1 與 Y2")
        if not isinstance(positions, dict) or set(positions) != {"Y1", "Y2"}:
            raise YAxesServiceError("config", "position_points 必須包含 Y1 與 Y2")
        return cls(
            command_points={str(axis): str(point) for axis, point in commands.items()},
            position_points={str(axis): str(point) for axis, point in positions.items()},
        )


class YAxesService(LifecycleTracked):
    """Position Y1 with M388 and Y2 with M398; never use M389/M399."""

    def __init__(
        self,
        plc_service: PointServiceProtocol,
        config: YAxesServiceConfig | None = None,
    ) -> None:
        self.plc_service = plc_service
        self.config = config or YAxesServiceConfig.load()
        self._init_status_tracker("YAxesService")

    @tracked_operation("precheck", "檢查 Y1/Y2 定位點位", "Y1/Y2 定位點位檢查完成")
    def precheck(self) -> None:
        if not self.plc_service.is_connected():
            raise YAxesServiceError("precheck", "PLC 尚未連線")
        for point_id in self.config.command_points.values():
            point = self.plc_service.get_point(point_id)
            if str(point.device).upper() != "M" or not point.writable:
                raise YAxesServiceError("precheck", "命令點位必須是可寫入的 M 點位", point_id=point_id)
        for point_id in self.config.position_points.values():
            point = self.plc_service.get_point(point_id)
            if str(point.device).upper() != "D":
                raise YAxesServiceError("precheck", "位置回授必須是 D 點位", point_id=point_id)

    @tracked_operation("read_positions", "讀取 Y1/Y2 位置", "Y1/Y2 位置讀取完成")
    def read_positions(self) -> dict[str, float]:
        try:
            return {
                axis: float(self.plc_service.read_point(point_id))
                for axis, point_id in self.config.position_points.items()
            }
        except Exception as exc:
            raise YAxesServiceError("read_positions", str(exc)) from exc

    @tracked_operation(
        "start_y1_positioning",
        "啟動 Y1 定位",
        "Y1 定位已啟動，等待 PLC 回授",
        success_status=LifecycleStatus.WAITING_SIGNAL,
    )
    def start_y1_positioning(self) -> None:
        self._start(("Y1",), "start_y1_positioning")

    @tracked_operation(
        "start_y2_positioning",
        "啟動 Y2 定位",
        "Y2 定位已啟動，等待 PLC 回授",
        success_status=LifecycleStatus.WAITING_SIGNAL,
    )
    def start_y2_positioning(self) -> None:
        self._start(("Y2",), "start_y2_positioning")

    @tracked_operation(
        "start_both_positioning",
        "啟動 Y1/Y2 定位",
        "Y1/Y2 定位已啟動，等待 PLC 回授",
        success_status=LifecycleStatus.WAITING_SIGNAL,
    )
    def start_both_positioning(self) -> None:
        self._start(("Y1", "Y2"), "start_both_positioning")

    @tracked_operation("stop", "停止 Y1/Y2 定位命令", "Y1/Y2 定位命令已停止")
    def stop(self) -> None:
        errors: list[str] = []
        for point_id in self.config.command_points.values():
            try:
                self.plc_service.write_point(point_id, False)
            except Exception as exc:
                errors.append(f"{point_id}: {exc}")
        if errors:
            raise YAxesServiceError("stop", "; ".join(errors))

    def _start(self, axes: tuple[str, ...], operation: str) -> None:
        active_points = tuple(self.config.command_points[axis] for axis in axes)
        active_point: str | None = None
        try:
            self.stop()
            for active_point in active_points:
                self.plc_service.write_point(active_point, True)
        except Exception as exc:
            try:
                self.stop()
            except Exception:
                pass
            raise YAxesServiceError(operation, str(exc), point_id=active_point) from exc
