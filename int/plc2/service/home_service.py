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


class HomeServiceError(RuntimeError):
    def __init__(self, operation: str, message: str, *, point_id: str | None = None) -> None:
        self.operation = operation
        self.point_id = point_id
        context = f" point={point_id}" if point_id else ""
        super().__init__(f"HomeService.{operation}{context}: {message}")


@dataclass(frozen=True)
class HomeServiceConfig:
    command_points: dict[str, str]
    position_points: dict[str, str]

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "HomeServiceConfig":
        raw = store.get_service("home")
        if raw is None:
            raise HomeServiceError("config", "找不到 services.yml 的 home 設定")
        commands = raw.get("command_points")
        positions = raw.get("position_points")
        if not isinstance(commands, dict) or not isinstance(positions, dict):
            raise HomeServiceError("config", "command_points/position_points 格式錯誤")
        if set(commands) != set(positions) or not commands:
            raise HomeServiceError("config", "歸零命令軸與位置回授軸不一致")
        return cls(
            {str(axis): str(point) for axis, point in commands.items()},
            {str(axis): str(point) for axis, point in positions.items()},
        )


class HomeService(LifecycleTracked):
    def __init__(
        self,
        plc_service: PointServiceProtocol,
        config: HomeServiceConfig | None = None,
    ) -> None:
        self.plc_service = plc_service
        self.config = config or HomeServiceConfig.load()
        self._init_status_tracker("HomeService")

    @property
    def is_connected(self) -> bool:
        return self.plc_service.is_connected()

    @tracked_operation("precheck", "檢查歸零點位", "歸零點位檢查完成")
    def precheck(self) -> None:
        if not self.is_connected:
            raise HomeServiceError("precheck", "PLC 尚未連線")
        for point_id in self.config.command_points.values():
            point = self.plc_service.get_point(point_id)
            if str(point.device).upper() != "M" or not point.writable:
                raise HomeServiceError("precheck", "歸零命令點位必須是可寫 M 點", point_id=point_id)
        for point_id in self.config.position_points.values():
            point = self.plc_service.get_point(point_id)
            if str(point.device).upper() != "D":
                raise HomeServiceError("precheck", "位置回授點位必須是 D 點", point_id=point_id)

    def set_home_commands(self, enabled: bool) -> None:
        self._set_status(LifecycleStatus.RUNNING, "set_home_commands", "寫入歸零命令")
        written: list[str] = []
        try:
            for point_id in self.config.command_points.values():
                self.plc_service.write_point(point_id, enabled)
                written.append(point_id)
        except Exception as exc:
            if enabled:
                for point_id in written:
                    try:
                        self.plc_service.write_point(point_id, False)
                    except Exception:
                        pass
            failed = next(
                (point for point in self.config.command_points.values() if point not in written),
                None,
            )
            error = HomeServiceError("set_home_commands", str(exc), point_id=failed)
            self._set_status(LifecycleStatus.ERROR, "set_home_commands", str(error))
            raise error from exc
        if enabled:
            self._set_status(
                LifecycleStatus.WAITING_SIGNAL,
                "set_home_commands",
                "歸零命令已啟動，等待 PLC 回授",
            )
        else:
            self._set_status(
                LifecycleStatus.SUCCESS,
                "set_home_commands",
                "歸零命令已停止",
            )

    @tracked_operation("read_positions", "讀取歸零位置", "歸零位置讀取完成")
    def read_positions(self) -> dict[str, float]:
        positions: dict[str, float] = {}
        for axis, point_id in self.config.position_points.items():
            try:
                value = self.plc_service.read_point(point_id)
            except Exception as exc:
                raise HomeServiceError("read_positions", str(exc), point_id=point_id) from exc
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise HomeServiceError("read_positions", f"回傳值無效: {value!r}", point_id=point_id)
            positions[axis] = float(value)
        return positions
