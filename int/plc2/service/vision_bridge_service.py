from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    from config.loader import CONFIG_STORE, ConfigStore
    from lifecycle import LifecycleTracked, tracked_operation
except ModuleNotFoundError:
    from plc2.config.loader import CONFIG_STORE, ConfigStore
    from plc2.lifecycle import LifecycleTracked, tracked_operation


LOGGER = logging.getLogger(__name__)


class VisionBridgeServiceError(RuntimeError):
    def __init__(self, operation: str, message: str) -> None:
        self.operation = operation
        super().__init__(f"VisionBridgeService.{operation}: {message}")


@dataclass(frozen=True)
class VisionBridgeServiceConfig:
    notify_file: Path
    height_feedback_file: Path
    done_signal_file: Path
    height_json_path: tuple[str, ...]
    default_timeout_seconds: float
    poll_interval_seconds: float

    @classmethod
    def load(cls, store: ConfigStore = CONFIG_STORE) -> "VisionBridgeServiceConfig":
        raw = store.get_service("vision_bridge") or {}
        return cls(
            notify_file=Path(str(raw.get("notify_file", "runtime/vision_bridge_notify.json"))),
            height_feedback_file=Path(str(raw.get("height_feedback_file", "runtime/vision_height_feedback.json"))),
            done_signal_file=Path(str(raw.get("done_signal_file", "runtime/vision_done_signal.json"))),
            height_json_path=tuple(str(item) for item in raw.get("height_json_path", ("height_mm",))),
            default_timeout_seconds=float(raw.get("default_timeout_seconds", 30.0)),
            poll_interval_seconds=float(raw.get("poll_interval_seconds", 0.1)),
        )


class VisionBridgeService(LifecycleTracked):
    """File-based bridge for RealSense/CANBus handshakes until the transport is fixed."""

    def __init__(self, config: VisionBridgeServiceConfig | None = None) -> None:
        self.config = config or VisionBridgeServiceConfig.load()
        self._init_status_tracker("VisionBridgeService")

    @tracked_operation("start", "通知 RealSense/CANBus 啟動", "RealSense/CANBus 啟動通知已送出")
    def start(self) -> None:
        self.notify("start")

    @tracked_operation("notify", "通知 RealSense/CANBus 流程階段", "RealSense/CANBus 流程階段已通知")
    def notify(self, phase: str, **payload: Any) -> None:
        data = {
            "phase": phase,
            "updated_at": time.time(),
            **payload,
        }
        path = self.config.notify_file
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        LOGGER.info("Vision bridge notified phase=%s payload=%s", phase, payload)

    @tracked_operation("wait_signal", "等待 RealSense/CANBus 信號", "RealSense/CANBus 信號已收到")
    def wait_signal(self, cancel_event, timeout_seconds: float | None = None) -> None:
        self._wait_for_file(self.config.height_feedback_file, cancel_event, timeout_seconds)

    @tracked_operation("wait_height", "等待 RealSense/CANBus 回傳高度", "RealSense/CANBus 高度已收到")
    def wait_height(self, cancel_event, timeout_seconds: float | None = None) -> float:
        deadline = time.monotonic() + (timeout_seconds or self.config.default_timeout_seconds)
        while time.monotonic() < deadline:
            if cancel_event.is_set():
                raise VisionBridgeServiceError("wait_height", "流程已取消")
            if self.config.height_feedback_file.exists():
                raw = json.loads(self.config.height_feedback_file.read_text(encoding="utf-8"))
                value: Any = raw
                for key in self.config.height_json_path:
                    if not isinstance(value, dict) or key not in value:
                        raise VisionBridgeServiceError("wait_height", f"高度檔缺少欄位: {'.'.join(self.config.height_json_path)}")
                    value = value[key]
                height = float(value)
                if not math.isfinite(height):
                    raise VisionBridgeServiceError("wait_height", f"高度不是有限數值: {height!r}")
                return height
            cancel_event.wait(self.config.poll_interval_seconds)
        raise VisionBridgeServiceError("wait_height", f"等待高度超過 {timeout_seconds or self.config.default_timeout_seconds:g} 秒")

    @tracked_operation("wait_done", "等待 RealSense/CANBus 完成信號", "RealSense/CANBus 完成信號已收到")
    def wait_done(self, cancel_event, timeout_seconds: float | None = None) -> None:
        self._wait_for_file(self.config.done_signal_file, cancel_event, timeout_seconds)

    def _wait_for_file(self, path: Path, cancel_event, timeout_seconds: float | None) -> None:
        deadline = time.monotonic() + (timeout_seconds or self.config.default_timeout_seconds)
        while time.monotonic() < deadline:
            if cancel_event.is_set():
                raise VisionBridgeServiceError("wait_signal", "流程已取消")
            if path.exists():
                return
            cancel_event.wait(self.config.poll_interval_seconds)
        raise VisionBridgeServiceError("wait_signal", f"等待信號超過 {timeout_seconds or self.config.default_timeout_seconds:g} 秒")
