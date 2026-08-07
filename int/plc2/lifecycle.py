from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from functools import wraps
from typing import Any, Callable


class LifecycleStatus(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    WAITING_SIGNAL = "waiting_signal"
    SUCCESS = "success"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    ERROR = "error"

    @property
    def terminal(self) -> bool:
        return self in {
            LifecycleStatus.SUCCESS,
            LifecycleStatus.CANCELLED,
            LifecycleStatus.TIMEOUT,
            LifecycleStatus.ERROR,
        }


@dataclass(frozen=True)
class StatusSnapshot:
    component: str
    status: LifecycleStatus
    step: str
    message: str
    updated_at: float
    data: dict[str, Any] = field(default_factory=dict)

    @property
    def succeeded(self) -> bool:
        return self.status is LifecycleStatus.SUCCESS

    def as_dict(self) -> dict[str, Any]:
        return {
            "component": self.component,
            "status": self.status.value,
            "step": self.step,
            "message": self.message,
            "updated_at": self.updated_at,
            "data": dict(self.data),
        }


StatusCallback = Callable[[StatusSnapshot], None]


class StatusTracker:
    def __init__(self, component: str, callback: StatusCallback | None = None) -> None:
        self.component = component
        self.callback = callback
        self._lock = threading.RLock()
        self._snapshot = StatusSnapshot(
            component=component,
            status=LifecycleStatus.PENDING,
            step="pending",
            message="尚未開始",
            updated_at=time.time(),
        )

    @property
    def snapshot(self) -> StatusSnapshot:
        with self._lock:
            item = self._snapshot
            return StatusSnapshot(
                component=item.component,
                status=item.status,
                step=item.step,
                message=item.message,
                updated_at=item.updated_at,
                data=dict(item.data),
            )

    @property
    def status(self) -> LifecycleStatus:
        return self.snapshot.status

    def update(
        self,
        status: LifecycleStatus,
        step: str,
        message: str,
        **data: Any,
    ) -> StatusSnapshot:
        item = StatusSnapshot(
            component=self.component,
            status=status,
            step=step,
            message=message,
            updated_at=time.time(),
            data=dict(data),
        )
        with self._lock:
            self._snapshot = item
        if self.callback is not None:
            self.callback(item)
        return item


class LifecycleTracked:
    _status_tracker: StatusTracker

    def _init_status_tracker(
        self,
        component: str,
        callback: StatusCallback | None = None,
    ) -> None:
        self._status_tracker = StatusTracker(component, callback)

    @property
    def status(self) -> LifecycleStatus:
        return self._status_tracker.status

    @property
    def status_snapshot(self) -> StatusSnapshot:
        return self._status_tracker.snapshot

    def _set_status(
        self,
        status: LifecycleStatus,
        step: str,
        message: str,
        **data: Any,
    ) -> StatusSnapshot:
        return self._status_tracker.update(status, step, message, **data)


def tracked_operation(
    step: str,
    running_message: str,
    success_message: str,
    *,
    success_status: LifecycleStatus = LifecycleStatus.SUCCESS,
):
    def decorate(method):
        @wraps(method)
        def wrapped(self: LifecycleTracked, *args, **kwargs):
            self._set_status(LifecycleStatus.RUNNING, step, running_message)
            try:
                value = method(self, *args, **kwargs)
            except Exception as exc:
                self._set_status(LifecycleStatus.ERROR, step, str(exc))
                raise
            data: dict[str, Any] = {}
            if isinstance(value, dict):
                data["result"] = dict(value)
            elif isinstance(value, (str, int, float, bool)) or value is None:
                data["result"] = value
            self._set_status(success_status, step, success_message, **data)
            return value

        return wrapped

    return decorate
