"""FastAPI Backend：提供 PLC 讀寫 API。

- 使用 config.loader 載入 PLC / 點位設定
- 透過 plc.client.MitsubishiPLCClient 進行 MC Protocol 通訊
- 將讀寫狀態印到 terminal 並寫入 log 檔
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, List, Optional
import threading
import time

from fastapi import Depends, FastAPI, HTTPException, Query, status
from pydantic import BaseModel, Field

from config.loader import CONFIG_STORE, PointDefinition
from plc.client import PLCConnectionError
from flow.home_flow import HomeFlow
from flow.main_cycle_flow import (
    MainCycleFlow,
    MainCycleState,
    Step1Command,
    TransferDirection,
)
from flow.vision_height_flow import VisionHeightFlow
from lifecycle import LifecycleStatus
from service.home_service import HomeService
from service.lift_service import LiftService, LiftServiceError
from service.plc_service import PLC_SERVICE, PlcPointValidationError, PlcServiceError
from service.y_axes_service import YAxesService
from service.pallet_transfer_service import PalletTransferService
from service.vision_bridge_service import VisionBridgeService
from service.arm_vision_workflow_service import ArmVisionWorkflowService
from service.arm_camera_home_interlock import ARM_CAMERA_HOME_INTERLOCK
from service.slot_vacuum_service import (
    CargoPurpose,
    SlotOccupancy,
    SlotVacuumMode,
    SlotVacuumService,
    SlotVacuumServiceError,
)
from service.middle_vacuum_service import (
    MiddleVacuumMode,
    MiddleVacuumService,
    MiddleVacuumServiceError,
)

import logging
import sys

# ===== Logging 設定 =====

BASE_DIR = Path(__file__).resolve().parent.parent
LOG_DIR = BASE_DIR / "logs"
LOG_DIR.mkdir(exist_ok=True)
LOG_FILE = LOG_DIR / "plc_backend.log"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s - %(message)s",
)

logger = logging.getLogger("plc_backend")
logger.setLevel(logging.INFO)
logger.propagate = False

if not logger.handlers:
    # 寫入檔案的 handler
    file_handler = logging.FileHandler(LOG_FILE, encoding="utf-8")
    file_handler.setFormatter(
        logging.Formatter("%(asctime)s [%(levelname)s] %(name)s - %(message)s")
    )
    logger.addHandler(file_handler)

    # 印到終端機的 handler（避免依賴 basicConfig）
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(
        logging.Formatter("%(asctime)s [%(levelname)s] %(name)s - %(message)s")
    )
    logger.addHandler(console_handler)


def _progress_logger(flow_name: str):
    last_logged_at = 0.0
    last_state: str | None = None

    def report(progress) -> None:
        nonlocal last_logged_at, last_state
        now = time.monotonic()
        state = getattr(progress.state, "value", str(progress.state))
        lifecycle_status = getattr(progress.status, "value", str(progress.status))
        if state == last_state and now - last_logged_at < 1.0:
            return
        last_logged_at = now
        last_state = state
        step = getattr(progress, "step", state)
        logger.info(
            "%s running status=%s state=%s step=%s elapsed=%.1fs message=%s",
            flow_name,
            lifecycle_status,
            state,
            step,
            progress.elapsed_seconds,
            progress.message,
        )

    return report


def _request_flow_cancellation(flow, cancel_event: threading.Event, flow_name: str) -> APIResponse:
    snapshot = flow.status_snapshot
    already_finished = snapshot.status.terminal
    # Always set the event.  A new request may be inside a precheck while the
    # lifecycle snapshot still reflects the previous terminal run.
    cancel_event.set()
    logger.info(
        "%s cancellation requested already_finished=%s status=%s",
        flow_name,
        already_finished,
        snapshot.status.value,
    )
    return APIResponse(
        ok=True,
        data={
            "cancel_requested": True,
            "already_finished": already_finished,
            "flow": snapshot.as_dict(),
        },
    )


def print_write_error(message: str) -> None:
    """Print write failures to the console immediately for field debugging."""

    print(f"[WRITE ERROR] {message}", file=sys.stderr, flush=True)


from pathlib import Path as _Path

_VERSION_FILE = _Path(__file__).resolve().parent.parent / "VERSION"
try:
    APP_VERSION = _VERSION_FILE.read_text(encoding="utf-8").strip()
except FileNotFoundError:
    APP_VERSION = "0.0.0"

app = FastAPI(title="PLC Backend API", version=APP_VERSION)


# ===== 資料模型 =====


class APIResponse(BaseModel):
    ok: bool = True
    data: Optional[object] = None
    error: Optional[str] = None


class WriteRegistersBody(BaseModel):
    device: str
    start: int = Field(..., ge=0)
    values: List[int]


class WriteByPointBody(BaseModel):
    point_id: str
    value: Any


class SlotStateBody(BaseModel):
    occupancy: SlotOccupancy
    cargo_id: Optional[str] = None
    purpose: CargoPurpose = CargoPurpose.UNKNOWN
    shelf_id: Optional[str] = None
    shelf_level: Optional[int] = Field(default=None, ge=0)
    confirm_release: bool = False


class SlotVacuumModeBody(BaseModel):
    mode: SlotVacuumMode
    confirm_release: bool = False


class MiddleVacuumBody(BaseModel):
    mode: MiddleVacuumMode


class MainCycleStep1Body(BaseModel):
    slot: str
    action: str
    height_mm: float
    forward_mm: Optional[float] = None
    x_speed: Optional[int] = Field(default=None, ge=1, le=32767)
    y1_speed: Optional[int] = Field(default=None, ge=1, le=32767)


class MainCycleSecondStepBody(BaseModel):
    transfer_direction: TransferDirection = TransferDirection.Y1_TO_Y2
    x_speed: Optional[int] = Field(default=None, ge=1, le=32767)
    height_reference_depth_mm: Optional[float] = Field(default=None, gt=0)


class ArmPoseBody(BaseModel):
    pose: str


class PointOut(BaseModel):
    id: str
    name: str
    plc: str
    device: str
    address: int
    type: str
    group: str
    writable: bool


class PointValueOut(BaseModel):
    point_id: str
    value: float | int | bool


# ===== 簡單的 auth stub（之後可換成真正 auth） =====


def get_current_user() -> str:
    """暫時回傳固定 user id，之後可改為 JWT / OAuth。"""

    return "demo_user"


# One shared point-oriented service owns all PLC clients and type conversion.
PLC_MANAGER = PLC_SERVICE
HOME_SERVICE = HomeService(PLC_SERVICE)
LIFT_SERVICE = LiftService(PLC_SERVICE)
Y_AXES_SERVICE = YAxesService(PLC_SERVICE)
SLOT_VACUUM_SERVICE = SlotVacuumService(PLC_SERVICE)
MIDDLE_VACUUM_SERVICE = MiddleVacuumService(PLC_SERVICE)
PALLET_TRANSFER_SERVICE = PalletTransferService(PLC_SERVICE)
VISION_BRIDGE_SERVICE = VisionBridgeService()
ARM_VISION_WORKFLOW_SERVICE = ArmVisionWorkflowService(
    target_height_validator=LIFT_SERVICE.validate_target_height,
)
HOME_FLOW = HomeFlow(HOME_SERVICE)
VISION_HEIGHT_FLOW = VisionHeightFlow(LIFT_SERVICE)
MAIN_CYCLE_FLOW = MainCycleFlow(
    LIFT_SERVICE,
    PALLET_TRANSFER_SERVICE,
    MIDDLE_VACUUM_SERVICE,
    VISION_BRIDGE_SERVICE,
    arm_vision_service=ARM_VISION_WORKFLOW_SERVICE if ARM_VISION_WORKFLOW_SERVICE.enabled else None,
    slot_vacuum_service=SLOT_VACUUM_SERVICE,
)
FLOW_LOCK = threading.Lock()
HOME_CANCEL_EVENT = threading.Event()
VISION_HEIGHT_CANCEL_EVENT = threading.Event()
MAIN_CYCLE_CANCEL_EVENT = threading.Event()
SLOT_VACUUM_SERVICE.start_watchdog()

# FastAPI startup intentionally performs no CAN motion. HOME and STANDBY are
# explicit operator/TXT/API actions so launching action_main.py cannot move the
# arm or camera unexpectedly.

# ===== Endpoint =====


@app.get("/health", response_model=APIResponse)
async def health_check() -> APIResponse:
    """健康檢查 endpoint。"""

    logger.info("Health check called")
    return APIResponse(ok=True, data={"status": "ok"})


@app.get("/lifecycle/status", response_model=APIResponse)
async def lifecycle_status() -> APIResponse:
    components = (
        PLC_SERVICE,
        HOME_SERVICE,
        LIFT_SERVICE,
        Y_AXES_SERVICE,
        SLOT_VACUUM_SERVICE,
        MIDDLE_VACUUM_SERVICE,
        PALLET_TRANSFER_SERVICE,
        VISION_BRIDGE_SERVICE,
        ARM_VISION_WORKFLOW_SERVICE,
        HOME_FLOW,
        VISION_HEIGHT_FLOW,
        MAIN_CYCLE_FLOW,
    )
    data = {
        item.status_snapshot.component: item.status_snapshot.as_dict()
        for item in components
    }
    data["ArmCameraHomeInterlock"] = ARM_CAMERA_HOME_INTERLOCK.snapshot.as_dict()
    data["ArmCameraHomeInterlock"]["not_home_maximum_height_mm"] = (
        LIFT_SERVICE.config.arm_camera_not_home_maximum_height_mm
    )
    data["ArmCameraHomeInterlock"]["home_maximum_height_mm"] = (
        LIFT_SERVICE.config.maximum_height_mm
    )
    return APIResponse(ok=True, data=data)


@app.post("/arm-camera-home/confirm", response_model=APIResponse)
def confirm_arm_camera_home() -> APIResponse:
    """Read ID142～ID145 and unlock heights above 695 mm only on stable HOME."""

    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="已有 PLC／手臂流程執行中，無法同時確認 HOME",
        )
    try:
        logger.info("Arm/camera HOME confirmation started")
        result = ARM_VISION_WORKFLOW_SERVICE.confirm_home()
        result["not_home_maximum_height_mm"] = (
            LIFT_SERVICE.arm_camera_not_home_maximum_height_mm
        )
        result["home_maximum_height_mm"] = LIFT_SERVICE.config.maximum_height_mm
        logger.info("Arm/camera HOME confirmed angles=%s", result.get("angles"))
        return APIResponse(ok=True, data=result)
    except Exception as exc:  # noqa: BLE001
        snapshot = ARM_CAMERA_HOME_INTERLOCK.snapshot
        logger.error("Arm/camera HOME confirmation failed error=%s", exc)
        return APIResponse(
            ok=False,
            data={
                **snapshot.as_dict(),
                "not_home_maximum_height_mm": (
                    LIFT_SERVICE.arm_camera_not_home_maximum_height_mm
                ),
                "home_maximum_height_mm": LIFT_SERVICE.config.maximum_height_mm,
            },
            error=str(exc),
        )
    finally:
        FLOW_LOCK.release()


def _move_can_component_home(component: str) -> APIResponse:
    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="已有 PLC／手臂流程執行中，無法同時回 HOME",
        )
    try:
        logger.info("CAN component HOME started component=%s", component)
        result = ARM_VISION_WORKFLOW_SERVICE.move_home_component(component)
        logger.info(
            "CAN component HOME confirmed component=%s angles=%s",
            component,
            result.get("angles"),
        )
        return APIResponse(ok=True, data=result)
    except Exception as exc:  # noqa: BLE001
        logger.error("CAN component HOME failed component=%s error=%s", component, exc)
        return APIResponse(ok=False, error=str(exc))
    finally:
        FLOW_LOCK.release()


@app.post("/arm-camera-home/move-arm", response_model=APIResponse)
def move_arm_home() -> APIResponse:
    """Move ID142/ID143 to HOME and confirm their stable readback."""

    return _move_can_component_home("arm")


@app.post("/arm-camera-home/move-camera", response_model=APIResponse)
def move_camera_home() -> APIResponse:
    """Move ID144/ID145 to HOME and confirm their stable readback."""

    return _move_can_component_home("camera")


@app.post("/plc/connect", response_model=APIResponse)
async def plc_connect(plc: str = "main_plc") -> APIResponse:
    """手動建立與指定 PLC 的連線。"""

    logger.info("API /plc/connect plc=%s", plc)
    try:
        PLC_MANAGER.connect(plc)
        SLOT_VACUUM_SERVICE.enforce_required_vacuum()
    except HTTPException:
        raise
    except (PLCConnectionError, PlcServiceError, SlotVacuumServiceError) as exc:
        logger.error("PLC 連線失敗 plc=%s error=%s", plc, exc)
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=str(exc),
        ) from exc

    plc_def = CONFIG_STORE.get_plc(plc)
    return APIResponse(
        ok=True,
        data={
            "plc": plc,
            "connected": True,
            "host": plc_def.host if plc_def is not None else None,
            "port": plc_def.port if plc_def is not None else None,
            "unit": plc_def.unit if plc_def is not None else None,
        },
    )


@app.post("/plc/disconnect", response_model=APIResponse)
async def plc_disconnect(plc: str = "main_plc") -> APIResponse:
    """關閉與指定 PLC 的連線。"""

    logger.info("API /plc/disconnect plc=%s", plc)
    PLC_MANAGER.disconnect(plc)
    return APIResponse(ok=True, data={"plc": plc, "connected": False})


@app.get("/slots", response_model=APIResponse)
async def get_slot_states() -> APIResponse:
    try:
        states = SLOT_VACUUM_SERVICE.list_states(include_vacuum=True)
    except Exception as exc:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=str(exc),
        ) from exc
    return APIResponse(ok=True, data=states)


@app.put("/slots/{side}", response_model=APIResponse)
async def update_slot_state(side: str, body: SlotStateBody) -> APIResponse:
    try:
        state_item = SLOT_VACUUM_SERVICE.update_state(
            side,
            body.occupancy,
            cargo_id=body.cargo_id,
            purpose=body.purpose,
            shelf_id=body.shelf_id,
            shelf_level=body.shelf_level,
            confirm_release=body.confirm_release,
        )
        states = SLOT_VACUUM_SERVICE.list_states(include_vacuum=True)
    except Exception as exc:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc
    return APIResponse(
        ok=True,
        data={"updated": state_item.as_dict(), "slots": states},
    )


@app.post("/slots/{side}/manual-release", response_model=APIResponse)
def manual_release_slot_vacuum(side: str) -> APIResponse:
    """Operator-confirmed timed break-vacuum pulse, then all outputs OFF."""

    try:
        state_item = SLOT_VACUUM_SERVICE.manual_release(side)
        states = SLOT_VACUUM_SERVICE.list_states(include_vacuum=True)
    except Exception as exc:
        logger.error("手動釋放載貨側真空失敗 side=%s error=%s", side, exc)
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc
    logger.warning("操作員已手動釋放載貨側真空 side=%s", side.upper())
    return APIResponse(
        ok=True,
        data={"updated": state_item.as_dict(), "slots": states},
    )


@app.post("/slots/enforce", response_model=APIResponse)
async def enforce_slot_vacuum() -> APIResponse:
    try:
        enforced = SLOT_VACUUM_SERVICE.enforce_required_vacuum()
    except Exception as exc:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=str(exc),
        ) from exc
    return APIResponse(ok=True, data={"enforced": enforced})


@app.get("/side-vacuum/{side}", response_model=APIResponse)
async def get_side_vacuum(side: str) -> APIResponse:
    try:
        state_item = SLOT_VACUUM_SERVICE.read_side_state(side, include_vacuum=True)
    except SlotVacuumServiceError as exc:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=str(exc),
        ) from exc
    return APIResponse(ok=True, data=state_item)


@app.put("/side-vacuum/{side}", response_model=APIResponse)
async def set_side_vacuum(side: str, body: SlotVacuumModeBody) -> APIResponse:
    try:
        state_item = SLOT_VACUUM_SERVICE.set_manual_mode(
            side,
            body.mode,
            confirm_release=body.confirm_release,
        )
    except (SlotVacuumServiceError, PlcServiceError, ValueError) as exc:
        logger.error(
            "側邊真空切換失敗 side=%s mode=%s error=%s",
            side,
            body.mode.value,
            exc,
        )
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc
    logger.info(
        "側邊真空切換完成 side=%s mode=%s",
        side.upper(),
        state_item.get("mode"),
    )
    return APIResponse(ok=True, data=state_item)


@app.get("/middle-vacuum", response_model=APIResponse)
async def get_middle_vacuum() -> APIResponse:
    try:
        state_item = MIDDLE_VACUUM_SERVICE.read_state()
    except Exception as exc:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=str(exc),
        ) from exc
    return APIResponse(ok=True, data=state_item)


@app.put("/middle-vacuum", response_model=APIResponse)
async def set_middle_vacuum(body: MiddleVacuumBody) -> APIResponse:
    try:
        state_item = MIDDLE_VACUUM_SERVICE.set_mode(body.mode)
    except (MiddleVacuumServiceError, PlcServiceError) as exc:
        logger.error("中間真空切換失敗 mode=%s error=%s", body.mode.value, exc)
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc
    logger.info(
        "中間真空切換完成 mode=%s M54=%s M56=%s",
        state_item["mode"],
        state_item["vacuum_on"],
        state_item["break_vacuum_on"],
    )
    return APIResponse(ok=True, data=state_item)


@app.get("/points", response_model=APIResponse)
async def list_points(group: Optional[str] = None) -> APIResponse:
    """列出點位定義（可依 group 過濾）。"""

    logger.info("API /points group=%s", group)
    points = CONFIG_STORE.list_points(group=group)
    out = [
        PointOut(
            id=p.id,
            name=p.name,
            plc=p.plc,
            device=p.device,
            address=p.address,
            type=p.type,
            group=p.group,
            writable=p.writable,
        )
        for p in points
    ]
    logger.info("/points 回傳 %s 筆資料", len(out))
    return APIResponse(ok=True, data=out)


@app.get("/points/{point_id}", response_model=APIResponse)
async def get_point(point_id: str) -> APIResponse:
    logger.info("API /points/%s", point_id)
    point = CONFIG_STORE.get_point(point_id)
    if point is None:
        logger.warning("查無 point 定義 point_id=%s", point_id)
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"point '{point_id}' not found",
        )
    out = PointOut(
        id=point.id,
        name=point.name,
        plc=point.plc,
        device=point.device,
        address=point.address,
        type=point.type,
        group=point.group,
        writable=point.writable,
    )
    return APIResponse(ok=True, data=out)


@app.get("/plc/registers/by-points", response_model=APIResponse)
async def get_registers_by_points(
    ids: List[str] = Query(..., description="point_id 清單"),
    plc: Optional[str] = None,
    user: str = Depends(get_current_user),  # noqa: ARG001
) -> APIResponse:
    """Read configured points through the shared point-oriented service."""

    results: List[PointValueOut] = []

    for pid in ids:
        point = CONFIG_STORE.get_point(pid)
        if point is None:
            logger.warning("get_registers_by_points: 找不到 point_id=%s", pid)
            continue

        if plc is not None and plc != point.plc:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=(
                    f"point '{pid}' belongs to PLC '{point.plc}', "
                    f"not requested PLC '{plc}'"
                ),
            )

        try:
            value = PLC_SERVICE.read_point(pid)

            logger.info(
                "by-points 讀取成功 point_id=%s plc=%s device=%s addr=%s value=%s",
                pid,
                point.plc,
                point.device,
                point.address,
                value,
            )
            results.append(PointValueOut(point_id=pid, value=value))
        except (PLCConnectionError, PlcServiceError) as exc:
            logger.error(
                "by-points 讀取失敗 point_id=%s plc=%s error=%s",
                pid,
                point.plc,
                exc,
            )
            # 若是連線被中止，關閉 client，讓下次呼叫時重新連線
            if "10053" in str(exc):
                PLC_SERVICE.disconnect(point.plc)
                break
            continue

    return APIResponse(ok=True, data=[r.model_dump() for r in results])


@app.get("/plc/registers", response_model=APIResponse)
async def get_registers(
    device: str,
    start: int,
    count: int = 1,
    plc: str = "main_plc",
    user: str = Depends(get_current_user),  # noqa: ARG001
) -> APIResponse:
    """讀取指定裝置暫存器。"""

    logger.info(
        "API /plc/registers plc=%s device=%s start=%s count=%s",
        plc,
        device,
        start,
        count,
    )

    try:
        if device.upper() == "D":
            data = PLC_MANAGER.read_d_register(plc, start, count=count)
        else:
            # 其他以 bit 處理
            bits = PLC_MANAGER.read_bit_device(plc, device.upper(), start, count=count)
            data = [int(b) for b in bits]
        logger.info(
            "讀取成功 plc=%s device=%s start=%s count=%s data=%s",
            plc,
            device,
            start,
            count,
            data,
        )
    except (PLCConnectionError, PlcServiceError) as exc:
        logger.error(
            "讀取失敗 plc=%s device=%s start=%s count=%s error=%s",
            plc,
            device,
            start,
            count,
            exc,
        )
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=str(exc),
        ) from exc

    return APIResponse(ok=True, data=data)


@app.post("/plc/registers/write", response_model=APIResponse)
async def write_registers(
    body: WriteRegistersBody,
    plc: str = "main_plc",
    user: str = Depends(get_current_user),  # noqa: ARG001
) -> APIResponse:
    """直接以 device + address 寫入暫存器。"""

    logger.info(
        "API /plc/registers/write plc=%s device=%s start=%s values=%s",
        plc,
        body.device,
        body.start,
        body.values,
    )

    if not body.values:
        logger.warning("寫入時 values 為空，拒絕執行")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="values 不可為空",
        )

    try:
        if body.device.upper() == "D":
            PLC_MANAGER.write_d_register(plc, body.start, body.values)
        else:
            bool_values = [bool(v) for v in body.values]
            PLC_MANAGER.write_bit_device(plc, body.device.upper(), body.start, bool_values)
        logger.info(
            "寫入成功 plc=%s device=%s start=%s values=%s",
            plc,
            body.device,
            body.start,
            body.values,
        )
    except (LiftServiceError, SlotVacuumServiceError, MiddleVacuumServiceError) as exc:
        print_write_error(
            f"/plc/registers/write plc={plc} device={body.device} "
            f"start={body.start} values={body.values} safety_error={exc}"
        )
        logger.warning(
            "安全互鎖拒絕寫入 plc=%s device=%s start=%s values=%s error=%s",
            plc,
            body.device,
            body.start,
            body.values,
            exc,
        )
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=str(exc),
        ) from exc
    except (PLCConnectionError, PlcServiceError) as exc:
        print_write_error(
            f"/plc/registers/write plc={plc} device={body.device} "
            f"start={body.start} values={body.values} error={exc}"
        )
        logger.error(
            "寫入失敗 plc=%s device=%s start=%s values=%s error=%s",
            plc,
            body.device,
            body.start,
            body.values,
            exc,
        )
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=str(exc),
        ) from exc

    return APIResponse(ok=True, data={"written": len(body.values)})


@app.post("/plc/registers/write-by-point", response_model=APIResponse)
async def write_by_point(
    body: WriteByPointBody,
    plc: Optional[str] = None,
    user: str = Depends(get_current_user),  # noqa: ARG001
) -> APIResponse:
    """Write a configured point through the shared point-oriented service."""

    logger.info(
        "API /plc/registers/write-by-point point_id=%s value=%s plc=%s",
        body.point_id,
        body.value,
        plc,
    )

    point: PointDefinition | None = CONFIG_STORE.get_point(body.point_id)
    if point is None:
        print_write_error(
            f"/plc/registers/write-by-point point_id={body.point_id} "
            f"value={body.value} error=point not found"
        )
        logger.warning("查無 point 定義，無法寫入 point_id=%s", body.point_id)
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"point '{body.point_id}' not found",
        )

    if plc is not None and plc != point.plc:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=(
                f"point '{point.id}' belongs to PLC '{point.plc}', "
                f"not requested PLC '{plc}'"
            ),
        )

    try:
        PLC_SERVICE.write_point(point.id, body.value)

        logger.info(
            "write-by-point 成功 point_id=%s name=%s plc=%s value=%s",
            point.id,
            point.name,
            point.plc,
            body.value,
        )
    except (LiftServiceError, SlotVacuumServiceError, MiddleVacuumServiceError) as exc:
        print_write_error(
            f"/plc/registers/write-by-point point_id={point.id} "
            f"plc={point.plc} value={body.value} safety_error={exc}"
        )
        logger.warning(
            "安全互鎖拒絕 point 寫入 point_id=%s plc=%s value=%s error=%s",
            point.id,
            point.plc,
            body.value,
            exc,
        )
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=str(exc),
        ) from exc
    except PlcPointValidationError as exc:
        print_write_error(
            f"/plc/registers/write-by-point point_id={point.id} "
            f"plc={point.plc} value={body.value} validation_error={exc}"
        )
        logger.warning(
            "point 寫入驗證失敗 point_id=%s plc=%s value=%s error=%s",
            point.id,
            point.plc,
            body.value,
            exc,
        )
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(exc),
        ) from exc
    except (PLCConnectionError, PlcServiceError) as exc:
        print_write_error(
            f"/plc/registers/write-by-point point_id={point.id} "
            f"plc={point.plc} value={body.value} error={exc}"
        )
        logger.error(
            "write-by-point 失敗 point_id=%s plc=%s error=%s",
            point.id,
            point.plc,
            exc,
        )
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=str(exc),
        ) from exc

    return APIResponse(
        ok=True,
        data={
            "point_id": point.id,
            "name": point.name,
            "value": body.value,
            "plc": point.plc,
        },
    )


@app.post("/flows/home/run", response_model=APIResponse)
def run_home_flow() -> APIResponse:
    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="已有 PLC Flow 執行中")
    HOME_CANCEL_EVENT.clear()
    try:
        logger.info("HomeFlow started")
        result = HOME_FLOW.run(HOME_CANCEL_EVENT, _progress_logger("HomeFlow"))
        logger.info(
            "HomeFlow completed state=%s elapsed=%.1fs message=%s",
            result.state.value,
            result.elapsed_seconds,
            result.message,
        )
        return APIResponse(
            ok=result.succeeded,
            data={
                "status": result.status.value,
                "state": result.state.value,
                "step": result.step,
                "message": result.message,
                "elapsed_seconds": result.elapsed_seconds,
                "positions": result.positions,
            },
            error=None if result.succeeded else result.message,
        )
    finally:
        FLOW_LOCK.release()


@app.post("/flows/home/cancel", response_model=APIResponse)
def cancel_home_flow() -> APIResponse:
    return _request_flow_cancellation(HOME_FLOW, HOME_CANCEL_EVENT, "HomeFlow")


@app.post("/flows/vision-height/run", response_model=APIResponse)
def run_vision_height_flow() -> APIResponse:
    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="已有 PLC Flow 執行中")
    VISION_HEIGHT_CANCEL_EVENT.clear()
    try:
        logger.info("VisionHeightFlow started")
        result = VISION_HEIGHT_FLOW.run(
            VISION_HEIGHT_CANCEL_EVENT,
            _progress_logger("VisionHeightFlow"),
        )
        logger.info(
            "VisionHeightFlow completed state=%s elapsed=%.1fs message=%s",
            result.state.value,
            result.elapsed_seconds,
            result.message,
        )
        return APIResponse(
            ok=result.succeeded,
            data={
                "status": result.status.value,
                "state": result.state.value,
                "step": result.step,
                "message": result.message,
                "elapsed_seconds": result.elapsed_seconds,
                "height_mm": result.height_mm,
            },
            error=None if result.succeeded else result.message,
        )
    finally:
        FLOW_LOCK.release()


@app.post("/flows/vision-height/cancel", response_model=APIResponse)
def cancel_vision_height_flow() -> APIResponse:
    return _request_flow_cancellation(
        VISION_HEIGHT_FLOW,
        VISION_HEIGHT_CANCEL_EVENT,
        "VisionHeightFlow",
    )


def _main_cycle_result_data(result) -> dict[str, Any]:
    home_snapshot = ARM_CAMERA_HOME_INTERLOCK.snapshot
    effective_maximum_height_mm = (
        LIFT_SERVICE.config.maximum_height_mm
        if home_snapshot.all_home_confirmed
        else LIFT_SERVICE.arm_camera_not_home_maximum_height_mm
    )
    return {
        "status": result.status.value,
        "state": result.state.value,
        "phase": result.phase.value,
        "step": result.step,
        "message": result.message,
        "elapsed_seconds": result.elapsed_seconds,
        "height_mm": result.height_mm,
        "vision_height_mm": result.vision_height_mm,
        "transfer_direction": result.transfer_direction,
        "arm_camera_home": home_snapshot.as_dict(),
        "effective_maximum_height_mm": effective_maximum_height_mm,
    }


def _main_cycle_response_after_home_postcheck(
    step_label: str,
    result,
    *,
    extra_data: dict[str, Any] | None = None,
) -> APIResponse:
    """Confirm arm and camera HOME after every completed main-cycle step."""

    data = _main_cycle_result_data(result)
    if extra_data:
        data.update(extra_data)
    original_result = {
        "status": result.status.value,
        "state": result.state.value,
        "step": result.step,
        "message": result.message,
    }
    MAIN_CYCLE_FLOW._set_status(
        LifecycleStatus.RUNNING,
        "post_step_home_check",
        f"{step_label}已結束，正在確認手臂與 Camera 是否都在 HOME",
        phase=result.phase.value,
        height_mm=result.height_mm,
        vision_height_mm=result.vision_height_mm,
        transfer_direction=result.transfer_direction,
    )
    logger.info("%s post-step HOME confirmation started", step_label)
    try:
        # The motion step has already issued its stop commands. Use a fresh
        # event so an earlier cancellation does not skip this read-only check.
        home_postcheck = ARM_VISION_WORKFLOW_SERVICE.confirm_home(
            threading.Event()
        )
    except Exception as exc:  # noqa: BLE001
        snapshot = ARM_CAMERA_HOME_INTERLOCK.snapshot
        message = (
            f"{step_label}已結束，但手臂／Camera HOME 結束確認未通過：{exc}"
        )
        if not result.succeeded:
            message = f"{result.message}；此外，手臂／Camera HOME 結束確認未通過：{exc}"
        stop_unconfirmed = result.state is MainCycleState.STOP_UNCONFIRMED
        data.update(
            {
                "status": "error",
                "state": (
                    MainCycleState.STOP_UNCONFIRMED.value
                    if stop_unconfirmed
                    else MainCycleState.ERROR.value
                ),
                "step": "post_step_home_check",
                "message": message,
                "step_result": original_result,
                "home_postcheck": {
                    "confirmed": False,
                    "arm_home_confirmed": False,
                    "camera_home_confirmed": False,
                    "error": str(exc),
                },
                "arm_camera_home": snapshot.as_dict(),
                "effective_maximum_height_mm": (
                    LIFT_SERVICE.arm_camera_not_home_maximum_height_mm
                ),
            }
        )
        MAIN_CYCLE_FLOW._set_status(
            LifecycleStatus.ERROR,
            "post_step_home_check",
            message,
            state=data["state"],
            phase=result.phase.value,
            height_mm=result.height_mm,
            vision_height_mm=result.vision_height_mm,
            transfer_direction=result.transfer_direction,
        )
        logger.error("%s", message)
        return APIResponse(ok=False, data=data, error=message)

    snapshot = ARM_CAMERA_HOME_INTERLOCK.snapshot
    home_postcheck = dict(home_postcheck)
    home_postcheck.update(
        {
            "confirmed": True,
            "arm_home_confirmed": True,
            "camera_home_confirmed": True,
        }
    )
    data.update(
        {
            "home_postcheck": home_postcheck,
            "arm_camera_home": snapshot.as_dict(),
            "effective_maximum_height_mm": LIFT_SERVICE.config.maximum_height_mm,
        }
    )
    message = f"{result.message}；手臂與 Camera HOME 已確認"
    data["message"] = message
    if result.succeeded:
        MAIN_CYCLE_FLOW._set_status(
            LifecycleStatus.SUCCESS,
            "post_step_home_check",
            message,
            state=result.state.value,
            phase=result.phase.value,
            height_mm=result.height_mm,
            vision_height_mm=result.vision_height_mm,
            transfer_direction=result.transfer_direction,
        )
        logger.info(
            "%s post-step HOME confirmed angles=%s",
            step_label,
            home_postcheck.get("angles"),
        )
        return APIResponse(ok=True, data=data)

    MAIN_CYCLE_FLOW._set_status(
        result.status,
        result.step,
        message,
        state=result.state.value,
        phase=result.phase.value,
        height_mm=result.height_mm,
        vision_height_mm=result.vision_height_mm,
        transfer_direction=result.transfer_direction,
        home_postcheck=home_postcheck,
    )
    logger.info(
        "%s ended with status=%s; post-step HOME nevertheless confirmed",
        step_label,
        result.status.value,
    )
    return APIResponse(ok=False, data=data, error=result.message)


def _main_cycle_command(body: MainCycleStep1Body) -> Step1Command:
    return Step1Command(
        slot=body.slot,
        action=body.action,
        height_mm=body.height_mm,
        forward_mm=body.forward_mm,
        x_speed=body.x_speed,
        y1_speed=body.y1_speed,
    )


def _run_independent_main_cycle_step(
    step_label: str,
    runner: Callable[[], Any],
    *,
    before_home_precheck: Callable[[], Any] | None = None,
    require_arm_camera_home: bool = True,  #新判斷
) -> APIResponse:
    """Atomically prepare, confirm HOME, and run an independent block."""

    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="已有 PLC Flow 執行中")
    MAIN_CYCLE_CANCEL_EVENT.clear()
    try:
        preparation = None
        if before_home_precheck is not None:
            logger.info("%s safe-height preparation started", step_label)
            try:
                preparation = before_home_precheck()
            except Exception as exc:  # noqa: BLE001
                snapshot = ARM_CAMERA_HOME_INTERLOCK.snapshot
                cancelled = MAIN_CYCLE_CANCEL_EVENT.is_set()
                message = (
                    f"{step_label}已取消：560mm 定位中止，子流程未啟動"
                    if cancelled
                    else f"{step_label}執行前無法定位並確認 560mm：{exc}"
                )
                MAIN_CYCLE_FLOW._set_status(
                    LifecycleStatus.CANCELLED if cancelled else LifecycleStatus.ERROR,
                    "independent_second_step_to_safe_height",
                    message,
                    phase=MAIN_CYCLE_FLOW.phase.value,
                )
                logger.error("%s", message)
                return APIResponse(
                    ok=False,
                    data={
                        "status": "cancelled" if cancelled else "error",
                        "state": "cancelled" if cancelled else "error",
                        "phase": MAIN_CYCLE_FLOW.phase.value,
                        "step": "independent_second_step_to_safe_height",
                        "message": message,
                        "elapsed_seconds": 0.0,
                        "height_mm": None,
                        "vision_height_mm": None,
                        "transfer_direction": None,
                        "arm_camera_home": snapshot.as_dict(),
                        "effective_maximum_height_mm": (
                            LIFT_SERVICE.config.maximum_height_mm
                            if snapshot.all_home_confirmed
                            else LIFT_SERVICE.arm_camera_not_home_maximum_height_mm
                        ),
                    },
                    error=message,
                )
                
                # PLC 獨立流程可選擇不受手臂／Camera HOME 狀態影響。
        # LiftService 的高度限制仍然有效：
        # 未確認 HOME 時仍只能定位到安全上限以下。
        if not require_arm_camera_home:
            logger.warning(
                "%s skipped arm/camera HOME precheck for PLC-independent operation",
                step_label,
            )

            if MAIN_CYCLE_CANCEL_EVENT.is_set():
                message = f"{step_label}已取消，子流程未啟動"
                return APIResponse(
                    ok=False,
                    data={
                        "status": "cancelled",
                        "state": "cancelled",
                        "phase": MAIN_CYCLE_FLOW.phase.value,
                        "step": "independent_cancelled",
                        "message": message,
                        "height_mm": preparation,
                        "arm_camera_home": (
                            ARM_CAMERA_HOME_INTERLOCK.snapshot.as_dict()
                        ),
                        "effective_maximum_height_mm": (
                            LIFT_SERVICE.arm_camera_not_home_maximum_height_mm
                        ),
                        "home_precheck_skipped": True,
                    },
                    error=message,
                )

            result = runner()
            data = _main_cycle_result_data(result)
            data["home_precheck_skipped"] = True

            if preparation is not None:
                data["safe_height_precheck_mm"] = preparation

            logger.warning(
                "%s completed without arm/camera HOME precheck or postcheck",
                step_label,
            )

            return APIResponse(
                ok=result.succeeded,
                data=data,
                error=None if result.succeeded else result.message,
            )        
        #新增程式碼
        MAIN_CYCLE_FLOW.begin_independent_precheck(step_label)
        logger.info("%s HOME precheck started", step_label)
        try:
            home_precheck = ARM_VISION_WORKFLOW_SERVICE.confirm_home(
                MAIN_CYCLE_CANCEL_EVENT
            )
        except Exception as exc:  # noqa: BLE001
            snapshot = ARM_CAMERA_HOME_INTERLOCK.snapshot
            cancelled = MAIN_CYCLE_CANCEL_EVENT.is_set()
            message = (
                f"{step_label}已取消：HOME 預檢中止，子流程未啟動"
                if cancelled
                else f"{step_label}執行前的四軸 HOME 確認未通過：{exc}"
            )
            MAIN_CYCLE_FLOW.finish_independent_precheck(
                LifecycleStatus.CANCELLED if cancelled else LifecycleStatus.ERROR,
                message,
            )
            logger.error("%s", message)
            return APIResponse(
                ok=False,
                data={
                    "status": "cancelled" if cancelled else "error",
                    "state": "cancelled" if cancelled else "error",
                    "phase": MAIN_CYCLE_FLOW.phase.value,
                    "step": "independent_home_precheck",
                    "message": message,
                    "elapsed_seconds": 0.0,
                    "height_mm": preparation,
                    "vision_height_mm": None,
                    "transfer_direction": None,
                    "arm_camera_home": snapshot.as_dict(),
                    "effective_maximum_height_mm": (
                        LIFT_SERVICE.arm_camera_not_home_maximum_height_mm
                    ),
                },
                error=message,
            )

        logger.info(
            "%s HOME precheck passed angles=%s",
            step_label,
            home_precheck.get("angles"),
        )
        if MAIN_CYCLE_CANCEL_EVENT.is_set():
            message = f"{step_label}已取消：HOME 確認後收到取消要求，子流程未啟動"
            MAIN_CYCLE_FLOW.finish_independent_precheck(
                LifecycleStatus.CANCELLED,
                message,
            )
            return APIResponse(
                ok=False,
                data={
                    "status": "cancelled",
                    "state": "cancelled",
                    "phase": MAIN_CYCLE_FLOW.phase.value,
                    "step": "independent_home_precheck",
                    "message": message,
                    "elapsed_seconds": 0.0,
                    "height_mm": preparation,
                    "vision_height_mm": None,
                    "transfer_direction": None,
                    "arm_camera_home": ARM_CAMERA_HOME_INTERLOCK.snapshot.as_dict(),
                    "effective_maximum_height_mm": (
                        LIFT_SERVICE.config.maximum_height_mm
                    ),
                },
                error=message,
            )
        result = runner()
        extra_data: dict[str, Any] = {"home_precheck": home_precheck}
        if preparation is not None:
            extra_data["safe_height_precheck_mm"] = preparation
        return _main_cycle_response_after_home_postcheck(
            step_label,
            result,
            extra_data=extra_data,
        )
    finally:
        FLOW_LOCK.release()


@app.post("/flows/main-cycle/first-step", response_model=APIResponse)
def run_main_cycle_first_step(body: MainCycleStep1Body) -> APIResponse:
    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="已有 PLC Flow 執行中")
    MAIN_CYCLE_CANCEL_EVENT.clear()
    try:
        logger.info("MainCycleFlow first step started body=%s", body.dict())
        result = MAIN_CYCLE_FLOW.run_first_step(
            _main_cycle_command(body),
            MAIN_CYCLE_CANCEL_EVENT,
            _progress_logger("MainCycleFlow"),
        )
        return _main_cycle_response_after_home_postcheck(
            "第一步",
            result,
        )
    finally:
        FLOW_LOCK.release()


@app.post("/flows/main-cycle/second-step", response_model=APIResponse)
def run_main_cycle_second_step(
    body: MainCycleSecondStepBody | None = None,
) -> APIResponse:
    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="已有 PLC Flow 執行中")
    MAIN_CYCLE_CANCEL_EVENT.clear()
    try:
        direction = (
            TransferDirection.Y1_TO_Y2
            if body is None
            else body.transfer_direction
        )
        logger.info(
            "MainCycleFlow second step started direction=%s",
            direction.value,
        )
        result = MAIN_CYCLE_FLOW.run_second_step(
            MAIN_CYCLE_CANCEL_EVENT,
            _progress_logger("MainCycleFlow"),
            transfer_direction=direction,
            x_speed=None if body is None else body.x_speed,
            height_reference_depth_mm=(
                None if body is None else body.height_reference_depth_mm
            ),
        )
        return _main_cycle_response_after_home_postcheck(
            "第二步",
            result,
        )
    finally:
        FLOW_LOCK.release()


@app.post("/flows/main-cycle/final-step", response_model=APIResponse)
def run_main_cycle_final_step(body: MainCycleStep1Body) -> APIResponse:
    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="已有 PLC Flow 執行中")
    MAIN_CYCLE_CANCEL_EVENT.clear()
    try:
        logger.info("MainCycleFlow final step started body=%s", body.dict())
        result = MAIN_CYCLE_FLOW.run_final_step(
            _main_cycle_command(body),
            MAIN_CYCLE_CANCEL_EVENT,
            _progress_logger("MainCycleFlow"),
        )
        return _main_cycle_response_after_home_postcheck(
            "第三步",
            result,
        )
    finally:
        FLOW_LOCK.release()


@app.post("/flows/main-cycle/independent/first-step", response_model=APIResponse)
def run_independent_main_cycle_first_step(body: MainCycleStep1Body) -> APIResponse:
    logger.info("Independent main-cycle first step requested body=%s", body.dict())
    return _run_independent_main_cycle_step(
        "獨立第一步",
        lambda: MAIN_CYCLE_FLOW.run_independent_first_step(
            _main_cycle_command(body),
            MAIN_CYCLE_CANCEL_EVENT,
            _progress_logger("MainCycleFlow"),
        ),
        require_arm_camera_home=False,
    )

@app.post(
    "/flows/main-cycle/independent/second-step",
    response_model=APIResponse,
)
def run_independent_main_cycle_second_step(
    body: MainCycleSecondStepBody | None = None,
) -> APIResponse:
    """獨立第二步：

    STANDBY
    → 手臂與 Camera 四軸同步回 HOME 並確認
    → PLC 移到視覺高度
    → 執行視覺吸取／搬運
    → 成功後維持 HOME
    """

    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="已有 PLC／手臂流程執行中",
        )

    MAIN_CYCLE_CANCEL_EVENT.clear()

    try:
        direction = (
            TransferDirection.Y1_TO_Y2
            if body is None
            else body.transfer_direction
        )

        logger.info(
            "Independent main-cycle second step requested direction=%s",
            direction.value,
        )

        # 1. ID142～ID145 同步回 HOME，並統一確認四軸讀值。
        logger.info("獨立第二步：手臂與 Camera 四軸同步回 HOME")

        home_pose_result = ARM_VISION_WORKFLOW_SERVICE.move_named_pose("HOME")

        if MAIN_CYCLE_CANCEL_EVENT.is_set():
            message = "獨立第二步已取消：四軸同步回 HOME 後停止"
            return APIResponse(
                ok=False,
                data={
                    "status": "cancelled",
                    "step": "move_home_pose",
                    "message": message,
                    "home_pose": home_pose_result,
                },
                error=message,
            )

        home_precheck = home_pose_result

        # 2. HOME 確認完成後，PLC 才移動到視覺高度
        logger.info("獨立第二步：PLC 移到視覺高度")

        safe_height_mm = (
            MAIN_CYCLE_FLOW.prepare_independent_second_step_height(
                MAIN_CYCLE_CANCEL_EVENT,
                _progress_logger("MainCycleFlow"),
                x_speed=None if body is None else body.x_speed,
            )
        )

        if MAIN_CYCLE_CANCEL_EVENT.is_set():
            message = "獨立第二步已取消：視覺高度定位後停止"
            return APIResponse(
                ok=False,
                data={
                    "status": "cancelled",
                    "step": "prepare_vision_height",
                    "message": message,
                    "home_pose": home_pose_result,
                    "home_precheck": home_precheck,
                    "safe_height_mm": safe_height_mm,
                },
                error=message,
            )

        # 3. 執行視覺辨識、吸取及搬運
        logger.info("獨立第二步：開始視覺吸取流程")

        result = MAIN_CYCLE_FLOW.run_independent_second_step(
            MAIN_CYCLE_CANCEL_EVENT,
            _progress_logger("MainCycleFlow"),
            transfer_direction=direction,
            x_speed=None if body is None else body.x_speed,
            height_reference_depth_mm=(
                None if body is None else body.height_reference_depth_mm
            ),
        )

        data = _main_cycle_result_data(result)

        data.update(
            {
                "home_pose": home_pose_result,
                "home_precheck": home_precheck,
                "safe_height_mm": safe_height_mm,
            }
        )

        # 第二步失敗或取消時，不要自動移動馬達
        if not result.succeeded:
            data.update(
                {
                    "standby_pose_attempted": False,
                    "final_pose": "unknown",
                }
            )

            return APIResponse(
                ok=False,
                data=data,
                error=result.message,
            )

        # 4. 手臂流程的成功條件已包含四軸 HOME 角度回讀。
        # 完成後保持 HOME，不再自動移動到 STANDBY。
        home_snapshot = ARM_CAMERA_HOME_INTERLOCK.snapshot
        if not home_snapshot.all_home_confirmed:
            raise RuntimeError(
                "獨立第二步雖回報完成，但四軸 HOME 互鎖狀態未確認"
            )

        data.update(
            {
                "standby_pose_attempted": False,
                "standby_pose_confirmed": False,
                "final_pose": "HOME",
                "home_postcheck_skipped": False,
                "home_confirmation_source": "arm_vision_workflow",
                "arm_camera_home": home_snapshot.as_dict(),
                "effective_maximum_height_mm": LIFT_SERVICE.config.maximum_height_mm,
            }
        )

        logger.info("獨立第二步完成，四軸 HOME 已確認並維持 HOME")
        return APIResponse(
            ok=True,
            data=data,
        )

    except Exception as exc:  # noqa: BLE001
        logger.exception(
            "獨立第二步執行失敗 error=%s",
            exc,
        )

        snapshot = ARM_CAMERA_HOME_INTERLOCK.snapshot

        return APIResponse(
            ok=False,
            data={
                "status": "error",
                "state": "error",
                "step": "independent_second_step",
                "message": str(exc),
                "arm_camera_home": snapshot.as_dict(),
                "effective_maximum_height_mm": (
                    LIFT_SERVICE.config.maximum_height_mm
                    if snapshot.all_home_confirmed
                    else LIFT_SERVICE.arm_camera_not_home_maximum_height_mm
                ),
            },
            error=str(exc),
        )

    finally:
        FLOW_LOCK.release()

def _move_arm_camera_pose(pose_name: str) -> APIResponse:
    normalized_pose = str(pose_name).strip().upper()
    if normalized_pose not in {"HOME", "STANDBY"}:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="pose 必須是 HOME 或 STANDBY",
        )

    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"已有 PLC／手臂流程執行中，無法移動到 {normalized_pose}",
        )

    try:
        logger.info("Manual four-axis pose movement started pose=%s", normalized_pose)

        result = ARM_VISION_WORKFLOW_SERVICE.move_named_pose(
            normalized_pose
        )

        logger.info(
            "Manual four-axis pose movement completed pose=%s angles=%s",
            normalized_pose,
            result.get("angles"),
        )

        return APIResponse(
            ok=True,
            data=result,
        )

    except Exception as exc:  # noqa: BLE001
        logger.error(
            "Manual four-axis pose movement failed pose=%s error=%s",
            normalized_pose,
            exc,
        )

        return APIResponse(
            ok=False,
            error=str(exc),
        )

    finally:
        FLOW_LOCK.release()


@app.post("/arm-camera-pose/move", response_model=APIResponse)
def move_arm_camera_pose(body: ArmPoseBody) -> APIResponse:
    """Synchronously dispatch and confirm HOME or STANDBY for ID142-ID145."""

    return _move_arm_camera_pose(body.pose)


@app.post("/arm-camera-standby/move", response_model=APIResponse)
def move_arm_camera_standby() -> APIResponse:
    """Backward-compatible STANDBY endpoint."""

    return _move_arm_camera_pose("STANDBY")


@app.post("/flows/main-cycle/independent/third-step", response_model=APIResponse)
def run_independent_main_cycle_third_step(body: MainCycleStep1Body) -> APIResponse:
    logger.info("Independent main-cycle third step requested body=%s", body.dict())
    return _run_independent_main_cycle_step(
        "獨立第三步",
        lambda: MAIN_CYCLE_FLOW.run_independent_third_step(
            _main_cycle_command(body),
            MAIN_CYCLE_CANCEL_EVENT,
            _progress_logger("MainCycleFlow"),
        ),
        require_arm_camera_home=False,
    )


@app.post("/flows/main-cycle/cancel", response_model=APIResponse)
def cancel_main_cycle_flow() -> APIResponse:
    return _request_flow_cancellation(
        MAIN_CYCLE_FLOW,
        MAIN_CYCLE_CANCEL_EVENT,
        "MainCycleFlow",
    )


@app.post("/flows/main-cycle/reset", response_model=APIResponse)
def reset_main_cycle_flow() -> APIResponse:
    if not FLOW_LOCK.acquire(blocking=False):
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="已有 PLC Flow 執行中")
    try:
        MAIN_CYCLE_CANCEL_EVENT.clear()
        MAIN_CYCLE_FLOW.reset()
        logger.info("MainCycleFlow reset")
        return APIResponse(ok=True, data={"phase": MAIN_CYCLE_FLOW.phase.value})
    finally:
        FLOW_LOCK.release()
