#!/usr/bin/env python3
"""
canbus_daemon.py — Persistent CAN-bus / robotic-arm connection server.

Opens the CAN connection (via canbus/ArmController) once and keeps it
connected for as long as this process runs, instead of every caller
reconnecting from scratch (which used to cost a serial reopen + a hardcoded
1-second "warming up" delay on every single arm action). Other processes
(d435_control.py, via canbus/remote_arm_controller.py) send commands to this
daemon over local-only HTTP instead of opening the serial port themselves.

Only this process may hold the CAN connection open at a time — run it before
anything that needs to move the arm, and leave it running for the session:

    python canbus_daemon.py

Manual diagnostic tools (canbus/2motor_sync.py) still work standalone, but
only while this daemon is NOT running — the existing cross-process lock file
(see canbus/motor_service.py) will correctly reject a second connection
attempt while the daemon holds the port.

Endpoints (127.0.0.1 by default):
    GET  /health              -> connection status
    GET  /positions           -> ArmController.read_positions()
    POST /go_to_point         -> ArmController.go_to_point(point_name, current_positions)
    POST /run_targets         -> ArmController.run_targets(targets)
    POST /absolute_position   -> MotorService.absolute_position_control(motor_id, angle, speed)
    POST /stop_all            -> ArmController.stop_all()
    POST /shutdown_all        -> ArmController.shutdown_all()
    POST /reset_input_buffer  -> MotorService._serial.reset_input_buffer()
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
from pathlib import Path
from typing import Any

import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

BASE_DIR = Path(__file__).resolve().parent
CANBUS_DIR = BASE_DIR / "canbus"
if str(CANBUS_DIR) not in sys.path:
    sys.path.insert(0, str(CANBUS_DIR))

from arm_controller import ArmController  # noqa: E402

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8757

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(message)s")
logger = logging.getLogger("canbus_daemon")

controller = ArmController()
command_lock = threading.Lock()
app = FastAPI()


class GoToPointBody(BaseModel):
    point_name: str
    current_positions: dict[str, float] | None = None


class RunTargetsBody(BaseModel):
    targets: dict[str, float]


class AbsolutePositionBody(BaseModel):
    motor_id: int
    angle_degrees: float
    max_speed_dps: int = 500


@app.get("/health")
def health() -> dict[str, Any]:
    return {"status": "ok", "connected": controller.is_connected}


@app.get("/positions")
def positions() -> dict[str, Any]:
    with command_lock:
        return controller.read_positions()


@app.post("/go_to_point")
def go_to_point(body: GoToPointBody) -> dict[str, Any]:
    with command_lock:
        return controller.go_to_point(body.point_name, body.current_positions)


@app.post("/run_targets")
def run_targets(body: RunTargetsBody) -> dict[str, Any]:
    with command_lock:
        return controller.run_targets(body.targets)


@app.post("/absolute_position")
def absolute_position(body: AbsolutePositionBody) -> dict[str, Any]:
    with command_lock:
        try:
            return controller.service.absolute_position_control(
                body.motor_id, body.angle_degrees, body.max_speed_dps
            )
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=502, detail=str(exc)) from exc


@app.post("/stop_all")
def stop_all() -> dict[str, Any]:
    with command_lock:
        return controller.stop_all()


@app.post("/shutdown_all")
def shutdown_all() -> dict[str, Any]:
    with command_lock:
        return controller.shutdown_all()


@app.post("/reset_input_buffer")
def reset_input_buffer() -> dict[str, Any]:
    with command_lock:
        if hasattr(controller.service, "_serial") and controller.service._serial:
            controller.service._serial.reset_input_buffer()
    return {"status": "ok"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    logger.info("Connecting to CAN bus...")
    logger.info(controller.connect())

    def handle_termination(signum, _frame) -> None:
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, handle_termination)

    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    finally:
        logger.info("Disconnecting CAN bus...")
        controller.disconnect()


if __name__ == "__main__":
    main()
