#!/usr/bin/env python3
"""
d435_camera_daemon.py — Persistent RealSense D435 frame server.

Opens the RealSense pipeline once and keeps it streaming for as long as this
process runs, continuously caching the latest aligned RGB-D frame. Other
processes (d435_control.py, calibration_ui.py) fetch that cached frame over a
local-only HTTP endpoint instead of each opening/closing their own pipeline.

Only this process may hold the RealSense pipeline open at a time — run it
before anything that needs a camera frame, and leave it running for the
session:

    python d435_camera_daemon.py

Endpoints (127.0.0.1 by default):
    GET /health  -> liveness + streaming status
    GET /frame   -> latest cached aligned color+depth frame (base64 PNG) +
                    depth_scale + color intrinsics
"""
from __future__ import annotations

import argparse
import base64
import logging
import signal
import threading
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import pyrealsense2 as rs
import uvicorn
from fastapi import FastAPI, HTTPException

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8756
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_FPS = 30
WARMUP_FRAME_COUNT = 15
FRAME_WAIT_TIMEOUT_MS = 1000
STALE_FRAME_WARNING_SECONDS = 2.0

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(message)s")
logger = logging.getLogger("d435_camera_daemon")


@dataclass
class CachedFrame:
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    depth_scale: float
    intrinsics: dict[str, float]
    captured_at: float


class CameraStreamer:
    """Owns the RealSense pipeline and keeps one cached frame up to date."""

    def __init__(self) -> None:
        self._pipeline = rs.pipeline()
        self._align = rs.align(rs.stream.color)
        self._lock = threading.Lock()
        self._latest: CachedFrame | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._depth_scale: float = 0.0
        self._intrinsics: dict[str, float] = {}

    def start(self) -> None:
        config = rs.config()
        config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FRAME_FPS)
        config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FRAME_FPS)

        logger.info("Starting RealSense pipeline...")
        profile = self._pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self._depth_scale = float(depth_sensor.get_depth_scale())
        logger.info("depth_scale = %s", self._depth_scale)

        color_profile = profile.get_stream(rs.stream.color)
        intr = color_profile.as_video_stream_profile().get_intrinsics()
        self._intrinsics = {
            "fx": float(intr.fx),
            "fy": float(intr.fy),
            "cx": float(intr.ppx),
            "cy": float(intr.ppy),
        }
        logger.info("intrinsics: %s", self._intrinsics)

        logger.info("Warming up camera (%d frames)...", WARMUP_FRAME_COUNT)
        for _ in range(WARMUP_FRAME_COUNT):
            self._pipeline.wait_for_frames(timeout_ms=FRAME_WAIT_TIMEOUT_MS)

        # Populate the cache with one frame synchronously so /frame never
        # races an empty cache right after startup.
        self._capture_once()

        self._thread = threading.Thread(target=self._run, name="camera-streamer", daemon=True)
        self._thread.start()
        logger.info("Camera streaming started.")

    def _capture_once(self) -> None:
        frames = self._pipeline.wait_for_frames(timeout_ms=FRAME_WAIT_TIMEOUT_MS)
        aligned = self._align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return
        color_bgr = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())
        with self._lock:
            self._latest = CachedFrame(
                color_bgr=color_bgr,
                depth_raw=depth_raw,
                depth_scale=self._depth_scale,
                intrinsics=self._intrinsics,
                captured_at=time.time(),
            )

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._capture_once()
            except Exception:  # noqa: BLE001
                logger.exception("Frame capture failed; retrying")
                time.sleep(0.2)

    def latest(self) -> CachedFrame | None:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        logger.info("Stopping RealSense pipeline...")
        try:
            self._pipeline.stop()
        except Exception:  # noqa: BLE001
            logger.exception("Error while stopping pipeline")


streamer = CameraStreamer()
app = FastAPI()


def _encode_png_base64(image: np.ndarray) -> str:
    ok, buffer = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("Failed to encode frame as PNG")
    return base64.b64encode(buffer.tobytes()).decode("ascii")


@app.get("/health")
def health() -> dict[str, Any]:
    cached = streamer.latest()
    if cached is None:
        return {"status": "starting", "streaming": False}
    age_seconds = time.time() - cached.captured_at
    return {
        "status": "ok",
        "streaming": True,
        "last_frame_age_seconds": age_seconds,
        "stale": age_seconds > STALE_FRAME_WARNING_SECONDS,
    }


@app.get("/frame")
def frame() -> dict[str, Any]:
    cached = streamer.latest()
    if cached is None:
        raise HTTPException(status_code=503, detail="Camera not streaming yet")
    return {
        "color_png_base64": _encode_png_base64(cached.color_bgr),
        "depth_png_base64": _encode_png_base64(cached.depth_raw),
        "depth_scale": cached.depth_scale,
        "intrinsics": cached.intrinsics,
        "captured_at": cached.captured_at,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    streamer.start()

    def handle_termination(signum, _frame) -> None:
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, handle_termination)

    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    finally:
        streamer.stop()


if __name__ == "__main__":
    main()
