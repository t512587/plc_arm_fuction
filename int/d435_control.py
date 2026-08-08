#!/usr/bin/env python3
"""
d435_control.py — Vision-guided suction arm control (dual direction)

Uses canbus/ library (ArmController + MotorService + waveshare_usbcana)
for motor control instead of raw python-can.

Flow:
  LView execute:
    HOME → LView → capture / API detect / calc ID142 & ID143
         → HOME → LGrap → predicted left target
         → [manual Enter: assume PLC pick done]
         → ID143 OuterMid(-) = HOME_ID143 - 180
         → ID142 right_mirror
         → ID143 right outer branch = (2*HOME_ID143 - left_ID143) - 360
         → [manual Enter: assume PLC place done]
         → ID143 OuterMid(-) → ID142 HOME → ID143 HOME → HOME

  RView execute:
    HOME → RView → capture / API detect / calc ID142 & ID143
         → HOME → RGrap → predicted right target
         → [manual Enter: assume PLC pick done]
         → ID143 OuterMid(+) = HOME_ID143 + 180
         → ID142 left_mirror
         → ID143 left outer branch = (2*HOME_ID143 - right_ID143) + 360
         → [manual Enter: assume PLC place done]
         → ID143 OuterMid(+) → ID142 HOME → ID143 HOME → HOME

Notes:
  - PLC is NOT controlled here; Enter is only a manual handoff simulation.
  - After the predicted left target, this version does NOT return to LGrap before going right.
  - Return path does NOT go back through the left target; it unwinds ID143 through OuterMid and returns HOME.
  - ID142 is mirrored normally.
  - ID143 uses the outer branch:
      LView left→right uses HOME-180 and normal_mirror-360.
      RView right→left uses HOME+180 and normal_mirror+360.
"""
from __future__ import annotations

import argparse
import base64
import json
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import requests


# ---------------------------------------------------------------------------
# Add canbus/ to import path so its bare-import modules resolve
# ---------------------------------------------------------------------------
BASE_DIR = Path(__file__).resolve().parent
CANBUS_DIR = BASE_DIR / "canbus"
if str(CANBUS_DIR) not in sys.path:
    sys.path.insert(0, str(CANBUS_DIR))

from arm_controller import ArmController  # noqa: E402
from arm_config import (  # noqa: E402
    HOME_SPEED_DPS,
    MAX_MOVE_DEGREES,
    MOTORS,
    MOTOR_ANGLE_LIMITS,
)


# ===========================================================================
# Settings
# ===========================================================================

# Local GPU server. If you want to use the demo server instead, replace these
# two constants with the demo URL and Basic Auth tuple.
SERVER_URL = "https://demo.bizlion.com.tw/tmts/suction-hotspot/detect"
SERVER_AUTH = ("tmts", "N1++jI8eBOLTogEL0gLz5ehBhTqv50cjKonThISlrQo=")
#SERVER_URL = "http://192.168.50.233:8000/detect"
#SERVER_AUTH = None

# Default depth filter sent to server, per view.
# Unit: meters.
# None = do not override server default depth_range.
# You can also override at runtime with --depth-min / --depth-max.
DEPTH_RANGE_BY_VIEW = {
    "LView": (0.60, 0.82),
    "RView": (0.60, 0.82),
}

ROI_BY_VIEW = {
    # LView ROI measured from the latest camera setup.
    "LView": (202, 117, 460, 312),

    # RView ROI placeholder. Measure/adjust this before RView calibration.
    "RView": (210, 186, 480, 388),
}

# RealSense camera intrinsics are read at runtime from capture_realsense_frame().
# Pixel/depth conversion:
#   x_cm = (u - cx) * z_m / fx * 100
#   y_cm = (v - cy) * z_m / fy * 100

# ---------------------------------------------------------------------------
# View-specific calibration mapping  (camera coords → motor angles)
#
# Labels use "ID 142" format (with space) to match canbus/ convention.
# Each view MUST be calibrated separately. Do not mix LView and RView samples.
# Model:
#   angle = a*x + b*y + c*x^2 + d*x*y + e*y^2 + f
#
# RView is prepared here as a slot. After fitting RView data, paste its coeffs
# into CALIB_BY_VIEW["RView"] and its limits into PREDICTION_LIMITS_BY_VIEW["RView"].
# No function code needs to change.
# ---------------------------------------------------------------------------
CALIB_BY_VIEW = {
    "LView": {
        # Second-car LView calibration, fitted from 24 samples.
        # ID 142 RMSE ~= 1.684 deg, max error ~= 3.722 deg
        "ID 142": (-3.41643143, -0.85823981, 0.05802321, -0.09017803, 0.10025649, 58.07462456),
        # ID 143 RMSE ~= 1.743 deg, max error ~= 4.249 deg
        "ID 143": (1.34783807, 3.65204515, -0.13589233, -0.01394930, -0.13336646, -88.48138011),
    },

    # Second-car RView calibration, fitted from 21 samples.
    # ID 142 RMSE ~= 1.765 deg, max error ~= 3.945 deg
    # ID 143 RMSE ~= 2.106 deg, max error ~= 5.839 deg
    "RView": {
        "ID 142": (3.50379437, -0.77081708, -0.03201293, -0.09647447, -0.07305849, -48.91500422),
        "ID 143": (-1.42868671, 3.63586380, 0.10614333, -0.00233213, 0.09877633, 41.54382802),
    },
}

# View-specific safety clamp for predicted pick angles.
# These limits are only for vision-predicted target angles; outer-branch transfer
# still uses general motor limits from canbus/arm_config.py.
PREDICTION_LIMITS_BY_VIEW = {
    "LView": {
        # ID142 mechanical safety limit is HOME(6) ± 90°.
        "ID 142": (9.0, 96.0),
        "ID 143": (-172.0, -62.0),
    },

    "RView": {
        "ID 142": (-100.0, 14.0),
        "ID 143": (15.0, 127.0),
    },
}

REQUIRED_MOTORS = ("ID 142", "ID 143", "ID 144", "ID 145")
HOME_COMPONENT_MOTORS = {
    "arm": ("ID 142", "ID 143"),
    "camera": ("ID 144", "ID 145"),
    # Cross-component groups for safe HOME ordering:
    # "small" = both small arms first (clear collision zone)
    # "big"   = both big arms second (safe to sweep after small arms retract)
    "small": ("ID 143", "ID 144"),
    "big": ("ID 142", "ID 145"),
}
INITIAL_HOME_CONFIRMED_MARKER = "[POSE_STATE] INITIAL_HOME_CONFIRMED"
ALL_HOME_CONFIRMED_MARKER = "[POSE_STATE] ALL_HOME_CONFIRMED"
ARM_HOME_CONFIRMED_MARKER = "[POSE_STATE] ARM_HOME_CONFIRMED"
CAMERA_HOME_CONFIRMED_MARKER = "[POSE_STATE] CAMERA_HOME_CONFIRMED"
SMALL_HOME_CONFIRMED_MARKER = "[POSE_STATE] SMALL_HOME_CONFIRMED"
BIG_HOME_CONFIRMED_MARKER = "[POSE_STATE] BIG_HOME_CONFIRMED"
NAMED_POSE_CONFIRMED_MARKER = "[POSE_STATE] NAMED_POSE_CONFIRMED"
ARM_STOP_CONFIRMED_MARKER = "[POSE_STATE] ARM_STOP_CONFIRMED"
ARM_STOP_UNCONFIRMED_MARKER = "[POSE_STATE] ARM_STOP_UNCONFIRMED"
POSE_RESULT_PREFIX = "[POSE_RESULT] "

# Pi-side object shape filter disabled.
# Detection quality is checked manually using control_result.png.
SUCTION_OBJECT_FILTER = {}


# ===========================================================================
# Vision / detection helpers
# ===========================================================================

def get_view_calib(view: str) -> dict[str, tuple[float, float, float, float, float, float]] | None:
    """Return calibration coefficients for a view, or None if not calibrated yet."""
    if view not in CALIB_BY_VIEW:
        raise RuntimeError(f"Unknown view: {view}. Available: {list(CALIB_BY_VIEW)}")
    return CALIB_BY_VIEW[view]


def get_view_prediction_limits(view: str) -> dict[str, tuple[float, float]] | None:
    """Return prediction angle limits for a view, or None if not calibrated yet."""
    if view not in PREDICTION_LIMITS_BY_VIEW:
        raise RuntimeError(
            f"Unknown view: {view}. Available: {list(PREDICTION_LIMITS_BY_VIEW)}"
        )
    return PREDICTION_LIMITS_BY_VIEW[view]


def has_view_mapping(view: str) -> bool:
    """True if both calibration coefficients and prediction limits exist."""
    return get_view_calib(view) is not None and get_view_prediction_limits(view) is not None


def clamp_angle(view: str, motor_label: str, angle: float) -> float:
    """Clamp predicted angle to the selected view's safe range."""
    limits_by_motor = get_view_prediction_limits(view)
    if limits_by_motor is None:
        raise RuntimeError(
            f"{view} mapping is not calibrated yet. Run calibration first, then paste "
            f"CALIB_BY_VIEW['{view}'] and PREDICTION_LIMITS_BY_VIEW['{view}']."
        )

    low, high = limits_by_motor[motor_label]
    return max(low, min(high, angle))


def pixel_depth_to_camera_xy_cm(
    u: int, v: int, z_m: float,
    fx: float, fy: float, cx: float, cy: float,
) -> tuple[float, float]:
    """Convert pixel (u, v) + depth (m) to camera-frame XY in cm."""
    x_cm = (u - cx) * z_m / fx * 100.0
    y_cm = (v - cy) * z_m / fy * 100.0
    return x_cm, y_cm


def eval_linear_model(coef: tuple[float, float, float], x_cm: float, y_cm: float) -> float:
    a, b, c = coef
    return a * x_cm + b * y_cm + c


def eval_quad_model(
    coef: tuple[float, float, float, float, float, float],
    x_cm: float,
    y_cm: float,
) -> float:
    a, b, c, d, e, f = coef
    return (
        a * x_cm
        + b * y_cm
        + c * x_cm * x_cm
        + d * x_cm * y_cm
        + e * y_cm * y_cm
        + f
    )


def predict_view_angles(view: str, x_cm: float, y_cm: float) -> dict[str, float]:
    """Predict motor angles from camera coordinates for the selected view."""
    calib = get_view_calib(view)
    if calib is None:
        raise RuntimeError(
            f"{view} mapping is not calibrated yet. For calibration capture, use the "
            f"printed camera_xyz_cm x/y and manually record ID142/ID143."
        )

    id142 = eval_quad_model(calib["ID 142"], x_cm, y_cm)
    id143 = eval_quad_model(calib["ID 143"], x_cm, y_cm)

    print(f"[MAP] {view} direct angle: ID142={id142:.2f}, ID143={id143:.2f}")

    id142 = clamp_angle(view, "ID 142", id142)
    id143 = clamp_angle(view, "ID 143", id143)

    return {
        "ID 142": round(id142, 2),
        "ID 143": round(id143, 2),
    }


def decode_mask_png_base64(mask_b64: str) -> np.ndarray:
    mask_bytes = base64.b64decode(mask_b64)
    mask_arr = np.frombuffer(mask_bytes, dtype=np.uint8)
    mask = cv2.imdecode(mask_arr, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise RuntimeError("Failed to decode mask_png_base64")
    return mask


def normalize_mask_to_full_image(
    mask: np.ndarray,
    color_shape: tuple[int, int, int],
    roi: tuple[int, int, int, int] | None,
) -> np.ndarray:
    full_h, full_w = color_shape[:2]

    if mask.shape[:2] == (full_h, full_w):
        return mask

    if roi is not None:
        x1, y1, x2, y2 = roi
        roi_w = x2 - x1
        roi_h = y2 - y1

        if mask.shape[:2] == (roi_h, roi_w):
            full_mask = np.zeros((full_h, full_w), dtype=np.uint8)
            full_mask[y1:y2, x1:x2] = mask
            print("[INFO] Mask was ROI-sized; pasted it back to full image.")
            return full_mask

    raise RuntimeError(
        f"Mask size {mask.shape[:2]} does not match full image {(full_h, full_w)}."
    )


def save_depth_visualization(depth_raw: np.ndarray, filename: str) -> None:
    depth_vis = cv2.convertScaleAbs(depth_raw, alpha=0.03)
    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    cv2.imwrite(filename, depth_vis)
    print(f"[INFO] Saved {filename}")


def capture_realsense_frame() -> tuple[np.ndarray, np.ndarray, float, dict]:
    """Capture one aligned RGB-D frame and return color intrinsics.

    Returns:
        color_bgr, depth_raw, depth_scale, intrinsics
        intrinsics = {"fx": float, "fy": float, "cx": float, "cy": float}
    """
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("[INFO] Starting RealSense pipeline...")
    profile = pipeline.start(config)

    try:
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        print(f"[INFO] depth_scale = {depth_scale}")

        color_profile = profile.get_stream(rs.stream.color)
        intr = color_profile.as_video_stream_profile().get_intrinsics()
        intrinsics = {
            "fx": float(intr.fx),
            "fy": float(intr.fy),
            "cx": float(intr.ppx),
            "cy": float(intr.ppy),
        }
        print(
            f"[INFO] intrinsics: fx={intr.fx:.2f} fy={intr.fy:.2f} "
            f"cx={intr.ppx:.2f} cy={intr.ppy:.2f}"
        )

        align = rs.align(rs.stream.color)

        print("[INFO] Warming up camera...")
        for _ in range(15):
            pipeline.wait_for_frames(timeout_ms=1000)

        print("[INFO] Capturing one aligned RGB-D frame...")
        frames = pipeline.wait_for_frames(timeout_ms=1000)
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            raise RuntimeError("Failed to get color/depth frame")

        color_bgr = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())

        print(f"[INFO] color shape = {color_bgr.shape}, dtype = {color_bgr.dtype}")
        print(f"[INFO] depth shape = {depth_raw.shape}, dtype = {depth_raw.dtype}")
        print(f"[INFO] depth min/max = {depth_raw.min()} / {depth_raw.max()}")

        return color_bgr, depth_raw, depth_scale, intrinsics

    finally:
        print("[INFO] Stopping RealSense pipeline...")
        pipeline.stop()

def call_detection_server(
    color_bgr: np.ndarray,
    depth_raw: np.ndarray,
    depth_scale: float,
    roi: tuple[int, int, int, int],
    depth_range: tuple[float, float] | None,
) -> dict:
    ok_color, color_png = cv2.imencode(".png", color_bgr)
    ok_depth, depth_png = cv2.imencode(".png", depth_raw)

    if not ok_color or not ok_depth:
        raise RuntimeError("Failed to encode color/depth PNG")

    roi_x1, roi_y1, roi_x2, roi_y2 = roi

    request_data = {
        "depth_scale": str(depth_scale),
        "roi_x1": str(roi_x1),
        "roi_y1": str(roi_y1),
        "roi_x2": str(roi_x2),
        "roi_y2": str(roi_y2),
    }

    if depth_range is not None:
        depth_min_m, depth_max_m = depth_range
        request_data["depth_min_m"] = str(depth_min_m)
        request_data["depth_max_m"] = str(depth_max_m)

    print(f"[INFO] Sending full image + ROI to {SERVER_URL}")
    print(f"[INFO] request_data = {request_data}")

    response = requests.post(
        SERVER_URL,
        files={
            "color": ("color.png", color_png.tobytes(), "image/png"),
            "depth": ("depth.png", depth_png.tobytes(), "image/png"),
        },
        data=request_data,
        auth=SERVER_AUTH,
        timeout=60,
    )

    response.raise_for_status()
    return response.json()


# ===========================================================================
# Visualization / JSON output
# ===========================================================================

def draw_control_result(
    color_bgr: np.ndarray,
    roi: tuple[int, int, int, int],
    u: int,
    v: int,
    z_m: float,
    mask: np.ndarray | None,
    target_angles: dict[str, float],
) -> dict:
    vis = color_bgr.copy()

    rx1, ry1, rx2, ry2 = roi
    cv2.rectangle(vis, (rx1, ry1), (rx2, ry2), (255, 0, 0), 2)
    cv2.putText(
        vis, "ROI", (rx1, max(20, ry1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2,
    )

    bbox = None
    area_px = None
    width_px = None
    height_px = None

    if mask is not None:
        mask = normalize_mask_to_full_image(mask, color_bgr.shape, roi)

        overlay = color_bgr.copy()
        overlay[mask > 0] = (0, 0, 255)
        vis = cv2.addWeighted(vis, 0.7, overlay, 0.3, 0)

        ys, xs = np.where(mask > 0)

        if len(xs) > 0:
            x1, x2 = int(xs.min()), int(xs.max())
            y1, y2 = int(ys.min()), int(ys.max())

            bbox = [x1, y1, x2, y2]
            area_px = int(np.count_nonzero(mask))
            width_px = x2 - x1 + 1
            height_px = y2 - y1 + 1

            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.circle(vis, (u, v), 8, (0, 255, 255), -1)
    cv2.circle(vis, (u, v), 20, (0, 255, 255), 2)

    # Display labels — use compact "ID142" format for readability
    id142_val = target_angles.get("ID 142", 0.0)
    id143_val = target_angles.get("ID 143", 0.0)

    label1 = f"({u},{v}) {z_m:.3f}m"
    label2 = f"ID142={id142_val:.2f}, ID143={id143_val:.2f}"

    cv2.putText(
        vis, label1, (u + 10, max(20, v - 15)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
    )
    cv2.putText(
        vis, label2, (20, 455),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
    )

    cv2.imwrite("control_result.png", vis)
    print("[INFO] Saved control_result.png")

    return {
        "bbox_px": bbox,
        "area_px": area_px,
        "width_px": width_px,
        "height_px": height_px,
    }


def validate_suction_object_info(object_info: dict) -> list[str]:
    """Object-shape safety filter disabled.

    Return an empty warning list so the program will not abort based on
    area/width/height/aspect ratio. ROI safety and motor angle limits still remain.
    """
    return []


# ===========================================================================
# Motor helpers  (using ArmController from canbus/)
# ===========================================================================

def read_current_angles(controller: ArmController) -> dict[str, float]:
    """Read all motor angles. Returns {"ID 142": float, ...}."""
    result = controller.read_positions()
    updates = result.get("_updates", {})
    angles = {label: float(val) for label, val in updates.items()}
    print(f"[READ] Current angles: {angles}")
    return angles


def verify_required_motors_online(angles: dict[str, float]) -> None:
    """Verify all required motors responded before moving."""
    missing = [label for label in REQUIRED_MOTORS if label not in angles]

    if missing:
        raise RuntimeError(
            "Required motors not online: "
            + ", ".join(missing)
            + ". Check motor power / CAN wiring / motor ID."
        )

    print(f"[INFO] Required motors online: {list(REQUIRED_MOTORS)}")


def wait_until_named_pose(
    controller: ArmController,
    point_name: str,
    *,
    tolerance_degrees: float,
    stable_reads: int,
    timeout_seconds: float,
    poll_interval_seconds: float,
    motor_labels: tuple[str, ...] = REQUIRED_MOTORS,
) -> dict[str, float]:
    """Confirm selected CAN axes are stably inside a named pose window."""
    if tolerance_degrees < 0:
        raise ValueError("tolerance_degrees must not be negative")
    if stable_reads <= 0:
        raise ValueError("stable_reads must be greater than zero")
    if timeout_seconds <= 0 or poll_interval_seconds <= 0:
        raise ValueError("HOME confirmation timing must be greater than zero")

    configured_pose = controller.point_config.get(point_name)
    if not isinstance(configured_pose, dict):
        raise RuntimeError(f"Named pose is not configured: {point_name}")
    missing_targets = [
        label for label in motor_labels if label not in configured_pose
    ]
    if missing_targets:
        raise RuntimeError(
            f"{point_name} is missing required motors: {', '.join(missing_targets)}"
        )
    targets = {
        label: float(configured_pose[label])
        for label in motor_labels
    }

    consecutive = 0
    deadline = time.monotonic() + timeout_seconds
    last_angles: dict[str, float] = {}
    while True:
        last_angles = read_current_angles(controller)
        missing_readbacks = [
            label for label in motor_labels if label not in last_angles
        ]
        errors = {
            label: abs(last_angles[label] - targets[label])
            for label in motor_labels
            if label in last_angles
        }
        within_tolerance = (
            not missing_readbacks
            and all(error <= tolerance_degrees for error in errors.values())
        )
        consecutive = consecutive + 1 if within_tolerance else 0
        print(
            f"[POSE_CHECK] {point_name} stable={consecutive}/{stable_reads} "
            f"missing={missing_readbacks} errors={errors}",
            flush=True,
        )
        if consecutive >= stable_reads:
            return {
                label: float(last_angles[label])
                for label in motor_labels
            }
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"{point_name} was not confirmed within {timeout_seconds:g}s; "
                f"last_angles={last_angles}"
            )
        time.sleep(poll_interval_seconds)


def confirm_home_pose(
    controller: ArmController,
    *,
    tolerance_degrees: float,
    stable_reads: int,
    timeout_seconds: float,
    poll_interval_seconds: float,
    final_confirmation: bool,
) -> dict[str, float]:
    """Emit a marker only after stable four-axis HOME readback."""
    angles = wait_until_named_pose(
        controller,
        "HOME",
        tolerance_degrees=tolerance_degrees,
        stable_reads=stable_reads,
        timeout_seconds=timeout_seconds,
        poll_interval_seconds=poll_interval_seconds,
    )
    marker = (
        ALL_HOME_CONFIRMED_MARKER
        if final_confirmation
        else INITIAL_HOME_CONFIRMED_MARKER
    )
    print(marker, flush=True)
    print(
        POSE_RESULT_PREFIX
        + json.dumps(
            {"pose": "HOME", "angles": angles},
            ensure_ascii=False,
            sort_keys=True,
        ),
        flush=True,
    )
    return angles


def move_component_home_pose(
    controller: ArmController,
    component: str,
    *,
    settle_sec: float,
    tolerance_degrees: float,
    stable_reads: int,
    timeout_seconds: float,
    poll_interval_seconds: float,
) -> dict[str, float]:
    """Move a motor group to HOME and confirm readback.

    Supported components:
      "arm"    — ID142 + ID143 (small arm first, then big arm)
      "camera" — ID144 + ID145 (camera small first, then camera big)
      "small"  — ID143 + ID144 (both small arms, independent axes, simultaneous)
      "big"    — ID142 + ID145 (both big arms, independent axes, simultaneous)

    For safe full-HOME: call "small" first, confirm, then "big".
    This prevents big arms from sweeping into the pillar before small arms clear.
    """

    try:
        motor_labels = HOME_COMPONENT_MOTORS[component]
    except KeyError as exc:
        raise ValueError(f"Unknown HOME component: {component!r}") from exc
    home = controller.point_config.get("HOME")
    if not isinstance(home, dict):
        raise RuntimeError("Named pose is not configured: HOME")
    missing_targets = [label for label in motor_labels if label not in home]
    if missing_targets:
        raise RuntimeError("HOME is missing: " + ", ".join(missing_targets))

    current = read_current_angles(controller)
    missing_readbacks = [label for label in motor_labels if label not in current]
    if missing_readbacks:
        raise RuntimeError(
            "Required component motors not online: " + ", ".join(missing_readbacks)
        )
    targets = {label: float(home[label]) for label in motor_labels}
    for label, target in targets.items():
        difference = abs(target - current[label])
        if difference > MAX_MOVE_DEGREES:
            raise RuntimeError(
                f"Safety blocked: {label} would move {difference:.1f}° "
                f"(current={current[label]:.2f} -> target={target:.2f}); "
                f"maximum is {MAX_MOVE_DEGREES}°"
            )
        check_general_motor_limits(label, target)

    # --- Movement ordering ---
    # "arm" / "camera": same kinematic chain, small link must retract first
    #   to create clearance before big link sweeps.
    # "small" / "big": independent axes on different chains, can move
    #   simultaneously — no collision risk within the group.
    SEQUENTIAL_ORDER: dict[str, tuple[str, ...]] = {
        "arm": ("ID 143", "ID 142"),
        "camera": ("ID 144", "ID 145"),
    }

    sequential = SEQUENTIAL_ORDER.get(component)
    if sequential is not None:
        # Same-chain pair: small link first, wait, then big link
        print(
            f"[MOVE] Return {component} motors to HOME "
            f"(small link first): {list(sequential)}"
        )
        for index, label in enumerate(sequential):
            target = targets[label]
            motor_id = MOTORS[label]
            response = controller.service.absolute_position_control(
                motor_id, target, HOME_SPEED_DPS,
            )
            print(
                f"[CAN] {label} → HOME {target:.2f}° @ {HOME_SPEED_DPS} dps | "
                f"raw={response.get('raw', '')}",
                flush=True,
            )
            if index == 0 and settle_sec > 0:
                time.sleep(settle_sec)
    else:
        # Cross-chain group ("small" or "big"): independent axes, send all
        print(
            f"[MOVE] Return {component} motors to HOME "
            f"(independent axes): {list(motor_labels)}"
        )
        for label in motor_labels:
            target = targets[label]
            motor_id = MOTORS[label]
            response = controller.service.absolute_position_control(
                motor_id, target, HOME_SPEED_DPS,
            )
            print(
                f"[CAN] {label} → HOME {target:.2f}° @ {HOME_SPEED_DPS} dps | "
                f"raw={response.get('raw', '')}",
                flush=True,
            )
    if settle_sec > 0:
        time.sleep(settle_sec)

    angles = wait_until_named_pose(
        controller,
        "HOME",
        tolerance_degrees=tolerance_degrees,
        stable_reads=stable_reads,
        timeout_seconds=timeout_seconds,
        poll_interval_seconds=poll_interval_seconds,
        motor_labels=motor_labels,
    )
    _COMPONENT_MARKERS = {
        "arm": ARM_HOME_CONFIRMED_MARKER,
        "camera": CAMERA_HOME_CONFIRMED_MARKER,
        "small": SMALL_HOME_CONFIRMED_MARKER,
        "big": BIG_HOME_CONFIRMED_MARKER,
    }
    marker = _COMPONENT_MARKERS[component]
    print(marker, flush=True)
    print(
        POSE_RESULT_PREFIX
        + json.dumps(
            {"pose": "HOME", "component": component, "angles": angles},
            ensure_ascii=False,
            sort_keys=True,
        ),
        flush=True,
    )
    return angles


def move_named_pose(
    controller: ArmController,
    point_name: str,
    *,
    settle_sec: float,
    tolerance_degrees: float,
    stable_reads: int,
    timeout_seconds: float,
    poll_interval_seconds: float,
) -> dict[str, float]:
    """移動四顆馬達到指定儲存姿態，並確認角度穩定到位。"""

    configured_pose = controller.point_config.get(point_name)
    if not isinstance(configured_pose, dict):
        raise RuntimeError(f"Named pose is not configured: {point_name}")

    missing = [
        label
        for label in REQUIRED_MOTORS
        if label not in configured_pose
    ]
    if missing:
        raise RuntimeError(
            f"{point_name} 缺少馬達角度：{', '.join(missing)}"
        )

    safe_go_to_point(
        controller,
        point_name,
        settle_sec,
    )

    angles = wait_until_named_pose(
        controller,
        point_name,
        tolerance_degrees=tolerance_degrees,
        stable_reads=stable_reads,
        timeout_seconds=timeout_seconds,
        poll_interval_seconds=poll_interval_seconds,
    )

    print(NAMED_POSE_CONFIRMED_MARKER, flush=True)
    print(
        POSE_RESULT_PREFIX
        + json.dumps(
            {
                "pose": point_name,
                "angles": angles,
            },
            ensure_ascii=False,
            sort_keys=True,
        ),
        flush=True,
    )

    return angles


def stop_all_motors_confirmed(controller: ArmController) -> bool:
    """Stop all four motors and report whether every stop command succeeded."""
    result = controller.stop_all()
    for key, value in result.items():
        print(f"[STOP] {key}: {value}", flush=True)
    confirmed = all(
        isinstance(result.get(label), str)
        and result[label].startswith("OK")
        for label in REQUIRED_MOTORS
    )
    print(
        ARM_STOP_CONFIRMED_MARKER
        if confirmed
        else ARM_STOP_UNCONFIRMED_MARKER,
        flush=True,
    )
    return confirmed


def safe_go_to_point(
    controller: ArmController,
    point_name: str,
    settle_sec: float,
) -> None:
    """Move to a named point with safety checks, then wait for settling."""
    print(f"[MOVE] Go to {point_name}")

    # Read current positions for MAX_MOVE_DEGREES safety check
    current = read_current_angles(controller)
    result = controller.go_to_point(point_name, current_positions=current)

    if "safety" in result:
        raise RuntimeError(f"Safety blocked: {result['safety']}")
    if "error" in result:
        raise RuntimeError(f"Error: {result['error']}")

    # Check individual motor results for errors
    for label in MOTORS:
        val = result.get(label, "")
        if isinstance(val, str) and val.startswith("ERROR"):
            raise RuntimeError(f"{label}: {val}")

    print(f"[MOVE] Waiting {settle_sec:.1f}s for {point_name} to settle...")
    time.sleep(settle_sec)


def step_pause(message: str, auto_step: bool = False, wait_sec: float = 0.0) -> None:
    """Pause between demo stages so movement can be checked visually.

    auto_step=False: wait for Enter, like the previous manual segmented test.
    auto_step=True: only wait wait_sec seconds, for later automated demos.
    """
    if auto_step:
        if wait_sec > 0:
            print(f"[STEP] {message}  auto-continue after {wait_sec:.1f}s")
            time.sleep(wait_sec)
        else:
            print(f"[STEP] {message}  auto-continue")
        return

    input(f"\n[STEP] {message}\nPress Enter to continue... ")


def check_prediction_angle_limits(view: str, motor_label: str, angle: float) -> None:
    """Check view-specific prediction limits only."""
    limits_by_motor = get_view_prediction_limits(view)
    if limits_by_motor is None:
        raise RuntimeError(f"{view} prediction limits are not configured yet.")

    limits = limits_by_motor.get(motor_label)
    if limits:
        lo, hi = limits
        if angle < lo or angle > hi:
            raise RuntimeError(
                f"BLOCKED. {motor_label} target={angle:.2f}° outside {view} "
                f"prediction limit [{lo:.1f}°, {hi:.1f}°]"
            )


def check_angle_limits(motor_label: str, angle: float, view: str = "LView") -> None:
    """Check prediction limits for the selected view and general motor limits.

    This is a legacy helper for single-motor target moves. Normal outer-branch
    transfer uses check_general_motor_limits() instead.
    """
    if has_view_mapping(view):
        check_prediction_angle_limits(view, motor_label, angle)

    general_limits = MOTOR_ANGLE_LIMITS.get(motor_label)
    if general_limits:
        lo, hi = general_limits
        if angle < lo or angle > hi:
            raise RuntimeError(
                f"BLOCKED. {motor_label} target={angle:.2f}° outside motor limit "
                f"[{lo:.1f}°, {hi:.1f}°]"
            )


def move_single_motor_angle(
    controller: ArmController,
    motor_label: str,
    angle: float,
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """Move one motor with safety limit checks."""
    check_angle_limits(motor_label, angle)

    service = controller.service
    motor_id = MOTORS[motor_label]
    resp = service.absolute_position_control(motor_id, angle, target_speed_dps)
    print(
        f"[CAN] {motor_label} → {angle:.2f}° @ {target_speed_dps} dps | "
        f"raw={resp.get('raw', '')}"
    )
    if settle_sec > 0:
        print(f"[MOVE] Waiting {settle_sec:.1f}s for {motor_label} settle...")
        time.sleep(settle_sec)


def retract_small_arm_to_safe(
    controller: ArmController,
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """After PLC pick + lift, retract only ID143 first.

    Keep ID142 near the target first, so the suction head does not sweep
    across the box/cords while still extended. This is not a full LGrap return.
    """
    lgrap = controller.point_config.get("LGrap", {})
    if "ID 143" not in lgrap:
        raise RuntimeError("LGrap does not contain ID 143 safe angle")

    angle = float(lgrap["ID 143"])
    print("[MOVE] Post-pick small-arm retract: ID 143 first")
    move_single_motor_angle(
        controller=controller,
        motor_label="ID 143",
        angle=angle,
        target_speed_dps=target_speed_dps,
        settle_sec=settle_sec,
    )


def move_to_target_angles(
    controller: ArmController,
    view: str,
    target_angles: dict[str, float],
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """
    Move ID 142 and ID 143 to predicted target angles.

    Uses service.absolute_position_control() directly so we can set
    a configurable speed (go_to_point / run_targets use fixed speeds).
    Big arm (ID 142) first, then small arm (ID 143), for safety.
    """
    print("[MOVE] Move suction arm to predicted target angles")
    for label, angle in target_angles.items():
        print(f"[TARGET] {label} = {angle:.2f}°")

    # Safety: check prediction-specific limits for this view
    for label, angle in target_angles.items():
        check_prediction_angle_limits(view, label, angle)

    # Safety: also check general motor limits from arm_config
    for label, angle in target_angles.items():
        general_limits = MOTOR_ANGLE_LIMITS.get(label)
        if general_limits:
            lo, hi = general_limits
            if angle < lo or angle > hi:
                raise RuntimeError(
                    f"BLOCKED. {label} target={angle:.2f}° outside motor limit "
                    f"[{lo:.1f}°, {hi:.1f}°]"
                )

    service = controller.service

    # Move big arm (ID 142) first
    if "ID 142" in target_angles:
        motor_id = MOTORS["ID 142"]
        angle = target_angles["ID 142"]
        resp = service.absolute_position_control(motor_id, angle, target_speed_dps)
        print(
            f"[CAN] ID 142 → {angle:.2f}° @ {target_speed_dps} dps | "
            f"raw={resp.get('raw', '')}"
        )
        time.sleep(0.5)

    # Then small arm (ID 143)
    if "ID 143" in target_angles:
        motor_id = MOTORS["ID 143"]
        angle = target_angles["ID 143"]
        resp = service.absolute_position_control(motor_id, angle, target_speed_dps)
        print(
            f"[CAN] ID 143 → {angle:.2f}° @ {target_speed_dps} dps | "
            f"raw={resp.get('raw', '')}"
        )

    print(f"[MOVE] Waiting {settle_sec:.1f}s at target...")
    time.sleep(settle_sec)


def retract_to_lgrap(
    controller: ArmController,
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """Return from predicted suction target back to LGrap.

    Wiring-safe rule for returning from target:
      ID 143 small arm first → ID 142 big arm second → full LGrap pose.

    This follows the reverse of the route used to enter the target and helps
    avoid wire/tube snagging near the bin.
    """
    print("[MOVE] Return suction arm back to LGrap via ordered motion")

    lgrap = controller.point_config.get("LGrap", {})
    service = controller.service

    # Small arm (ID 143) first to unwind/clear the suction side.
    if "ID 143" in lgrap:
        motor_id = MOTORS["ID 143"]
        angle = float(lgrap["ID 143"])
        resp = service.absolute_position_control(motor_id, angle, target_speed_dps)
        print(f"[CAN] ID 143 → LGrap {angle:.2f}° | raw={resp.get('raw', '')}")
        time.sleep(0.8)

    # Big arm (ID 142) second.
    if "ID 142" in lgrap:
        motor_id = MOTORS["ID 142"]
        angle = float(lgrap["ID 142"])
        resp = service.absolute_position_control(motor_id, angle, target_speed_dps)
        print(f"[CAN] ID 142 → LGrap {angle:.2f}° | raw={resp.get('raw', '')}")
        time.sleep(0.8)

    # Full LGrap pose (ID 144 / ID 145 too). This keeps the same saved point.
    current = read_current_angles(controller)
    result = controller.go_to_point("LGrap", current_positions=current)
    if "safety" in result:
        raise RuntimeError(f"LGrap safety blocked: {result['safety']}")
    if "error" in result:
        raise RuntimeError(f"LGrap error: {result['error']}")

    print(f"[MOVE] Waiting {settle_sec:.1f}s for LGrap settle...")
    time.sleep(settle_sec)


def check_general_motor_limits(motor_label: str, angle: float) -> None:
    """Check general motor limits only.

    The pick target uses prediction limits, but the mirrored right-side demo
    point can be positive on ID 143, so it must not use the LView prediction
    clamp [-130, -35].
    """
    general_limits = MOTOR_ANGLE_LIMITS.get(motor_label)
    if general_limits:
        lo, hi = general_limits
        if angle < lo or angle > hi:
            raise RuntimeError(
                f"BLOCKED. {motor_label} target={angle:.2f}° outside motor limit "
                f"[{lo:.1f}°, {hi:.1f}°]"
            )


def move_single_motor_angle_general(
    controller: ArmController,
    motor_label: str,
    angle: float,
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """Move one motor with general motor limits only."""
    check_general_motor_limits(motor_label, angle)

    service = controller.service
    motor_id = MOTORS[motor_label]
    resp = service.absolute_position_control(motor_id, angle, target_speed_dps)
    print(
        f"[CAN] {motor_label} → {angle:.2f}° @ {target_speed_dps} dps | "
        f"raw={resp.get('raw', '')}"
    )
    if settle_sec > 0:
        print(f"[MOVE] Waiting {settle_sec:.1f}s for {motor_label} settle...")
        time.sleep(settle_sec)


def get_home_angles(controller: ArmController, home_point: str = "HOME") -> dict[str, float]:
    """Read saved HOME angles from point_config.json."""
    home = controller.point_config.get(home_point, {})
    if not home:
        raise RuntimeError(f"Home point {home_point} not found in point_config.json")

    required = ["ID 142", "ID 143"]
    missing = [label for label in required if label not in home]
    if missing:
        raise RuntimeError("HOME is missing: " + ", ".join(missing))

    return {label: float(home[label]) for label in required}


def get_id143_outer_mid_angle(controller: ArmController) -> float:
    """Compute the small-arm outside transfer angle from HOME.

    Rule agreed on-site:
      OuterMid_ID143 = HOME_ID143 - 180
    """
    home = get_home_angles(controller)
    outer_mid = home["ID 143"] - 180.0
    check_general_motor_limits("ID 143", outer_mid)
    print(
        f"[OUTER] Compute ID143 OuterMid from HOME: "
        f"{home['ID 143']:.2f} - 180 = {outer_mid:.2f}°"
    )
    return round(outer_mid, 2)


def move_id143_to_outer_mid(
    controller: ArmController,
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """Move only ID143 to the outside transfer angle."""
    outer_mid = get_id143_outer_mid_angle(controller)
    print(f"[OUTER] Move ID143 to OuterMid = {outer_mid:.2f}°")
    move_single_motor_angle_general(
        controller=controller,
        motor_label="ID 143",
        angle=outer_mid,
        target_speed_dps=target_speed_dps,
        settle_sec=settle_sec,
    )


def compute_right_outer_branch_angles(
    controller: ArmController,
    target_angles: dict[str, float],
    home_point: str = "HOME",
) -> dict[str, float]:
    """Compute the right-side outer-branch pose.

    Agreed rule:
      - ID142 big arm DOES mirror around HOME:
            right_ID142 = 2*HOME_ID142 - left_ID142
      - ID143 small arm also computes the normal HOME mirror first:
            right_ID143_normal = 2*HOME_ID143 - left_ID143
        but it must use the outside branch:
            right_ID143_outer = right_ID143_normal - 360

    This keeps the correct big-arm mirror motion while preventing the small arm
    from jumping from OuterMid back through the inner-line shortcut.
    """
    home = get_home_angles(controller, home_point)

    missing = [label for label in ("ID 142", "ID 143") if label not in target_angles]
    if missing:
        raise RuntimeError("target_angles missing: " + ", ".join(missing))

    home_id142 = float(home["ID 142"])
    home_id143 = float(home["ID 143"])
    left_id142 = float(target_angles["ID 142"])
    left_id143 = float(target_angles["ID 143"])

    right_id142 = 2.0 * home_id142 - left_id142
    right_id143_normal = 2.0 * home_id143 - left_id143
    right_id143_outer = right_id143_normal - 360.0

    right_angles = {
        "ID 142": round(right_id142, 2),
        "ID 143": round(right_id143_outer, 2),
    }

    print("[RIGHT] Compute right-side pose: ID142 mirror + ID143 outer branch")
    print(
        f"[RIGHT] ID 142: HOME={home_id142:.2f}, left={left_id142:.2f}, "
        f"right_mirror={right_id142:.2f}"
    )
    print(
        f"[RIGHT] ID 143: HOME={home_id143:.2f}, left={left_id143:.2f}, "
        f"normal_mirror={right_id143_normal:.2f}, outer_branch={right_id143_outer:.2f}"
    )

    for label, angle in right_angles.items():
        check_general_motor_limits(label, angle)

    return right_angles

def move_to_right_outer_branch_pose_via_outer_mid(
    controller: ArmController,
    right_angles: dict[str, float],
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """Move from left target to right outer-branch pose through OuterMid.

    Transfer route:
      ID143 → OuterMid
      → ID142 → right mirror angle
      → ID143 → right outer-branch angle
    """
    print("[MOVE] Outside transfer: left target → OuterMid → right outer-branch pose")
    print("[MOVE] Order: ID143 OuterMid → ID142 right_mirror → ID143 right_outer")

    move_id143_to_outer_mid(controller, target_speed_dps, 0.8)

    if "ID 142" in right_angles:
        move_single_motor_angle_general(
            controller, "ID 142", right_angles["ID 142"], target_speed_dps, 0.8
        )

    if "ID 143" in right_angles:
        move_single_motor_angle_general(
            controller, "ID 143", right_angles["ID 143"], target_speed_dps, settle_sec
        )


def return_home_from_right_outer_branch_via_outer_mid(
    controller: ArmController,
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """Return from right outer-branch pose back to HOME through OuterMid.

    Return route:
      ID143 → OuterMid
      → ID142 → HOME
      → ID143 → HOME
      → full HOME named pose

    This unwinds the small-arm wire through the outside line without going
    back through the original left suction target.
    """
    print("[MOVE] Return route: right outer-branch pose → OuterMid → HOME")
    print("[MOVE] Order: ID143 OuterMid → ID142 HOME → ID143 HOME → HOME")

    home = get_home_angles(controller)

    move_id143_to_outer_mid(controller, target_speed_dps, 0.8)

    move_single_motor_angle_general(
        controller, "ID 142", home["ID 142"], target_speed_dps, 0.8
    )

    move_single_motor_angle_general(
        controller, "ID 143", home["ID 143"], target_speed_dps, settle_sec
    )




def get_id143_outer_mid_angle_plus(controller: ArmController) -> float:
    """Compute the positive-side outside transfer angle from HOME.

    RView right→left route uses the symmetric outside branch:
      OuterMidPlus_ID143 = HOME_ID143 + 180
    """
    home = get_home_angles(controller)
    outer_mid = home["ID 143"] + 180.0
    check_general_motor_limits("ID 143", outer_mid)
    print(
        f"[OUTER] Compute ID143 OuterMidPlus from HOME: "
        f"{home['ID 143']:.2f} + 180 = {outer_mid:.2f}°"
    )
    return round(outer_mid, 2)


def move_id143_to_outer_mid_plus(
    controller: ArmController,
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """Move only ID143 to the positive outside transfer angle."""
    outer_mid = get_id143_outer_mid_angle_plus(controller)
    print(f"[OUTER] Move ID143 to OuterMidPlus = {outer_mid:.2f}°")
    move_single_motor_angle_general(
        controller=controller,
        motor_label="ID 143",
        angle=outer_mid,
        target_speed_dps=target_speed_dps,
        settle_sec=settle_sec,
    )


def compute_left_outer_branch_angles(
    controller: ArmController,
    target_angles: dict[str, float],
    home_point: str = "HOME",
) -> dict[str, float]:
    """Compute the left-side outer-branch pose from a right-side target.

    Symmetric rule for RView right→left:
      - ID142 mirrors around HOME:
            left_ID142 = 2*HOME_ID142 - right_ID142
      - ID143 computes the normal HOME mirror first:
            left_ID143_normal = 2*HOME_ID143 - right_ID143
        then uses the positive outside branch:
            left_ID143_outer = left_ID143_normal + 360

    This is the mirror of LView left→right, which uses -360. The positive
    branch keeps ID143 moving through HOME+180 instead of jumping through the
    inner-line shortcut.
    """
    home = get_home_angles(controller, home_point)

    missing = [label for label in ("ID 142", "ID 143") if label not in target_angles]
    if missing:
        raise RuntimeError("target_angles missing: " + ", ".join(missing))

    home_id142 = float(home["ID 142"])
    home_id143 = float(home["ID 143"])
    right_id142 = float(target_angles["ID 142"])
    right_id143 = float(target_angles["ID 143"])

    left_id142 = 2.0 * home_id142 - right_id142
    left_id143_normal = 2.0 * home_id143 - right_id143
    left_id143_outer = left_id143_normal + 360.0

    left_angles = {
        "ID 142": round(left_id142, 2),
        "ID 143": round(left_id143_outer, 2),
    }

    print("[LEFT] Compute left-side pose: ID142 mirror + ID143 positive outer branch")
    print(
        f"[LEFT] ID 142: HOME={home_id142:.2f}, right={right_id142:.2f}, "
        f"left_mirror={left_id142:.2f}"
    )
    print(
        f"[LEFT] ID 143: HOME={home_id143:.2f}, right={right_id143:.2f}, "
        f"normal_mirror={left_id143_normal:.2f}, outer_branch={left_id143_outer:.2f}"
    )

    for label, angle in left_angles.items():
        check_general_motor_limits(label, angle)

    return left_angles


def move_to_left_outer_branch_pose_via_outer_mid(
    controller: ArmController,
    left_angles: dict[str, float],
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """Move from right target to left outer-branch pose through OuterMidPlus.

    Transfer route:
      ID143 → OuterMidPlus
      → ID142 → left mirror angle
      → ID143 → left outer-branch angle
    """
    print("[MOVE] Outside transfer: right target → OuterMidPlus → left outer-branch pose")
    print("[MOVE] Order: ID143 OuterMidPlus → ID142 left_mirror → ID143 left_outer")

    move_id143_to_outer_mid_plus(controller, target_speed_dps, 0.8)

    if "ID 142" in left_angles:
        move_single_motor_angle_general(
            controller, "ID 142", left_angles["ID 142"], target_speed_dps, 0.8
        )

    if "ID 143" in left_angles:
        move_single_motor_angle_general(
            controller, "ID 143", left_angles["ID 143"], target_speed_dps, settle_sec
        )


def return_home_from_left_outer_branch_via_outer_mid(
    controller: ArmController,
    target_speed_dps: int,
    settle_sec: float,
) -> None:
    """Return from left outer-branch pose back to HOME through OuterMidPlus.

    Return route:
      ID143 → OuterMidPlus
      → ID142 → HOME
      → ID143 → HOME
      → full HOME named pose
    """
    print("[MOVE] Return route: left outer-branch pose → OuterMidPlus → HOME")
    print("[MOVE] Order: ID143 OuterMidPlus → ID142 HOME → ID143 HOME → HOME")

    home = get_home_angles(controller)

    move_id143_to_outer_mid_plus(controller, target_speed_dps, 0.8)

    move_single_motor_angle_general(
        controller, "ID 142", home["ID 142"], target_speed_dps, 0.8
    )

    move_single_motor_angle_general(
        controller, "ID 143", home["ID 143"], target_speed_dps, settle_sec
    )


def get_pregrasp_point_for_view(view: str) -> str:
    """Return the named pre-grasp pose for the selected camera view."""
    if view == "LView":
        return "LGrap"
    if view == "RView":
        return "RGrap"
    raise RuntimeError(f"Unknown view: {view}")


def build_route_plan(view: str) -> list[str]:
    """Build a compact route plan for JSON output."""
    if view == "LView":
        return [
            "HOME", "LView", "capture_and_detect",
            "HOME", "LGrap", "predicted_left_suction_angles",
            "manual_fake_plc_pick_done", "outer_mid_id143_minus",
            "id142_right_mirror", "id143_right_outer_branch",
            "manual_fake_plc_place_done", "outer_mid_id143_minus",
            "id142_home", "id143_home", "HOME",
        ]
    if view == "RView":
        return [
            "HOME", "RView", "capture_and_detect",
            "HOME", "RGrap", "predicted_right_suction_angles",
            "manual_fake_plc_pick_done", "outer_mid_id143_plus",
            "id142_left_mirror", "id143_left_outer_branch",
            "manual_fake_plc_place_done", "outer_mid_id143_plus",
            "id142_home", "id143_home", "HOME",
        ]
    raise RuntimeError(f"Unknown view: {view}")


# ===========================================================================
# Full pick flow
# ===========================================================================

def run_post_detection_pick_flow(
    controller: ArmController,
    view: str,
    target_angles: dict[str, float],
    settle_sec: float,
    target_speed_dps: int,
    target_wait_sec: float,
    pick_wait_sec: float,
    place_point: str,
    auto_step: bool,
    home_tolerance_degrees: float,
    home_stable_reads: int,
    home_timeout_seconds: float,
    home_poll_interval_seconds: float,
) -> None:
    """Dual-direction outer-branch demo route.

    View determines direction:
      LView: left target → right place
      RView: right target → left place

    PLC is not controlled here. Enter/manual waits are only handoff points.
    """
    if view not in ("LView", "RView"):
        raise RuntimeError(f"Unknown view for execute flow: {view}")

    pregrasp_point = get_pregrasp_point_for_view(view)

    if view == "LView":
        pick_side = "left"
        place_side = "right"
    else:
        pick_side = "right"
        place_side = "left"

    # Check both the direct pick target and the later mirrored transfer pose
    # before moving toward or picking the object.
    for label, angle in target_angles.items():
        check_prediction_angle_limits(view, label, angle)
        check_general_motor_limits(label, angle)
    transfer_angles = (
        compute_right_outer_branch_angles(controller, target_angles)
        if view == "LView"
        else compute_left_outer_branch_angles(controller, target_angles)
    )

    print("[FLOW] Vision finished. Return HOME before grasp route.")
    step_pause(f"Move from {view} back to HOME before pick route", auto_step, 0.5)
    safe_go_to_point(controller, "HOME", settle_sec)
    home_angles = wait_until_named_pose(
        controller,
        "HOME",
        tolerance_degrees=home_tolerance_degrees,
        stable_reads=home_stable_reads,
        timeout_seconds=home_timeout_seconds,
        poll_interval_seconds=home_poll_interval_seconds,
    )
    print(
        POSE_RESULT_PREFIX
        + json.dumps(
            {
                "pose": f"HOME_BEFORE_{pregrasp_point.upper()}",
                "angles": home_angles,
            },
            ensure_ascii=False,
            sort_keys=True,
        ),
        flush=True,
    )

    print(f"[FLOW] Enter pre-grasp safe pose ({pregrasp_point})")
    step_pause(f"Move HOME → {pregrasp_point}", auto_step, 0.5)
    safe_go_to_point(controller, pregrasp_point, settle_sec)

    step_pause(
        f"Move {pregrasp_point} → predicted {pick_side} suction target. "
        "Order: ID142 first, then ID143",
        auto_step,
        0.5,
    )
    move_to_target_angles(
        controller=controller,
        view=view,
        target_angles=target_angles,
        target_speed_dps=target_speed_dps,
        settle_sec=target_wait_sec,
    )

    print(f"\n[HANDOFF] Arm is at {pick_side} target suction position.")
    print("[HANDOFF] PLC is not controlled by this script.")
    print("[HANDOFF] Press Enter after you assume the object is picked and clear.")
    if pick_wait_sec > 0:
        print(f"[HANDOFF] Suggested PLC wait reference = {pick_wait_sec:.1f}s")
    step_pause(
        f"FAKE PLC PICK DONE: object attached and safe to transfer {place_side}",
        auto_step,
        pick_wait_sec,
    )

    if view == "LView":
        step_pause(
            "Move left target → ID143 OuterMid(-) → ID142 right_mirror → ID143 right_outer",
            auto_step,
            0.5,
        )
        move_to_right_outer_branch_pose_via_outer_mid(
            controller=controller,
            right_angles=transfer_angles,
            target_speed_dps=target_speed_dps,
            settle_sec=settle_sec,
        )

        print("\n[HANDOFF] Arm is at right outer-branch place position.")
        print("[HANDOFF] PLC is not controlled by this script.")
        print("[HANDOFF] Press Enter after you assume the object is released and clear.")
        step_pause(
            "FAKE PLC PLACE DONE: object released and safe to return",
            auto_step,
            pick_wait_sec,
        )

        step_pause(
            "Return right_outer → ID143 OuterMid(-) → ID142 HOME → ID143 HOME → HOME",
            auto_step,
            0.5,
        )
        return_home_from_right_outer_branch_via_outer_mid(
            controller=controller,
            target_speed_dps=target_speed_dps,
            settle_sec=settle_sec,
        )

    else:
        step_pause(
            "Move right target → ID143 OuterMid(+) → ID142 left_mirror → ID143 left_outer",
            auto_step,
            0.5,
        )
        move_to_left_outer_branch_pose_via_outer_mid(
            controller=controller,
            left_angles=transfer_angles,
            target_speed_dps=target_speed_dps,
            settle_sec=settle_sec,
        )

        print("\n[HANDOFF] Arm is at left outer-branch place position.")
        print("[HANDOFF] PLC is not controlled by this script.")
        print("[HANDOFF] Press Enter after you assume the object is released and clear.")
        step_pause(
            "FAKE PLC PLACE DONE: object released and safe to return",
            auto_step,
            pick_wait_sec,
        )

        step_pause(
            "Return left_outer → ID143 OuterMid(+) → ID142 HOME → ID143 HOME → HOME",
            auto_step,
            0.5,
        )
        return_home_from_left_outer_branch_via_outer_mid(
            controller=controller,
            target_speed_dps=target_speed_dps,
            settle_sec=settle_sec,
        )

    safe_go_to_point(controller, "HOME", settle_sec)

    print(f"[INFO] {view} {pick_side}→{place_side} pick-and-place demo flow finished. Arm returned HOME.")


def confirm_motor_action(view: str, execute: bool, target_speed_dps: int, yes: bool) -> bool:
    """Interactive safety confirmation before moving motors."""
    if yes:
        return True

    if execute:
        if view == "LView":
            route = (
                "HOME → LView → detect → HOME → LGrap → left target → fake pick Enter "
                "→ ID143 OuterMid(-) → ID142 right_mirror → ID143 right_outer "
                "→ fake place Enter → ID143 OuterMid(-) → ID142 HOME → ID143 HOME → HOME"
            )
        elif view == "RView":
            route = (
                "HOME → RView → detect → HOME → RGrap → right target → fake pick Enter "
                "→ ID143 OuterMid(+) → ID142 left_mirror → ID143 left_outer "
                "→ fake place Enter → ID143 OuterMid(+) → ID142 HOME → ID143 HOME → HOME"
            )
        else:
            raise RuntimeError(f"Unknown view: {view}")
    else:
        route = f"HOME → {view} → detect → HOME"

    print(f"\n[SAFETY] Route: {route}")
    print(f"[SAFETY] Target speed = {target_speed_dps} dps")
    input("[SAFETY] Press Enter to start (Ctrl+C to abort)... ")
    return True


# ===========================================================================
# Main
# ===========================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Vision-guided suction arm control v3 dual-direction (uses canbus/ library).\n"
            "Default: connect → HOME → selected view → capture/detect → HOME (no pick).\n"
            "--execute with LView: left pick → right place.\n"
            "--execute with RView: right pick → left place.\n"
            "--dry-run: skip all motor movement, just capture + detect."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--view", default="LView", choices=["LView", "RView"])
    parser.add_argument(
        "--depth-min", type=float, default=None,
        help="Override server depth_min_m for this run. Must be used with --depth-max.",
    )
    parser.add_argument(
        "--depth-max", type=float, default=None,
        help="Override server depth_max_m for this run. Must be used with --depth-min.",
    )
    parser.add_argument(
        "--disable-depth-range", action="store_true",
        help="Do not send depth_min_m/depth_max_m; use server default depth_range.",
    )
    parser.add_argument(
        "--execute", action="store_true",
        help="Run pick-and-place demo flow: LView left→right, RView right→left",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Skip all motor movement; just capture + detect (assumes camera already in position)",
    )
    parser.add_argument("--yes", action="store_true", help="Skip safety prompt")
    parser.add_argument(
        "--target-speed", type=int, default=200,
        help="Speed (dps) for moving to predicted target (default: 200)",
    )
    parser.add_argument(
        "--settle", type=float, default=2.5,
        help="Wait seconds after named poses (default: 2.5)",
    )
    parser.add_argument(
        "--target-wait", type=float, default=2.0,
        help="Wait seconds at predicted target (default: 2.0)",
    )
    parser.add_argument(
        "--pick-wait", type=float, default=2.0,
        help="Reference wait seconds for PLC/vacuum handoff when --auto-step is used (default: 2.0)",
    )
    parser.add_argument(
        "--place-point", default="RGrap",
        help="Unused in HOME-mirror mode; kept only for backward-compatible command lines",
    )
    parser.add_argument(
        "--auto-step", action="store_true",
        help="Do not wait for Enter at each segment; auto-continue using short waits",
    )
    parser.add_argument(
        "--ignore-object-safety", action="store_true",
        help="No-op: object-shape safety filter is disabled in this version",
    )
    parser.add_argument(
        "--home-tolerance", type=float, default=1.0,
        help="Maximum HOME readback error per motor in degrees (default: 1.0)",
    )
    parser.add_argument(
        "--home-stable-reads", type=int, default=3,
        help="Consecutive four-axis HOME readbacks required (default: 3)",
    )
    parser.add_argument(
        "--home-timeout", type=float, default=15.0,
        help="Seconds allowed for HOME readback confirmation (default: 15)",
    )
    parser.add_argument(
        "--home-poll", type=float, default=0.2,
        help="Seconds between HOME readbacks (default: 0.2)",
    )
    home_mode = parser.add_mutually_exclusive_group()
    home_mode.add_argument(
        "--confirm-home-only",
        action="store_true",
        help="Read and confirm all four HOME angles without moving any motor or camera",
    )
    home_mode.add_argument(
        "--move-home-only",
        choices=tuple(HOME_COMPONENT_MOTORS),
        help="Move only arm (ID142/143) or camera (ID144/145) to HOME and confirm it",
    )
    home_mode.add_argument(
        "--move-pose-only",
        metavar="POSE_NAME",
        help="Move all four motors to a named pose and confirm stable readback",
    )
    args = parser.parse_args()

    if (
        args.confirm_home_only
        or args.move_home_only
        or args.move_pose_only
    ):
        controller = ArmController()
        try:
            print(f"[INFO] {controller.connect()}")
            print("[INFO] Warming up CAN adapter (1s)...")
            time.sleep(1.0)

            if hasattr(controller.service, "_serial") and controller.service._serial:
                controller.service._serial.reset_input_buffer()

            if args.move_home_only:
                move_component_home_pose(
                    controller,
                    args.move_home_only,
                    settle_sec=args.settle,
                    tolerance_degrees=args.home_tolerance,
                    stable_reads=args.home_stable_reads,
                    timeout_seconds=args.home_timeout,
                    poll_interval_seconds=args.home_poll,
                )
            elif args.move_pose_only:
                move_named_pose(
                    controller,
                    args.move_pose_only,
                    settle_sec=args.settle,
                    tolerance_degrees=args.home_tolerance,
                    stable_reads=args.home_stable_reads,
                    timeout_seconds=args.home_timeout,
                    poll_interval_seconds=args.home_poll,
                )
            else:
                confirm_home_pose(
                    controller,
                    tolerance_degrees=args.home_tolerance,
                    stable_reads=args.home_stable_reads,
                    timeout_seconds=args.home_timeout,
                    poll_interval_seconds=args.home_poll,
                    final_confirmation=True,
                )
        finally:
            controller.disconnect()
            print("[INFO] Controller disconnected.")
        return

    view = args.view
    dry_run = args.dry_run
    use_motors = not dry_run  # default: motors ON; --dry-run: motors OFF

    if view not in ROI_BY_VIEW:
        raise RuntimeError(f"Unknown view: {view}. Available: {list(ROI_BY_VIEW)}")

    if args.execute and not has_view_mapping(view):
        raise RuntimeError(f"{view} mapping is not calibrated yet; cannot execute.")

    if (args.depth_min is None) != (args.depth_max is None):
        raise RuntimeError("--depth-min and --depth-max must be provided together.")

    if args.disable_depth_range:
        depth_range = None
    elif args.depth_min is not None and args.depth_max is not None:
        depth_range = (float(args.depth_min), float(args.depth_max))
    else:
        depth_range = DEPTH_RANGE_BY_VIEW.get(view)

    roi = ROI_BY_VIEW[view]
    controller: ArmController | None = None

    def handle_termination(signum, _frame) -> None:
        if controller is not None:
            stop_all_motors_confirmed(controller)
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, handle_termination)

    mode_label = "DRY-RUN" if dry_run else ("EXECUTE (full pick)" if args.execute else "VISION ONLY")
    print(f"[INFO] VIEW = {view}")
    print(f"[INFO] ROI = {roi}")
    print(f"[INFO] depth_range = {depth_range}")
    print(f"[INFO] mapping_ready = {has_view_mapping(view)}")
    print(f"[INFO] mode = {mode_label}")

    try:
        # ==============================================================
        # Connect + go to camera pose (unless --dry-run)
        # ==============================================================
        if use_motors:
            if not confirm_motor_action(
                view=view,
                execute=args.execute,
                target_speed_dps=args.target_speed,
                yes=args.yes,
            ):
                return

            controller = ArmController()
            connect_msg = controller.connect()
            print(f"[INFO] {connect_msg}")

            # Waveshare USB-CAN-A needs warm-up after configuration.
            # In the GUI, users naturally wait between Connect and first
            # command; in script mode we must add an explicit delay.
            print("[INFO] Warming up CAN adapter (1s)...")
            time.sleep(1.0)

            # Flush any stale bytes left in serial RX buffer
            if hasattr(controller.service, '_serial') and controller.service._serial:
                controller.service._serial.reset_input_buffer()

            # Verify CAN is alive with a test read
            print("[INFO] Verifying CAN communication...")
            test_angles = read_current_angles(controller)
            if not test_angles:
                print("[WARN] No motor responded to test read. Retrying after 1s...")
                time.sleep(1.0)
                if hasattr(controller.service, '_serial') and controller.service._serial:
                    controller.service._serial.reset_input_buffer()
                test_angles = read_current_angles(controller)
                if not test_angles:
                    raise RuntimeError(
                        "CAN bus connected but no motors responded. "
                        "Check: (1) motors powered on? (2) CAN wiring OK? "
                        "(3) config.json bitrate matches hardware?"
                    )
            verify_required_motors_online(test_angles)
            print(f"[INFO] CAN verified. {len(test_angles)} motors online.")

            # HOME first, confirm all four axes, then move to the selected view.
            safe_go_to_point(controller, "HOME", args.settle)
            confirm_home_pose(
                controller,
                tolerance_degrees=args.home_tolerance,
                stable_reads=args.home_stable_reads,
                timeout_seconds=args.home_timeout,
                poll_interval_seconds=args.home_poll,
                final_confirmation=False,
            )
            safe_go_to_point(controller, view, args.settle)

        # ==============================================================
        # Vision pipeline (always runs)
        # ==============================================================
        color_bgr, depth_raw, depth_scale, cam_intr = capture_realsense_frame()

        cv2.imwrite("control_color.png", color_bgr)
        cv2.imwrite("control_depth.png", depth_raw)
        save_depth_visualization(depth_raw, "control_depth_vis.png")

        data = call_detection_server(
            color_bgr=color_bgr,
            depth_raw=depth_raw,
            depth_scale=depth_scale,
            roi=roi,
            depth_range=depth_range,
        )

        result = data.get("result")

        if result is None:
            print("[INFO] No suction point found.")
            if use_motors and controller is not None:
                print("[FLOW] No target. Return HOME for safety.")
                safe_go_to_point(controller, "HOME", args.settle)
            return

        u = int(result["u"])
        v = int(result["v"])
        z_m = float(result["z_m"])

        print(f"[INFO] suction point = ({u}, {v}), z_m={z_m:.3f} m")

        # Mask handling
        mask = None
        mask_b64 = result.get("mask_png_base64")

        if mask_b64:
            mask = decode_mask_png_base64(mask_b64)
            cv2.imwrite("control_mask.png", mask)
            print("[INFO] Saved control_mask.png")
        else:
            print("[WARN] result has no mask_png_base64")

        # Camera coords → motor angles
        x_cm, y_cm = pixel_depth_to_camera_xy_cm(
            u, v, z_m,
            fx=cam_intr["fx"], fy=cam_intr["fy"],
            cx=cam_intr["cx"], cy=cam_intr["cy"],
        )
        if has_view_mapping(view):
            target_angles = predict_view_angles(view, x_cm, y_cm)
        else:
            target_angles = {}
            print(
                f"[WARN] {view} mapping is not calibrated yet. "
                "This run will output camera_xyz_cm only; no motor target angle is available."
            )

        rx1, ry1, rx2, ry2 = roi
        inside_roi = rx1 <= u <= rx2 and ry1 <= v <= ry2

        object_info = draw_control_result(
            color_bgr=color_bgr,
            roi=roi,
            u=u,
            v=v,
            z_m=z_m,
            mask=mask,
            target_angles=target_angles,
        )

        object_safety_warnings = validate_suction_object_info(object_info)
        if object_safety_warnings:
            print("[WARN] Suspicious suction object/mask:")
            for warning in object_safety_warnings:
                print(f"  - {warning}")

        width_cm_est = None
        height_cm_est = None

        if object_info["width_px"] is not None:
            width_cm_est = object_info["width_px"] * z_m / cam_intr["fx"] * 100.0

        if object_info["height_px"] is not None:
            height_cm_est = object_info["height_px"] * z_m / cam_intr["fy"] * 100.0

        # ==============================================================
        # Output JSON (use compact "ID142" keys for backward compat)
        # ==============================================================
        target_angles_compact = {
            k.replace(" ", ""): v for k, v in target_angles.items()
        }

        control_target = {
            "view": view,
            "roi": list(roi),
            "suction_pixel": {"u": u, "v": v},
            "depth_m": z_m,
            "camera_xyz_cm": {
                "x": round(x_cm, 3),
                "y": round(y_cm, 3),
                "z": round(z_m * 100.0, 3),
            },
            "object_info": {
                **object_info,
                "width_cm_est": (
                    None if width_cm_est is None else round(width_cm_est, 3)
                ),
                "height_cm_est": (
                    None if height_cm_est is None else round(height_cm_est, 3)
                ),
            },
            "predicted_suction_angles": (target_angles_compact if target_angles else None),
            "route_plan": build_route_plan(view),
            "safety": {
                "inside_roi": inside_roi,
                "execute": args.execute,
                "target_speed_dps": args.target_speed,
                "mapping_ready": has_view_mapping(view),
                "depth_range_m": (None if depth_range is None else list(depth_range)),
                "prediction_angle_limits": (
                    None if get_view_prediction_limits(view) is None
                    else {
                        k.replace(" ", ""): v
                        for k, v in get_view_prediction_limits(view).items()
                    }
                ),
                "settle_sec": args.settle,
                "target_wait_sec": args.target_wait,
                "pick_wait_sec": args.pick_wait,
                "place_point": args.place_point,
                "auto_step": args.auto_step,
                "ignore_object_safety": args.ignore_object_safety,
                "object_safety_warnings": object_safety_warnings,
                "object_filter": SUCTION_OBJECT_FILTER,
            },
        }

        print("\n[CONTROL TARGET]")
        print(json.dumps(control_target, indent=2, ensure_ascii=False))

        Path("control_target.json").write_text(
            json.dumps(control_target, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        print("[INFO] Saved control_target.json")

        # ==============================================================
        # ROI safety gate
        # ==============================================================
        if not inside_roi:
            print("[WARN] Suction point is outside ROI. Abort execution for safety.")
            if use_motors and controller is not None:
                print("[FLOW] Target outside ROI. Return HOME for safety.")
                safe_go_to_point(controller, "HOME", args.settle)
            return

        # ==============================================================
        # Object-shape safety gate
        # ==============================================================
        if object_safety_warnings and not args.ignore_object_safety:
            print("[WARN] Object safety check failed. Abort execution for safety.")
            print("[WARN] Use --ignore-object-safety only if you intentionally want to override this.")
            if use_motors and controller is not None:
                print("[FLOW] Suspicious target. Return HOME for safety.")
                safe_go_to_point(controller, "HOME", args.settle)
            return

        # ==============================================================
        # Execute pick flow / vision-only / dry-run
        # ==============================================================
        if args.execute and controller is not None:
            # Full flow: pick the object
            if not target_angles:
                raise RuntimeError(f"{view} has no predicted target angles; cannot execute.")
            run_post_detection_pick_flow(
                controller=controller,
                view=view,
                target_angles=target_angles,
                settle_sec=args.settle,
                target_speed_dps=args.target_speed,
                target_wait_sec=args.target_wait,
                pick_wait_sec=args.pick_wait,
                place_point=args.place_point,
                auto_step=args.auto_step,
                home_tolerance_degrees=args.home_tolerance,
                home_stable_reads=args.home_stable_reads,
                home_timeout_seconds=args.home_timeout,
                home_poll_interval_seconds=args.home_poll,
            )
            confirm_home_pose(
                controller,
                tolerance_degrees=args.home_tolerance,
                stable_reads=args.home_stable_reads,
                timeout_seconds=args.home_timeout,
                poll_interval_seconds=args.home_poll,
                final_confirmation=True,
            )
        elif use_motors and controller is not None:
            # Vision-only: just go HOME after detection
            print("\n[VISION ONLY] Detection complete. Returning HOME.")
            safe_go_to_point(controller, "HOME", args.settle)
            print("\nTo run full pick flow next time:")
            print(
                f"  uv run python {Path(__file__).name} --view {view} "
                f"--execute --target-speed {args.target_speed}"
            )
        else:
            # Dry-run: no motors at all
            print("\n[DRY RUN] No motor command sent.")
            print("To run with motors:")
            print(f"  uv run python {Path(__file__).name} --view {view}")
            print("To run full pick flow:")
            print(
                f"  uv run python {Path(__file__).name} --view {view} "
                f"--execute --target-speed {args.target_speed}"
            )

    finally:
        if controller is not None:
            controller.disconnect()
            print("[INFO] Controller disconnected.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise
