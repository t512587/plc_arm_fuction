#!/usr/bin/env python3
"""
calibration_ui.py

Semi-automatic suction-point calibration UI for Raspberry Pi.

Purpose:
  1) Choose LView / RView
  2) Run the vision script to move HOME -> View -> capture -> detect -> HOME
  3) Display control_result.png
  4) Save control_target.json + images into a record folder
  5) User decides whether to keep this sample
  6) User manually reads ID142 / ID143 using 2motor_sync.py and types them here
  7) Save calibration_points_LView.csv or calibration_points_RView.csv
  8) Fit quadratic mapping and output coefficients

Expected files in the same folder:
  - d435_control.py
  - calibration_ui_safe.py

Run on Raspberry Pi desktop:
  cd ~/int
  uv run python calibration_ui.py

If uv is not needed in your environment, edit VISION_CMD_PREFIX below.
"""

from __future__ import annotations

import csv
import json
import math
import shutil
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Any


# =============================================================================
# User settings
# =============================================================================

BASE_DIR = Path(__file__).resolve().parent

# Canonical vision+arm control script.
VISION_SCRIPT_CANDIDATES = [
    BASE_DIR / "d435_control.py",
]


def get_vision_script() -> Path | None:
    for path in VISION_SCRIPT_CANDIDATES:
        if path.exists():
            return path
    return None


# Current project normally runs scripts with: uv run python xxx.py
# If you want direct python, change this to [sys.executable].
VISION_CMD_PREFIX = ["uv", "run", "python"]

RECORD_ROOT = BASE_DIR / "calibration_records"
MIN_RECOMMENDED_POINTS = 20

FIELDNAMES = [
    "point",
    "view",
    "x_cm",
    "y_cm",
    "id142",
    "id143",
    "u",
    "v",
    "z_m",
    "control_json",
    "result_image",
    "note",
]


# =============================================================================
# CSV helpers
# =============================================================================

def csv_path_for_view(view: str) -> Path:
    return BASE_DIR / f"calibration_points_{view}.csv"


def result_path_for_view(view: str) -> Path:
    return BASE_DIR / f"calibration_fit_result_{view}.txt"


def load_rows(view: str) -> list[dict[str, str]]:
    path = csv_path_for_view(view)
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def save_rows(view: str, rows: list[dict[str, Any]]) -> None:
    path = csv_path_for_view(view)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in FIELDNAMES})


def next_point_index(view: str) -> int:
    rows = load_rows(view)
    nums: list[int] = []
    for row in rows:
        try:
            nums.append(int(float(row.get("point", ""))))
        except ValueError:
            pass
    return max(nums) + 1 if nums else 1


# =============================================================================
# Quadratic fit helpers
# =============================================================================

def design_terms(x: float, y: float) -> list[float]:
    # angle = a*x + b*y + c*x^2 + d*x*y + e*y^2 + f
    return [x, y, x * x, x * y, y * y, 1.0]


def solve_linear_system(A: list[list[float]], b: list[float]) -> list[float]:
    """Solve A x = b using Gaussian elimination with partial pivoting."""
    n = len(A)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]

    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[pivot][col]) < 1e-12:
            raise ValueError("矩陣奇異或資料點分布不足，請增加/分散校正點。")

        if pivot != col:
            M[col], M[pivot] = M[pivot], M[col]

        div = M[col][col]
        for j in range(col, n + 1):
            M[col][j] /= div

        for r in range(n):
            if r == col:
                continue
            factor = M[r][col]
            if abs(factor) < 1e-15:
                continue
            for j in range(col, n + 1):
                M[r][j] -= factor * M[col][j]

    return [M[i][n] for i in range(n)]


def fit_quadratic(rows: list[dict[str, str]], target_key: str) -> tuple[list[float], list[dict[str, float]]]:
    samples: list[tuple[float, float, float]] = []
    for row in rows:
        try:
            x = float(row["x_cm"])
            y = float(row["y_cm"])
            angle = float(row[target_key])
            samples.append((x, y, angle))
        except (KeyError, ValueError):
            pass

    if len(samples) < 6:
        raise ValueError("二次模型至少需要 6 點；建議 20 點以上。")

    m = 6
    XtX = [[0.0 for _ in range(m)] for _ in range(m)]
    Xty = [0.0 for _ in range(m)]

    for x, y, angle in samples:
        t = design_terms(x, y)
        for i in range(m):
            Xty[i] += t[i] * angle
            for j in range(m):
                XtX[i][j] += t[i] * t[j]

    coef = solve_linear_system(XtX, Xty)

    details: list[dict[str, float]] = []
    for x, y, angle in samples:
        t = design_terms(x, y)
        pred = sum(coef[i] * t[i] for i in range(m))
        err = pred - angle
        details.append({"x": x, "y": y, "actual": angle, "pred": pred, "err": err})

    return coef, details


def rmse(details: list[dict[str, float]]) -> float:
    return math.sqrt(sum(d["err"] ** 2 for d in details) / len(details))


def max_abs_error(details: list[dict[str, float]]) -> float:
    return max(abs(d["err"]) for d in details)


def format_coef(coef: list[float]) -> str:
    return "(" + ", ".join(f"{v:.8f}" for v in coef) + ")"


def suggest_limits(rows: list[dict[str, str]], key: str, pad: float = 10.0) -> tuple[float, float]:
    vals: list[float] = []
    for row in rows:
        try:
            vals.append(float(row[key]))
        except (KeyError, ValueError):
            pass
    if not vals:
        return (0.0, 0.0)
    return (math.floor(min(vals) - pad), math.ceil(max(vals) + pad))


def build_result_text(view: str, rows: list[dict[str, str]]) -> str:
    coef142, det142 = fit_quadratic(rows, "id142")
    coef143, det143 = fit_quadratic(rows, "id143")

    lim142 = suggest_limits(rows, "id142", pad=10.0)
    lim143 = suggest_limits(rows, "id143", pad=10.0)

    lines: list[str] = []
    lines.append("=== Calibration Fit Result ===")
    lines.append(f"View: {view}")
    lines.append(f"CSV: {csv_path_for_view(view).resolve()}")
    lines.append(f"Samples: {len(rows)}")
    lines.append("")
    lines.append("Model:")
    lines.append("  angle = a*x + b*y + c*x^2 + d*x*y + e*y^2 + f")
    lines.append("")
    lines.append("ID142 error:")
    lines.append(f"  RMSE = {rmse(det142):.3f} deg")
    lines.append(f"  MAX  = {max_abs_error(det142):.3f} deg")
    lines.append("")
    lines.append("ID143 error:")
    lines.append(f"  RMSE = {rmse(det143):.3f} deg")
    lines.append(f"  MAX  = {max_abs_error(det143):.3f} deg")
    lines.append("")
    lines.append("Paste this into main script:")
    lines.append("")
    lines.append(f'CALIB_BY_VIEW["{view}"] = {{')
    lines.append(f'    "ID 142": {format_coef(coef142)},')
    lines.append(f'    "ID 143": {format_coef(coef143)},')
    lines.append("}")
    lines.append("")
    lines.append(f'PREDICTION_LIMITS_BY_VIEW["{view}"] = {{')
    lines.append(f'    "ID 142": ({lim142[0]:.1f}, {lim142[1]:.1f}),')
    lines.append(f'    "ID 143": ({lim143[0]:.1f}, {lim143[1]:.1f}),')
    lines.append("}")
    lines.append("")
    lines.append("Per-point errors:")
    lines.append("point,id142_actual,id142_pred,id142_err,id143_actual,id143_pred,id143_err")
    for i, (d142, d143) in enumerate(zip(det142, det143), start=1):
        lines.append(
            f"{i},"
            f"{d142['actual']:.2f},{d142['pred']:.2f},{d142['err']:.2f},"
            f"{d143['actual']:.2f},{d143['pred']:.2f},{d143['err']:.2f}"
        )

    return "\n".join(lines)


# =============================================================================
# UI app
# =============================================================================

class CalibrationUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Suction Point Calibration UI")

        self.current_target: dict[str, Any] | None = None
        self.current_record_dir: Path | None = None
        self.current_result_image: Path | None = None
        self.photo: tk.PhotoImage | None = None

        self.view_var = tk.StringVar(value="LView")
        self.speed_var = tk.StringVar(value="40")
        self.id142_var = tk.StringVar()
        self.id143_var = tk.StringVar()
        self.note_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Ready.")

        self._build_ui()
        self.refresh_point_count()

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(2, weight=1)

        control = ttk.LabelFrame(main, text="1) Capture / detect", padding=8)
        control.grid(row=0, column=0, columnspan=2, sticky="ew")
        control.columnconfigure(8, weight=1)

        ttk.Label(control, text="View").grid(row=0, column=0, padx=4)
        view_box = ttk.Combobox(control, textvariable=self.view_var, values=["LView", "RView"], width=8, state="readonly")
        view_box.grid(row=0, column=1, padx=4)
        view_box.bind("<<ComboboxSelected>>", lambda _e: self.refresh_point_count())

        ttk.Label(control, text="Target speed").grid(row=0, column=2, padx=4)
        ttk.Entry(control, textvariable=self.speed_var, width=8).grid(row=0, column=3, padx=4)

        ttk.Button(control, text="Run capture/detect", command=self.run_capture).grid(row=0, column=4, padx=8)
        ttk.Button(control, text="Show record folder", command=self.show_record_folder).grid(row=0, column=5, padx=4)

        self.count_label = ttk.Label(control, text="")
        self.count_label.grid(row=0, column=6, padx=12)

        image_box = ttk.LabelFrame(main, text="2) Check control_result.png", padding=8)
        image_box.grid(row=1, column=0, rowspan=2, sticky="nsew", pady=8)
        image_box.rowconfigure(0, weight=1)
        image_box.columnconfigure(0, weight=1)

        self.image_label = ttk.Label(image_box, text="Run capture/detect to show image here.", anchor="center")
        self.image_label.grid(row=0, column=0, sticky="nsew")

        data_box = ttk.LabelFrame(main, text="3) Accept point after manual alignment", padding=8)
        data_box.grid(row=1, column=1, sticky="new", padx=(8, 0), pady=8)

        self.target_text = tk.Text(data_box, width=58, height=14)
        self.target_text.grid(row=0, column=0, columnspan=4, sticky="ew")

        ttk.Label(data_box, text="Actual ID142").grid(row=1, column=0, padx=4, pady=8, sticky="e")
        ttk.Entry(data_box, textvariable=self.id142_var, width=12).grid(row=1, column=1, padx=4, pady=8, sticky="w")

        ttk.Label(data_box, text="Actual ID143").grid(row=1, column=2, padx=4, pady=8, sticky="e")
        ttk.Entry(data_box, textvariable=self.id143_var, width=12).grid(row=1, column=3, padx=4, pady=8, sticky="w")

        ttk.Label(data_box, text="Note").grid(row=2, column=0, padx=4, pady=4, sticky="e")
        ttk.Entry(data_box, textvariable=self.note_var, width=40).grid(row=2, column=1, columnspan=3, padx=4, pady=4, sticky="ew")

        ttk.Button(data_box, text="Accept / save this point", command=self.accept_point).grid(row=3, column=0, columnspan=2, padx=4, pady=8, sticky="ew")
        ttk.Button(data_box, text="Reject this point", command=self.reject_point).grid(row=3, column=2, padx=4, pady=8, sticky="ew")
        ttk.Button(data_box, text="Delete last accepted", command=self.delete_last).grid(row=3, column=3, padx=4, pady=8, sticky="ew")

        fit_box = ttk.LabelFrame(main, text="4) Fit model", padding=8)
        fit_box.grid(row=2, column=1, sticky="nsew", padx=(8, 0), pady=(0, 8))
        fit_box.columnconfigure(0, weight=1)
        fit_box.rowconfigure(1, weight=1)

        ttk.Button(fit_box, text="Fit current view", command=self.fit_current_view).grid(row=0, column=0, sticky="ew")
        self.fit_text = tk.Text(fit_box, width=58, height=14)
        self.fit_text.grid(row=1, column=0, sticky="nsew", pady=(8, 0))

        bottom = ttk.Frame(main)
        bottom.grid(row=3, column=0, columnspan=2, sticky="ew")
        bottom.columnconfigure(0, weight=1)
        ttk.Label(bottom, textvariable=self.status_var).grid(row=0, column=0, sticky="w")

    def set_status(self, text: str) -> None:
        self.status_var.set(text)
        self.root.update_idletasks()

    def write_target_text(self, text: str) -> None:
        self.target_text.delete("1.0", tk.END)
        self.target_text.insert(tk.END, text)

    def write_fit_text(self, text: str) -> None:
        self.fit_text.delete("1.0", tk.END)
        self.fit_text.insert(tk.END, text)

    def refresh_point_count(self) -> None:
        view = self.view_var.get()
        rows = load_rows(view)
        self.count_label.config(text=f"{view}: {len(rows)} / {MIN_RECOMMENDED_POINTS} pts")

    def run_capture_threaded(self) -> None:
        thread = threading.Thread(target=self.run_capture, daemon=True)
        thread.start()

    def run_capture(self) -> None:
        view = self.view_var.get()
        try:
            speed = int(float(self.speed_var.get()))
        except ValueError:
            messagebox.showerror("Input error", "Target speed must be a number.")
            return

        vision_script = get_vision_script()
        if vision_script is None:
            msg = "Cannot find any vision script:\n" + "\n".join(str(p) for p in VISION_SCRIPT_CANDIDATES)
            self.set_status("Missing vision script.")
            messagebox.showerror("Missing script", msg)
            return

        self.current_target = None
        self.current_record_dir = None
        self.current_result_image = None
        self.write_target_text("")
        self.id142_var.set("")
        self.id143_var.set("")

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        record_dir = RECORD_ROOT / view / timestamp
        record_dir.mkdir(parents=True, exist_ok=True)

        cmd = VISION_CMD_PREFIX + [str(vision_script), "--view", view, "--target-speed", str(speed), "--yes"]

        self.set_status(f"Running: {' '.join(cmd)}")
        self.write_target_text("Running capture/detect...\nThis will move HOME -> View -> capture -> HOME.\n")

        try:
            proc = subprocess.run(
                cmd,
                cwd=BASE_DIR,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=180,
            )
        except subprocess.TimeoutExpired:
            self.set_status("Timeout.")
            messagebox.showerror("Timeout", "Vision script timed out.")
            return
        except Exception as exc:
            self.set_status("Failed.")
            messagebox.showerror("Run failed", str(exc))
            return

        log_path = record_dir / "run_log.txt"
        log_path.write_text(proc.stdout, encoding="utf-8")

        copied: dict[str, Path] = {}
        for name in [
            "control_target.json",
            "control_result.png",
            "control_color.png",
            "control_depth.png",
            "control_depth_vis.png",
            "control_mask.png",
        ]:
            src = BASE_DIR / name
            if src.exists():
                dst = record_dir / name
                try:
                    shutil.copy2(src, dst)
                    copied[name] = dst
                except Exception:
                    pass

        if proc.returncode != 0:
            self.set_status("Vision script returned error.")
            self.write_target_text(proc.stdout)
            messagebox.showerror("Vision script error", f"See log:\n{log_path}")
            return

        target_path = copied.get("control_target.json")
        result_image = copied.get("control_result.png")

        if target_path is None:
            self.set_status("No control_target.json. Probably no suction point found.")
            self.write_target_text(
                "No control_target.json was created.\n"
                "Usually this means: No suction point found.\n\n"
                f"Record folder:\n{record_dir}\n\n"
                "Run log tail:\n" + proc.stdout[-3000:]
            )
            self.load_image(copied.get("control_color.png") or copied.get("control_depth_vis.png"))
            self.current_record_dir = record_dir
            return

        try:
            target = json.loads(target_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.set_status("Failed to read control_target.json.")
            messagebox.showerror("JSON error", str(exc))
            return

        self.current_target = target
        self.current_record_dir = record_dir
        self.current_result_image = result_image

        self.load_image(result_image or copied.get("control_color.png"))

        xyz = target.get("camera_xyz_cm", {})
        pixel = target.get("suction_pixel", {})
        depth = target.get("depth_m", "")
        inside_roi = target.get("safety", {}).get("inside_roi", "")

        summary = {
            "view": target.get("view"),
            "roi": target.get("roi"),
            "u": pixel.get("u"),
            "v": pixel.get("v"),
            "depth_m": depth,
            "x_cm": xyz.get("x"),
            "y_cm": xyz.get("y"),
            "z_cm": xyz.get("z"),
            "inside_roi": inside_roi,
            "record_dir": str(record_dir),
        }

        self.write_target_text(
            "Detected suction point.\n"
            "1) Check the displayed image.\n"
            "2) If yellow point/mask is correct, use 2motor_sync.py:\n"
            "   SHUTDOWN ALL -> manually align suction head -> Read Current Positions.\n"
            "3) Type actual ID142 / ID143 here, then Accept.\n\n"
            + json.dumps(summary, indent=2, ensure_ascii=False)
        )
        self.set_status(f"Capture done. Record saved: {record_dir}")
        self.refresh_point_count()

    def load_image(self, path: Path | None) -> None:
        if path is None or not path.exists():
            self.image_label.config(text="No image found.", image="")
            self.photo = None
            return

        try:
            photo = tk.PhotoImage(file=str(path))
        except Exception as exc:
            self.image_label.config(text=f"Cannot display image:\n{path}\n{exc}", image="")
            self.photo = None
            return

        self.photo = photo
        self.image_label.config(image=self.photo, text="")

    def show_record_folder(self) -> None:
        if self.current_record_dir:
            messagebox.showinfo("Record folder", str(self.current_record_dir))
        else:
            messagebox.showinfo("Record folder", "No current record folder yet.")

    def accept_point(self) -> None:
        if self.current_target is None:
            messagebox.showwarning("No target", "Run capture/detect first.")
            return

        view = str(self.current_target.get("view") or self.view_var.get())
        xyz = self.current_target.get("camera_xyz_cm") or {}
        pixel = self.current_target.get("suction_pixel") or {}

        try:
            x_cm = float(xyz["x"])
            y_cm = float(xyz["y"])
            id142 = float(self.id142_var.get())
            id143 = float(self.id143_var.get())
        except Exception:
            messagebox.showerror(
                "Missing data",
                "Need camera x/y and actual ID142 / ID143.\n"
                "Please type the actual motor angles read from 2motor_sync.py.",
            )
            return

        point = next_point_index(view)
        rows = load_rows(view)
        rows.append({
            "point": point,
            "view": view,
            "x_cm": f"{x_cm:.3f}",
            "y_cm": f"{y_cm:.3f}",
            "id142": f"{id142:.2f}",
            "id143": f"{id143:.2f}",
            "u": "" if pixel.get("u", "") == "" else str(pixel.get("u")),
            "v": "" if pixel.get("v", "") == "" else str(pixel.get("v")),
            "z_m": "" if self.current_target.get("depth_m", "") == "" else f"{float(self.current_target.get('depth_m')):.6f}",
            "control_json": "" if self.current_record_dir is None else str(self.current_record_dir / "control_target.json"),
            "result_image": "" if self.current_result_image is None else str(self.current_result_image),
            "note": self.note_var.get().strip(),
        })
        save_rows(view, rows)

        self.set_status(f"[OK] Saved {view} point {point}.")
        self.write_target_text(
            f"[OK] Saved {view} point {point}.\n\n"
            f"x={x_cm:.3f}, y={y_cm:.3f}, ID142={id142:.2f}, ID143={id143:.2f}\n"
            f"CSV: {csv_path_for_view(view)}"
        )
        self.id142_var.set("")
        self.id143_var.set("")
        self.note_var.set("")
        self.refresh_point_count()

    def reject_point(self) -> None:
        if self.current_record_dir:
            self.set_status(f"Rejected current point. Raw record kept: {self.current_record_dir}")
            self.write_target_text(
                "Rejected current point.\n"
                "No row was added to CSV.\n\n"
                f"Raw record is still kept here:\n{self.current_record_dir}"
            )
        else:
            self.set_status("Rejected current point.")
            self.write_target_text("Rejected current point. No row was added to CSV.")

        self.current_target = None
        self.id142_var.set("")
        self.id143_var.set("")
        self.note_var.set("")

    def delete_last(self) -> None:
        view = self.view_var.get()
        rows = load_rows(view)
        if not rows:
            messagebox.showinfo("Delete last", f"No data in {csv_path_for_view(view)}.")
            return

        last = rows[-1]
        ok = messagebox.askyesno(
            "Delete last accepted point",
            f"Delete last {view} point?\n\n"
            f"point={last.get('point')}\n"
            f"x={last.get('x_cm')}, y={last.get('y_cm')}\n"
            f"ID142={last.get('id142')}, ID143={last.get('id143')}",
        )
        if not ok:
            return

        rows.pop()
        save_rows(view, rows)
        self.set_status(f"Deleted last {view} point.")
        self.refresh_point_count()

    def fit_current_view(self) -> None:
        view = self.view_var.get()
        rows = load_rows(view)

        if len(rows) < 6:
            messagebox.showwarning(
                "Not enough points",
                f"{view} has {len(rows)} points. Need at least 6; recommended {MIN_RECOMMENDED_POINTS}.",
            )
            return

        try:
            text = build_result_text(view, rows)
        except Exception as exc:
            messagebox.showerror("Fit failed", str(exc))
            return

        result_path = result_path_for_view(view)
        result_path.write_text(text, encoding="utf-8")
        self.write_fit_text(text)
        self.set_status(f"[OK] Fit result saved: {result_path}")


def main() -> None:
    root = tk.Tk()
    CalibrationUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()