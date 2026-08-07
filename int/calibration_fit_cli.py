#!/usr/bin/env python3
"""
calibration_fit_cli.py

Second-car LView calibration data collector + quadratic fitting tool.

No uv required. No numpy required. Pure Python standard library.

Data needed per point:
  x_cm, y_cm, actual ID142 angle, actual ID143 angle

Optional:
  paste CONTROL TARGET JSON and the script extracts:
    camera_xyz_cm.x
    camera_xyz_cm.y
    suction_pixel.u
    suction_pixel.v
    depth_m

Output model:
  angle = a*x + b*y + c*x^2 + d*x*y + e*y^2 + f

Run:
  python3 calibration_fit_cli.py

Files written:
  calibration_points.csv
  calibration_fit_result.txt
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

CSV_PATH = Path("calibration_points.csv")
RESULT_PATH = Path("calibration_fit_result.txt")

FIELDNAMES = [
    "point",
    "x_cm",
    "y_cm",
    "id142",
    "id143",
    "u",
    "v",
    "z_m",
    "note",
]


# -----------------------------
# Basic IO helpers
# -----------------------------

def prompt_float(label: str, default: float | None = None, required: bool = True) -> float | None:
    while True:
        suffix = "" if default is None else f" [{default}]"
        s = input(f"{label}{suffix}: ").strip()
        if not s and default is not None:
            return default
        if not s and not required:
            return None
        try:
            return float(s)
        except ValueError:
            print("  請輸入數字。")


def prompt_int(label: str, default: int | None = None, required: bool = True) -> int | None:
    while True:
        suffix = "" if default is None else f" [{default}]"
        s = input(f"{label}{suffix}: ").strip()
        if not s and default is not None:
            return default
        if not s and not required:
            return None
        try:
            return int(float(s))
        except ValueError:
            print("  請輸入整數。")


def load_rows() -> list[dict[str, str]]:
    if not CSV_PATH.exists():
        return []
    with CSV_PATH.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows


def save_rows(rows: list[dict[str, Any]]) -> None:
    with CSV_PATH.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in FIELDNAMES})


def next_point_index(rows: list[dict[str, str]]) -> int:
    nums: list[int] = []
    for row in rows:
        try:
            nums.append(int(float(row.get("point", ""))))
        except ValueError:
            pass
    return (max(nums) + 1) if nums else 1


# -----------------------------
# CONTROL TARGET JSON parsing
# -----------------------------

def read_multiline_json() -> str:
    print("\n貼上 [CONTROL TARGET] 的 JSON。")
    print("貼完後輸入一行 END 結束。")
    print("只貼 {...} 那段就好，也可以包含 [CONTROL TARGET] 前綴。")
    lines: list[str] = []
    while True:
        line = input()
        if line.strip() == "END":
            break
        lines.append(line)
    return "\n".join(lines)


def extract_json_object(text: str) -> dict[str, Any]:
    text = text.strip()
    text = text.replace("[CONTROL TARGET]", "").strip()

    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end <= start:
        raise ValueError("找不到 JSON 物件。")
    return json.loads(text[start:end + 1])


def point_from_control_target(data: dict[str, Any]) -> dict[str, Any]:
    xyz = data.get("camera_xyz_cm") or {}
    suction_pixel = data.get("suction_pixel") or {}

    if "x" not in xyz or "y" not in xyz:
        raise ValueError("JSON 裡找不到 camera_xyz_cm.x / camera_xyz_cm.y")

    return {
        "x_cm": float(xyz["x"]),
        "y_cm": float(xyz["y"]),
        "u": suction_pixel.get("u", ""),
        "v": suction_pixel.get("v", ""),
        "z_m": data.get("depth_m", ""),
    }


# -----------------------------
# Least squares without numpy
# -----------------------------

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
        details.append({
            "x": x,
            "y": y,
            "actual": angle,
            "pred": pred,
            "err": err,
        })

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


def build_result_text(rows: list[dict[str, str]]) -> str:
    coef142, det142 = fit_quadratic(rows, "id142")
    coef143, det143 = fit_quadratic(rows, "id143")

    lim142 = suggest_limits(rows, "id142", pad=10.0)
    lim143 = suggest_limits(rows, "id143", pad=10.0)

    lines: list[str] = []
    lines.append("=== Calibration Fit Result ===")
    lines.append(f"CSV: {CSV_PATH.resolve()}")
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
    lines.append("CALIB_LVIEW_QUAD = {")
    lines.append(f'    "ID 142": {format_coef(coef142)},')
    lines.append(f'    "ID 143": {format_coef(coef143)},')
    lines.append("}")
    lines.append("")
    lines.append("PREDICTION_ANGLE_LIMITS = {")
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


# -----------------------------
# Menu actions
# -----------------------------

def print_rows(rows: list[dict[str, str]]) -> None:
    if not rows:
        print("\n目前沒有資料。")
        return

    print("\n目前資料：")
    print("pt | x_cm     y_cm     ID142     ID143     u      v      z_m")
    print("---+-------------------------------------------------------------")
    for row in rows:
        print(
            f"{row.get('point',''):>2} | "
            f"{row.get('x_cm',''):>7} "
            f"{row.get('y_cm',''):>7} "
            f"{row.get('id142',''):>8} "
            f"{row.get('id143',''):>8} "
            f"{row.get('u',''):>6} "
            f"{row.get('v',''):>6} "
            f"{row.get('z_m',''):>8}"
        )


def add_point_manual(rows: list[dict[str, str]]) -> None:
    pt = next_point_index(rows)
    print(f"\n新增第 {pt} 點：手動輸入 x_cm / y_cm")
    x = prompt_float("x_cm")
    y = prompt_float("y_cm")
    id142 = prompt_float("實際 ID142 角度")
    id143 = prompt_float("實際 ID143 角度")
    u = prompt_int("u，可空白", required=False)
    v = prompt_int("v，可空白", required=False)
    z = prompt_float("z_m，可空白", required=False)
    note = input("note，可空白: ").strip()

    rows.append({
        "point": pt,
        "x_cm": f"{x:.3f}",
        "y_cm": f"{y:.3f}",
        "id142": f"{id142:.2f}",
        "id143": f"{id143:.2f}",
        "u": "" if u is None else str(u),
        "v": "" if v is None else str(v),
        "z_m": "" if z is None else f"{z:.6f}",
        "note": note,
    })
    save_rows(rows)
    print(f"[OK] 已存第 {pt} 點到 {CSV_PATH}")


def add_point_from_json(rows: list[dict[str, str]]) -> None:
    pt = next_point_index(rows)
    text = read_multiline_json()
    try:
        data = extract_json_object(text)
        p = point_from_control_target(data)
    except Exception as exc:
        print(f"[ERROR] JSON 解析失敗: {exc}")
        return

    print(
        f"\n從 CONTROL TARGET 抓到：x={p['x_cm']:.3f}, y={p['y_cm']:.3f}, "
        f"u={p.get('u')}, v={p.get('v')}, z={p.get('z_m')}"
    )
    id142 = prompt_float("實際 ID142 角度")
    id143 = prompt_float("實際 ID143 角度")
    note = input("note，可空白: ").strip()

    rows.append({
        "point": pt,
        "x_cm": f"{float(p['x_cm']):.3f}",
        "y_cm": f"{float(p['y_cm']):.3f}",
        "id142": f"{id142:.2f}",
        "id143": f"{id143:.2f}",
        "u": "" if p.get("u", "") == "" else str(p.get("u")),
        "v": "" if p.get("v", "") == "" else str(p.get("v")),
        "z_m": "" if p.get("z_m", "") == "" else f"{float(p.get('z_m')):.6f}",
        "note": note,
    })
    save_rows(rows)
    print(f"[OK] 已存第 {pt} 點到 {CSV_PATH}")


def delete_last(rows: list[dict[str, str]]) -> None:
    if not rows:
        print("沒有資料可刪。")
        return
    last = rows[-1]
    print(f"準備刪除最後一點：point={last.get('point')}, x={last.get('x_cm')}, y={last.get('y_cm')}")
    ans = input("確定刪除？輸入 y: ").strip().lower()
    if ans == "y":
        rows.pop()
        save_rows(rows)
        print("[OK] 已刪除。")
    else:
        print("取消。")


def fit_and_save(rows: list[dict[str, str]]) -> None:
    if len(rows) < 6:
        print("資料太少，二次模型至少 6 點，建議 20 點。")
        return

    try:
        text = build_result_text(rows)
    except Exception as exc:
        print(f"[ERROR] Fit 失敗: {exc}")
        return

    RESULT_PATH.write_text(text, encoding="utf-8")
    print("\n" + text)
    print(f"\n[OK] 結果已存到 {RESULT_PATH}")


def main() -> None:
    print("Second-car LView calibration collector / fitter")
    print("No uv required. Run with: python3 calibration_fit_cli.py")
    print(f"CSV file: {CSV_PATH.resolve()}")

    rows = load_rows()
    if rows:
        print(f"[INFO] 已讀取 {len(rows)} 筆資料。")
    else:
        print("[INFO] 尚未有資料，會建立 calibration_points.csv。")

    while True:
        print("\n==== MENU ====")
        print(f"目前點數：{len(load_rows())} / 建議 20")
        print("1) 貼 CONTROL TARGET JSON + 輸入 ID142/ID143")
        print("2) 手動輸入 x_cm/y_cm + ID142/ID143")
        print("3) 顯示目前資料")
        print("4) Fit 二次模型並輸出 CALIB_LVIEW_QUAD")
        print("5) 刪除最後一點")
        print("6) 離開")
        choice = input("選擇: ").strip()

        rows = load_rows()

        if choice == "1":
            add_point_from_json(rows)
        elif choice == "2":
            add_point_manual(rows)
        elif choice == "3":
            print_rows(rows)
        elif choice == "4":
            fit_and_save(rows)
        elif choice == "5":
            delete_last(rows)
        elif choice == "6":
            print("Bye.")
            return
        else:
            print("請輸入 1~6。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n使用者中止。")
