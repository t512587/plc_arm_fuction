"""Tkinter UI:

- 左側顯示 group 清單（來自 points.yml）
- 右側顯示各點位與目前數值
- 上方提供連線/斷線按鈕
- 透過 HTTP 呼叫 FastAPI：
  - /plc/connect, /plc/disconnect
  - /points?group=...
  - /plc/registers/by-points?ids=...
"""

from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
import time
import math
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, font as tkfont
from tkinter import messagebox, simpledialog, ttk
from typing import Any, Callable, Dict, List

import requests

from config.loader import CONFIG_STORE, PointDefinition
try:
    from task_file import TaskFileError, TaskStep, parse_task_file
except ModuleNotFoundError:
    from plc2.task_file import TaskFileError, TaskStep, parse_task_file

API_BASE_URL = "http://127.0.0.1:8000"
DEFAULT_PLC_NAME = "main_plc"
SLOT_SIDES = ("Y1", "Y2")
FLOW_COMPONENT_NAMES = {
    "home": "HomeFlow",
    "vision-height": "VisionHeightFlow",
    "main-cycle": "MainCycleFlow",
}
FLOW_TERMINAL_STATUSES = {"success", "cancelled", "timeout", "error"}
HOME_FLOW_REQUEST_TIMEOUT_SECONDS = 210
MAIN_CYCLE_STEP_REQUEST_TIMEOUT_SECONDS = 360
MAIN_CYCLE_SECOND_STEP_REQUEST_TIMEOUT_SECONDS = 720
CANCEL_STATUS_POLL_INTERVAL_MS = 250
CANCEL_STATUS_CONFIRM_TIMEOUT_SECONDS = 15.0
SECOND_STEP_STATUS_POLL_INTERVAL_MS = 500
CONTROL_RESULT_POLL_INTERVAL_MS = 500
CONTROL_RESULT_MAX_WIDTH = 360
CONTROL_RESULT_MAX_HEIGHT = 300
CONTROL_RESULT_PATH = Path(__file__).resolve().parents[2] / "control_result.png"
SECOND_STEP_LABELS = {
    "independent_second_step_to_safe_height": "先將升降機自動定位到 560mm",
    "independent_home_precheck": "560mm 到位後確認四軸 HOME",
    "post_step_home_check": "步驟結束：確認手臂與 Camera HOME",
    "step2_precheck": "檢查 PLC 與升降服務",
    "step2_to_vision_height": "升降機移動到 560mm 視覺高度",
    "step2_arm_vision_pick_place": "D435i／CANBus 手臂取放",
    "step2_confirm_safe_height_before_arm_motion": "確認目前動作所需安全高度後放行手臂",
    "step2_pick_height": "取料：升降與吸真空",
    "step2_safe_height_before_transfer": "換邊前：保持吸附並下降到 460mm",
    "step2_arm_transfer": "安全高度已確認：手臂換邊",
    "step2_place_height": "放料：保持 460mm 並切換破真空",
    "step2_safe_height_before_home": "回 HOME 前：保持並確認 460mm",
    "step2_arm_return_home": "安全高度已確認：手臂返回 HOME",
    "step2_arm_home_confirmed": "四顆馬達 HOME 已確認",
    "step2_start_bridge": "啟動 RealSense／CANBus 橋接",
    "step2_wait_height": "等待辨識高度",
    "step2_m54_height": "移動到取料高度並開啟 M54",
    "step2_return_after_m54": "取料後返回並保持 460mm",
    "step2_m56_cross_side_height": "保持 460mm 並切換 M54 OFF／M56 ON",
    "step2_hold_cross_side_height_after_m56": "破真空後維持 460mm",
    "step2_wait_done": "等待 RealSense／CANBus 完成",
}
CJK_FONT_CANDIDATES = (
    "Noto Sans CJK TC",
    "Noto Sans CJK SC",
    "Noto Sans TC",
    "Noto Sans SC",
    "Noto Sans",
    "Droid Sans Fallback",
    "AR PL UMing TW",
    "AR PL UKai TW",
    "WenQuanYi Zen Hei",
    "Microsoft JhengHei UI",
    "Microsoft JhengHei",
    "PingFang TC",
    "Heiti TC",
    "Source Han Sans TW",
    "Source Han Sans TC",
)


class MainWindow(tk.Tk):
    columns = ("id", "name", "device", "address", "value", "writable")
    headers = {
        "id": "ID",
        "name": "名稱",
        "device": "裝置",
        "address": "位址",
        "value": "數值",
        "writable": "可寫入",
    }

    def __init__(self) -> None:
        super().__init__()
        self.title("PLC Monitor")
        self.geometry("1500x900")
        self.minsize(1100, 700)
        self._configure_fonts()

        self._connected = False
        self._current_points: List[PointDefinition] = []
        self._values: Dict[str, str] = {}
        self._active_flow: str | None = None
        self._active_flow_label: str | None = None
        self._flow_started_at: float | None = None
        self._flow_timer_id: str | None = None
        self._flow_cancelling = False
        self._flow_cancel_started_at: float | None = None
        self._flow_cancel_deadline: float | None = None
        self._flow_cancel_poll_id: str | None = None
        self._flow_cancel_poll_in_flight = False
        self._stop_unconfirmed = False
        self._ui_events: queue.Queue[Callable[[], None]] = queue.Queue()
        self.side_vacuum_vars = {
            side: tk.StringVar(value="真空：未知 / 破真空：未知")
            for side in SLOT_SIDES
        }
        self.middle_vacuum_var = tk.StringVar(value="M54：未知 / M56：未知")
        self.vacuum_controls: list[ttk.Button] = []
        self.arm_home_confirm_var = tk.StringVar(value="手臂／Camera HOME：尚未確認")
        self._arm_home_confirming = False
        self.main_cycle_first_slot_var = tk.StringVar(value="Y1")
        self.main_cycle_first_action_var = tk.StringVar(value="吸")
        self.main_cycle_first_height_var = tk.StringVar(value="560")
        self.main_cycle_first_forward_var = tk.StringVar(value="")
        self.main_cycle_transfer_direction_var = tk.StringVar(value="Y1_TO_Y2")
        self.main_cycle_final_slot_var = tk.StringVar(value="Y2")
        self.main_cycle_final_action_var = tk.StringVar(value="推")
        self.main_cycle_final_height_var = tk.StringVar(value="560")
        self.main_cycle_final_forward_var = tk.StringVar(value="")
        self.task_file_path_var = tk.StringVar(value="No TXT task loaded")
        self._task_file_steps: list[TaskStep] = []
        self._task_file_control_settings: dict[str, str] = {}
        self._task_file_running = False
        self._task_file_index = 0
        self.main_cycle_phase = "ready_first_step"
        self.main_cycle_phase_var = tk.StringVar(value="完整流程準備完成：先執行第一步")
        self.main_cycle_run_text_var = tk.StringVar(value="開始完整流程")
        self.main_cycle_controls: list[tk.Widget] = []
        self.main_cycle_first_controls: list[tk.Widget] = []
        self.main_cycle_second_controls: list[tk.Widget] = []
        self.main_cycle_final_controls: list[tk.Widget] = []
        self._main_cycle_gate_window: tk.Toplevel | None = None
        self._second_step_active = False
        self._second_step_started_at: float | None = None
        self._second_step_window: tk.Toplevel | None = None
        self._second_step_poll_id: str | None = None
        self._second_step_poll_in_flight = False
        self._second_step_last_message = ""
        self._second_step_stage_var = tk.StringVar(value="準備執行第二步")
        self._second_step_state_var = tk.StringVar(value="等待狀態")
        self._second_step_message_var = tk.StringVar(value="尚未收到後端訊息")
        self._second_step_elapsed_var = tk.StringVar(value="第二步經過時間：0.0 秒")
        self._second_step_height_var = tk.StringVar(value="升降機高度：等待資料")
        self._second_step_detected_height_var = tk.StringVar(
            value="影像辨識原始高度：等待資料"
        )
        self._second_step_target_height_var = tk.StringVar(
            value="升降目標 D500：等待資料"
        )
        self._second_step_history: tk.Listbox | None = None
        self._second_step_progressbar: ttk.Progressbar | None = None
        self._second_step_cancel_button: ttk.Button | None = None
        self._control_result_photo: tk.PhotoImage | None = None
        self._control_result_mtime_ns: int | None = None
        self.control_result_status_var = tk.StringVar(
            value=f"等待辨識結果：{CONTROL_RESULT_PATH.name}"
        )

        self.status_var = tk.StringVar(value="未連線")

        self._build_widgets()
        self._load_groups()
        self.after(50, self._drain_ui_events)
        self._refresh_control_result_image(force=True)
        self.after(CONTROL_RESULT_POLL_INTERVAL_MS, self._poll_control_result_image)

    # ===== UI =====

    def _post_ui(self, callback: Callable[[], None]) -> None:
        """Safely enqueue a Tk callback from a background worker."""

        self._ui_events.put(callback)

    def _drain_ui_events(self) -> None:
        """Run worker results on Tk's main thread."""

        while True:
            try:
                callback = self._ui_events.get_nowait()
            except queue.Empty:
                break
            try:
                callback()
            except Exception as exc:  # noqa: BLE001
                self.report_callback_exception(type(exc), exc, exc.__traceback__)
        self.after(50, self._drain_ui_events)

    def _configure_fonts(self) -> None:
        self._refresh_x_font_path()
        available_families = set(tkfont.families(self))
        selected_family = self._select_cjk_family(available_families)

        named_fonts = (
            "TkDefaultFont",
            "TkTextFont",
            "TkMenuFont",
            "TkHeadingFont",
            "TkCaptionFont",
            "TkSmallCaptionFont",
            "TkIconFont",
            "TkTooltipFont",
        )
        for font_name in named_fonts:
            try:
                tkfont.nametofont(font_name).configure(family=selected_family, size=11)
            except tk.TclError:
                continue

        self.default_font = tkfont.Font(
            self,
            name="PlcCjkDefaultFont",
            family=selected_family,
            size=11,
        )
        self.heading_font = tkfont.Font(
            self,
            name="PlcCjkHeadingFont",
            family=selected_family,
            size=11,
            weight="bold",
        )

        if self.default_font.actual("family") == "fixed":
            self.default_font = "taipei16"
            self.heading_font = "taipei24"

        default_font_name = self.default_font.name if isinstance(self.default_font, tkfont.Font) else self.default_font
        heading_font_name = self.heading_font.name if isinstance(self.heading_font, tkfont.Font) else self.heading_font

        self.option_add("*Font", default_font_name)
        self.option_add("*font", default_font_name)
        self.option_add("*Dialog.msg.font", default_font_name)
        self.option_add("*Dialog.dtl.font", default_font_name)
        self.option_add("*Listbox.font", default_font_name)
        self.option_add("*Menu.font", default_font_name)

        style = ttk.Style(self)
        style.configure(".", font=default_font_name)
        style.configure("TButton", font=default_font_name)
        style.configure("TLabel", font=default_font_name)
        style.configure("TEntry", font=default_font_name)
        style.configure("TFrame", font=default_font_name)
        style.configure("TLabelframe", font=default_font_name)
        style.configure("TLabelframe.Label", font=default_font_name)
        style.configure("Treeview", font=default_font_name, rowheight=28)
        style.configure("Treeview.Heading", font=heading_font_name)

        print(
            f"[Tk font] selected={selected_family!r} "
            f"default={self._font_debug_name(self.default_font)} "
            f"heading={self._font_debug_name(self.heading_font)}",
            file=sys.stderr,
            flush=True,
        )

    def _refresh_x_font_path(self) -> None:
        if not os.environ.get("DISPLAY"):
            return
        for command in (["xset", "+fp", "/usr/share/fonts/X11/misc"], ["xset", "fp", "rehash"]):
            try:
                subprocess.run(
                    command,
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2,
                )
            except (OSError, subprocess.SubprocessError):
                pass

    @staticmethod
    def _font_debug_name(font: tkfont.Font | str) -> str:
        if isinstance(font, tkfont.Font):
            return font.actual("family")
        return font

    def _select_cjk_family(self, available_families: set[str]) -> str:
        for family in CJK_FONT_CANDIDATES:
            if family in available_families and self._fontconfig_matches(family):
                return family
        return self._fontconfig_cjk_family() or tkfont.nametofont("TkDefaultFont").actual("family")

    @staticmethod
    def _fontconfig_cjk_family() -> str | None:
        for family in CJK_FONT_CANDIDATES:
            if MainWindow._fontconfig_matches(family):
                return family

        return None

    @staticmethod
    def _fontconfig_matches(family: str) -> bool:
        aliases = {
            "microsoft jhenghei ui": {"microsoft jhenghei"},
            "microsoft jhenghei": {"microsoft jhenghei ui"},
            "source han sans tw": {"source han sans tc"},
            "source han sans tc": {"source han sans tw"},
        }
        try:
            result = subprocess.run(
                ["fc-match", "--format=%{family}", f"{family}:lang=zh-tw"],
                check=False,
                capture_output=True,
                text=True,
                timeout=1,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False

        matched_families = {
            name.strip().casefold()
            for name in result.stdout.split(",")
            if name.strip()
        }
        expected_families = {family.casefold()}
        expected_families.update(name.casefold() for name in aliases.get(family.casefold(), set()))
        return bool(matched_families & expected_families)

    def _build_widgets(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        top_bar = ttk.Frame(self, padding=(10, 8))
        top_bar.grid(row=0, column=0, sticky="ew")
        top_bar.columnconfigure(1, weight=1)

        ttk.Label(top_bar, textvariable=self.status_var).grid(row=0, column=0, sticky="w")

        button_frame = ttk.Frame(top_bar)
        button_frame.grid(row=0, column=2, sticky="e")
        self.btn_connect = ttk.Button(button_frame, text="連線", command=self.on_connect_clicked)
        self.btn_disconnect = ttk.Button(
            button_frame,
            text="斷線",
            command=self.on_disconnect_clicked,
            state=tk.DISABLED,
        )
        self.btn_connect.grid(row=0, column=0, padx=(0, 8))
        self.btn_disconnect.grid(row=0, column=1)

        flow_frame = ttk.Frame(top_bar)
        flow_frame.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        self.btn_home_flow = ttk.Button(
            flow_frame,
            text="一鍵回原點",
            command=self.on_home_flow_clicked,
            state=tk.DISABLED,
        )
        self.btn_vision_flow = ttk.Button(
            flow_frame,
            text="視覺高度 560mm",
            command=self.on_vision_height_flow_clicked,
            state=tk.DISABLED,
        )
        self.btn_confirm_arm_home = ttk.Button(
            flow_frame,
            text="確認手臂／Camera HOME",
            command=self.on_confirm_arm_home_clicked,
            state=tk.DISABLED,
        )
        self.btn_camera_home = ttk.Button(
            flow_frame,
            text="Camera 回 HOME",
            command=lambda: self.on_move_component_home_clicked("camera"),
            state=tk.DISABLED,
        )
        self.btn_arm_home = ttk.Button(
            flow_frame,
            text="手臂回 HOME",
            command=lambda: self.on_move_component_home_clicked("arm"),
            state=tk.DISABLED,
        )
        self.btn_cancel_flow = ttk.Button(
            flow_frame,
            text="取消流程",
            command=self.on_cancel_flow_clicked,
            state=tk.DISABLED,
        )
        self.btn_home_flow.grid(row=0, column=0, padx=(0, 8))
        self.btn_vision_flow.grid(row=0, column=1, padx=(0, 8))
        self.btn_camera_home.grid(row=0, column=2, padx=(0, 8))
        self.btn_arm_home.grid(row=0, column=3, padx=(0, 8))
        self.btn_confirm_arm_home.grid(row=0, column=4, padx=(0, 8))
        self.btn_cancel_flow.grid(row=0, column=5, padx=(0, 8))
        ttk.Label(flow_frame, textvariable=self.arm_home_confirm_var).grid(
            row=0,
            column=6,
            sticky="w",
        )

        vacuum_frame = ttk.LabelFrame(
            top_bar,
            text="真空／破真空快捷控制",
            padding=(8, 6),
        )
        vacuum_frame.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        vacuum_frame.columnconfigure(4, weight=1)
        side_rows = (
            ("Y1", "Y1（真空 M50／破真空 M51）"),
            ("Y2", "Y2（真空 M52／破真空 M53）"),
        )
        for row, (side, label) in enumerate(side_rows):
            ttk.Label(vacuum_frame, text=label, width=28).grid(
                row=row,
                column=0,
                sticky="w",
            )
            for column, (button_label, mode) in enumerate(
                (("開啟真空", "vacuum"), ("開啟破真空", "break_vacuum"), ("全部關閉", "off")),
                start=1,
            ):
                button = ttk.Button(
                    vacuum_frame,
                    text=button_label,
                    command=lambda selected_side=side, selected_mode=mode: self._set_side_vacuum(
                        selected_side,
                        selected_mode,
                    ),
                    state=tk.DISABLED,
                )
                button.grid(row=row, column=column, padx=(0, 8), pady=(0, 4))
                self.vacuum_controls.append(button)
            ttk.Label(vacuum_frame, textvariable=self.side_vacuum_vars[side]).grid(
                row=row,
                column=4,
                sticky="w",
            )

        ttk.Label(
            vacuum_frame,
            text="中間（真空 M54／破真空 M56）",
            width=28,
        ).grid(row=2, column=0, sticky="w")
        for column, (button_label, mode) in enumerate(
            (("開啟真空", "vacuum"), ("開啟破真空", "break_vacuum"), ("全部關閉", "off")),
            start=1,
        ):
            button = ttk.Button(
                vacuum_frame,
                text=button_label,
                command=lambda selected_mode=mode: self._set_middle_vacuum(selected_mode),
                state=tk.DISABLED,
            )
            button.grid(row=2, column=column, padx=(0, 8))
            self.vacuum_controls.append(button)
        ttk.Label(vacuum_frame, textvariable=self.middle_vacuum_var).grid(
            row=2,
            column=4,
            sticky="w",
        )
        self.btn_vacuum_refresh = ttk.Button(
            vacuum_frame,
            text="重新讀取全部狀態",
            command=self._refresh_all_vacuum,
            state=tk.DISABLED,
        )
        self.btn_vacuum_refresh.grid(row=0, column=5, rowspan=3, padx=(12, 0))

        cycle_frame = ttk.LabelFrame(
            top_bar,
            text="完整流程與三個獨立子流程",
            padding=(8, 6),
        )
        cycle_frame.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        cycle_frame.columnconfigure(0, weight=1)
        cycle_frame.columnconfigure(1, weight=1)
        cycle_frame.columnconfigure(2, weight=1)

        first_frame = ttk.LabelFrame(cycle_frame, text="① 第一步：貨盤動作", padding=(8, 6))
        first_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        ttk.Label(first_frame, text="貨盤").grid(row=0, column=0, sticky="e")
        first_slot_combo = ttk.Combobox(
            first_frame,
            textvariable=self.main_cycle_first_slot_var,
            values=("Y1", "Y2", "none"),
            width=6,
            state=tk.DISABLED,
        )
        first_slot_combo.grid(row=0, column=1, padx=(4, 8))
        ttk.Label(first_frame, text="動作").grid(row=0, column=2, sticky="e")
        first_action_combo = ttk.Combobox(
            first_frame,
            textvariable=self.main_cycle_first_action_var,
            values=("吸", "推", "none"),
            width=6,
            state=tk.DISABLED,
        )
        first_action_combo.grid(row=0, column=3, padx=(4, 0))
        ttk.Label(first_frame, text="高度 D500").grid(row=1, column=0, sticky="e", pady=(6, 0))
        first_height_entry = ttk.Entry(
            first_frame,
            textvariable=self.main_cycle_first_height_var,
            width=8,
            state=tk.DISABLED,
        )
        first_height_entry.grid(row=1, column=1, padx=(4, 8), pady=(6, 0))
        ttk.Label(first_frame, text="前進距離").grid(row=1, column=2, sticky="e", pady=(6, 0))
        first_forward_entry = ttk.Entry(
            first_frame,
            textvariable=self.main_cycle_first_forward_var,
            width=8,
            state=tk.DISABLED,
        )
        first_forward_entry.grid(row=1, column=3, padx=(4, 0), pady=(6, 0))
        ttk.Label(
            first_frame,
            text=(
                "Y1 使用 D510；Y2 使用 D560\n"
                "吸取規則：選 Y1 會持續吸住 Y2；選 Y2 會持續吸住 Y1"
            ),
            wraplength=330,
            justify=tk.LEFT,
        ).grid(row=2, column=0, columnspan=4, sticky="w", pady=(5, 0))
        ttk.Label(
            first_frame,
            text="安全上限：手臂＋Camera HOME 已確認 1450mm；否則 695mm",
            foreground="#9a3412",
            wraplength=330,
            justify=tk.LEFT,
        ).grid(row=3, column=0, columnspan=4, sticky="w", pady=(5, 0))
        self.btn_independent_first_step = ttk.Button(
            first_frame,
            text="獨立執行第一步",
            command=lambda: self.on_independent_main_cycle_step_clicked("first"),
            state=tk.DISABLED,
        )
        self.btn_independent_first_step.grid(
            row=4,
            column=0,
            columnspan=4,
            sticky="ew",
            pady=(8, 0),
        )

        second_frame = ttk.LabelFrame(cycle_frame, text="② 第二步：視覺＋手臂取放", padding=(8, 6))
        second_frame.grid(row=0, column=1, sticky="nsew", padx=6)
        ttk.Label(
            second_frame,
            text="選擇搬運方向後，自動執行對應 Camera 視角、\nCANBus 手臂取放及 M54／M56 真空交接。",
            justify=tk.LEFT,
        ).grid(row=0, column=0, sticky="w")
        direction_frame = ttk.LabelFrame(
            second_frame,
            text="搬運方向",
            padding=(8, 5),
        )
        direction_frame.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        for column, (value, label) in enumerate(
            (
                ("Y1_TO_Y2", "Y1 → Y2\nRView 右取左放"),
                ("Y2_TO_Y1", "Y2 → Y1\nLView 左取右放"),
            )
        ):
            direction_button = ttk.Radiobutton(
                direction_frame,
                text=label,
                value=value,
                variable=self.main_cycle_transfer_direction_var,
                state=tk.DISABLED,
            )
            direction_button.grid(
                row=0,
                column=column,
                sticky="w",
                padx=(0, 12) if column == 0 else 0,
            )
            self.main_cycle_second_controls.append(direction_button)
        self.btn_second_step_progress = ttk.Button(
            second_frame,
            text="查看第二步進度",
            command=self._show_second_step_progress,
            state=tk.DISABLED,
        )
        height_result_frame = ttk.LabelFrame(
            second_frame,
            text="影像辨識／升降高度",
            padding=(8, 5),
        )
        height_result_frame.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        ttk.Label(
            height_result_frame,
            textvariable=self._second_step_detected_height_var,
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            height_result_frame,
            textvariable=self._second_step_target_height_var,
            foreground="#075985",
        ).grid(row=1, column=0, sticky="w", pady=(3, 0))
        self.btn_second_step_progress.grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Label(
            second_frame,
            text="安全上限：固定 695mm（任何設定或辨識結果皆不可超過）",
            foreground="#9a3412",
            wraplength=300,
            justify=tk.LEFT,
        ).grid(row=4, column=0, sticky="w", pady=(7, 0))
        self.btn_independent_second_step = ttk.Button(
            second_frame,
            text="獨立執行第二步",
            command=lambda: self.on_independent_main_cycle_step_clicked("second"),
            state=tk.DISABLED,
        )
        self.btn_independent_second_step.grid(row=5, column=0, sticky="ew", pady=(8, 0))

        final_frame = ttk.LabelFrame(cycle_frame, text="③ 第三步：貨盤動作", padding=(8, 6))
        final_frame.grid(row=0, column=2, sticky="nsew", padx=(6, 0))
        ttk.Label(final_frame, text="貨盤").grid(row=0, column=0, sticky="e")
        final_slot_combo = ttk.Combobox(
            final_frame,
            textvariable=self.main_cycle_final_slot_var,
            values=("Y1", "Y2", "none"),
            width=6,
            state=tk.DISABLED,
        )
        final_slot_combo.grid(row=0, column=1, padx=(4, 8))
        ttk.Label(final_frame, text="動作").grid(row=0, column=2, sticky="e")
        final_action_combo = ttk.Combobox(
            final_frame,
            textvariable=self.main_cycle_final_action_var,
            values=("吸", "推", "none"),
            width=6,
            state=tk.DISABLED,
        )
        final_action_combo.grid(row=0, column=3, padx=(4, 0))
        ttk.Label(final_frame, text="高度 D500").grid(row=1, column=0, sticky="e", pady=(6, 0))
        final_height_entry = ttk.Entry(
            final_frame,
            textvariable=self.main_cycle_final_height_var,
            width=8,
            state=tk.DISABLED,
        )
        final_height_entry.grid(row=1, column=1, padx=(4, 8), pady=(6, 0))
        ttk.Label(final_frame, text="前進距離").grid(row=1, column=2, sticky="e", pady=(6, 0))
        final_forward_entry = ttk.Entry(
            final_frame,
            textvariable=self.main_cycle_final_forward_var,
            width=8,
            state=tk.DISABLED,
        )
        final_forward_entry.grid(row=1, column=3, padx=(4, 0), pady=(6, 0))
        ttk.Label(
            final_frame,
            text=(
                "Y1 使用 D510；Y2 使用 D560\n"
                "放回規則：選擇該側＋推，才會關閉該側真空並破真空"
            ),
            wraplength=330,
            justify=tk.LEFT,
        ).grid(row=2, column=0, columnspan=4, sticky="w", pady=(5, 0))
        ttk.Label(
            final_frame,
            text="安全上限：第二步結束並確認手臂＋Camera HOME 後 1450mm；否則 695mm",
            foreground="#9a3412",
            wraplength=330,
            justify=tk.LEFT,
        ).grid(row=3, column=0, columnspan=4, sticky="w", pady=(5, 0))
        self.btn_independent_third_step = ttk.Button(
            final_frame,
            text="獨立執行第三步",
            command=lambda: self.on_independent_main_cycle_step_clicked("third"),
            state=tk.DISABLED,
        )
        self.btn_independent_third_step.grid(
            row=4,
            column=0,
            columnspan=4,
            sticky="ew",
            pady=(8, 0),
        )

        actions_frame = ttk.Frame(cycle_frame)
        actions_frame.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        actions_frame.columnconfigure(0, weight=1)
        ttk.Label(
            actions_frame,
            text="完整流程：每一步完成後會跳出視窗，由操作員確認是否執行下一步",
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 5))
        ttk.Label(actions_frame, textvariable=self.main_cycle_phase_var).grid(
            row=1,
            column=0,
            sticky="w",
        )
        self.btn_main_cycle_run = ttk.Button(
            actions_frame,
            textvariable=self.main_cycle_run_text_var,
            command=self.on_main_cycle_run_clicked,
            state=tk.DISABLED,
        )
        self.btn_main_cycle_reset = ttk.Button(
            actions_frame,
            text="重置總流程",
            command=self.on_main_cycle_reset_clicked,
            state=tk.DISABLED,
        )
        self.btn_main_cycle_run.grid(row=1, column=1, padx=(8, 8))
        self.btn_main_cycle_reset.grid(row=1, column=2)

        task_file_frame = ttk.LabelFrame(
            cycle_frame,
            text="TXT task file",
            padding=(8, 6),
        )
        task_file_frame.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        task_file_frame.columnconfigure(1, weight=1)
        self.btn_task_file_load = ttk.Button(
            task_file_frame,
            text="Load TXT",
            command=self.on_load_task_file_clicked,
            state=tk.DISABLED,
        )
        self.btn_task_file_run = ttk.Button(
            task_file_frame,
            text="Run TXT",
            command=self.on_run_task_file_clicked,
            state=tk.DISABLED,
        )
        self.btn_task_file_load.grid(row=0, column=0, padx=(0, 8), sticky="w")
        ttk.Label(task_file_frame, textvariable=self.task_file_path_var).grid(
            row=0,
            column=1,
            sticky="ew",
        )
        self.btn_task_file_run.grid(row=0, column=2, padx=(8, 0), sticky="e")
        task_list_frame = ttk.Frame(task_file_frame)
        task_list_frame.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        task_list_frame.columnconfigure(0, weight=1)
        self.task_file_list = tk.Listbox(task_list_frame, height=4, exportselection=False)
        self.task_file_list.grid(row=0, column=0, sticky="ew")
        task_file_scroll = ttk.Scrollbar(
            task_list_frame,
            orient=tk.VERTICAL,
            command=self.task_file_list.yview,
        )
        task_file_scroll.grid(row=0, column=1, sticky="ns")
        self.task_file_list.configure(yscrollcommand=task_file_scroll.set)
        self.main_cycle_first_controls.extend(
            (
                first_slot_combo,
                first_action_combo,
                first_height_entry,
                first_forward_entry,
                self.btn_independent_first_step,
            )
        )
        self.main_cycle_final_controls.extend(
            (
                final_slot_combo,
                final_action_combo,
                final_height_entry,
                final_forward_entry,
                self.btn_independent_third_step,
            )
        )
        self.main_cycle_second_controls.append(self.btn_independent_second_step)
        self.main_cycle_controls.extend(
            (
                *self.main_cycle_first_controls,
                *self.main_cycle_second_controls,
                *self.main_cycle_final_controls,
                self.btn_main_cycle_run,
                self.btn_main_cycle_reset,
                self.btn_task_file_load,
                self.btn_task_file_run,
            )
        )

        paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        paned.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))

        left_frame = ttk.Frame(paned)
        left_frame.rowconfigure(0, weight=1)
        left_frame.columnconfigure(0, weight=1)
        group_list_font = self.default_font.name if isinstance(self.default_font, tkfont.Font) else self.default_font
        self.group_list = tk.Listbox(left_frame, exportselection=False, font=group_list_font)
        self.group_list.grid(row=0, column=0, sticky="nsew")
        group_scrollbar = ttk.Scrollbar(left_frame, orient=tk.VERTICAL, command=self.group_list.yview)
        group_scrollbar.grid(row=0, column=1, sticky="ns")
        self.group_list.configure(yscrollcommand=group_scrollbar.set)
        self.group_list.bind("<<ListboxSelect>>", self.on_group_changed)

        right_frame = ttk.Frame(paned)
        right_frame.rowconfigure(1, weight=1)
        right_frame.columnconfigure(0, weight=1)

        table_actions = ttk.Frame(right_frame)
        table_actions.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        table_actions.columnconfigure(0, weight=1)
        self.btn_refresh_current_group = ttk.Button(
            table_actions,
            text="重新讀取目前群組",
            command=self.on_refresh_current_group_clicked,
            state=tk.DISABLED,
        )
        self.btn_refresh_current_group.grid(row=0, column=1, sticky="e")

        self.table = ttk.Treeview(
            right_frame,
            columns=self.columns,
            show="headings",
            selectmode="browse",
        )
        for column in self.columns:
            self.table.heading(column, text=self.headers[column])

        self.table.column("id", width=180, anchor=tk.W)
        self.table.column("name", width=220, anchor=tk.W)
        self.table.column("device", width=70, anchor=tk.CENTER)
        self.table.column("address", width=80, anchor=tk.E)
        self.table.column("value", width=140, anchor=tk.CENTER)
        self.table.column("writable", width=80, anchor=tk.CENTER)

        self.table.grid(row=1, column=0, sticky="nsew")
        y_scrollbar = ttk.Scrollbar(right_frame, orient=tk.VERTICAL, command=self.table.yview)
        y_scrollbar.grid(row=1, column=1, sticky="ns")
        x_scrollbar = ttk.Scrollbar(right_frame, orient=tk.HORIZONTAL, command=self.table.xview)
        x_scrollbar.grid(row=2, column=0, sticky="ew")
        self.table.configure(yscrollcommand=y_scrollbar.set, xscrollcommand=x_scrollbar.set)
        self.table.bind("<Double-1>", self.on_table_double_click)

        result_frame = ttk.LabelFrame(
            paned,
            text="最新影像辨識結果",
            padding=(8, 6),
        )
        result_frame.rowconfigure(0, weight=1)
        result_frame.columnconfigure(0, weight=1)
        self.control_result_image_label = ttk.Label(
            result_frame,
            text=f"尚未找到 {CONTROL_RESULT_PATH.name}",
            anchor=tk.CENTER,
            justify=tk.CENTER,
        )
        self.control_result_image_label.grid(row=0, column=0, sticky="nsew")
        ttk.Label(
            result_frame,
            textvariable=self.control_result_status_var,
            anchor=tk.CENTER,
            justify=tk.CENTER,
            wraplength=CONTROL_RESULT_MAX_WIDTH,
        ).grid(row=1, column=0, sticky="ew", pady=(6, 0))
        ttk.Button(
            result_frame,
            text="重新載入圖片",
            command=lambda: self._refresh_control_result_image(force=True),
        ).grid(row=2, column=0, pady=(6, 0))

        paned.add(left_frame, weight=0)
        paned.add(right_frame, weight=1)
        paned.add(result_frame, weight=0)

    def _poll_control_result_image(self) -> None:
        """Refresh the preview when the vision process replaces its PNG output."""

        self._refresh_control_result_image()
        self.after(CONTROL_RESULT_POLL_INTERVAL_MS, self._poll_control_result_image)

    def _refresh_control_result_image(self, *, force: bool = False) -> None:
        try:
            image_stat = CONTROL_RESULT_PATH.stat()
        except FileNotFoundError:
            if force or self._control_result_mtime_ns is not None:
                self._control_result_photo = None
                self._control_result_mtime_ns = None
                self.control_result_image_label.configure(
                    image="",
                    text=f"尚未找到 {CONTROL_RESULT_PATH.name}",
                )
                self.control_result_status_var.set("執行影像辨識後會自動顯示最新結果")
            return
        except OSError as exc:
            self.control_result_status_var.set(f"無法讀取辨識圖片：{exc}")
            return

        if not force and image_stat.st_mtime_ns == self._control_result_mtime_ns:
            return

        try:
            photo = tk.PhotoImage(file=str(CONTROL_RESULT_PATH))
            original_width = photo.width()
            original_height = photo.height()
            subsample = max(
                1,
                math.ceil(original_width / CONTROL_RESULT_MAX_WIDTH),
                math.ceil(original_height / CONTROL_RESULT_MAX_HEIGHT),
            )
            if subsample > 1:
                photo = photo.subsample(subsample, subsample)
        except (OSError, tk.TclError) as exc:
            # cv2.imwrite writes directly to the destination. A polling tick can
            # therefore land while the PNG is incomplete; leave the old preview
            # visible and retry on the next tick.
            self.control_result_status_var.set(f"圖片載入中，稍後自動重試：{exc}")
            return

        self._control_result_photo = photo
        self._control_result_mtime_ns = image_stat.st_mtime_ns
        self.control_result_image_label.configure(image=photo, text="")
        updated_at = time.strftime(
            "%Y-%m-%d %H:%M:%S",
            time.localtime(image_stat.st_mtime),
        )
        self.control_result_status_var.set(
            f"{original_width}×{original_height}｜更新時間 {updated_at}"
        )

    # ===== HTTP 小工具 =====

    def _request(self, method: str, path: str, **kwargs: Any) -> dict[str, Any] | None:
        url = API_BASE_URL + path
        timeout = kwargs.pop("timeout", 5)
        try:
            resp = requests.request(method, url, timeout=timeout, **kwargs)
            resp.raise_for_status()
            return resp.json()
        except requests.HTTPError as exc:
            response = exc.response
            status_code = response.status_code if response is not None else "unknown"
            response_text = response.text if response is not None else ""
            print(
                f"[WRITE/HTTP ERROR] {method} {url} status={status_code} "
                f"payload={kwargs.get('json')} response={response_text}",
                file=sys.stderr,
                flush=True,
            )
            messagebox.showerror(
                "HTTP 錯誤",
                f"請求 {url} 失敗:\n{exc}\n\n{response_text}",
                parent=self,
            )
            return None
        except Exception as exc:  # noqa: BLE001
            print(
                f"[WRITE/HTTP ERROR] {method} {url} payload={kwargs.get('json')} error={exc}",
                file=sys.stderr,
                flush=True,
            )
            messagebox.showerror("HTTP 錯誤", f"請求 {url} 失敗:\n{exc}", parent=self)
            return None

    # ===== PLC 寫入（依 point_id） =====

    def write_point_value(self, point_id: str, value: str) -> bool:
        """呼叫後端 API 以 point_id 寫入數值。"""

        if not self._connected:
            messagebox.showwarning("尚未連線", '請先按上方"連線" 按鈕', parent=self)
            return False

        try:
            payload: dict[str, Any] = {"point_id": point_id, "value": int(value)}
        except ValueError:
            try:
                payload = {"point_id": point_id, "value": float(value)}
            except ValueError:
                payload = {"point_id": point_id, "value": value}

        data = self._request("POST", "/plc/registers/write-by-point", json=payload)
        if not data or not data.get("ok"):
            print(
                f"[WRITE ERROR] point_id={point_id} value={value} response={data}",
                file=sys.stderr,
                flush=True,
            )
            return False

        return True

    # ===== 資料顯示 =====

    def _load_groups(self) -> None:
        groups = sorted({p.group for p in CONFIG_STORE.list_points() if p.group})
        for group in groups:
            self.group_list.insert(tk.END, group)
        if groups:
            self.group_list.selection_set(0)
            self.group_list.activate(0)
            self._show_group(groups[0])

    def _show_group(self, group: str) -> None:
        points = CONFIG_STORE.list_points(group=group)
        self._current_points = points
        self._values = {p.id: "-" for p in points}

        if self._connected:
            self._refresh_point_values(points)

        self._render_points()

    def _refresh_point_values(self, points: List[PointDefinition]) -> None:
        ids = [p.id for p in points]
        if not ids:
            return

        params = [("ids", pid) for pid in ids]
        data = self._request("GET", "/plc/registers/by-points", params=params)
        if not data or not data.get("ok"):
            return

        for item in data.get("data", []):
            pid = str(item.get("point_id"))
            self._values[pid] = str(item.get("value"))

    def _refresh_current_group(self) -> bool:
        if not self._connected:
            return False
        if not self._current_points:
            self.status_var.set("目前沒有可重新讀取的點位")
            return False

        self._refresh_point_values(self._current_points)
        self._render_points()
        self.status_var.set("目前群組已重新讀取")
        return True

    def _refresh_all_vacuum(self) -> None:
        if not self._connected:
            return
        completed = True
        for side in SLOT_SIDES:
            data = self._request("GET", f"/side-vacuum/{side}")
            if data and data.get("ok") and isinstance(data.get("data"), dict):
                self._render_side_vacuum(side, data["data"])
            else:
                self.side_vacuum_vars[side].set("真空：讀取失敗 / 破真空：讀取失敗")
                completed = False
        completed = self._refresh_middle_vacuum() and completed
        self.status_var.set("真空狀態已更新" if completed else "真空狀態僅完成部分更新")

    def _render_side_vacuum(self, side: str, state: dict[str, Any]) -> None:
        vacuum = "ON" if state.get("vacuum_on") is True else "OFF"
        break_vacuum = "ON" if state.get("break_vacuum_on") is True else "OFF"
        self.side_vacuum_vars[side].set(f"真空：{vacuum} / 破真空：{break_vacuum}")

    def _set_side_vacuum(self, side: str, mode: str) -> None:
        if not self._connected:
            messagebox.showwarning("尚未連線", "請先連線 PLC", parent=self)
            return
        if self._active_flow is not None:
            messagebox.showwarning("流程執行中", "請等待目前 Flow 結束後再控制真空", parent=self)
            return
        mode_labels = {
            "vacuum": f"開啟 {side} 真空",
            "break_vacuum": f"開啟 {side} 破真空",
            "off": f"關閉 {side} 全部真空輸出",
        }
        confirm_release = mode in {"break_vacuum", "off"}
        if confirm_release and not messagebox.askyesno(
            f"確認{mode_labels[mode]}",
            f"警告：{mode_labels[mode]}後貨物可能立即掉落。\n\n"
            "請確認貨物已放妥或有安全承接，且手臂已停止。\n"
            "此操作也會解除該側持續真空守護。確定繼續嗎？",
            icon="warning",
            parent=self,
        ):
            return
        self.status_var.set(f"正在{mode_labels[mode]}...")
        data = self._request(
            "PUT",
            f"/side-vacuum/{side}",
            json={"mode": mode, "confirm_release": confirm_release},
        )
        if not data or not data.get("ok") or not isinstance(data.get("data"), dict):
            self.status_var.set(f"{mode_labels[mode]}失敗")
            return
        self._render_side_vacuum(side, data["data"])
        self.status_var.set(f"{mode_labels[mode]}完成")

    def _refresh_middle_vacuum(self) -> bool:
        if not self._connected:
            return False
        data = self._request("GET", "/middle-vacuum")
        if not data or not data.get("ok") or not isinstance(data.get("data"), dict):
            self.middle_vacuum_var.set("M54：讀取失敗 / M56：讀取失敗")
            self.status_var.set("中間真空狀態讀取失敗")
            return False
        self._render_middle_vacuum(data["data"])
        return True

    def _render_middle_vacuum(self, state: dict[str, Any]) -> None:
        vacuum = "ON" if state.get("vacuum_on") is True else "OFF"
        break_vacuum = "ON" if state.get("break_vacuum_on") is True else "OFF"
        mode_labels = {
            "off": "全部關閉",
            "vacuum": "中間真空中",
            "break_vacuum": "中間破真空中",
            "invalid": "異常：兩者同時開啟",
        }
        mode = str(state.get("mode") or "unknown")
        self.middle_vacuum_var.set(
            f"M54：{vacuum} / M56：{break_vacuum}（{mode_labels.get(mode, mode)}）"
        )

    def _set_middle_vacuum(self, mode: str) -> None:
        if not self._connected:
            messagebox.showwarning("尚未連線", "請先連線 PLC", parent=self)
            return
        if self._active_flow is not None:
            messagebox.showwarning("流程執行中", "請等待目前 Flow 結束後再控制中間真空", parent=self)
            return
        mode_labels = {
            "off": "全部關閉",
            "vacuum": "開啟中間真空 M54",
            "break_vacuum": "開啟中間破真空 M56",
        }
        if mode in {"break_vacuum", "off"} and not messagebox.askyesno(
            f"確認{mode_labels[mode]}",
            f"{mode_labels[mode]}可能會使中間位置的貨物掉落。\n"
            "請確認貨物已可安全釋放，是否繼續？",
            icon="warning",
            parent=self,
        ):
            return
        self.status_var.set(f"正在{mode_labels[mode]}...")
        data = self._request("PUT", "/middle-vacuum", json={"mode": mode})
        if not data or not data.get("ok") or not isinstance(data.get("data"), dict):
            self.status_var.set(f"{mode_labels[mode]}失敗")
            return
        self._render_middle_vacuum(data["data"])
        self.status_var.set(f"{mode_labels[mode]}完成")

    def _set_vacuum_controls(self, enabled: bool) -> None:
        enabled = enabled and not self._stop_unconfirmed
        state = tk.NORMAL if enabled else tk.DISABLED
        for control in self.vacuum_controls:
            control.configure(state=state)
        self.btn_vacuum_refresh.configure(state=state)

    def _render_points(self) -> None:
        self.table.delete(*self.table.get_children())
        for point in self._current_points:
            value = self._display_value(point)
            self.table.insert(
                "",
                tk.END,
                iid=point.id,
                values=(
                    point.id,
                    point.name,
                    point.device,
                    point.address,
                    value,
                    "是" if point.writable else "否",
                ),
            )

    def _display_value(self, point: PointDefinition) -> str:
        value = self._values.get(point.id, "-")
        if self._is_bit(point) and value in {"True", "False", "true", "false"}:
            return "ON" if value.lower() == "true" else "OFF"
        return value

    @staticmethod
    def _is_bit(point: PointDefinition) -> bool:
        return point.device.upper() in {"M", "X", "Y"} and point.type == "bit"

    def _get_point(self, point_id: str) -> PointDefinition | None:
        for point in self._current_points:
            if point.id == point_id:
                return point
        return None

    # ===== 事件處理 =====

    def on_group_changed(self, event: tk.Event) -> None:  # noqa: ARG002
        selection = self.group_list.curselection()
        if not selection:
            return
        group = self.group_list.get(selection[0])
        self._show_group(group)

    def on_refresh_current_group_clicked(self) -> None:
        self._refresh_current_group()

    def _set_can_home_controls(self, enabled: bool) -> None:
        enabled = enabled and not self._stop_unconfirmed
        state = tk.NORMAL if enabled else tk.DISABLED
        self.btn_camera_home.configure(state=state)
        self.btn_arm_home.configure(state=state)
        self.btn_confirm_arm_home.configure(state=state)

    def on_connect_clicked(self) -> None:
        resp = self._request("POST", "/plc/connect", params={"plc": DEFAULT_PLC_NAME})
        if not resp or not resp.get("ok"):
            return

        self._connected = True
        data = resp.get("data") if isinstance(resp.get("data"), dict) else {}
        host = data.get("host")
        port = data.get("port")
        if host and port:
            self.status_var.set(f"已連線 ({DEFAULT_PLC_NAME} {host}:{port})")
        else:
            self.status_var.set(f"已連線 ({DEFAULT_PLC_NAME})")
        self.btn_connect.configure(state=tk.DISABLED)
        self.btn_disconnect.configure(state=tk.NORMAL)
        self.btn_refresh_current_group.configure(state=tk.NORMAL)
        launch_state = tk.DISABLED if self._stop_unconfirmed else tk.NORMAL
        self.btn_home_flow.configure(state=launch_state)
        self.btn_vision_flow.configure(state=launch_state)
        self._set_can_home_controls(True)
        self._set_main_cycle_controls(True)
        self._set_vacuum_controls(True)
        self._refresh_all_vacuum()

        selection = self.group_list.curselection()
        if selection:
            self._show_group(self.group_list.get(selection[0]))

    def on_disconnect_clicked(self) -> None:
        if self._active_flow is not None or self._arm_home_confirming:
            messagebox.showwarning("流程執行中", "請先取消目前的 PLC Flow，再斷線", parent=self)
            return
        self._request("POST", "/plc/disconnect", params={"plc": DEFAULT_PLC_NAME})
        self._connected = False
        self._close_main_cycle_continue_gate()
        self.status_var.set("未連線")
        self.btn_connect.configure(state=tk.NORMAL)
        self.btn_disconnect.configure(state=tk.DISABLED)
        self.btn_refresh_current_group.configure(state=tk.DISABLED)
        self.btn_home_flow.configure(state=tk.DISABLED)
        self.btn_vision_flow.configure(state=tk.DISABLED)
        self._set_can_home_controls(False)
        self.btn_cancel_flow.configure(state=tk.DISABLED)
        self._set_main_cycle_controls(False)
        self._set_vacuum_controls(False)

    def on_table_double_click(self, event: tk.Event) -> None:
        row_id = self.table.identify_row(event.y)
        column = self.table.identify_column(event.x)
        if not row_id or column != "#5":
            return

        point = self._get_point(row_id)
        if point is None or not point.writable:
            return

        if self._is_bit(point):
            current = self._values.get(point.id, "False").lower() == "true"
            new_value = "False" if current else "True"
        else:
            current_value = self._values.get(point.id, "")
            new_value = simpledialog.askstring(
                "寫入點位",
                f"{point.name} ({point.id})",
                initialvalue="" if current_value == "-" else current_value,
                parent=self,
            )
            if new_value is None:
                return

        vacuum_release_sides = {
            "Y1_VAC_ON": "Y1",
            "Y2_VAC_ON": "Y2",
        }
        if point.id in vacuum_release_sides and new_value == "False":
            self._set_side_vacuum(vacuum_release_sides[point.id], "off")
            return

        ok = self.write_point_value(point.id, new_value)
        if not ok:
            messagebox.showwarning(
                "寫入失敗",
                f"寫入點位 {point.name} ({point.id}) 失敗，請檢查日誌。",
                parent=self,
            )
            return

        self._values[point.id] = new_value
        self._render_points()

    def on_home_flow_clicked(self) -> None:
        self._start_flow("home", "/flows/home/run", "一鍵回原點")

    def on_vision_height_flow_clicked(self) -> None:
        self._start_flow("vision-height", "/flows/vision-height/run", "視覺高度 560mm")

    def on_move_component_home_clicked(self, component: str) -> None:
        if self._active_flow is not None or self._arm_home_confirming:
            messagebox.showwarning(
                "流程執行中",
                "請等待目前流程結束後再回 HOME",
                parent=self,
            )
            return
        labels = {
            "camera": "Camera（ID144／ID145）",
            "arm": "手臂（ID142／ID143）",
        }
        label = labels[component]
        if not messagebox.askyesno(
            f"確認 {label} 回 HOME",
            f"即將移動 {label} 回到已校正的 HOME 角度。\n\n"
            "請確認移動範圍內無人員與障礙物，升降機與 Y 軸均已停止。\n\n"
            "確定要繼續嗎？",
            icon="warning",
            parent=self,
        ):
            return

        self._arm_home_confirming = True
        self.arm_home_confirm_var.set(f"{label}：正在回 HOME…")
        self.status_var.set(f"正在移動並確認 {label} HOME")
        self._set_can_home_controls(False)
        self.btn_disconnect.configure(state=tk.DISABLED)
        self.btn_home_flow.configure(state=tk.DISABLED)
        self.btn_vision_flow.configure(state=tk.DISABLED)
        self._set_main_cycle_controls(False)
        self._set_vacuum_controls(False)

        def worker() -> None:
            payload: dict[str, Any] | None = None
            error: str | None = None
            try:
                response = requests.post(
                    API_BASE_URL + f"/arm-camera-home/move-{component}",
                    timeout=60,
                )
                response.raise_for_status()
                payload = response.json()
            except Exception as exc:  # noqa: BLE001
                error = str(exc)
            self._post_ui(
                lambda: self._finish_component_home(component, payload, error)
            )

        threading.Thread(
            target=worker,
            daemon=True,
            name=f"{component}-home-move",
        ).start()

    def _finish_component_home(
        self,
        component: str,
        payload: dict[str, Any] | None,
        request_error: str | None,
    ) -> None:
        self._arm_home_confirming = False
        if self._connected and self._active_flow is None:
            self._set_can_home_controls(True)
            self.btn_disconnect.configure(state=tk.NORMAL)
            self.btn_home_flow.configure(state=tk.NORMAL)
            self.btn_vision_flow.configure(state=tk.NORMAL)
            self._set_main_cycle_controls(True)
            self._set_vacuum_controls(True)

        labels = {
            "camera": "Camera（ID144／ID145）",
            "arm": "手臂（ID142／ID143）",
        }
        label = labels[component]
        if request_error is not None:
            self.arm_home_confirm_var.set(f"{label} HOME：請求失敗")
            self.status_var.set(f"{label} HOME 請求失敗：{request_error}")
            messagebox.showerror("回 HOME 失敗", request_error, parent=self)
            return

        data = payload.get("data") if payload and isinstance(payload.get("data"), dict) else {}
        if payload and payload.get("ok"):
            angles = data.get("angles") if isinstance(data.get("angles"), dict) else {}
            angle_text = "、".join(
                f"{motor}={float(value):.2f}°"
                for motor, value in angles.items()
            )
            self.arm_home_confirm_var.set(
                f"{label} HOME 已確認；四軸尚待確認（上限 695mm）"
            )
            self.status_var.set(f"{label} HOME 完成：{angle_text}")
            messagebox.showinfo(
                "回 HOME 完成",
                f"{label} 已連續穩定讀回 HOME。\n\n{angle_text}\n\n"
                "若要解鎖 1450mm，請再執行「確認手臂／Camera HOME」。",
                parent=self,
            )
            return

        reason = str((payload or {}).get("error") or f"{label} HOME 未完成")
        self.arm_home_confirm_var.set(f"{label} HOME：失敗（上限 695mm）")
        self.status_var.set(reason)
        messagebox.showwarning("回 HOME 失敗", reason, parent=self)

    def on_confirm_arm_home_clicked(self) -> None:
        if self._active_flow is not None or self._arm_home_confirming:
            messagebox.showwarning(
                "流程執行中",
                "請等待目前流程結束後再確認 HOME",
                parent=self,
            )
            return

        self._arm_home_confirming = True
        self.arm_home_confirm_var.set("手臂／Camera HOME：正在讀取四軸角度…")
        self.status_var.set("正在確認 ID142～ID145 HOME；此動作不會移動手臂")
        self._set_can_home_controls(False)
        self.btn_disconnect.configure(state=tk.DISABLED)
        self.btn_home_flow.configure(state=tk.DISABLED)
        self.btn_vision_flow.configure(state=tk.DISABLED)
        self._set_main_cycle_controls(False)
        self._set_vacuum_controls(False)

        def worker() -> None:
            payload: dict[str, Any] | None = None
            error: str | None = None
            try:
                response = requests.post(
                    API_BASE_URL + "/arm-camera-home/confirm",
                    timeout=30,
                )
                response.raise_for_status()
                payload = response.json()
            except Exception as exc:  # noqa: BLE001
                error = str(exc)
            self._post_ui(
                lambda: self._finish_arm_home_confirmation(payload, error)
            )

        threading.Thread(
            target=worker,
            daemon=True,
            name="arm-camera-home-confirm",
        ).start()

    def _finish_arm_home_confirmation(
        self,
        payload: dict[str, Any] | None,
        request_error: str | None,
        *,
        show_dialog: bool = True,
    ) -> None:
        self._arm_home_confirming = False
        if self._connected and self._active_flow is None:
            self._set_can_home_controls(True)
            self.btn_disconnect.configure(state=tk.NORMAL)
            self.btn_home_flow.configure(state=tk.NORMAL)
            self.btn_vision_flow.configure(state=tk.NORMAL)
            self._set_main_cycle_controls(True)
            self._set_vacuum_controls(True)

        if request_error is not None:
            self.arm_home_confirm_var.set("手臂／Camera HOME：確認請求失敗")
            self.status_var.set(f"HOME 確認請求失敗：{request_error}")
            if show_dialog:
                messagebox.showerror("HOME 確認失敗", request_error, parent=self)
            return

        data = payload.get("data") if payload and isinstance(payload.get("data"), dict) else {}
        if payload and payload.get("ok"):
            angles = data.get("angles") if isinstance(data.get("angles"), dict) else {}
            angle_text = "、".join(
                f"{label}={float(value):.2f}°"
                for label, value in angles.items()
            )
            maximum = float(data.get("home_maximum_height_mm") or 1450.0)
            self.arm_home_confirm_var.set(
                f"手臂／Camera HOME：已確認，可用至 {maximum:g}mm"
            )
            self.status_var.set(f"四軸 HOME 已確認：{angle_text}")
            if show_dialog:
                messagebox.showinfo(
                    "HOME 已確認",
                    f"ID142～ID145 已連續穩定回讀 HOME。\n\n"
                    f"{angle_text}\n\n升降高度上限已恢復為 {maximum:g}mm。",
                    parent=self,
                )
            return

        reason = str(
            (payload or {}).get("error")
            or data.get("reason")
            or "四軸角度未通過 HOME 確認"
        )
        self.arm_home_confirm_var.set("手臂／Camera HOME：未確認（上限 695mm）")
        self.status_var.set(f"HOME 未確認：{reason}")
        if show_dialog:
            messagebox.showwarning("HOME 未確認", reason, parent=self)

    def _update_arm_home_from_main_cycle(
        self,
        payload: dict[str, Any],
    ) -> None:
        data = payload.get("data") if isinstance(payload.get("data"), dict) else {}
        home = (
            data.get("arm_camera_home")
            if isinstance(data.get("arm_camera_home"), dict)
            else {}
        )
        maximum = float(data.get("effective_maximum_height_mm") or 695.0)
        if home.get("all_home_confirmed"):
            self.arm_home_confirm_var.set(
                f"手臂／Camera HOME：已確認，可用至 {maximum:g}mm"
            )
            return
        self.arm_home_confirm_var.set(
            f"手臂／Camera HOME：未確認（上限 {maximum:g}mm）"
        )

    def on_load_task_file_clicked(self) -> None:
        path = filedialog.askopenfilename(
            parent=self,
            title="Load TXT task file",
            filetypes=(("Text files", "*.txt"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            steps = parse_task_file(path)
        except (OSError, TaskFileError) as exc:
            self._task_file_steps = []
            self.task_file_path_var.set("No valid TXT task loaded")
            self.task_file_list.delete(0, tk.END)
            self.btn_task_file_run.configure(state=tk.DISABLED)
            messagebox.showerror("TXT task error", str(exc), parent=self)
            return

        self._task_file_steps = steps
        self._task_file_control_settings = (
            dict(steps[0].values)
            if steps and steps[0].type == "control_settings"
            else {}
        )
        self.task_file_path_var.set(path)
        self.task_file_list.delete(0, tk.END)
        for step in steps:
            self.task_file_list.insert(tk.END, self._task_step_summary(step))
        if self._connected and not self._active_flow and not self._stop_unconfirmed:
            self.btn_task_file_run.configure(state=tk.NORMAL)
        self.status_var.set(f"Loaded TXT task: {len(steps)} steps")

    def on_run_task_file_clicked(self) -> None:
        if not self._connected:
            messagebox.showwarning("TXT task", "Please connect PLC first.", parent=self)
            return
        if self._active_flow is not None:
            messagebox.showwarning("TXT task", "A flow is already running.", parent=self)
            return
        if not self._task_file_steps:
            messagebox.showwarning("TXT task", "Load a TXT task file first.", parent=self)
            return
        preview = "\n".join(
            self._task_step_summary(step) for step in self._task_file_steps[:8]
        )
        if len(self._task_file_steps) > 8:
            preview += f"\n... {len(self._task_file_steps) - 8} more steps"
        if not messagebox.askyesno(
            "Run TXT task",
            "Run these TXT task steps in order?\n\n"
            f"{preview}\n\n"
            "Execution stops on the first error, timeout, or cancellation.",
            icon="warning",
            parent=self,
        ):
            return

        self._task_file_running = True
        self._task_file_index = 0
        self._active_flow = "main-cycle"
        self._active_flow_label = "TXT task"
        self._flow_started_at = time.monotonic()
        self._flow_cancelling = False
        self._flow_cancel_started_at = None
        self._flow_cancel_deadline = None
        self.status_var.set("TXT task started")
        self.btn_home_flow.configure(state=tk.DISABLED)
        self.btn_vision_flow.configure(state=tk.DISABLED)
        self._set_can_home_controls(False)
        self.btn_cancel_flow.configure(state=tk.NORMAL)
        self._set_main_cycle_controls(False)
        self._set_vacuum_controls(False)
        self._update_flow_elapsed()
        self.after(0, self._run_next_task_file_step)

    def _run_next_task_file_step(self) -> None:
        if not self._task_file_running:
            return
        if self._task_file_index >= len(self._task_file_steps):
            self._task_file_running = False
            self._finish_flow(
                "TXT task",
                {
                    "ok": True,
                    "data": {
                        "status": "success",
                        "state": "success",
                        "step": "txt_task",
                        "message": f"TXT task completed: {len(self._task_file_steps)} steps",
                        "elapsed_seconds": (
                            time.monotonic() - self._flow_started_at
                            if self._flow_started_at is not None
                            else 0.0
                        ),
                    },
                },
                None,
                expected_flow="main-cycle",
            )
            self.main_cycle_phase = "ready_first_step"
            self._refresh_main_cycle_phase_ui()
            return

        step = self._task_file_steps[self._task_file_index]
        self.task_file_list.selection_clear(0, tk.END)
        self.task_file_list.selection_set(self._task_file_index)
        self.task_file_list.see(self._task_file_index)
        self.status_var.set(
            f"TXT task step {step.index}/{len(self._task_file_steps)}: "
            f"{self._task_step_summary(step)}"
        )

        if step.type == "wait":
            milliseconds = int(float(step.values["seconds"]) * 1000)
            self.after(milliseconds, self._complete_current_task_file_step)
            return
        if step.type == "control_settings":
            self._complete_current_task_file_step()
            return
        if step.type == "confirm":
            if messagebox.askyesno(
                "TXT task confirm",
                step.values.get("message", f"Confirm step {step.index}"),
                parent=self,
            ):
                self._complete_current_task_file_step()
            else:
                self._finish_task_file_error(
                    f"TXT task cancelled at confirm step {step.index}"
                )
            return

        requests_to_run = self._task_step_requests(step)
        self._run_task_file_request_chain(step, requests_to_run, 0)

    def _complete_current_task_file_step(self) -> None:
        if not self._task_file_running:
            return
        self._task_file_index += 1
        self.after(0, self._run_next_task_file_step)

    def _run_task_file_request_chain(
        self,
        step: TaskStep,
        requests_to_run: list[tuple[str, str, dict[str, Any] | None, float]],
        request_index: int,
    ) -> None:
        if not self._task_file_running:
            return
        if request_index >= len(requests_to_run):
            self._complete_current_task_file_step()
            return

        label, path, json_payload, timeout = requests_to_run[request_index]

        def worker() -> None:
            payload: dict[str, Any] | None = None
            error: str | None = None
            try:
                response = requests.post(
                    API_BASE_URL + path,
                    json=json_payload,
                    timeout=timeout,
                )
                response.raise_for_status()
                payload = response.json()
            except Exception as exc:  # noqa: BLE001
                error = str(exc)
            self._post_ui(
                lambda: self._handle_task_file_request_result(
                    step,
                    requests_to_run,
                    request_index,
                    label,
                    payload,
                    error,
                )
            )

        threading.Thread(
            target=worker,
            daemon=True,
            name=f"txt-task-step-{step.index}-{request_index + 1}",
        ).start()

    def _handle_task_file_request_result(
        self,
        step: TaskStep,
        requests_to_run: list[tuple[str, str, dict[str, Any] | None, float]],
        request_index: int,
        label: str,
        payload: dict[str, Any] | None,
        request_error: str | None,
    ) -> None:
        if not self._task_file_running:
            return
        if payload is not None:
            update_home = getattr(self, "_update_arm_home_from_main_cycle", None)
            if callable(update_home):
                update_home(payload)
        if request_error is not None:
            self._finish_task_file_error(f"{label} request failed: {request_error}")
            return
        if not payload or not payload.get("ok"):
            data = payload.get("data") if payload else {}
            if not isinstance(data, dict):
                data = {}
            message = str(
                (payload or {}).get("error")
                or data.get("message")
                or f"{label} failed"
            )
            self._finish_task_file_error(message, payload)
            return
        self._run_task_file_request_chain(step, requests_to_run, request_index + 1)

    def _finish_task_file_error(
        self,
        message: str,
        payload: dict[str, Any] | None = None,
    ) -> None:
        self._task_file_running = False
        if payload is None or not isinstance(payload.get("data"), dict):
            payload = {
                "ok": False,
                "data": {
                    "status": "error",
                    "state": "error",
                    "step": "txt_task",
                    "message": message,
                    "elapsed_seconds": (
                        time.monotonic() - self._flow_started_at
                        if self._flow_started_at is not None
                        else 0.0
                    ),
                },
                "error": message,
            }
        self._finish_flow(
            "TXT task",
            payload,
            None,
            expected_flow="main-cycle",
        )
        self.main_cycle_phase = "ready_first_step"
        self._refresh_main_cycle_phase_ui()

    def _task_step_requests(
        self,
        step: TaskStep,
    ) -> list[tuple[str, str, dict[str, Any] | None, float]]:
        if step.type == "pallet":
            settings = getattr(self, "_task_file_control_settings", {})
            payload: dict[str, Any] = {
                "slot": step.values["slot"],
                "action": step.values["action"],
                "height_mm": float(step.values["height_mm"]),
                "forward_mm": (
                    None
                    if step.values.get("forward_mm") in {None, ""}
                    else float(step.values["forward_mm"])
                ),
            }
            if settings:
                payload["x_speed"] = int(settings["x_speed"])
                payload["y1_speed"] = int(settings["y1_speed"])
            return [
                (
                    f"TXT step {step.index} pallet",
                    "/flows/main-cycle/independent/first-step",
                    payload,
                    MAIN_CYCLE_STEP_REQUEST_TIMEOUT_SECONDS,
                )
            ]
        if step.type == "vision_transfer":
            repeat = int(step.values.get("repeat", "1"))
            settings = getattr(self, "_task_file_control_settings", {})
            payload = {
                "transfer_direction": step.values["transfer_direction"],
            }
            if settings:
                payload["x_speed"] = int(settings["x_speed"])
                payload["height_reference_depth_mm"] = float(
                    settings["height_reference_depth_mm"]
                )
            return [
                (
                    f"TXT step {step.index} vision_transfer repeat "
                    f"{iteration}/{repeat}",
                    "/flows/main-cycle/independent/second-step",
                    payload,
                    MAIN_CYCLE_SECOND_STEP_REQUEST_TIMEOUT_SECONDS,
                )
                for iteration in range(1, repeat + 1)
            ]
        if step.type == "home":
            target = step.values["target"]
            requests_to_run: list[tuple[str, str, dict[str, Any] | None, float]] = []
            if target == "plc":
                requests_to_run.append(
                    (
                        f"TXT step {step.index} PLC home",
                        "/flows/home/run",
                        None,
                        HOME_FLOW_REQUEST_TIMEOUT_SECONDS,
                    )
                )
            if target == "arm":
                requests_to_run.append(
                    (
                        f"TXT step {step.index} arm home",
                        "/arm-camera-home/move-arm",
                        None,
                        60,
                    )
                )
            if target == "camera":
                requests_to_run.append(
                    (
                        f"TXT step {step.index} camera home",
                        "/arm-camera-home/move-camera",
                        None,
                        60,
                    )
                )
            return requests_to_run
        if step.type == "arm_pose":
            pose = step.values["pose"]
            if pose in {"HOME", "STANDBY"}:
                return [
                    (
                        f"TXT step {step.index} arm pose {pose}",
                        "/arm-camera-pose/move",
                        {"pose": pose},
                        60,
                    )
                ]
            raise ValueError(f"Unsupported arm pose: {pose}")
        raise ValueError(f"Unsupported executable task type: {step.type}")

    @staticmethod
    def _task_step_summary(step: TaskStep) -> str:
        if step.type == "control_settings":
            return (
                f"{step.index}. control_settings "
                f"X速度={step.values.get('x_speed')} "
                f"Y1速度={step.values.get('y1_speed')} "
                f"深度基準={step.values.get('height_reference_depth_mm')}mm"
            )
        if step.type == "pallet":
            forward = step.values.get("forward_mm", "")
            return (
                f"{step.index}. pallet "
                f"slot={step.values.get('slot')} action={step.values.get('action')} "
                f"height={step.values.get('height_mm')} forward={forward}"
            )
        if step.type == "vision_transfer":
            return (
                f"{step.index}. vision_transfer "
                f"direction={step.values.get('transfer_direction')} "
                f"repeat={step.values.get('repeat', '1')}"
            )
        if step.type == "home":
            return f"{step.index}. home target={step.values.get('target')}"
        if step.type == "wait":
            return f"{step.index}. wait seconds={step.values.get('seconds')}"
        if step.type == "confirm":
            return f"{step.index}. confirm {step.values.get('message', '')}"
        if step.type == "arm_pose":
            return f"{step.index}. arm_pose pose={step.values.get('pose')}"
        return f"{step.index}. {step.type}"

    def on_main_cycle_run_clicked(self) -> None:
        if self.main_cycle_phase == "ready_first_step":
            self._start_main_cycle_sequence()
        elif self.main_cycle_phase in {"waiting_step2", "waiting_final_step1"}:
            self._show_main_cycle_continue_gate(self.main_cycle_phase)
        elif self.main_cycle_phase == "complete":
            messagebox.showinfo("總流程已完成", "請先重置總流程，再開始下一輪。", parent=self)
        else:
            messagebox.showwarning("總流程階段不正確", f"目前階段：{self.main_cycle_phase}", parent=self)

    def on_independent_main_cycle_step_clicked(self, step_name: str) -> None:
        if not self._connected:
            messagebox.showwarning("尚未連線", "請先連線 PLC", parent=self)
            return
        if self._active_flow is not None:
            messagebox.showwarning(
                "流程執行中",
                "請先等待目前步驟完成，或按下安全停止後再切換子流程。",
                parent=self,
            )
            return

        step_labels = {
            "first": "第一步",
            "second": "第二步",
            "third": "第三步",
        }
        if step_name not in step_labels:
            raise ValueError(f"未知的獨立子流程：{step_name}")
        label = step_labels[step_name]

        if self.main_cycle_phase != "ready_first_step":
            if not messagebox.askyesno(
                "切換到獨立子流程",
                "目前完整流程尚未重置。\n\n"
                f"切換到獨立{label}會結束完整流程的等待狀態，之後若要重新執行完整流程，"
                "會再從第一步開始。\n\n確定要繼續嗎？",
                icon="warning",
                parent=self,
            ):
                return
            reset_payload = self._request("POST", "/flows/main-cycle/reset")
            if not reset_payload or not reset_payload.get("ok"):
                return
            self._close_main_cycle_continue_gate()
            self.main_cycle_phase = "ready_first_step"
            self._refresh_main_cycle_phase_ui()

        request_payload: dict[str, Any]
        if step_name == "first":
            parsed = self._main_cycle_payload(
                step_label="獨立第一步",
                slot_var=self.main_cycle_first_slot_var,
                action_var=self.main_cycle_first_action_var,
                height_var=self.main_cycle_first_height_var,
                forward_var=self.main_cycle_first_forward_var,
            )
            if parsed is None:
                return
            request_payload = parsed
        elif step_name == "second":
            direction = self.main_cycle_transfer_direction_var.get().strip()
            if direction not in {"Y1_TO_Y2", "Y2_TO_Y1"}:
                messagebox.showerror(
                    "子流程輸入錯誤",
                    "第二步搬運方向必須是 Y1 → Y2 或 Y2 → Y1",
                    parent=self,
                )
                return
            request_payload = {"transfer_direction": direction}
        else:
            parsed = self._main_cycle_payload(
                step_label="獨立第三步",
                slot_var=self.main_cycle_final_slot_var,
                action_var=self.main_cycle_final_action_var,
                height_var=self.main_cycle_final_height_var,
                forward_var=self.main_cycle_final_forward_var,
            )
            if parsed is None:
                return
            request_payload = parsed

        if not messagebox.askyesno(
            f"確認獨立執行{label}",
            f"即將單獨執行{label}，完成後不會自動進入其他步驟。\n\n"
            "請確認設備移動範圍內無人員或障礙物。\n\n確定要執行嗎？",
            icon="warning",
            parent=self,
        ):
            return
        self._start_independent_main_cycle_step(step_name, label, request_payload)

    def on_main_cycle_reset_clicked(self) -> None:
        if not self._connected:
            messagebox.showwarning("尚未連線", "請先連線 PLC", parent=self)
            return
        if self._active_flow is not None:
            messagebox.showwarning("流程執行中", "請等待目前 Flow 結束後再重置", parent=self)
            return
        data = self._request("POST", "/flows/main-cycle/reset")
        if not data or not data.get("ok"):
            return
        self._close_main_cycle_continue_gate()
        self.main_cycle_phase = "ready_first_step"
        self._refresh_main_cycle_phase_ui()
        self.status_var.set("總流程已重置")

    def _start_independent_main_cycle_step(
        self,
        step_name: str,
        label: str,
        request_payload: dict[str, Any],
    ) -> None:
        self._active_flow = "main-cycle"
        self._active_flow_label = f"獨立{label}"
        self._flow_started_at = time.monotonic()
        self._flow_cancelling = False
        self._flow_cancel_started_at = None
        self._flow_cancel_deadline = None
        self.status_var.set(f"獨立{label}已開始")
        self.main_cycle_phase_var.set(f"{label}子流程獨立執行中")
        self.btn_home_flow.configure(state=tk.DISABLED)
        self.btn_vision_flow.configure(state=tk.DISABLED)
        self._set_can_home_controls(False)
        self.btn_cancel_flow.configure(state=tk.NORMAL)
        self._set_main_cycle_controls(False)
        self._set_vacuum_controls(False)
        self._update_flow_elapsed()
        if step_name == "second":
            self._begin_second_step_progress()

        endpoint = f"/flows/main-cycle/independent/{step_name}-step"
        timeout = (
            MAIN_CYCLE_SECOND_STEP_REQUEST_TIMEOUT_SECONDS
            if step_name == "second"
            else MAIN_CYCLE_STEP_REQUEST_TIMEOUT_SECONDS
        )

        def worker() -> None:
            payload: dict[str, Any] | None = None
            error: str | None = None
            try:
                response = requests.post(
                    API_BASE_URL + endpoint,
                    json=request_payload,
                    timeout=timeout,
                )
                response.raise_for_status()
                payload = response.json()
            except Exception as exc:  # noqa: BLE001
                error = str(exc)
            self._post_ui(
                lambda result_payload=payload, request_error=error: (
                    self._finish_independent_main_cycle_step(
                        step_name,
                        label,
                        result_payload,
                        request_error,
                    )
                )
            )

        threading.Thread(
            target=worker,
            daemon=True,
            name=f"plc-flow-independent-{step_name}",
        ).start()

    def _finish_independent_main_cycle_step(
        self,
        step_name: str,
        label: str,
        payload: dict[str, Any] | None,
        request_error: str | None,
    ) -> None:
        update_home = getattr(self, "_update_arm_home_from_main_cycle", None)
        if payload is not None and callable(update_home):
            update_home(payload)
        self._finish_flow(
            f"獨立{label}",
            payload,
            request_error,
            expected_flow="main-cycle",
        )
        self.main_cycle_phase = "ready_first_step"
        self._refresh_main_cycle_phase_ui()

    def on_cancel_flow_clicked(self) -> None:
        if self._active_flow is None or self._flow_cancelling:
            return
        self._task_file_running = False
        active_flow = self._active_flow
        label = self._active_flow_label or "PLC Flow"
        self._flow_cancelling = True
        self._flow_cancel_started_at = time.monotonic()
        self._flow_cancel_deadline = (
            self._flow_cancel_started_at + CANCEL_STATUS_CONFIRM_TIMEOUT_SECONDS
        )
        self.status_var.set(f"正在取消 {label}，請等待設備停止...")
        self.btn_cancel_flow.configure(state=tk.DISABLED)
        if self._second_step_active:
            self._second_step_state_var.set("正在取消，等待設備停止")
            if self._second_step_cancel_button is not None:
                self._second_step_cancel_button.configure(state=tk.DISABLED)

        def worker() -> None:
            try:
                response = requests.post(
                    f"{API_BASE_URL}/flows/{active_flow}/cancel",
                    timeout=5,
                )
                response.raise_for_status()
                payload = response.json()
                self._post_ui(
                    lambda: self._handle_cancel_response(active_flow, label, payload),
                )
            except Exception as exc:  # noqa: BLE001
                self._post_ui(
                    lambda error=str(exc): self._handle_cancel_error(active_flow, label, error),
                )

        threading.Thread(target=worker, daemon=True, name="plc-flow-cancel").start()

    def _handle_cancel_response(
        self,
        flow_name: str,
        label: str,
        payload: dict[str, Any],
    ) -> None:
        if self._active_flow != flow_name:
            return
        data = payload.get("data") if isinstance(payload.get("data"), dict) else {}
        snapshot = data.get("flow") if isinstance(data.get("flow"), dict) else {}
        if data.get("already_finished") and self._finish_flow_from_snapshot(
            flow_name,
            label,
            snapshot,
        ):
            return
        self._schedule_cancel_status_poll(flow_name, label, delay_ms=0)

    def _handle_cancel_error(self, flow_name: str, label: str, error: str) -> None:
        if self._active_flow != flow_name:
            return
        self._flow_cancelling = False
        self._flow_cancel_started_at = None
        self._flow_cancel_deadline = None
        self.btn_cancel_flow.configure(state=tk.NORMAL)
        if self._second_step_active:
            self._second_step_state_var.set("執行中（取消請求失敗）")
            if self._second_step_cancel_button is not None:
                self._second_step_cancel_button.configure(state=tk.NORMAL)
        self.status_var.set(f"{label}取消請求失敗，流程狀態仍在確認中")
        messagebox.showerror("取消失敗", error, parent=self)

    def _schedule_cancel_status_poll(
        self,
        flow_name: str,
        label: str,
        *,
        delay_ms: int = CANCEL_STATUS_POLL_INTERVAL_MS,
    ) -> None:
        if self._active_flow != flow_name or not self._flow_cancelling:
            return
        self._stop_cancel_status_poll()
        self._flow_cancel_poll_id = self.after(
            delay_ms,
            lambda: self._poll_cancelled_flow(flow_name, label),
        )

    def _poll_cancelled_flow(self, flow_name: str, label: str) -> None:
        self._flow_cancel_poll_id = None
        if self._active_flow != flow_name or not self._flow_cancelling:
            return
        if (
            self._flow_cancel_deadline is not None
            and time.monotonic() >= self._flow_cancel_deadline
        ):
            self._stop_flow_timer()
            self.status_var.set(
                f"{label}取消已送出，但未確認設備停止；請檢查設備並依現場 SOP 停止"
            )
            messagebox.showwarning(
                "尚未確認設備停止",
                "取消請求已送出，但後端未在 10 秒內回報終止狀態。\n"
                "請勿繼續操作，並依現場 SOP 確認或停止設備。",
                parent=self,
            )
            return
        if self._flow_cancel_poll_in_flight:
            self._schedule_cancel_status_poll(flow_name, label)
            return
        self._flow_cancel_poll_in_flight = True

        def worker() -> None:
            snapshot: dict[str, Any] | None = None
            try:
                response = requests.get(f"{API_BASE_URL}/lifecycle/status", timeout=3)
                response.raise_for_status()
                payload = response.json()
                data = payload.get("data") if isinstance(payload.get("data"), dict) else {}
                component = FLOW_COMPONENT_NAMES.get(flow_name)
                item = data.get(component) if component else None
                if isinstance(item, dict):
                    snapshot = item
            except Exception:  # noqa: BLE001
                snapshot = None
            self._post_ui(
                lambda: self._handle_cancel_status(flow_name, label, snapshot),
            )

        threading.Thread(
            target=worker,
            daemon=True,
            name=f"plc-flow-{flow_name}-cancel-status",
        ).start()

    def _handle_cancel_status(
        self,
        flow_name: str,
        label: str,
        snapshot: dict[str, Any] | None,
    ) -> None:
        self._flow_cancel_poll_in_flight = False
        if self._active_flow != flow_name or not self._flow_cancelling:
            return
        if snapshot is not None and self._finish_flow_from_snapshot(
            flow_name,
            label,
            snapshot,
        ):
            return
        self._schedule_cancel_status_poll(flow_name, label)

    def _finish_flow_from_snapshot(
        self,
        flow_name: str,
        label: str,
        snapshot: dict[str, Any],
    ) -> bool:
        lifecycle_status = str(snapshot.get("status") or "")
        if lifecycle_status not in FLOW_TERMINAL_STATUSES:
            return False
        snapshot_data = (
            snapshot.get("data") if isinstance(snapshot.get("data"), dict) else {}
        )
        data = {
            **snapshot_data,
            "status": lifecycle_status,
            "state": str(snapshot_data.get("state") or lifecycle_status),
            "step": str(snapshot.get("step") or ""),
            "message": str(snapshot.get("message") or "流程已結束"),
        }
        if self._flow_started_at is not None:
            data.setdefault(
                "elapsed_seconds",
                time.monotonic() - self._flow_started_at,
            )
        return self._finish_flow(
            label,
            {
                "ok": lifecycle_status == "success",
                "data": data,
                "error": None if lifecycle_status == "success" else data["message"],
            },
            None,
            expected_flow=flow_name,
        )

    def _start_flow(self, flow_name: str, path: str, label: str) -> None:
        if not self._connected:
            messagebox.showwarning("尚未連線", "請先連線 PLC", parent=self)
            return
        if self._active_flow is not None:
            messagebox.showwarning("流程執行中", "已有 PLC Flow 執行中", parent=self)
            return

        self._active_flow = flow_name
        self._active_flow_label = label
        self._flow_started_at = time.monotonic()
        self._flow_cancelling = False
        self._flow_cancel_started_at = None
        self._flow_cancel_deadline = None
        self.status_var.set(f"{label}已開始，正在執行前置檢查...")
        self.btn_home_flow.configure(state=tk.DISABLED)
        self.btn_vision_flow.configure(state=tk.DISABLED)
        self._set_can_home_controls(False)
        self.btn_cancel_flow.configure(state=tk.NORMAL)
        self._set_main_cycle_controls(False)
        self._set_vacuum_controls(False)
        self._update_flow_elapsed()

        def worker() -> None:
            try:
                request_timeout = (
                    HOME_FLOW_REQUEST_TIMEOUT_SECONDS if flow_name == "home" else 75
                )
                response = requests.post(API_BASE_URL + path, timeout=request_timeout)
                response.raise_for_status()
                payload = response.json()
                self._post_ui(
                    lambda: self._finish_flow(
                        label,
                        payload,
                        None,
                        expected_flow=flow_name,
                    ),
                )
            except Exception as exc:  # noqa: BLE001
                self._post_ui(
                    lambda error=str(exc): self._finish_flow(
                        label,
                        None,
                        error,
                        expected_flow=flow_name,
                    ),
                )

        threading.Thread(
            target=worker,
            daemon=True,
            name=f"plc-flow-{flow_name}",
        ).start()

    def _finish_flow(
        self,
        label: str,
        payload: dict[str, Any] | None,
        request_error: str | None,
        *,
        expected_flow: str | None = None,
        show_result_dialog: bool = True,
    ) -> bool:
        if expected_flow is not None and self._active_flow != expected_flow:
            return False
        self._stop_cancel_status_poll()
        self._stop_second_step_progress()
        self._stop_flow_timer()
        self._active_flow = None
        self._active_flow_label = None
        self._flow_started_at = None
        self._flow_cancelling = False
        self._flow_cancel_started_at = None
        self._flow_cancel_deadline = None
        self._flow_cancel_poll_in_flight = False
        self.btn_cancel_flow.configure(state=tk.DISABLED)
        data = payload.get("data") if payload else {}
        if not isinstance(data, dict):
            data = {}
        stop_unconfirmed = str(data.get("state") or "") == "stop_unconfirmed"
        if stop_unconfirmed:
            self._stop_unconfirmed = True
        if self._connected:
            launch_state = tk.DISABLED if self._stop_unconfirmed else tk.NORMAL
            self.btn_home_flow.configure(state=launch_state)
            self.btn_vision_flow.configure(state=launch_state)
            self._set_can_home_controls(not self._stop_unconfirmed)
            self._set_main_cycle_controls(not self._stop_unconfirmed)
            self._set_vacuum_controls(not self._stop_unconfirmed)
            if not self._stop_unconfirmed:
                self._refresh_all_vacuum()

        if request_error is not None:
            self.status_var.set(f"{label}請求失敗")
            if show_result_dialog:
                messagebox.showerror(f"{label}失敗", request_error, parent=self)
            return True

        message = str(data.get("message") or payload.get("error") or "流程沒有回傳訊息")
        succeeded = bool(payload and payload.get("ok"))
        details = self._flow_result_details(data, message)
        if stop_unconfirmed:
            self.status_var.set(f"{label}停止未確認：禁止繼續操作")
            if show_result_dialog:
                messagebox.showerror(
                    "設備停止未確認",
                    details
                    + "\n\n系統已鎖定動作控制。請勿繼續操作，並依現場急停／停機 SOP "
                    "確認設備完全停止後，再由合格人員重新啟動系統。",
                    parent=self,
                )
        elif succeeded:
            self.status_var.set(f"{label}執行完畢：{message}")
            if show_result_dialog:
                messagebox.showinfo(f"{label}執行完畢", details, parent=self)
        elif data.get("state") == "cancelled":
            self.status_var.set(f"{label}已取消：{message}")
            if show_result_dialog:
                messagebox.showwarning(f"{label}已取消", details, parent=self)
        else:
            self.status_var.set(f"{label}未完成：{message}")
            if show_result_dialog:
                messagebox.showwarning(f"{label}未完成", details, parent=self)

        selection = self.group_list.curselection()
        if selection:
            self._show_group(self.group_list.get(selection[0]))
        return True

    def _update_flow_elapsed(self) -> None:
        if self._active_flow is None or self._flow_started_at is None:
            return
        elapsed = time.monotonic() - self._flow_started_at
        label = self._active_flow_label or "PLC Flow"
        if self._flow_cancelling:
            cancel_elapsed = (
                time.monotonic() - self._flow_cancel_started_at
                if self._flow_cancel_started_at is not None
                else 0.0
            )
            self.status_var.set(
                f"正在取消 {label}：流程已執行 {elapsed:.1f} 秒，"
                f"取消等待 {cancel_elapsed:.1f} 秒"
            )
        else:
            self.status_var.set(f"{label}執行中：已執行 {elapsed:.1f} 秒，請勿重複操作")
        self._flow_timer_id = self.after(1000, self._update_flow_elapsed)

    def _stop_flow_timer(self) -> None:
        if self._flow_timer_id is None:
            return
        try:
            self.after_cancel(self._flow_timer_id)
        except tk.TclError:
            pass
        self._flow_timer_id = None

    def _stop_cancel_status_poll(self) -> None:
        if self._flow_cancel_poll_id is None:
            return
        try:
            self.after_cancel(self._flow_cancel_poll_id)
        except tk.TclError:
            pass
        self._flow_cancel_poll_id = None

    @staticmethod
    def _flow_result_details(data: dict[str, Any], message: str) -> str:
        state_labels = {
            "pending": "尚未開始",
            "running": "執行中",
            "waiting_signal": "等待 PLC 訊號",
            "success": "成功",
            "cancelled": "已取消",
            "stop_unconfirmed": "停止未確認",
            "timeout": "逾時",
            "error": "失敗",
        }
        step_labels = {
            "precheck": "前置檢查",
            "command_home": "送出歸零命令",
            "wait_zero": "等待位置歸零",
            "vacuum": "開啟真空",
            "move_to_height": "移動至視覺高度",
        }
        status = str(data.get("status") or data.get("state") or "unknown")
        state = str(data.get("state") or "unknown")
        step = str(data.get("step") or "unknown")
        elapsed = float(data.get("elapsed_seconds") or 0.0)
        lines = [
            message,
            f"流程狀態：{state_labels.get(status, status)}",
            f"細節狀態：{state}",
            f"步驟：{step_labels.get(step, step)}",
            f"耗時：{elapsed:.1f} 秒",
        ]
        positions = data.get("positions")
        if isinstance(positions, dict) and positions:
            values = "、".join(f"{axis}={float(value):g}mm" for axis, value in positions.items())
            lines.append(f"目前位置：{values}")
        elif data.get("height_mm") is not None:
            lines.append(f"目前高度：{float(data['height_mm']):g}mm")
        if data.get("vision_height_mm") is not None:
            lines.append(f"視覺回傳高度：{float(data['vision_height_mm']):g}mm")
        direction = str(data.get("transfer_direction") or "")
        if direction == "Y1_TO_Y2":
            lines.append("第二步方向：Y1 → Y2（RView 右取左放）")
        elif direction == "Y2_TO_Y1":
            lines.append("第二步方向：Y2 → Y1（LView 左取右放）")
        if data.get("phase") is not None:
            lines.append(f"總流程階段：{data['phase']}")
        home_postcheck = data.get("home_postcheck")
        if isinstance(home_postcheck, dict):
            if home_postcheck.get("confirmed"):
                angles = home_postcheck.get("angles")
                angle_text = ""
                if isinstance(angles, dict) and angles:
                    angle_text = "（" + "、".join(
                        f"{key}={float(value):.2f}°"
                        for key, value in angles.items()
                    ) + "）"
                lines.append(
                    f"結束 HOME 檢查：手臂及 Camera 均已確認{angle_text}"
                )
            else:
                lines.append(
                    "結束 HOME 檢查：未通過；"
                    + str(home_postcheck.get("error") or "原因不明")
                )
        return "\n".join(lines)

    def _begin_second_step_progress(self) -> None:
        if self._active_flow != "main-cycle":
            return
        self.arm_home_confirm_var.set(
            "手臂／Camera HOME：第二步進行中（固定上限 695mm）"
        )
        self.main_cycle_phase_var.set("總流程：第二步執行中")
        self._second_step_active = True
        self._second_step_started_at = time.monotonic()
        self._second_step_last_message = ""
        direction_text = (
            "Y1 → Y2／RView"
            if self.main_cycle_transfer_direction_var.get() == "Y1_TO_Y2"
            else "Y2 → Y1／LView"
        )
        self._second_step_stage_var.set(f"啟動第二步：{direction_text}")
        self._second_step_state_var.set("執行中")
        self._second_step_message_var.set("正在送出第二步命令…")
        self._second_step_elapsed_var.set("第二步經過時間：0.0 秒")
        self._second_step_height_var.set("升降機高度：等待資料")
        self._second_step_detected_height_var.set("影像辨識原始高度：等待資料")
        self._second_step_target_height_var.set("升降目標 D500：等待資料")
        self.btn_second_step_progress.configure(state=tk.NORMAL)
        self._show_second_step_progress()
        self._schedule_second_step_status_poll(0)

    def _show_second_step_progress(self) -> None:
        if self._second_step_window is not None:
            try:
                if self._second_step_window.winfo_exists():
                    self._second_step_window.deiconify()
                    self._second_step_window.lift()
                    return
            except tk.TclError:
                pass

        window = tk.Toplevel(self)
        self._second_step_window = window
        window.title("第二步執行進度")
        window.geometry("680x450")
        window.minsize(580, 390)
        window.transient(self)
        window.protocol("WM_DELETE_WINDOW", self._hide_second_step_progress)
        window.columnconfigure(0, weight=1)
        window.rowconfigure(4, weight=1)

        heading = ttk.Frame(window, padding=(16, 14, 16, 8))
        heading.grid(row=0, column=0, sticky="ew")
        heading.columnconfigure(0, weight=1)
        ttk.Label(
            heading,
            textvariable=self._second_step_stage_var,
            font=self.heading_font,
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            heading,
            textvariable=self._second_step_state_var,
        ).grid(row=0, column=1, sticky="e")

        progressbar = ttk.Progressbar(window, mode="indeterminate")
        progressbar.grid(row=1, column=0, sticky="ew", padx=16)
        self._second_step_progressbar = progressbar
        if self._second_step_active:
            progressbar.start(12)

        info = ttk.Frame(window, padding=(16, 10))
        info.grid(row=2, column=0, sticky="ew")
        info.columnconfigure(0, weight=1)
        ttk.Label(info, textvariable=self._second_step_elapsed_var).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(info, textvariable=self._second_step_height_var).grid(
            row=1, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Label(
            info,
            textvariable=self._second_step_detected_height_var,
        ).grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Label(
            info,
            textvariable=self._second_step_target_height_var,
            foreground="#075985",
        ).grid(row=3, column=0, sticky="w", pady=(4, 0))
        ttk.Label(
            info,
            textvariable=self._second_step_message_var,
            wraplength=600,
            justify=tk.LEFT,
        ).grid(row=4, column=0, sticky="ew", pady=(8, 0))

        history_frame = ttk.LabelFrame(window, text="最近動作", padding=(8, 6))
        history_frame.grid(row=4, column=0, sticky="nsew", padx=16, pady=(0, 10))
        history_frame.columnconfigure(0, weight=1)
        history_frame.rowconfigure(0, weight=1)
        history = tk.Listbox(
            history_frame,
            height=6,
            activestyle="none",
            exportselection=False,
            font=self.default_font,
        )
        history.grid(row=0, column=0, sticky="nsew")
        history_scrollbar = ttk.Scrollbar(
            history_frame,
            orient=tk.VERTICAL,
            command=history.yview,
        )
        history_scrollbar.grid(row=0, column=1, sticky="ns")
        history.configure(yscrollcommand=history_scrollbar.set)
        self._second_step_history = history
        if self._second_step_last_message:
            history.insert(tk.END, self._second_step_last_message)

        actions = ttk.Frame(window, padding=(16, 0, 16, 14))
        actions.grid(row=5, column=0, sticky="ew")
        actions.columnconfigure(0, weight=1)
        cancel_button = ttk.Button(
            actions,
            text="取消第二步",
            command=self.on_cancel_flow_clicked,
        )
        cancel_button.grid(row=0, column=1, padx=(0, 8))
        self._second_step_cancel_button = cancel_button
        ttk.Button(
            actions,
            text="隱藏視窗",
            command=self._hide_second_step_progress,
        ).grid(row=0, column=2)

    def _hide_second_step_progress(self) -> None:
        if self._second_step_window is None:
            return
        try:
            self._second_step_window.withdraw()
        except tk.TclError:
            self._second_step_window = None

    def _schedule_second_step_status_poll(
        self,
        delay_ms: int = SECOND_STEP_STATUS_POLL_INTERVAL_MS,
    ) -> None:
        if not self._second_step_active or self._active_flow != "main-cycle":
            return
        if self._second_step_poll_id is not None:
            try:
                self.after_cancel(self._second_step_poll_id)
            except tk.TclError:
                pass
        self._second_step_poll_id = self.after(delay_ms, self._poll_second_step_status)

    def _poll_second_step_status(self) -> None:
        self._second_step_poll_id = None
        if not self._second_step_active or self._active_flow != "main-cycle":
            return
        if self._second_step_poll_in_flight:
            self._schedule_second_step_status_poll()
            return
        self._second_step_poll_in_flight = True

        def worker() -> None:
            lifecycle_data: dict[str, Any] | None = None
            error: str | None = None
            try:
                response = requests.get(f"{API_BASE_URL}/lifecycle/status", timeout=3)
                response.raise_for_status()
                payload = response.json()
                data = payload.get("data")
                if isinstance(data, dict):
                    lifecycle_data = data
                else:
                    error = "後端沒有回傳流程狀態"
            except Exception as exc:  # noqa: BLE001
                error = str(exc)
            self._post_ui(
                lambda: self._handle_second_step_status(lifecycle_data, error)
            )

        threading.Thread(
            target=worker,
            daemon=True,
            name="plc-flow-main-cycle-progress",
        ).start()

    def _handle_second_step_status(
        self,
        lifecycle_data: dict[str, Any] | None,
        error: str | None,
    ) -> None:
        self._second_step_poll_in_flight = False
        if not self._second_step_active or self._active_flow != "main-cycle":
            return

        if self._second_step_started_at is not None:
            elapsed = time.monotonic() - self._second_step_started_at
            self._second_step_elapsed_var.set(f"第二步經過時間：{elapsed:.1f} 秒")

        if error is not None:
            self._second_step_state_var.set("狀態讀取暫時中斷")
        elif lifecycle_data is not None:
            main_snapshot = lifecycle_data.get("MainCycleFlow")
            arm_snapshot = lifecycle_data.get("ArmVisionWorkflowService")
            if not isinstance(main_snapshot, dict):
                main_snapshot = {}
            if not isinstance(arm_snapshot, dict):
                arm_snapshot = {}

            step = str(main_snapshot.get("step") or "step2_precheck")
            main_message = str(main_snapshot.get("message") or "")
            arm_message = str(arm_snapshot.get("message") or "")
            arm_step = str(arm_snapshot.get("step") or "")
            status = str(main_snapshot.get("status") or "running")
            data = (
                main_snapshot.get("data")
                if isinstance(main_snapshot.get("data"), dict)
                else {}
            )
            arm_data = (
                arm_snapshot.get("data")
                if isinstance(arm_snapshot.get("data"), dict)
                else {}
            )

            stage = SECOND_STEP_LABELS.get(step, step)
            if step in {
                "step2_arm_vision_pick_place",
                "step2_pick_height",
                "step2_place_height",
            }:
                stage = self._second_step_arm_stage(arm_step, stage)
            direction = str(data.get("transfer_direction") or "")
            if direction == "Y1_TO_Y2":
                stage = f"Y1 → Y2／RView｜{stage}"
            elif direction == "Y2_TO_Y1":
                stage = f"Y2 → Y1／LView｜{stage}"
            self._second_step_stage_var.set(stage)
            if self._flow_cancelling:
                self._second_step_state_var.set("正在取消，等待設備停止")
            else:
                self._second_step_state_var.set(self._second_step_status_label(status))

            message = (
                arm_message
                if step == "step2_arm_vision_pick_place" and arm_message
                else main_message
            )
            message = self._friendly_second_step_message(message)
            if message:
                self._second_step_message_var.set(message)
                self._append_second_step_history(message)

            height = data.get("height_mm")
            vision_height = data.get("vision_height_mm")
            detected_depth_m = arm_data.get("depth_m")
            target_height = arm_data.get("plc_height_mm")
            if target_height is None:
                target_height = arm_data.get("height_mm")
            if target_height is None:
                target_height = vision_height
            detected_height_mm, target_height_mm = self._second_step_height_values(
                detected_depth_m,
                target_height,
            )
            if detected_height_mm is not None:
                self._second_step_detected_height_var.set(
                    f"影像辨識原始高度：{detected_height_mm:.3f} mm"
                )
            if target_height_mm is not None:
                self._second_step_target_height_var.set(
                    f"升降目標 D500：{target_height_mm:.3f} mm"
                )
            height_parts: list[str] = []
            if height is not None:
                height_parts.append(f"目前 D500={float(height):g}mm")
            if vision_height is not None:
                height_parts.append(f"辨識工作高度={float(vision_height):g}mm")
            self._second_step_height_var.set(
                "升降機高度：" + ("；".join(height_parts) if height_parts else "等待資料")
            )

        self._schedule_second_step_status_poll()

    def _append_second_step_history(self, message: str) -> None:
        if not message or message == self._second_step_last_message:
            return
        self._second_step_last_message = message
        history = self._second_step_history
        if history is None:
            return
        try:
            history.insert(tk.END, message)
            while history.size() > 30:
                history.delete(0)
            history.see(tk.END)
        except tk.TclError:
            self._second_step_history = None

    def _stop_second_step_progress(self) -> None:
        self._second_step_active = False
        self._second_step_poll_in_flight = False
        if self._second_step_poll_id is not None:
            try:
                self.after_cancel(self._second_step_poll_id)
            except tk.TclError:
                pass
            self._second_step_poll_id = None
        if self._second_step_progressbar is not None:
            try:
                self._second_step_progressbar.stop()
            except tk.TclError:
                pass
        if self._second_step_window is not None:
            try:
                self._second_step_window.destroy()
            except tk.TclError:
                pass
        self._second_step_window = None
        self._second_step_history = None
        self._second_step_progressbar = None
        self._second_step_cancel_button = None
        self.btn_second_step_progress.configure(state=tk.DISABLED)

    @staticmethod
    def _second_step_status_label(status: str) -> str:
        return {
            "pending": "準備中",
            "running": "執行中",
            "waiting_signal": "等待設備訊號",
            "success": "已完成",
            "cancelled": "已取消",
            "timeout": "已逾時",
            "error": "發生錯誤",
        }.get(status, status or "執行中")

    @staticmethod
    def _second_step_height_values(
        detected_depth_m: Any,
        target_height_mm: Any,
    ) -> tuple[float | None, float | None]:
        """Convert lifecycle values into the two height readouts shown by the UI."""

        detected: float | None = None
        target: float | None = None
        try:
            if detected_depth_m is not None:
                detected = float(detected_depth_m) * 1000.0
        except (TypeError, ValueError):
            detected = None
        try:
            if target_height_mm is not None:
                target = float(target_height_mm)
        except (TypeError, ValueError):
            target = None
        return detected, target

    @staticmethod
    def _second_step_arm_stage(arm_step: str, fallback: str) -> str:
        return {
            "start": "啟動 D435i／CANBus 手臂程式",
            "arm_output": "D435i／CANBus 手臂執行中",
            "height_precheck": "檢查辨識高度與 695mm 安全上限",
            "waiting_safe_height": "等待升降機停止並確認目前動作安全高度",
            "safe_height_confirmed": "安全高度已確認，準備放行手臂",
            "pick_handoff": "取料位置交接",
            "place_handoff": "放料位置交接",
            "cancelling": "正在要求四顆 CAN 馬達停止",
            "cancel_stop_confirmed": "四顆 CAN 馬達已確認停止",
            "success": "手臂取放完成並確認 HOME",
            "cancelled": "正在取消手臂流程",
            "timeout": "手臂流程逾時",
            "error": "手臂流程發生錯誤",
        }.get(arm_step, fallback)

    @staticmethod
    def _friendly_second_step_message(message: str) -> str:
        text = message.removeprefix("第二步手臂：").strip()
        lower = text.lower()
        if "[pose_check] home" in lower:
            return f"HOME 角度回讀：{text.removeprefix('[POSE_CHECK] ').strip()}"
        if "arm_stop_unconfirmed" in lower:
            return f"警告：四顆 CAN 馬達停止尚未確認；{text}"
        translations = (
            ("verifying can communication", "正在確認 CANBus 與四顆馬達連線"),
            ("initial_home_confirmed", "啟動 HOME 已由四顆馬達角度回讀確認"),
            ("all_home_confirmed", "最終 HOME 已由四顆馬達角度回讀確認"),
            ("arm_stop_confirmed", "四顆 CAN 馬達已回覆停止命令"),
            ("go to home", "四顆馬達正在移動到 HOME"),
            ("go to lview", "相機臂正在移動到 LView"),
            ("go to rview", "相機臂正在移動到 RView"),
            ("starting realsense pipeline", "正在啟動 D435i 相機"),
            ("warming up camera", "D435i 相機暖機中"),
            ("capturing one aligned rgb-d frame", "正在拍攝對齊的彩色與深度影像"),
            ("sending full image", "正在送出影像進行吸取點辨識"),
            ("saved control_target.json", "辨識完成，正在檢查高度與安全條件"),
            ("left target suction position", "手臂已到左側吸取位置，準備升降與吸真空"),
            ("right target suction position", "手臂已到右側吸取位置，準備升降與吸真空"),
            (
                "right outer-branch place position",
                "手臂已到右側放料位置，準備升降與破真空",
            ),
            (
                "left outer-branch place position",
                "手臂已到左側放料位置，準備升降與破真空",
            ),
            ("returned home", "手臂正在返回 HOME"),
        )
        for marker, translated in translations:
            if marker in lower:
                return translated
        return text

    def _set_main_cycle_controls(self, enabled: bool) -> None:
        enabled = enabled and not self._stop_unconfirmed
        state = tk.NORMAL if enabled else tk.DISABLED
        for control in self.main_cycle_controls:
            if isinstance(control, ttk.Combobox):
                control.configure(state="readonly" if enabled else tk.DISABLED)
            else:
                control.configure(state=state)
        if enabled and not self._task_file_steps:
            self.btn_task_file_run.configure(state=tk.DISABLED)
        if enabled:
            self._refresh_main_cycle_phase_ui()

    @staticmethod
    def _set_controls_enabled(controls: list[tk.Widget], enabled: bool) -> None:
        for control in controls:
            if isinstance(control, ttk.Combobox):
                control.configure(state="readonly" if enabled else tk.DISABLED)
            else:
                control.configure(state=tk.NORMAL if enabled else tk.DISABLED)

    def _close_main_cycle_continue_gate(self) -> None:
        window = self._main_cycle_gate_window
        self._main_cycle_gate_window = None
        if window is None:
            return
        try:
            if window.winfo_exists():
                window.grab_release()
                window.destroy()
        except tk.TclError:
            pass

    def _show_main_cycle_continue_gate(self, phase: str) -> None:
        if phase not in {"waiting_step2", "waiting_final_step1"}:
            return
        if self._active_flow is not None:
            return
        if self._main_cycle_gate_window is not None:
            if self._main_cycle_gate_window.winfo_exists():
                self._main_cycle_gate_window.deiconify()
                self._main_cycle_gate_window.lift()
                self._main_cycle_gate_window.focus_force()
                return
            self._main_cycle_gate_window = None

        is_second_step = phase == "waiting_step2"
        step_name = "第二步" if is_second_step else "第三步"
        button_text = f"繼續進行{step_name}"
        if is_second_step:
            direction = (
                "Y1 → Y2（RView 右取左放）"
                if self.main_cycle_transfer_direction_var.get() == "Y1_TO_Y2"
                else "Y2 → Y1（LView 左取右放）"
            )
            heading = "第一步已完成"
            description = (
                f"搬運方向：{direction}\n"
                "請確認現場無人員或障礙物，Camera 與手臂位於 HOME。\n"
                "第二步移動手臂前會再次確認升降機為 560 ± 1 mm。"
            )
        else:
            heading = "第二步已完成"
            description = (
                "請確認 Camera 與四軸手臂都已返回 HOME，並確認第三步貨盤設定。\n"
                "若選擇「推」，會對所選 Y1／Y2 關閉持續真空並開啟破真空。\n"
                "只有 HOME 確認通過後，第三步才可使用最高 1450 mm。"
            )

        window = tk.Toplevel(self)
        self._main_cycle_gate_window = window
        window.title(f"{step_name}執行確認")
        window.resizable(False, False)
        window.transient(self)
        window.protocol("WM_DELETE_WINDOW", self._close_main_cycle_continue_gate)

        body = ttk.Frame(window, padding=(24, 20))
        body.grid(row=0, column=0, sticky="nsew")
        ttk.Label(
            body,
            text=heading,
            font=self.heading_font,
            anchor=tk.CENTER,
        ).grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        ttk.Label(
            body,
            text=f"準備進入{step_name}",
            font=self.default_font,
            anchor=tk.CENTER,
        ).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        ttk.Label(
            body,
            text=description,
            justify=tk.LEFT,
            wraplength=500,
        ).grid(row=2, column=0, columnspan=2, sticky="w", pady=(0, 18))
        ttk.Button(
            body,
            text="稍後再執行",
            command=self._close_main_cycle_continue_gate,
        ).grid(row=3, column=0, sticky="ew", padx=(0, 8))
        ttk.Button(
            body,
            text=button_text,
            command=lambda expected_phase=phase: self._continue_main_cycle_from_gate(
                expected_phase
            ),
        ).grid(row=3, column=1, sticky="ew", padx=(8, 0))
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        window.update_idletasks()
        x = self.winfo_rootx() + max(0, (self.winfo_width() - window.winfo_width()) // 2)
        y = self.winfo_rooty() + max(0, (self.winfo_height() - window.winfo_height()) // 2)
        window.geometry(f"+{x}+{y}")
        window.grab_set()
        window.lift()
        window.focus_force()

    def _continue_main_cycle_from_gate(self, expected_phase: str) -> None:
        if self.main_cycle_phase != expected_phase:
            self._close_main_cycle_continue_gate()
            messagebox.showwarning(
                "流程階段已變更",
                f"目前階段：{self.main_cycle_phase}，請重新確認。",
                parent=self,
            )
            return
        self._close_main_cycle_continue_gate()
        self._start_main_cycle_sequence(confirmed_phase=expected_phase)

    def _refresh_main_cycle_phase_ui(self) -> None:
        if (
            not self._connected
            or self._active_flow is not None
            or self._stop_unconfirmed
        ):
            return
        if self.main_cycle_phase == "ready_first_step":
            self.main_cycle_phase_var.set("完整流程準備完成：先執行第一步")
            self.main_cycle_run_text_var.set("開始完整流程")
            self._set_controls_enabled(self.main_cycle_first_controls, True)
            self._set_controls_enabled(self.main_cycle_second_controls, True)
            self._set_controls_enabled(self.main_cycle_final_controls, True)
            self.btn_main_cycle_run.configure(state=tk.NORMAL)
        elif self.main_cycle_phase == "waiting_step2":
            self.main_cycle_phase_var.set("第一步已完成；等待人工確認第二步")
            self.main_cycle_run_text_var.set("開啟第二步確認視窗")
            self._set_controls_enabled(self.main_cycle_first_controls, True)
            self._set_controls_enabled(self.main_cycle_second_controls, True)
            self._set_controls_enabled(self.main_cycle_final_controls, True)
            self.btn_main_cycle_run.configure(state=tk.NORMAL)
        elif self.main_cycle_phase == "waiting_final_step1":
            self.main_cycle_phase_var.set("第一、二步已完成；等待人工確認第三步")
            self.main_cycle_run_text_var.set("開啟第三步確認視窗")
            self._set_controls_enabled(self.main_cycle_first_controls, True)
            self._set_controls_enabled(self.main_cycle_second_controls, True)
            self._set_controls_enabled(self.main_cycle_final_controls, True)
            self.btn_main_cycle_run.configure(state=tk.NORMAL)
        elif self.main_cycle_phase == "complete":
            self.main_cycle_phase_var.set("三步驟完整流程已完成")
            self.main_cycle_run_text_var.set("完整流程已完成")
            self._set_controls_enabled(self.main_cycle_first_controls, True)
            self._set_controls_enabled(self.main_cycle_second_controls, True)
            self._set_controls_enabled(self.main_cycle_final_controls, True)
            self.btn_main_cycle_run.configure(state=tk.DISABLED)
        else:
            self.main_cycle_phase_var.set(f"目前流程階段：{self.main_cycle_phase}")
            self.btn_main_cycle_run.configure(state=tk.DISABLED)
        self.btn_main_cycle_reset.configure(state=tk.NORMAL)

    def _main_cycle_payload(
        self,
        *,
        step_label: str,
        slot_var: tk.StringVar,
        action_var: tk.StringVar,
        height_var: tk.StringVar,
        forward_var: tk.StringVar,
    ) -> dict[str, Any] | None:
        try:
            height = float(height_var.get().strip())
        except ValueError:
            messagebox.showerror(
                "總流程輸入錯誤",
                f"{step_label}的高度 D500 必須是數字",
                parent=self,
            )
            return None
        slot = slot_var.get().strip()
        action_text = action_var.get().strip()
        action = {"吸": "suck", "推": "push", "none": "none"}.get(action_text, action_text)
        if slot not in {"Y1", "Y2", "none"}:
            messagebox.showerror(
                "總流程輸入錯誤",
                f"{step_label}的貨盤必須是 Y1、Y2 或 none",
                parent=self,
            )
            return None
        if (slot == "none") != (action == "none"):
            messagebox.showerror(
                "總流程輸入錯誤",
                f"{step_label}選擇 none 時，貨盤與動作都必須是 none",
                parent=self,
            )
            return None
        payload: dict[str, Any] = {
            "slot": slot,
            "action": action,
            "height_mm": height,
            "forward_mm": None,
        }
        if slot.lower() != "none" and action != "none":
            point_name = "D510" if slot.upper() == "Y1" else "D560"
            try:
                payload["forward_mm"] = float(forward_var.get().strip())
            except ValueError:
                messagebox.showerror(
                    "總流程輸入錯誤",
                    f"{step_label}的前進距離 {point_name} 必須是數字",
                    parent=self,
                )
                return None
        return payload

    def _start_main_cycle_sequence(self, *, confirmed_phase: str | None = None) -> None:
        if not self._connected:
            messagebox.showwarning("尚未連線", "請先連線 PLC", parent=self)
            return
        if self._active_flow is not None:
            messagebox.showwarning("流程執行中", "已有 PLC Flow 執行中", parent=self)
            return

        start_phase = self.main_cycle_phase
        if (
            start_phase in {"waiting_step2", "waiting_final_step1"}
            and confirmed_phase != start_phase
        ):
            self._show_main_cycle_continue_gate(start_phase)
            return

        transfer_direction = self.main_cycle_transfer_direction_var.get().strip()
        first_payload: dict[str, Any] | None = None
        if start_phase == "ready_first_step":
            first_payload = self._main_cycle_payload(
                step_label="第一步",
                slot_var=self.main_cycle_first_slot_var,
                action_var=self.main_cycle_first_action_var,
                height_var=self.main_cycle_first_height_var,
                forward_var=self.main_cycle_first_forward_var,
            )
            if first_payload is None:
                return
        elif start_phase == "waiting_step2":
            if transfer_direction not in {"Y1_TO_Y2", "Y2_TO_Y1"}:
                messagebox.showerror(
                    "總流程輸入錯誤",
                    "第二步搬運方向必須是 Y1 → Y2 或 Y2 → Y1",
                    parent=self,
                )
                return
        elif start_phase == "waiting_final_step1":
            final_payload = self._main_cycle_payload(
                step_label="第三步",
                slot_var=self.main_cycle_final_slot_var,
                action_var=self.main_cycle_final_action_var,
                height_var=self.main_cycle_final_height_var,
                forward_var=self.main_cycle_final_forward_var,
            )
            if final_payload is None:
                return
        else:
            messagebox.showwarning(
                "總流程階段不正確",
                f"目前階段：{start_phase}",
                parent=self,
            )
            return

        self._active_flow = "main-cycle"
        self._active_flow_label = {
            "ready_first_step": "第一步",
            "waiting_step2": "第二步",
            "waiting_final_step1": "第三步",
        }[start_phase]
        self._flow_started_at = time.monotonic()
        self._flow_cancelling = False
        self._flow_cancel_started_at = None
        self._flow_cancel_deadline = None
        self.status_var.set(f"{self._active_flow_label}已開始")
        self.main_cycle_phase_var.set(
            {
                "ready_first_step": "① 第一步執行中",
                "waiting_step2": "② 從第二步繼續執行",
                "waiting_final_step1": "③ 第三步執行中",
            }.get(start_phase, "三步驟總流程執行中")
        )
        self.btn_home_flow.configure(state=tk.DISABLED)
        self.btn_vision_flow.configure(state=tk.DISABLED)
        self._set_can_home_controls(False)
        self.btn_cancel_flow.configure(state=tk.NORMAL)
        self._set_main_cycle_controls(False)
        self._set_vacuum_controls(False)
        self._update_flow_elapsed()

        def worker() -> None:
            latest_payload: dict[str, Any] | None = None
            fallback_phase = start_phase
            try:
                if start_phase == "ready_first_step":
                    self._post_ui(
                        lambda: self.arm_home_confirm_var.set(
                            "手臂／Camera HOME：完整流程啟動前確認中…"
                        )
                    )
                    confirm_response = requests.post(
                        API_BASE_URL + "/arm-camera-home/confirm",
                        timeout=30,
                    )
                    confirm_response.raise_for_status()
                    confirm_payload = confirm_response.json()
                    self._post_ui(
                        lambda payload=confirm_payload: self._finish_arm_home_confirmation(
                            payload,
                            None,
                            show_dialog=False,
                        )
                    )
                    if not confirm_payload.get("ok"):
                        raise RuntimeError(
                            str(
                                confirm_payload.get("error")
                                or "第一步前的四軸 HOME 確認未通過"
                            )
                        )

                    response = requests.post(
                        API_BASE_URL + "/flows/main-cycle/first-step",
                        json=first_payload,
                        timeout=MAIN_CYCLE_STEP_REQUEST_TIMEOUT_SECONDS,
                    )
                    response.raise_for_status()
                    latest_payload = response.json()
                    if not latest_payload.get("ok"):
                        self._post_ui(
                            lambda payload=latest_payload: self._finish_main_cycle_sequence(
                                payload,
                                None,
                                "ready_first_step",
                            )
                        )
                        return
                    fallback_phase = "waiting_step2"
                    self._post_ui(
                        lambda payload=latest_payload: self._finish_main_cycle_sequence(
                            payload,
                            None,
                            "waiting_step2",
                            expected_success_phase="waiting_step2",
                        )
                    )
                    return

                if start_phase == "waiting_step2":
                    self._post_ui(self._begin_second_step_progress)
                    second_response = requests.post(
                        API_BASE_URL + "/flows/main-cycle/second-step",
                        json={"transfer_direction": transfer_direction},
                        timeout=MAIN_CYCLE_SECOND_STEP_REQUEST_TIMEOUT_SECONDS,
                    )
                    second_response.raise_for_status()
                    latest_payload = second_response.json()
                    self._post_ui(
                        lambda payload=latest_payload: (
                            self._update_arm_home_from_main_cycle(payload)
                        )
                    )
                    if not latest_payload.get("ok"):
                        self._post_ui(
                            lambda payload=latest_payload: self._finish_main_cycle_sequence(
                                payload,
                                None,
                                "waiting_step2",
                            )
                        )
                        return
                    fallback_phase = "waiting_final_step1"
                    self._post_ui(
                        lambda payload=latest_payload: self._finish_main_cycle_sequence(
                            payload,
                            None,
                            "waiting_final_step1",
                            expected_success_phase="waiting_final_step1",
                        )
                    )
                    return

                final_response = requests.post(
                    API_BASE_URL + "/flows/main-cycle/final-step",
                    json=final_payload,
                    timeout=MAIN_CYCLE_STEP_REQUEST_TIMEOUT_SECONDS,
                )
                final_response.raise_for_status()
                latest_payload = final_response.json()
                self._post_ui(
                    lambda payload=latest_payload: self._finish_main_cycle_sequence(
                        payload,
                        None,
                        fallback_phase,
                        expected_success_phase="complete",
                    )
                )
            except Exception as exc:  # noqa: BLE001
                self._post_ui(
                    lambda error=str(exc), payload=latest_payload, phase=fallback_phase: (
                        self._finish_main_cycle_sequence(
                            payload,
                            error,
                            phase,
                        )
                    )
                )

        threading.Thread(target=worker, daemon=True, name="plc-flow-main-cycle").start()

    def _mark_main_cycle_stage(
        self,
        phase: str,
        message: str,
        *,
        close_second_step: bool = False,
    ) -> None:
        self.main_cycle_phase = phase
        self.main_cycle_phase_var.set(message)
        if close_second_step:
            self._stop_second_step_progress()

    def _finish_main_cycle_sequence(
        self,
        payload: dict[str, Any] | None,
        request_error: str | None,
        fallback_phase: str,
        *,
        expected_success_phase: str | None = None,
    ) -> None:
        data = (
            payload.get("data")
            if payload and isinstance(payload.get("data"), dict)
            else {}
        )
        succeeded = bool(payload and payload.get("ok") and request_error is None)
        response_phase = (
            expected_success_phase
            if succeeded and expected_success_phase is not None
            else str(data.get("phase") or fallback_phase)
        )
        waiting_for_confirmation = succeeded and response_phase in {
            "waiting_step2",
            "waiting_final_step1",
        }
        label = self._active_flow_label or "三步驟總流程"
        update_home = getattr(self, "_update_arm_home_from_main_cycle", None)
        if payload is not None and callable(update_home):
            update_home(payload)
        if not self._finish_flow(
            label,
            payload,
            request_error,
            expected_flow="main-cycle",
            show_result_dialog=not waiting_for_confirmation,
        ):
            return
        if not succeeded:
            step_result = data.get("step_result")
            operation_finished = (
                str(data.get("step") or "") == "post_step_home_check"
                and isinstance(step_result, dict)
                and str(step_result.get("status") or "") == "success"
            )
            self.main_cycle_phase = (
                response_phase if operation_finished else fallback_phase
            )
            self._refresh_main_cycle_phase_ui()
            return
        self.main_cycle_phase = response_phase
        self._refresh_main_cycle_phase_ui()
        if waiting_for_confirmation:
            self.after(
                100,
                lambda phase=response_phase: self._show_main_cycle_continue_gate(phase),
            )


def main() -> None:
    window = MainWindow()
    window.mainloop()


if __name__ == "__main__":
    main()
