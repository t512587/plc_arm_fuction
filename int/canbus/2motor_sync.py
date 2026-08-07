"""AMR Arm Control UI — thin Tkinter wrapper over ArmController."""
from __future__ import annotations

import threading
import traceback
from datetime import datetime
from pathlib import Path
from tkinter import BOTH, END, LEFT, RIGHT, TOP, X, Button, Entry, Frame, Label, StringVar, Text, Tk, ttk

from arm_config import (
    HOME_SPEED_DPS,
    MAX_SPEED_DPS,
    MOTORS,
    STEP_DEGREES,
    VACUUM_BIG,
    VACUUM_SMALL,
)
from arm_controller import ArmController
from tk_ui import configure_tk_ui


class FourMotorSyncUi:
    def __init__(self, root: Tk) -> None:
        self.root = root
        self.root.title("AMR Arm Control (ID142~ID145)")
        self.root.geometry("980x760")
        self.default_font, self.heading_font, self.mono_font = configure_tk_ui(root)
        self.ctrl = ArmController()
        self.status = StringVar(value="Disconnected")
        self.positions = {label: StringVar(value="0.00") for label in MOTORS}
        self.positions_loaded = False
        self.adjusted_labels: set[str] = set()
        self.running_actions: set[str] = set()
        self.log_file = self._create_log_file()
        self._build_ui()

    # ─── UI construction ────────────────────────────────────────────────

    def _build_ui(self) -> None:
        top = Frame(self.root, padx=12, pady=10)
        top.pack(side=TOP, fill=X)
        Label(
            top,
            text=(
                f"CAN link Data    Interface: {self.ctrl.config.can_interface}    "
                f"Channel: {self.ctrl.config.channel}    CAN: {self.ctrl.config.bitrate}    "
                f"Serial: {self.ctrl.config.serial_baudrate}    Speed: 0x{MAX_SPEED_DPS:04X}"
            ),
        ).pack(side=LEFT)
        Label(top, textvariable=self.status).pack(side=RIGHT)

        controls = Frame(self.root, padx=12, pady=6)
        controls.pack(side=TOP, fill=X)
        Button(controls, text="Connect", command=self._connect_direct).pack(
            side=LEFT, padx=(0, 8)
        )
        Button(controls, text="Disconnect", command=self._disconnect_direct).pack(
            side=LEFT, padx=(0, 8)
        )
        Button(
            controls,
            text="Read Current Positions",
            command=self._read_positions_direct,
        ).pack(side=LEFT, padx=(0, 8))
        Button(
            controls,
            text="HOME",
            command=lambda: self._go_to_point_direct("HOME"),
            fg="white",
            bg="#5cb85c",
        ).pack(side=LEFT, padx=(0, 8))

        # --- Point position buttons ---
        points_row = Frame(self.root, padx=12, pady=4)
        points_row.pack(side=TOP, fill=X)
        Label(points_row, text="Go To:").pack(side=LEFT, padx=(0, 6))
        point_colors = {
            "MOVE": "#f0ad4e",
            "LView": "#337ab7",
            "RView": "#337ab7",
            "LGrap": "#9b59b6",
            "RGrap": "#9b59b6",
        }
        for point_name in self.ctrl.point_config:
            if point_name == "HOME":
                continue
            bg_color = point_colors.get(point_name, "#555")
            Button(
                points_row,
                text=point_name,
                command=lambda pn=point_name: self._go_to_point_direct(pn),
                fg="white",
                bg=bg_color,
            ).pack(side=LEFT, padx=(0, 6))

        # --- Save current position ---
        save_row = Frame(self.root, padx=12, pady=4)
        save_row.pack(side=TOP, fill=X)
        Label(save_row, text="Save Current As:").pack(side=LEFT, padx=(0, 6))
        self.save_point_name = StringVar(value="HOME")
        point_names = ["HOME"] + [n for n in self.ctrl.point_config if n != "HOME"]
        ttk.Combobox(
            save_row,
            values=point_names,
            textvariable=self.save_point_name,
            state="readonly",
            width=10,
        ).pack(side=LEFT, padx=(0, 6))
        Button(
            save_row,
            text="Save",
            command=self._save_current_position,
        ).pack(side=LEFT, padx=(0, 8))

        # --- Vacuum arm dynamic control ---
        vacuum_row = Frame(self.root, padx=12, pady=4)
        vacuum_row.pack(side=TOP, fill=X)
        Label(vacuum_row, text="真空泵動態控制:").pack(side=LEFT, padx=(0, 6))
        Label(vacuum_row, text="偏移(cm)").pack(side=LEFT, padx=(0, 4))
        self.vacuum_offset = StringVar(value="0.0")
        Entry(vacuum_row, textvariable=self.vacuum_offset, width=8).pack(side=LEFT, padx=(0, 6))
        Label(vacuum_row, text="方向").pack(side=LEFT, padx=(0, 4))
        self.vacuum_direction = StringVar(value="L")
        ttk.Combobox(
            vacuum_row,
            values=["L", "R"],
            textvariable=self.vacuum_direction,
            state="readonly",
            width=3,
        ).pack(side=LEFT, padx=(0, 6))
        Button(
            vacuum_row,
            text="計算並移動",
            command=self._move_vacuum_dynamic_direct,
            fg="white",
            bg="#e67e22",
        ).pack(side=LEFT, padx=(0, 6))
        Button(
            vacuum_row,
            text="僅計算",
            command=self._calc_vacuum_only,
        ).pack(side=LEFT, padx=(0, 6))

        # --- Motor panels ---
        middle = Frame(self.root, padx=12, pady=10)
        middle.pack(side=TOP, fill=X)
        left = Frame(middle)
        left.pack(side=LEFT, fill=X, expand=True, padx=(0, 8))
        right = Frame(middle)
        right.pack(side=LEFT, fill=X, expand=True, padx=(8, 0))

        Label(left, text="真空泵臂", font=self.heading_font).pack(anchor="w")
        self._motor_panel(left, "ID 142").pack(fill=X, pady=(0, 10))
        self._motor_panel(left, "ID 143").pack(fill=X)
        Label(right, text="相機臂", font=self.heading_font).pack(anchor="w")
        self._motor_panel(right, "ID 144").pack(fill=X, pady=(0, 10))
        self._motor_panel(right, "ID 145").pack(fill=X)

        bottom = Frame(self.root, padx=12, pady=10)
        bottom.pack(side=TOP, fill=BOTH, expand=True)
        btn_row = Frame(bottom)
        btn_row.pack(anchor="w", pady=(0, 8))
        Button(
            btn_row,
            text="Run Selected Motor",
            command=self._run_selected_direct,
        ).pack(side=LEFT, padx=(0, 8))
        Button(
            btn_row,
            text="STOP ALL",
            command=self._stop_all_direct,
            fg="white",
            bg="#d9534f",
        ).pack(side=LEFT, padx=(0, 8))
        Button(
            btn_row,
            text="SHUTDOWN ALL",
            command=self._shutdown_all_direct,
            fg="white",
            bg="#5bc0de",
        ).pack(side=LEFT, padx=(0, 8))
        self.output = Text(bottom, height=16, wrap="word", font=self.mono_font)
        self.output.pack(fill=BOTH, expand=True)
        self._append("Ready. Connect -> Read Current Positions -> Adjust -> Run.\n")
        self._append(f"Log file: {self.log_file}\n")

    def _motor_panel(self, parent: Frame, label: str) -> Frame:
        panel = Frame(parent, padx=10, pady=10, relief="groove", borderwidth=1)
        Label(panel, text=label).pack(anchor="w")

        row = Frame(panel, pady=8)
        row.pack(fill=X)
        Label(row, text="Current Degree").pack(side=LEFT, padx=(0, 6))
        Entry(row, textvariable=self.positions[label], width=12).pack(side=LEFT)

        buttons = Frame(panel, pady=8)
        buttons.pack(fill=X)
        Button(buttons, text="<-1", command=lambda: self._adjust_degree(label, -STEP_DEGREES)).pack(
            side=LEFT, fill=X, expand=True, padx=(0, 6)
        )
        Button(buttons, text="+1>", command=lambda: self._adjust_degree(label, STEP_DEGREES)).pack(
            side=LEFT, fill=X, expand=True, padx=(6, 0)
        )
        return panel

    # ─── Button handlers ────────────────────────────────────────────────

    def _connect_direct(self) -> None:
        try:
            result = self.ctrl.connect()
            self.status.set("Connected")
            self._append("[connect] OK\n")
            self._append(f"  {result}\n\n")
        except Exception as exc:
            details = "".join(traceback.format_exception_only(type(exc), exc)).strip()
            self.status.set("Error")
            self._append(f"[connect] ERROR\n  {details}\n\n")

    def _disconnect_direct(self) -> None:
        try:
            result = self.ctrl.disconnect()
            self.status.set("Disconnected")
            self._append("[disconnect] OK\n")
            self._append(f"  {result}\n\n")
        except Exception as exc:
            details = "".join(traceback.format_exception_only(type(exc), exc)).strip()
            self._append(f"[disconnect] ERROR\n  {details}\n\n")

    def _read_positions_direct(self) -> None:
        if not self.ctrl.is_connected:
            self._append("[read_positions] ERROR\n")
            self._append("  CAN bus is not connected. Press Connect first.\n\n")
            return

        self._append("[read_positions] START\n")
        result = self.ctrl.read_positions()
        self._show_result("read_positions", result)
        self._append("[read_positions] DONE\n\n")

    def _go_to_point_direct(self, point_name: str) -> None:
        if not self.ctrl.is_connected:
            self._append(f"[{point_name}] ERROR\n")
            self._append("  CAN bus is not connected. Press Connect first.\n\n")
            return

        current = self._get_current_positions() if self.positions_loaded else None
        self._append(f"[{point_name}] START\n")
        result = self.ctrl.go_to_point(point_name, current)
        self._show_result(point_name, result)
        self._append(f"[{point_name}] DONE\n\n")

    def _adjust_degree(self, label: str, delta: float) -> None:
        if not self.positions_loaded:
            self._append(
                f"[adjust] BLOCKED {label}\n"
                "  Read Current Positions first.\n\n"
            )
            return

        try:
            current = float(self.positions[label].get())
        except ValueError:
            self._append(f"[adjust] ERROR {label}: current value is not a number.\n\n")
            return

        target = current + delta
        self.positions[label].set(f"{target:.2f}")
        self.adjusted_labels.add(label)
        self._append(f"[adjust] {label}: {current:.2f} -> {target:.2f} degree\n")

    def _run_selected_direct(self) -> None:
        if not self.ctrl.is_connected:
            self._append("[run] ERROR\n")
            self._append("  CAN bus is not connected. Press Connect first.\n\n")
            return

        if not self.positions_loaded:
            self._append("[run] BLOCKED\n")
            self._append("  Read Current Positions first.\n\n")
            return

        if not self.adjusted_labels:
            self._append("[run] BLOCKED\n")
            self._append("  No motor was adjusted. Use <-1 or +1 first.\n\n")
            return

        targets = {}
        for label in list(self.adjusted_labels):
            try:
                targets[label] = float(self.positions[label].get())
            except ValueError:
                self._append(f"[run] ERROR {label}: value is not a number.\n")
                continue

        self._append("[run] START\n")
        result = self.ctrl.run_targets(targets)
        self._show_result("run", result)
        self.adjusted_labels.clear()
        self._append("[run] DONE\n\n")

    def _stop_all_direct(self) -> None:
        if not self.ctrl.is_connected:
            self._append("[stop_all] ERROR\n")
            self._append("  CAN bus is not connected. If emergency, cut motor power directly.\n\n")
            return

        self._append("[stop_all] START\n")
        result = self.ctrl.stop_all()
        self._show_result("stop_all", result)
        self._append("[stop_all] DONE\n\n")

    def _shutdown_all_direct(self) -> None:
        if not self.ctrl.is_connected:
            self._append("[shutdown_all] ERROR\n")
            self._append("  CAN bus is not connected. If emergency, cut motor power directly.\n\n")
            return

        self._append("[shutdown_all] START\n")
        result = self.ctrl.shutdown_all()
        self._show_result("shutdown_all", result)
        self._append("[shutdown_all] DONE\n\n")

    def _save_current_position(self) -> None:
        point_name = self.save_point_name.get()
        if not self.positions_loaded:
            self._append("[save] ERROR\n  Read Current Positions first before saving.\n\n")
            return

        new_angles = {}
        for label in MOTORS:
            try:
                new_angles[label] = round(float(self.positions[label].get()), 2)
            except ValueError:
                self._append(f"[save] ERROR {label}: value is not a number.\n\n")
                return

        self.ctrl.save_position(point_name, new_angles)

        self._append(f'[save] OK saved "{point_name}" to point_config.json\n')
        for label, angle in new_angles.items():
            self._append(f"  {label}: {angle:.2f}\n")
        self._append("\n")

    def _calc_vacuum_only(self) -> None:
        try:
            offset = float(self.vacuum_offset.get())
        except ValueError:
            self._append("[calc] ERROR: offset is not a number.\n\n")
            return

        direction = self.vacuum_direction.get()
        result = self.ctrl.calc_vacuum_angles(direction, offset)

        self._append(f"[calc] Vacuum arm angles for direction={direction} offset={offset:.1f}cm\n")
        for key, value in result.items():
            self._append(f"  {key}: {value}\n")
        self._append("\n")

    def _move_vacuum_dynamic_direct(self) -> None:
        if not self.ctrl.is_connected:
            self._append("[vacuum_move] ERROR\n")
            self._append("  CAN bus is not connected. Press Connect first.\n\n")
            return

        if not self.positions_loaded:
            self._append("[vacuum_move] ERROR\n")
            self._append("  Read Current Positions first.\n\n")
            return

        try:
            offset = float(self.vacuum_offset.get())
        except ValueError:
            self._append("[vacuum_move] ERROR: offset is not a number.\n\n")
            return

        direction = self.vacuum_direction.get()
        current = self._get_current_positions()

        self._append(f"[vacuum_move] START direction={direction} offset={offset:.1f}cm\n")
        result = self.ctrl.move_vacuum_dynamic(direction, offset, current)

        if "error" in result:
            self._append(f"[vacuum_move] ERROR: {result['error']}\n\n")
            return
        if "safety" in result:
            self._append(f"[vacuum_move] {result['safety']}\n\n")
            return

        self._show_result("vacuum_move", result)
        self._append("[vacuum_move] DONE\n\n")

    # ─── Helpers ────────────────────────────────────────────────────────

    def _get_current_positions(self) -> dict[str, float]:
        """Get current position values from UI as floats."""
        positions = {}
        for label in MOTORS:
            try:
                positions[label] = float(self.positions[label].get())
            except ValueError:
                pass
        return positions

    def _show_result(self, label: str, result) -> None:
        if label == "connect":
            self.status.set("Connected")
        elif label == "disconnect":
            self.status.set("Disconnected")

        if isinstance(result, dict) and "_updates" in result:
            updates = result["_updates"]
            for motor_label, value in updates.items():
                self.positions[motor_label].set(value)

            if updates and result.get("_degree_positions", False):
                self.positions_loaded = True
                self.adjusted_labels.clear()
                self._append("  positions_loaded=True. You may adjust and run motors.\n")

        self._append(f"[{label}] OK\n")
        if isinstance(result, dict):
            for key, value in result.items():
                if key.startswith("_"):
                    continue
                self._append(f"  {key}: {value}\n")
        else:
            self._append(f"  {result}\n")
        self._append("\n")

    def _show_error(self, label: str, message: str) -> None:
        self._append(f"[{label}] ERROR\n  {message}\n\n")

    def _append(self, text: str) -> None:
        self.output.insert(END, text)
        self.output.see(END)
        self._write_log(text)

    def _write_log(self, text: str) -> None:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with self.log_file.open("a", encoding="utf-8") as file:
            for line in text.splitlines():
                file.write(f"{timestamp} {line}\n")

    @staticmethod
    def _create_log_file() -> Path:
        now = datetime.now()
        log_dir = Path(__file__).with_name("log") / now.strftime("%Y%m%d")
        log_dir.mkdir(parents=True, exist_ok=True)
        return log_dir / f"2motor_sync_{now.strftime('%H%M%S')}.log"


def main() -> None:
    root = Tk()
    FourMotorSyncUi(root)
    root.mainloop()


if __name__ == "__main__":
    main()
