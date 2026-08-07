from __future__ import annotations

import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any, Callable

from service_plc import PlcPointValue, ServicePlc


class ServicePlcUi(tk.Tk):
    columns = ("id", "name", "device", "address", "type", "value", "raw", "control")

    def __init__(self) -> None:
        super().__init__()
        self.title("PLC MC Protocol Service Test UI")
        self.geometry("1040x680")
        self.minsize(900, 560)

        self.service = ServicePlc()
        self.result_queue: queue.Queue[tuple[str, bool, object]] = queue.Queue()
        self.action_running = False
        self.status_var = tk.StringVar(value="未連線")
        self.d_value_var = tk.StringVar(value="")

        self._build_ui()
        self.after(100, self._poll_results)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        top = ttk.Frame(self, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        ttk.Button(top, text="PLC 連線", command=lambda: self._run("connect", self.service.connect)).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(top, text="PLC 斷線", command=lambda: self._run("disconnect", self.service.disconnect)).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(top, text="讀取全部", command=lambda: self._run("read_all", self.service.read_all_points)).pack(
            side=tk.LEFT, padx=(0, 12)
        )
        connection = self.service.connection
        ttk.Label(top, text=f"{connection.name}  {connection.host}:{connection.port}").pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.RIGHT)

        body = ttk.Frame(self, padding=(10, 0, 10, 10))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)

        self.table = ttk.Treeview(body, columns=self.columns, show="headings", selectmode="browse")
        headings = {
            "id": "ID",
            "name": "名稱",
            "device": "裝置",
            "address": "位址",
            "type": "型態",
            "value": "數值",
            "raw": "Raw",
            "control": "控制",
        }
        widths = {
            "id": 180,
            "name": 240,
            "device": 70,
            "address": 80,
            "type": 90,
            "value": 120,
            "raw": 130,
            "control": 180,
        }
        for column in self.columns:
            self.table.heading(column, text=headings[column])
            self.table.column(column, width=widths[column], anchor=tk.CENTER if column != "name" else tk.W)
        self.table.grid(row=0, column=0, sticky="nsew")
        self.table.bind("<<TreeviewSelect>>", self._on_selected)
        self.table.bind("<Double-1>", self._on_double_click)
        scrollbar = ttk.Scrollbar(body, orient=tk.VERTICAL, command=self.table.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.table.configure(yscrollcommand=scrollbar.set)

        editor = ttk.Frame(self, padding=(10, 0, 10, 10))
        editor.grid(row=2, column=0, sticky="ew")
        ttk.Label(editor, text="D 數值").pack(side=tk.LEFT, padx=(0, 6))
        ttk.Entry(editor, textvariable=self.d_value_var, width=14).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(editor, text="寫入選取 D", command=self._write_selected_d).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(editor, text="切換選取 M", command=self._toggle_selected_m).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(editor, text="讀取選取", command=self._read_selected).pack(side=tk.LEFT)

        self._render_defaults()

    def _render_defaults(self) -> None:
        self.table.delete(*self.table.get_children())
        for point in self.service.points:
            control = "SWITCH" if point.device == "M" else ("數值輸入" if point.writable else "唯讀")
            self.table.insert(
                "",
                tk.END,
                iid=point.id,
                values=(
                    point.id,
                    point.name,
                    point.device,
                    point.address,
                    self._point_type_text(point),
                    self._format_value(point.default),
                    "-",
                    control,
                ),
            )

    def _run(self, label: str, action: Callable[[], Any]) -> None:
        if self.action_running:
            self.status_var.set("忙碌中")
            return
        self.action_running = True
        self.status_var.set(f"{label} 執行中...")

        def worker() -> None:
            try:
                self.result_queue.put((label, True, action()))
            except Exception as exc:  # noqa: BLE001
                self.result_queue.put((label, False, str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def _poll_results(self) -> None:
        while True:
            try:
                label, ok, result = self.result_queue.get_nowait()
            except queue.Empty:
                break
            self.action_running = False
            if ok:
                self._handle_result(label, result)
            else:
                self.status_var.set(f"{label} 失敗")
                messagebox.showerror("PLC 錯誤", str(result), parent=self)
        self.after(100, self._poll_results)

    def _handle_result(self, label: str, result: Any) -> None:
        if label == "read_all" and isinstance(result, list):
            self._apply_point_values(result)
        elif label.startswith("read:") and isinstance(result, tuple):
            point_id, value, raw = result
            self._set_row_value(str(point_id), value, raw)
        elif label.startswith("write:") and isinstance(result, tuple):
            point_id, value = result
            self._set_row_value(str(point_id), value)
        self.status_var.set(f"{label} OK")

    def _apply_point_values(self, values: list[PlcPointValue]) -> None:
        for item in values:
            self._set_row_value(item.point.id, item.error or item.value, item.raw)

    def _set_row_value(self, point_id: str, value: Any, raw: Any = None) -> None:
        if not self.table.exists(point_id):
            return
        values = list(self.table.item(point_id, "values"))
        values[5] = self._format_value(value)
        if raw is not None:
            values[6] = str(raw)
        self.table.item(point_id, values=values)

    def _on_selected(self, _event: tk.Event) -> None:
        point = self._selected_point()
        if point is None or point.device != "D":
            return
        current = self.table.item(point.id, "values")[5]
        self.d_value_var.set(str(current))

    def _on_double_click(self, event: tk.Event) -> None:
        row_id = self.table.identify_row(event.y)
        if row_id:
            self.table.selection_set(row_id)
            self.table.focus(row_id)

        point = self._selected_point()
        if point is None:
            return
        if point.device == "M":
            self._toggle_selected_m()
        elif point.device == "D" and point.writable:
            self._open_d_edit_dialog(point.id)

    def _read_selected(self) -> None:
        point = self._selected_point()
        if point is None:
            return
        self._run(f"read:{point.id}", lambda: (point.id, *self.service.read_point_with_raw(point.id)))

    def _write_selected_d(self) -> None:
        point = self._selected_point()
        if point is None:
            return
        if point.device != "D":
            messagebox.showinfo("不是 D 點位", "請選取 D 數值點位。", parent=self)
            return
        if not point.writable:
            messagebox.showinfo("唯讀", f"{point.name} 不可寫入。", parent=self)
            return
        value = float(self.d_value_var.get())

        def action() -> tuple[str, float]:
            self.service.write_point(point.id, value)
            return point.id, value

        self._run(f"write:{point.id}", action)

    def _open_d_edit_dialog(self, point_id: str) -> None:
        point = self.service.get_point(point_id)
        if point.device != "D":
            return
        if not point.writable:
            messagebox.showinfo("唯讀", f"{point.name} 不可寫入。", parent=self)
            return

        current = self.table.item(point.id, "values")[5]
        dialog = tk.Toplevel(self)
        dialog.title(f"寫入 {point.name}")
        dialog.transient(self)
        dialog.geometry("380x170")
        dialog.minsize(360, 160)
        dialog.resizable(False, False)
        dialog.columnconfigure(0, weight=0)
        dialog.columnconfigure(1, weight=1)

        value_var = tk.StringVar(value=str(current))
        ttk.Label(
            dialog,
            text=f"{point.name}\n{point.id}  D{point.address}  {self._point_type_text(point)}",
            justify=tk.LEFT,
        ).grid(
            row=0,
            column=0,
            columnspan=2,
            sticky="w",
            padx=12,
            pady=(12, 6),
        )
        ttk.Label(dialog, text="數值").grid(row=1, column=0, sticky="w", padx=(12, 8), pady=6)
        entry = ttk.Entry(dialog, textvariable=value_var, width=18)
        entry.grid(row=1, column=1, sticky="ew", padx=(0, 12), pady=6)

        def submit() -> None:
            try:
                value = float(value_var.get())
            except ValueError:
                messagebox.showerror("數值錯誤", f"無法轉換為數值: {value_var.get()}", parent=dialog)
                return

            def action() -> tuple[str, float]:
                self.service.write_point(point.id, value)
                return point.id, value

            self._run(f"write:{point.id}", action)
            dialog.destroy()

        buttons = ttk.Frame(dialog)
        buttons.grid(row=2, column=0, columnspan=2, sticky="e", padx=12, pady=(6, 12))
        ttk.Button(buttons, text="寫入", command=submit).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(buttons, text="取消", command=dialog.destroy).pack(side=tk.LEFT)
        entry.focus_set()
        entry.selection_range(0, tk.END)
        dialog.bind("<Return>", lambda _event: submit())
        dialog.bind("<Escape>", lambda _event: dialog.destroy())
        self._center_dialog(dialog)
        dialog.wait_visibility()
        try:
            dialog.grab_set()
        except tk.TclError:
            dialog.after(50, dialog.grab_set)

    def _center_dialog(self, dialog: tk.Toplevel) -> None:
        dialog.update_idletasks()
        width = max(dialog.winfo_width(), dialog.winfo_reqwidth())
        height = max(dialog.winfo_height(), dialog.winfo_reqheight())
        parent_x = self.winfo_rootx()
        parent_y = self.winfo_rooty()
        parent_w = self.winfo_width()
        parent_h = self.winfo_height()
        x = parent_x + max((parent_w - width) // 2, 0)
        y = parent_y + max((parent_h - height) // 2, 0)
        dialog.geometry(f"{width}x{height}+{x}+{y}")

    def _toggle_selected_m(self) -> None:
        point = self._selected_point()
        if point is None:
            return
        if point.device != "M":
            messagebox.showinfo("不是 M 點位", "請選取 M switch 點位。", parent=self)
            return

        def action() -> tuple[str, bool]:
            return point.id, self.service.toggle_m_point(point.id)

        self._run(f"write:{point.id}", action)

    def _selected_point(self):
        selection = self.table.selection()
        if not selection:
            return None
        point_id = selection[0]
        try:
            return self.service.get_point(point_id)
        except KeyError:
            return None

    @staticmethod
    def _format_value(value: Any) -> str:
        if isinstance(value, bool):
            return "1" if value else "0"
        if isinstance(value, float):
            return f"{value:.1f}"
        return str(value)

    @staticmethod
    def _point_type_text(point) -> str:
        if point.device == "M":
            return "switch"
        if point.scale != 1.0:
            return f"{point.type} x{point.scale:g}"
        return point.type

    def _on_close(self) -> None:
        try:
            self.service.disconnect()
        except Exception:
            pass
        self.destroy()


def main() -> None:
    app = ServicePlcUi()
    app.mainloop()


if __name__ == "__main__":
    main()
