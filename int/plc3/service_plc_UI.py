from __future__ import annotations

import queue
import threading
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk
from typing import Any

from api import PLCConnectionError
from service_plc import ActionResult, ServicePLC


class ServicePlcUiApp:
    monitor_ids = ("Y1_CUR_POS", "Y2_CUR_POS", "X_CUR_POS")
    move_ids = ("Y1_MOVE", "Y2_MOVE", "X_MOVE")

    def __init__(self, root: tk.Tk, service: ServicePLC | None = None) -> None:
        self.root = root
        self.root.title("PLC Service UI")
        self.root.geometry("1380x900")
        self.root.minsize(1200, 760)
        self.service = service or ServicePLC()
        self.queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.current_group = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="未連線")
        self.message_var = tk.StringVar(value="-")
        self.monitor_vars = {point_id: tk.StringVar(value="-") for point_id in self.monitor_ids}
        self.move_vars = {point_id: tk.BooleanVar(value=False) for point_id in self.move_ids}
        self.backhome_timeout_var = tk.StringVar(value="60")
        self.moveto_position_var = tk.StringVar(value="none")
        self.moveto_action_var = tk.StringVar(value="none")
        self.moveto_height_var = tk.StringVar(value="0")
        self.moveto_depth_var = tk.StringVar(value="0")
        self.moveto_timeout_var = tk.StringVar(value="60")
        self.vacuum_action_var = tk.BooleanVar(value=True)
        self.vacuum_timeout_var = tk.StringVar(value="60")
        self.movetox_height_var = tk.StringVar(value="0")
        self.movetox_timeout_var = tk.StringVar(value="60")
        self._build_ui()
        self._render_groups()
        self._process_queue()
        self._schedule_monitor()

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=12)
        container.pack(fill="both", expand=True)
        container.columnconfigure(0, weight=1)
        container.rowconfigure(3, weight=1)
        container.rowconfigure(4, weight=1)

        top = ttk.Frame(container)
        top.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        top.columnconfigure(1, weight=1)
        ttk.Label(top, text="PLC 操作測試 UI", font=("", 16, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=1, sticky="w", padx=(16, 0))
        ttk.Button(top, text="連線", command=lambda: self._run_async("連線", self.service.connect)).grid(row=0, column=2, padx=(0, 8))
        ttk.Button(top, text="斷線", command=lambda: self._run_async("斷線", self.service.disconnect)).grid(row=0, column=3)

        monitor_frame = ttk.LabelFrame(container, text="即時監控 / 即時開關", padding=10)
        monitor_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        for col in range(6):
            monitor_frame.columnconfigure(col, weight=1)
        for idx, point_id in enumerate(self.monitor_ids):
            ttk.Label(monitor_frame, text=point_id).grid(row=0, column=idx * 2, sticky="w")
            ttk.Label(monitor_frame, textvariable=self.monitor_vars[point_id]).grid(row=0, column=idx * 2 + 1, sticky="w")
        for idx, point_id in enumerate(self.move_ids):
            ttk.Checkbutton(
                monitor_frame,
                text=point_id,
                variable=self.move_vars[point_id],
                command=lambda pid=point_id: self._toggle_move(pid),
            ).grid(row=1, column=idx * 2, columnspan=2, sticky="w", pady=(8, 0))

        function_frame = ttk.LabelFrame(container, text="四個重要 Function", padding=10)
        function_frame.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        for col in range(8):
            function_frame.columnconfigure(col, weight=1)

        ttk.Label(function_frame, text="backHOME timeout").grid(row=0, column=0, sticky="w")
        ttk.Entry(function_frame, textvariable=self.backhome_timeout_var, width=8).grid(row=0, column=1, sticky="w")
        ttk.Button(function_frame, text="執行 backHOME", command=self._call_backhome).grid(row=0, column=2, sticky="w")

        ttk.Label(function_frame, text="moveTo position").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(function_frame, textvariable=self.moveto_position_var, values=("none", "left", "right"), width=8, state="readonly").grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Label(function_frame, text="action").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Combobox(function_frame, textvariable=self.moveto_action_var, values=("none", "pull", "push"), width=8, state="readonly").grid(row=1, column=3, sticky="w", pady=(8, 0))
        ttk.Label(function_frame, text="height").grid(row=1, column=4, sticky="w", pady=(8, 0))
        ttk.Entry(function_frame, textvariable=self.moveto_height_var, width=8).grid(row=1, column=5, sticky="w", pady=(8, 0))
        ttk.Label(function_frame, text="depth").grid(row=1, column=6, sticky="w", pady=(8, 0))
        ttk.Entry(function_frame, textvariable=self.moveto_depth_var, width=8).grid(row=1, column=7, sticky="w", pady=(8, 0))
        ttk.Label(function_frame, text="timeout").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(function_frame, textvariable=self.moveto_timeout_var, width=8).grid(row=2, column=1, sticky="w", pady=(8, 0))
        ttk.Button(function_frame, text="執行 moveTo", command=self._call_moveto).grid(row=2, column=2, sticky="w", pady=(8, 0))

        ttk.Checkbutton(function_frame, text="VAC action(True=ON)", variable=self.vacuum_action_var).grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Label(function_frame, text="timeout").grid(row=3, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(function_frame, textvariable=self.vacuum_timeout_var, width=8).grid(row=3, column=3, sticky="w", pady=(8, 0))
        ttk.Button(function_frame, text="執行 VAC", command=self._call_vacuum).grid(row=3, column=4, sticky="w", pady=(8, 0))

        ttk.Label(function_frame, text="moveToX height").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(function_frame, textvariable=self.movetox_height_var, width=8).grid(row=4, column=1, sticky="w", pady=(8, 0))
        ttk.Label(function_frame, text="timeout").grid(row=4, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(function_frame, textvariable=self.movetox_timeout_var, width=8).grid(row=4, column=3, sticky="w", pady=(8, 0))
        ttk.Button(function_frame, text="執行 moveToX", command=self._call_movetox).grid(row=4, column=4, sticky="w", pady=(8, 0))

        middle = ttk.PanedWindow(container, orient="horizontal")
        middle.grid(row=3, column=0, sticky="nsew", pady=(0, 10))

        group_frame = ttk.LabelFrame(middle, text="Points Group", padding=10)
        group_frame.rowconfigure(0, weight=1)
        group_frame.columnconfigure(0, weight=1)
        self.group_list = tk.Listbox(group_frame, exportselection=False)
        self.group_list.grid(row=0, column=0, sticky="nsew")
        self.group_list.bind("<<ListboxSelect>>", self._on_group_selected)
        middle.add(group_frame, weight=1)

        point_frame = ttk.LabelFrame(middle, text="Points Grid", padding=10)
        point_frame.rowconfigure(0, weight=1)
        point_frame.columnconfigure(0, weight=1)
        columns = ("id", "name", "device", "address", "type", "writable", "value")
        self.point_table = ttk.Treeview(point_frame, columns=columns, show="headings")
        for col in columns:
            self.point_table.heading(col, text=col)
        self.point_table.column("id", width=170, anchor="w")
        self.point_table.column("name", width=220, anchor="w")
        self.point_table.column("device", width=60, anchor="center")
        self.point_table.column("address", width=70, anchor="e")
        self.point_table.column("type", width=70, anchor="center")
        self.point_table.column("writable", width=70, anchor="center")
        self.point_table.column("value", width=120, anchor="center")
        self.point_table.grid(row=0, column=0, sticky="nsew")
        self.point_table.bind("<Double-1>", self._edit_point)
        point_scroll = ttk.Scrollbar(point_frame, orient="vertical", command=self.point_table.yview)
        point_scroll.grid(row=0, column=1, sticky="ns")
        self.point_table.configure(yscrollcommand=point_scroll.set)
        middle.add(point_frame, weight=4)

        bottom = ttk.PanedWindow(container, orient="horizontal")
        bottom.grid(row=4, column=0, sticky="nsew")

        status_frame = ttk.LabelFrame(bottom, text="Function Status", padding=10)
        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(0, weight=1)
        ttk.Label(status_frame, textvariable=self.message_var, wraplength=500).grid(row=0, column=0, sticky="nw")
        bottom.add(status_frame, weight=2)

        log_frame = ttk.LabelFrame(bottom, text="Log", padding=10)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, height=12, state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=log_scroll.set)
        bottom.add(log_frame, weight=3)

    def _render_groups(self) -> None:
        self.group_list.delete(0, "end")
        for group in self.service.list_groups():
            self.group_list.insert("end", group)
        if self.group_list.size() > 0:
            self.group_list.selection_set(0)
            self._on_group_selected(None)

    def _schedule_monitor(self) -> None:
        threading.Thread(target=self._load_monitor, daemon=True).start()
        self.root.after(500, self._schedule_monitor)

    def _load_monitor(self) -> None:
        try:
            snapshot = self.service.read_monitor_snapshot()
            self.queue.put(("monitor", snapshot))
        except Exception as exc:
            self.queue.put(("error", str(exc)))

    def _process_queue(self) -> None:
        while True:
            try:
                event, payload = self.queue.get_nowait()
            except queue.Empty:
                break
            if event == "monitor":
                self._render_monitor(payload)
            elif event == "message":
                self.message_var.set(str(payload))
            elif event == "action":
                self._handle_action_result(payload)
            elif event == "error":
                self.message_var.set(str(payload))
            elif event == "point_refresh":
                self._refresh_points()
        self._refresh_log_view()
        self.root.after(100, self._process_queue)

    def _render_monitor(self, snapshot: dict[str, Any]) -> None:
        for point_id in self.monitor_ids:
            self.monitor_vars[point_id].set(str(snapshot.get(point_id, "-")))
        connected = "已連線" if snapshot.get("connected") else "未連線"
        self.status_var.set(f"{connected} / {snapshot.get('service_status', '-')}")
        current_message = str(snapshot.get("service_message", "-"))
        if current_message and current_message != "-":
            self.message_var.set(current_message)
        self._refresh_points()

    def _refresh_points(self) -> None:
        group = self.current_group.get() or None
        points = self.service.list_points(group)
        self.point_table.delete(*self.point_table.get_children())
        for point in points:
            try:
                value = self.service.api.read_point(point.id) if self.service.api.is_connected(point.plc) else "-"
            except Exception as exc:
                value = f"ERR: {exc}"
            self.point_table.insert(
                "",
                "end",
                iid=point.id,
                values=(point.id, point.name, point.device, point.address, point.type, "Y" if point.writable else "N", value),
            )

    def _on_group_selected(self, _event: Any) -> None:
        selected = self.group_list.curselection()
        if not selected:
            return
        self.current_group.set(self.group_list.get(selected[0]))
        self._refresh_points()

    def _toggle_move(self, point_id: str) -> None:
        enabled = self.move_vars[point_id].get()
        self._run_async(point_id, self.service.toggle_move, point_id, enabled)

    def _edit_point(self, _event: Any) -> None:
        selected = self.point_table.selection()
        if not selected:
            return
        point_id = selected[0]
        point = self.service.get_point(point_id)
        if not point.writable:
            messagebox.showinfo("不可修改", f"{point.id} 不可修改", parent=self.root)
            return
        try:
            current = self.service.read_point(point.id)
        except Exception:
            current = ""
        if point.device == "M" or point.type.lower() == "bit":
            answer = messagebox.askyesno("寫入 bit", f"{point.id}\n目前值: {current}\n按 Yes 寫入 ON，No 寫入 OFF", parent=self.root)
            self._run_async(f"write {point.id}", self.service.write_point, point.id, answer)
            return
        answer = simpledialog.askstring("寫入 D", f"{point.id} ({point.type})\n目前值: {current}\n請輸入新值", initialvalue=str(current), parent=self.root)
        if answer is None:
            return
        self._run_async(f"write {point.id}", self.service.write_point, point.id, answer)

    def _call_backhome(self) -> None:
        timeout = float(self.backhome_timeout_var.get())
        self._run_async("backHOME", self.service.back_home, timeout)

    def _call_moveto(self) -> None:
        position = self.moveto_position_var.get()
        action = self.moveto_action_var.get()
        height = int(self.moveto_height_var.get())
        depth = int(self.moveto_depth_var.get())
        timeout = float(self.moveto_timeout_var.get())
        self._run_async("moveTo", self.service.move_to, position, action, height, depth, timeout)

    def _call_vacuum(self) -> None:
        timeout = float(self.vacuum_timeout_var.get())
        self._run_async("VAC", self.service.vacuum, self.vacuum_action_var.get(), timeout)

    def _call_movetox(self) -> None:
        height = int(self.movetox_height_var.get())
        timeout = float(self.movetox_timeout_var.get())
        self._run_async("moveToX", self.service.move_to_x, height, timeout)

    def _run_async(self, label: str, func: Any, *args: Any) -> None:
        self.message_var.set(f"{label} 執行中...")

        def worker() -> None:
            try:
                result = func(*args)
                self.queue.put(("action", result))
            except PLCConnectionError as exc:
                self.queue.put(("error", str(exc)))
            except Exception as exc:
                self.queue.put(("error", str(exc)))
            finally:
                self.queue.put(("point_refresh", None))

        threading.Thread(target=worker, daemon=True).start()

    def _handle_action_result(self, payload: Any) -> None:
        if isinstance(payload, ActionResult):
            self.message_var.set(
                f"status={payload.status} message={payload.message} duration={payload.duration:.2f}s data={payload.data}"
            )
        elif isinstance(payload, str):
            self.message_var.set(payload)
        elif payload is not None:
            self.message_var.set(str(payload))

    def _refresh_log_view(self) -> None:
        logs = self.service.get_logs(300)
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.insert("1.0", "\n".join(logs))
        self.log_text.see("end")
        self.log_text.configure(state="disabled")


def main() -> None:
    root = tk.Tk()
    app = ServicePlcUiApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
