#!/usr/bin/env python3
"""Tkinter UI for pasting, validating, and running whitelisted Actions."""

from __future__ import annotations

import queue
import threading
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from fractions import Fraction
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Any

from action import (
    DEFAULT_ACTION_FILE,
    ActionFileError,
    build_action_registry,
    build_services,
    dated_log_path,
    execute_actions,
    parse_action_text,
    validate_action_names,
)


@dataclass(slots=True)
class ConfirmationRequest:
    event: threading.Event
    accepted: bool = False


class ActionUi:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("AMR Action Runner")
        self.root.geometry("1280x820")
        self.root.minsize(900, 600)

        self.status = tk.StringVar(root, "待命")
        self._messages: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._running = False
        self._services: tuple[Any, Any, Any, Any] | None = None
        self._worker: threading.Thread | None = None
        self._log_path = dated_log_path()
        self._log_lock = threading.Lock()
        self._result_image_path = Path.cwd() / "control_result.png"
        self._result_image_signature: tuple[int, int] | None = None
        self._result_image_widget_size: tuple[int, int] | None = None
        self._result_photo: tk.PhotoImage | None = None

        self._build()
        self._load_default(show_error=False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(50, self._poll_messages)
        self.root.after(100, self._poll_result_image)

    def _build(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        toolbar = ttk.Frame(outer)
        toolbar.pack(fill="x", pady=(0, 8))
        ttk.Button(toolbar, text="載入 action.txt", command=self._load_default).pack(
            side="left", padx=(0, 5)
        )
        self.validate_button = ttk.Button(toolbar, text="檢查語法", command=self._validate)
        self.validate_button.pack(side="left", padx=5)
        self.run_button = ttk.Button(toolbar, text="執行", command=self._run)
        self.run_button.pack(side="left", padx=5)
        self.reset_button = ttk.Button(toolbar, text="Reset", command=self._reset)
        self.reset_button.pack(side="left", padx=5)
        ttk.Button(toolbar, text="清除結果", command=self._clear_output).pack(
            side="left", padx=5
        )
        ttk.Button(toolbar, text="複製選取", command=self._copy_selected).pack(
            side="left", padx=5
        )
        ttk.Button(toolbar, text="複製全部", command=self._copy_all).pack(
            side="left", padx=5
        )
        self.stop_button = ttk.Button(toolbar, text="STOP ALL", command=self._stop_all)
        self.stop_button.pack(side="left", padx=15)
        ttk.Label(toolbar, textvariable=self.status).pack(side="right")

        panes = ttk.Panedwindow(outer, orient="horizontal")
        panes.pack(fill="both", expand=True)

        editor_frame = ttk.LabelFrame(panes, text="Action 文字（可直接貼上）", padding=6)
        output_frame = ttk.LabelFrame(panes, text="執行結果與錯誤", padding=6)
        panes.add(editor_frame, weight=1)
        panes.add(output_frame, weight=1)

        editor_panes = ttk.Panedwindow(editor_frame, orient="vertical")
        editor_panes.pack(fill="both", expand=True)

        editor_text_frame = ttk.Frame(editor_panes)
        image_frame = ttk.LabelFrame(
            editor_panes,
            text=f"偵測影像：{self._result_image_path.name}",
            padding=6,
        )
        editor_panes.add(editor_text_frame, weight=1)
        editor_panes.add(image_frame, weight=1)

        self.editor = self._editor_with_line_numbers(editor_text_frame)
        self.result_image_label = ttk.Label(
            image_frame,
            text=f"等待圖片更新：\n{self._result_image_path}",
            anchor="center",
            justify="center",
        )
        self.result_image_label.pack(fill="both", expand=True)
        self.output = self._text_with_scrollbar(output_frame, editable=False)
        self.output.tag_configure("error", foreground="#b00020")
        self.output.tag_configure("success", foreground="#087f23")
        self.output.tag_configure("info", foreground="#164a8a")
        self.output.bind("<Button-3>", self._show_copy_menu)
        self.output.bind("<Control-c>", lambda _event: self._copy_selected())

        self.copy_menu = tk.Menu(self.root, tearoff=False)
        self.copy_menu.add_command(label="複製選取", command=self._copy_selected)
        self.copy_menu.add_command(label="複製全部", command=self._copy_all)

        ttk.Label(
            outer,
            text=(
                "Reset 會保留 Action 文字、停止/斷開現有服務並重建狀態；"
                f"Reset 完成後請再次按「執行」。Log：{self._log_path}"
            ),
        ).pack(fill="x", pady=(8, 0))

    @staticmethod
    def _text_with_scrollbar(parent: tk.Misc, *, editable: bool) -> tk.Text:
        container = ttk.Frame(parent)
        container.pack(fill="both", expand=True)
        container.rowconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)
        text = tk.Text(container, wrap="none", undo=editable, font=("TkFixedFont", 11))
        text.grid(row=0, column=0, sticky="nsew")
        ybar = ttk.Scrollbar(container, orient="vertical", command=text.yview)
        ybar.grid(row=0, column=1, sticky="ns")
        xbar = ttk.Scrollbar(container, orient="horizontal", command=text.xview)
        xbar.grid(row=1, column=0, sticky="ew")
        text.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        if not editable:
            text.configure(state="disabled")
        return text

    def _editor_with_line_numbers(self, parent: tk.Misc) -> tk.Text:
        container = ttk.Frame(parent)
        container.pack(fill="both", expand=True)
        container.rowconfigure(0, weight=1)
        container.columnconfigure(1, weight=1)

        self.line_numbers = tk.Text(
            container,
            width=5,
            padx=4,
            takefocus=False,
            state="disabled",
            wrap="none",
            font=("TkFixedFont", 11),
            background="#eeeeee",
            foreground="#666666",
            relief="flat",
        )
        self.line_numbers.grid(row=0, column=0, sticky="ns")

        text = tk.Text(
            container, wrap="none", undo=True, font=("TkFixedFont", 11)
        )
        text.grid(row=0, column=1, sticky="nsew")

        def scroll_vertical(first: str, last: str) -> None:
            ybar.set(first, last)
            self.line_numbers.yview_moveto(first)

        def move_vertical(*args: str) -> None:
            text.yview(*args)
            self.line_numbers.yview(*args)

        ybar = ttk.Scrollbar(container, orient="vertical", command=move_vertical)
        ybar.grid(row=0, column=2, sticky="ns")
        xbar = ttk.Scrollbar(container, orient="horizontal", command=text.xview)
        xbar.grid(row=1, column=1, sticky="ew")
        text.configure(yscrollcommand=scroll_vertical, xscrollcommand=xbar.set)

        def schedule_update(_event: tk.Event[Any] | None = None) -> None:
            self.root.after_idle(self._update_line_numbers)

        for event_name in (
            "<KeyRelease>",
            "<ButtonRelease-1>",
            "<MouseWheel>",
            "<Button-4>",
            "<Button-5>",
            "<Configure>",
            "<<Paste>>",
            "<<Cut>>",
            "<<Undo>>",
            "<<Redo>>",
        ):
            text.bind(event_name, schedule_update, add="+")
        self.root.after_idle(self._update_line_numbers)
        return text

    def _update_line_numbers(self) -> None:
        if not hasattr(self, "editor") or not hasattr(self, "line_numbers"):
            return
        line_count = int(self.editor.index("end-1c").split(".")[0])
        content = "\n".join(str(number) for number in range(1, line_count + 1))
        first, _last = self.editor.yview()
        self.line_numbers.configure(state="normal")
        self.line_numbers.delete("1.0", "end")
        self.line_numbers.insert("1.0", content)
        self.line_numbers.configure(state="disabled")
        self.line_numbers.yview_moveto(first)

    def _load_default(self, show_error: bool = True) -> None:
        if self._running:
            return
        try:
            content = DEFAULT_ACTION_FILE.read_text(encoding="utf-8")
        except OSError as exc:
            if show_error:
                messagebox.showerror("載入失敗", str(exc), parent=self.root)
            return
        self.editor.delete("1.0", "end")
        self.editor.insert("1.0", content)
        self.status.set(f"已載入 {DEFAULT_ACTION_FILE.name}")

    def _parse_editor(self) -> list[Any]:
        actions = parse_action_text(self.editor.get("1.0", "end-1c"))
        validate_action_names(actions)
        if not actions:
            raise ActionFileError("沒有可執行的 Action")
        return actions

    def _validate(self) -> None:
        try:
            actions = self._parse_editor()
        except ActionFileError as exc:
            message = f"語法檢查失敗：{exc}"
            self._append(message + "\n", "error")
            self._write_log(message)
            self.status.set("語法錯誤")
            return
        message = f"語法檢查完成：共 {len(actions)} 個 Action。"
        self._append(message + "\n", "success")
        self._write_log(message)
        self.status.set("語法正確")

    def _run(self) -> None:
        if self._running:
            return
        try:
            actions = self._parse_editor()
        except ActionFileError as exc:
            message = f"ERROR: {exc}"
            self._append(message + "\n", "error")
            self._write_log(message)
            self.status.set("語法錯誤")
            return

        self._set_running(True, "執行中")
        message = f"開始執行，共 {len(actions)} 個 Action。"
        self._append("\n" + message + "\n", "info")
        self._write_log(message)
        self._worker = threading.Thread(
            target=self._execute_worker, args=(actions,), daemon=True
        )
        self._worker.start()

    def _execute_worker(self, actions: list[Any]) -> None:
        try:
            if self._services is None:
                self._services = build_services()
            registry = build_action_registry(*self._services)
            registry["hunam_ctrl"] = self._confirm_from_worker
            execute_actions(actions, registry, output=self._queue_output)
        except Exception as exc:
            message = f"{type(exc).__name__}: {exc}"
            self._write_log(f"ERROR: {message}")
            self._messages.put(("failed", message))
        else:
            self._write_log(f"全部完成，共 {len(actions)} 個 Action。")
            self._messages.put(("completed", len(actions)))

    def _confirm_from_worker(self) -> dict[str, bool]:
        request = ConfirmationRequest(threading.Event())
        self._messages.put(("confirm", request))
        request.event.wait()
        if not request.accepted:
            raise RuntimeError("操作員取消執行")
        return {"success": True, "confirmed": True}

    def _queue_output(self, message: str, **_kwargs: Any) -> None:
        self._write_log(message)
        self._messages.put(("output", message))

    def _write_log(self, message: str) -> None:
        timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
        try:
            with self._log_lock:
                self._log_path.parent.mkdir(parents=True, exist_ok=True)
                with self._log_path.open("a", encoding="utf-8") as log_file:
                    print(f"{timestamp} [UI] {message}", file=log_file)
        except OSError as exc:
            self._messages.put(("log_error", str(exc)))

    def _poll_messages(self) -> None:
        try:
            while True:
                kind, payload = self._messages.get_nowait()
                if kind == "output":
                    self._append(str(payload) + "\n")
                elif kind == "completed":
                    self._append(f"全部完成，共 {payload} 個 Action。\n", "success")
                    self._set_running(False, "完成")
                elif kind == "failed":
                    self._append(f"ERROR: {payload}\n", "error")
                    self._set_running(False, "執行失敗，請檢查後 Reset")
                elif kind == "reset_done":
                    self._append("Reset 完成，可以重新執行。\n", "success")
                    self._set_running(False, "Reset 完成")
                elif kind == "reset_failed":
                    self._append(f"Reset 部分失敗：{payload}\n", "error")
                    self._set_running(False, "Reset 完成（有警告）")
                elif kind == "confirm":
                    accepted = messagebox.askokcancel(
                        "人為確認",
                        "Action 執行到 hunam_ctrl()。\n確認現場安全後按「確定」繼續。",
                        parent=self.root,
                    )
                    payload.accepted = accepted
                    self._write_log(
                        "hunam_ctrl 操作員確認繼續" if accepted else "hunam_ctrl 操作員取消"
                    )
                    payload.event.set()
                elif kind == "log_error":
                    self._append(f"Log 寫入失敗：{payload}\n", "error")
        except queue.Empty:
            pass
        self.root.after(50, self._poll_messages)

    def _poll_result_image(self) -> None:
        """Reload control_result.png whenever the detector updates the file."""
        try:
            stat = self._result_image_path.stat()
            signature = (stat.st_mtime_ns, stat.st_size)
        except OSError:
            signature = None

        widget_size = (
            max(self.result_image_label.winfo_width() - 12, 1),
            max(self.result_image_label.winfo_height() - 12, 1),
        )
        size_changed = (
            self._result_image_widget_size is None
            or abs(widget_size[0] - self._result_image_widget_size[0]) >= 10
            or abs(widget_size[1] - self._result_image_widget_size[1]) >= 10
        )

        if signature is not None and (
            signature != self._result_image_signature or size_changed
        ):
            try:
                photo = tk.PhotoImage(file=str(self._result_image_path))
                scale = min(
                    widget_size[0] / photo.width(),
                    widget_size[1] / photo.height(),
                )
                ratio = Fraction(max(scale, 0.05)).limit_denominator(8)
                if ratio.numerator != ratio.denominator:
                    photo = photo.zoom(
                        ratio.numerator, ratio.numerator
                    ).subsample(ratio.denominator, ratio.denominator)
            except (OSError, tk.TclError):
                # The detector may still be writing the PNG; retry next poll.
                pass
            else:
                self._result_photo = photo
                self._result_image_signature = signature
                self._result_image_widget_size = widget_size
                self.result_image_label.configure(image=photo, text="")

        self.root.after(300, self._poll_result_image)

    def _reset(self) -> None:
        if self._running:
            messagebox.showwarning("無法 Reset", "Action 執行中，請先等待或使用 STOP ALL。", parent=self.root)
            return
        self._set_running(True, "Reset 中")
        self._worker = threading.Thread(target=self._reset_worker, daemon=True)
        self._worker.start()

    def _reset_worker(self) -> None:
        errors = self._cleanup_services()
        self._services = None
        self._write_log(
            "Reset 完成" if not errors else "Reset 部分失敗: " + "; ".join(errors)
        )
        self._messages.put(("reset_failed" if errors else "reset_done", "; ".join(errors)))

    def _cleanup_services(self) -> list[str]:
        if self._services is None:
            return []
        plc, canbus, d435i, amr = self._services
        errors: list[str] = []
        operations = (
            ("停止 RGB-D", lambda: d435i.stop_rgbd() if d435i.is_rgbd_running else None),
            ("停止 CAN 馬達", lambda: canbus.stop_all() if canbus.is_connected else None),
            ("斷開 CAN", lambda: canbus.disconnect() if canbus.is_connected else None),
            ("斷開 PLC", plc.disconnect),
            ("斷開 AMR", amr.disconnect),
        )
        for label, operation in operations:
            try:
                operation()
            except Exception as exc:
                errors.append(f"{label}: {exc}")
        return errors

    def _stop_all(self) -> None:
        if self._services is None:
            self._append("STOP ALL：目前尚未建立 CAN service。\n", "info")
            return
        canbus = self._services[1]

        def stop_worker() -> None:
            try:
                result = canbus.stop_all()
            except Exception as exc:
                self._queue_output(f"STOP ALL ERROR: {exc}")
            else:
                self._queue_output(f"STOP ALL result={result}")

        threading.Thread(target=stop_worker, daemon=True).start()

    def _clear_output(self) -> None:
        self.output.configure(state="normal")
        self.output.delete("1.0", "end")
        self.output.configure(state="disabled")

    def _copy_selected(self) -> str:
        try:
            text = self.output.get("sel.first", "sel.last")
        except tk.TclError:
            text = ""
        if text:
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
            self.status.set("已複製選取內容")
        return "break"

    def _copy_all(self) -> None:
        text = self.output.get("1.0", "end-1c")
        if text:
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
            self.status.set("已複製全部結果")

    def _show_copy_menu(self, event: tk.Event[Any]) -> None:
        self.copy_menu.tk_popup(event.x_root, event.y_root)

    def _append(self, text: str, tag: str | None = None) -> None:
        self.output.configure(state="normal")
        self.output.insert("end", text, (() if tag is None else (tag,)))
        self.output.see("end")
        self.output.configure(state="disabled")

    def _set_running(self, running: bool, status: str) -> None:
        self._running = running
        state = "disabled" if running else "normal"
        self.run_button.configure(state=state)
        self.validate_button.configure(state=state)
        self.reset_button.configure(state=state)
        self.status.set(status)

    def _on_close(self) -> None:
        if self._running:
            if not messagebox.askyesno(
                "執行中", "Action 仍在執行。確定要關閉 UI？", parent=self.root
            ):
                return
        self._cleanup_services()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ActionUi(root)
    root.mainloop()


if __name__ == "__main__":
    main()
