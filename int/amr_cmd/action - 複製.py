#!/usr/bin/env python3
"""Read action.txt and execute whitelisted AMR actions only."""

from __future__ import annotations

import argparse
import ast
import json
import sys
import time
from dataclasses import asdict, dataclass, is_dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Mapping


DEFAULT_ACTION_FILE = Path(__file__).with_name("action.txt")
DEFAULT_LOG_DIR = Path(__file__).with_name("log")

ALLOWED_ACTION_NAMES = frozenset(
    {
        "AMRconnect",
        "AMRdisconnect",
        "AMRread_map",
        "AMRgoto_position",
        "sleep",
        "hunam_ctrl",
    }
)


class ActionFileError(ValueError):
    """Invalid action.txt syntax or unsupported action."""


class ActionExecutionError(RuntimeError):
    """An action raised an exception or returned a failure result."""


@dataclass(frozen=True, slots=True)
class ActionCall:
    name: str
    args: tuple[Any, ...]
    kwargs: dict[str, Any]
    line: int


def sleep_action(t: int | float) -> dict[str, Any]:
    """Pause action execution for ``t`` milliseconds."""
    if isinstance(t, bool) or not isinstance(t, (int, float)):
        raise ValueError("t must be a number of milliseconds")
    if t < 0:
        raise ValueError("t must be greater than or equal to zero")
    time.sleep(float(t) / 1000.0)
    return {"success": True, "slept_ms": t}


def hunam_ctrl(input_fn: Callable[[str], str] = input) -> dict[str, Any]:
    """Wait for a human operator to press Enter before continuing."""
    input_fn("等待人為確認，請按 Enter 繼續...")
    return {"success": True, "confirmed": True}


def parse_action_text(text: str) -> list[ActionCall]:
    """Parse function-call-only syntax without executing arbitrary Python."""
    try:
        tree = ast.parse(text, filename="action.txt", mode="exec")
    except SyntaxError as exc:
        raise ActionFileError(f"第 {exc.lineno} 行語法錯誤: {exc.msg}") from exc

    calls: list[ActionCall] = []
    for statement in tree.body:
        if not isinstance(statement, ast.Expr) or not isinstance(
            statement.value, ast.Call
        ):
            raise ActionFileError(f"第 {statement.lineno} 行只允許 Action 呼叫")
        call = statement.value
        if not isinstance(call.func, ast.Name):
            raise ActionFileError(
                f"第 {statement.lineno} 行不允許屬性或動態呼叫"
            )
        if any(keyword.arg is None for keyword in call.keywords):
            raise ActionFileError(f"第 {statement.lineno} 行不允許 **kwargs")
        try:
            args = tuple(ast.literal_eval(arg) for arg in call.args)
            kwargs = {
                str(item.arg): ast.literal_eval(item.value)
                for item in call.keywords
            }
        except (ValueError, TypeError) as exc:
            raise ActionFileError(
                f"第 {statement.lineno} 行參數只允許常值"
            ) from exc
        calls.append(ActionCall(call.func.id, args, kwargs, statement.lineno))
    return calls


def load_actions(path: str | Path = DEFAULT_ACTION_FILE) -> list[ActionCall]:
    action_path = Path(path)
    if not action_path.is_file():
        raise ActionFileError(f"找不到 Action 檔案: {action_path}")
    return parse_action_text(action_path.read_text(encoding="utf-8"))


def validate_action_names(actions: list[ActionCall]) -> None:
    for action in actions:
        if action.name not in ALLOWED_ACTION_NAMES:
            raise ActionFileError(f"第 {action.line} 行未知 Action: {action.name}")


def build_amr_service() -> Any:
    """Create ServiceAmr only; never import PLC, CANbus, or D435 services."""
    try:
        from .base.service_amr import ServiceAmr
    except ImportError:  # Direct execution: python amr_cmd/action.py
        from base.service_amr import ServiceAmr
    return ServiceAmr()


def build_action_registry(amr: Any) -> dict[str, Callable[..., Any]]:
    """Expose AMR commands and hardware-independent flow controls."""
    return {
        "AMRconnect": amr.connect,
        "AMRdisconnect": amr.disconnect,
        "AMRread_map": amr.read_map,
        "AMRgoto_position": amr.goto_position,
        "sleep": sleep_action,
        "hunam_ctrl": hunam_ctrl,
    }


def _failure_message(result: Any) -> str | None:
    if not isinstance(result, Mapping):
        return None
    if result.get("ok") is False or result.get("success") is False:
        return str(result.get("message", "Action 回傳失敗"))
    if str(result.get("status", "")).lower() in {
        "error",
        "failed",
        "timeout",
    }:
        return str(result.get("message", result["status"]))
    code = result.get("code")
    if code is not None:
        try:
            if int(code) != 0:
                return str(result.get("msg") or result.get("message") or code)
        except (TypeError, ValueError):
            return f"AMR API 回傳無效 code: {code!r}"
    return None


def _display_result(result: Any) -> str:
    if is_dataclass(result):
        result = asdict(result)
    try:
        return json.dumps(result, ensure_ascii=False, default=str)
    except (TypeError, ValueError):
        return repr(result)


def execute_actions(
    actions: list[ActionCall],
    registry: Mapping[str, Callable[..., Any]],
    *,
    output: Callable[[str], None] = print,
) -> list[Any]:
    results: list[Any] = []
    total = len(actions)
    for index, action in enumerate(actions, start=1):
        function = registry.get(action.name)
        if function is None:
            raise ActionFileError(f"第 {action.line} 行未知 Action: {action.name}")
        output(f"[{index}/{total}] START line={action.line} action={action.name}")
        started_at = time.monotonic()
        try:
            result = function(*action.args, **action.kwargs)
        except Exception as exc:
            raise ActionExecutionError(
                f"第 {action.line} 行 {action.name} 執行失敗: {exc}"
            ) from exc
        failure = _failure_message(result)
        if failure is not None:
            raise ActionExecutionError(
                f"第 {action.line} 行 {action.name} 失敗: {failure}"
            )
        elapsed = time.monotonic() - started_at
        output(
            f"[{index}/{total}] DONE  action={action.name} "
            f"elapsed={elapsed:.3f}s result={_display_result(result)}"
        )
        results.append(result)
    return results


def _format_call(action: ActionCall) -> str:
    values = [repr(value) for value in action.args]
    values.extend(
        f"{key}={value!r}" for key, value in action.kwargs.items()
    )
    return f"{action.name}({', '.join(values)})"


def dated_log_path(now: datetime | None = None) -> Path:
    current = now or datetime.now()
    return DEFAULT_LOG_DIR / f"action_{current:%Y-%m-%d}.log"


def build_output(log_path: Path) -> Callable[..., None]:
    log_path.parent.mkdir(parents=True, exist_ok=True)

    def output(message: str, *, stream: Any = sys.stdout) -> None:
        print(message, file=stream)
        timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
        with log_path.open("a", encoding="utf-8") as log_file:
            print(f"{timestamp} {message}", file=log_file)

    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="依序執行 action.txt 的 AMR Action"
    )
    parser.add_argument(
        "action_file",
        nargs="?",
        default=str(DEFAULT_ACTION_FILE),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只解析並顯示，不控制 AMR",
    )
    args = parser.parse_args(argv)

    try:
        output = build_output(dated_log_path())
    except OSError as exc:
        print(f"ERROR: 無法建立 Action log: {exc}", file=sys.stderr)
        return 1

    try:
        actions = load_actions(args.action_file)
        validate_action_names(actions)
        if not actions:
            output("action.txt 沒有可執行 Action")
            return 0
        if args.dry_run:
            for index, action in enumerate(actions, start=1):
                output(f"{index}: line {action.line}: {_format_call(action)}")
            return 0

        amr = build_amr_service()
        execute_actions(actions, build_action_registry(amr), output=output)
        output(f"全部完成，共 {len(actions)} 個 Action")
        return 0
    except (ActionFileError, ActionExecutionError) as exc:
        output(f"ERROR: {exc}", stream=sys.stderr)
        return 1
    except KeyboardInterrupt:
        output("ERROR: 使用者中止執行", stream=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
