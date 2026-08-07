# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "fastapi",
#     "pymcprotocol",
#     "pyyaml",
#     "requests",
#     "uvicorn",
# ]
# ///
"""整合啟動腳本：先啟動 FastAPI，再啟動 Tkinter UI。

用法：
    python main.py

注意：
- 這個腳本會在背景 thread 啟動 uvicorn server（不使用 auto-reload）。
- 適用於開發 / 測試環境，正式環境建議分開管理 API 與 UI。
"""

from __future__ import annotations

import locale
import os
import threading
import time
import urllib.error
import urllib.request

import uvicorn

from pathlib import Path

from api.main import app


def configure_process_locale() -> None:
    """Use a UTF-8 CJK locale before Tk initializes its font fallback."""

    user_locale_dir = Path.home() / ".local/share/locale/usr/lib/locale"
    if user_locale_dir.exists():
        os.environ["LOCPATH"] = str(user_locale_dir)

    preferred_locales = ("zh_TW.UTF-8", "zh_TW.utf8", "C.UTF-8")
    for locale_name in preferred_locales:
        try:
            locale.setlocale(locale.LC_ALL, locale_name)
        except locale.Error:
            continue

        os.environ["LANG"] = locale_name
        os.environ["LC_CTYPE"] = locale_name
        os.environ["LC_ALL"] = locale_name
        if locale_name.startswith("zh_TW"):
            os.environ["LANGUAGE"] = "zh_TW:zh"
        return


def read_version() -> str:
    version_file = Path(__file__).resolve().parent / "VERSION"
    try:
        return version_file.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        return "0.0.0"


def run_api() -> None:
    """在背景 thread 啟動 uvicorn。"""

    config = uvicorn.Config(app=app, host="0.0.0.0", port=8000, log_level="info")
    server = uvicorn.Server(config)
    server.run()


def api_is_healthy(timeout_seconds: float = 0.5) -> bool:
    """Return True when an existing local backend is ready for the UI."""

    try:
        with urllib.request.urlopen(
            "http://127.0.0.1:8000/health",
            timeout=timeout_seconds,
        ) as response:
            return response.status == 200
    except (OSError, urllib.error.URLError):
        return False


def main() -> None:
    configure_process_locale()

    from ui.main import main as ui_main

    version = read_version()
    print(f"python-plc version: {version}")

    if api_is_healthy():
        print("FastAPI backend already healthy on port 8000; reusing it.")
    else:
        api_thread = threading.Thread(target=run_api, daemon=True)
        api_thread.start()
        for _attempt in range(30):
            if api_is_healthy():
                break
            time.sleep(0.1)
        else:
            print("WARNING: FastAPI backend did not become healthy on port 8000.")

    # 啟動 UI（在主 thread）
    ui_main()


if __name__ == "__main__":
    main()
