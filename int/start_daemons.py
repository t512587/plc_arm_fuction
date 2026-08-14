#!/usr/bin/env python3
"""
start_daemons.py — Launch the persistent camera + CAN-bus daemons together.

d435_camera_daemon.py and canbus_daemon.py are independent processes (one
crashing does not take the other down), but in normal operation you always
want both running for the whole session. This script starts both, prefixes
their output so it's clear which daemon printed what, and stops both
cleanly on Ctrl+C.

    python start_daemons.py
"""
from __future__ import annotations

import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

DAEMONS = [
    ("camera", BASE_DIR / "d435_camera_daemon.py"),
    ("canbus", BASE_DIR / "canbus_daemon.py"),
]


def _stream_output(name: str, process: subprocess.Popen) -> None:
    assert process.stdout is not None
    for line in process.stdout:
        print(f"[{name}] {line.rstrip()}", flush=True)


def main() -> None:
    processes: list[subprocess.Popen] = []
    threads: list[threading.Thread] = []

    for name, script_path in DAEMONS:
        process = subprocess.Popen(
            [sys.executable, "-u", str(script_path)],
            cwd=BASE_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        processes.append(process)
        thread = threading.Thread(
            target=_stream_output, args=(name, process), daemon=True
        )
        thread.start()
        threads.append(thread)
        print(f"[start_daemons] launched {name} (pid={process.pid})")

    def handle_signal(signum, _frame) -> None:
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, handle_signal)

    try:
        # Poll instead of process.wait()-ing sequentially so a crash in one
        # daemon is reported immediately without killing the other one,
        # which may still be perfectly usable on its own.
        reported_exit = {name: False for name, _ in DAEMONS}
        while any(process.poll() is None for process in processes):
            for (name, _), process in zip(DAEMONS, processes):
                if process.poll() is not None and not reported_exit[name]:
                    print(f"[start_daemons] {name} exited (code={process.returncode})")
                    reported_exit[name] = True
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        print("[start_daemons] stopping daemons...")
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
        for thread in threads:
            thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
