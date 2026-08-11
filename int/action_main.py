#!/usr/bin/env python3
"""Start the integrated PLC + D435 action UI from the int-amr root."""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent
PLC2_DIR = ROOT_DIR / "plc2"
PLC2_MAIN = PLC2_DIR / "main.py"


def prepare_import_path() -> None:
    """Make plc2's local packages win over root-level legacy modules."""

    root = str(ROOT_DIR)
    plc2 = str(PLC2_DIR)
    sys.path[:] = [path for path in sys.path if path not in {root, plc2}]
    sys.path.insert(0, plc2)
    sys.modules.pop("ui", None)
    sys.modules.pop("ui.main", None)


def main() -> None:
    os.chdir(PLC2_DIR)
    prepare_import_path()

    spec = importlib.util.spec_from_file_location("plc2_integrated_main", PLC2_MAIN)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load {PLC2_MAIN}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.main()


if __name__ == "__main__":
    main()
