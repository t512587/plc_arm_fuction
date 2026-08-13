#!/usr/bin/env python3
"""Start the integrated PLC + D435 action UI from the int-amr root."""

from __future__ import annotations

import argparse
import importlib.util
import json
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
    prepare_import_path()

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--height", type=float, help="執行 LView 自動任務的貨盤高度 (mm)")
    parser.add_argument("--forward-mm", type=float, default=400.0, help="Y1/Y2 前進距離 (mm，預設 400)")
    parser.add_argument("--yes", action="store_true", help="確認執行真實硬體自動任務")
    parser.add_argument("--dry-run", action="store_true", help="只顯示自動任務設定，不控制硬體")
    parser.add_argument("--status", action="store_true", help="顯示上一個自動任務的快照")
    args, remaining = parser.parse_known_args()

    if args.status or args.height is not None or args.dry_run:
        if remaining:
            raise SystemExit(f"不支援的參數: {' '.join(remaining)}")
        os.chdir(PLC2_DIR)
        from flow.auto_transfer_task import AutoTransferTask, CheckpointStore

        if args.status:
            data = CheckpointStore().load()
            print("沒有自動任務快照。" if data is None else json.dumps(data, ensure_ascii=False, indent=2))
            return
        if args.height is None:
            raise SystemExit("自動任務必須提供 --height，例如 --height 350")
        if args.dry_run:
            print(
                "AUTO TRANSFER DRY RUN\n"
                f"  view: LView\n  height: {args.height:g}mm\n"
                f"  forward: {args.forward_mm:g}mm\n"
                "  flow: lowest -> standby -> Y1/Y2 suck -> lowest -> vision/arm -> lowest -> Y1/Y2 push -> lowest"
            )
            return
        if not args.yes:
            raise SystemExit("此命令會控制真實硬體；請確認後加上 --yes，或先使用 --dry-run。")

        from service.arm_vision_workflow_service import ArmVisionWorkflowService
        from service.lift_service import LiftService
        from service.middle_vacuum_service import MiddleVacuumService
        from service.pallet_transfer_service import PalletTransferService
        from service.plc_service import PlcService

        plc = PlcService()
        plc.connect()
        try:
            lift = LiftService(plc)

            def plc_snapshot() -> dict[str, object]:
                return {
                    "M375_y1_move": plc.read_point("Y1_MOVE"),
                    "M374_y2_move": plc.read_point("Y2_MOVE"),
                    "M376_lift_position": plc.read_point("X_Move"),
                    "M379_lift_down": plc.read_point("X_MOVE_DOWN"),
                    "M50_y1_vacuum": plc.read_point("Y1_VAC_ON"),
                    "M51_y1_break_vacuum": plc.read_point("Y1_VAC_OFF"),
                    "M52_y2_vacuum": plc.read_point("Y2_VAC_ON"),
                    "M53_y2_break_vacuum": plc.read_point("Y2_VAC_OFF"),
                    "M54_middle_vacuum": plc.read_point("X_VAC_ON"),
                    "M56_middle_break_vacuum": plc.read_point("X_VAC_OFF"),
                }

            task = AutoTransferTask(
                lift,
                PalletTransferService(plc),
                MiddleVacuumService(plc),
                ArmVisionWorkflowService(target_height_validator=lift.validate_target_height),
                plc_snapshot_provider=plc_snapshot,
            )
            task.run(height_mm=args.height, forward_mm=args.forward_mm)
        finally:
            plc.disconnect()
        return

    if remaining:
        raise SystemExit(f"不支援的參數: {' '.join(remaining)}")

    os.chdir(PLC2_DIR)

    spec = importlib.util.spec_from_file_location("plc2_integrated_main", PLC2_MAIN)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load {PLC2_MAIN}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.main()


if __name__ == "__main__":
    main()
