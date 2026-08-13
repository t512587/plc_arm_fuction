from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

from service_plc import ServicePlc


@dataclass(frozen=True)
class CandidateBit:
    address: int
    name: str


@dataclass(frozen=True)
class WatchRegister:
    address: int
    name: str


CANDIDATE_BITS = (
    CandidateBit(10, "X回原點流程_內部"),
    CandidateBit(11, "Y回原點流程_內部"),
    CandidateBit(12, "Z回原點流程_內部"),
    CandidateBit(30, "回原點自保持"),
    CandidateBit(50, "左邊真空"),
    CandidateBit(51, "左邊破真空"),
    CandidateBit(52, "右邊真空"),
    CandidateBit(53, "右邊破真空"),
    CandidateBit(350, "X軸手動上升"),
    CandidateBit(351, "X軸手動下降"),
    CandidateBit(352, "Y軸手動前進"),
    CandidateBit(353, "Y軸手動後退"),
    CandidateBit(354, "右Y軸前進"),
    CandidateBit(355, "右Y軸後退"),
    CandidateBit(374, "未確認_R_FWD_POS"),
    CandidateBit(375, "未確認_L_FWD_POS"),
    CandidateBit(376, "未確認_LIFT_UP_POS"),
    CandidateBit(378, "X軸前進"),
    CandidateBit(379, "X軸後退"),
    CandidateBit(382, "Y軸前進"),
    CandidateBit(383, "Y軸後退"),
    CandidateBit(388, "Y軸前進"),
    CandidateBit(389, "Y軸後退"),
    CandidateBit(398, "Z軸前進"),
    CandidateBit(399, "Z軸後退"),
    CandidateBit(400, "Y回原點完成"),
    CandidateBit(405, "X回原點完成"),
    CandidateBit(410, "Z回原點完成"),
)

WATCH_REGISTERS = (
    WatchRegister(52, "X現在位置"),
    WatchRegister(62, "Y現在位置"),
    WatchRegister(72, "Z現在位置"),
    WatchRegister(300, "X手動速度"),
    WatchRegister(302, "Y1手動速度"),
    WatchRegister(304, "Y2手動速度"),
    WatchRegister(500, "左邊X移動位置"),
    WatchRegister(510, "左邊Y移動位置"),
    WatchRegister(550, "右邊X移動位置"),
    WatchRegister(560, "右邊Y移動位置"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="只讀監看 PLC 候選點位，用來反查真正的 HMI/PC 命令 bit。"
    )
    parser.add_argument("--interval", type=float, default=0.2, help="讀取間隔秒數，預設 0.2")
    parser.add_argument("--duration", type=float, default=60.0, help="監看秒數，預設 60")
    parser.add_argument("--show-all", action="store_true", help="每次都印出所有點位，而不是只印變化")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    service = ServicePlc()
    print(service.connect())
    print("只讀監看開始；請按原本會讓機台動作的 HMI/實體按鈕。")
    print("看到 M 點從 0->1 或 D 位置開始變化時，請記下那一行。")

    last_bits: dict[int, bool] = {}
    last_registers: dict[int, float] = {}
    started_at = time.monotonic()

    try:
        while time.monotonic() - started_at <= args.duration:
            timestamp = time.strftime("%H:%M:%S")
            changed_lines: list[str] = []

            for bit in CANDIDATE_BITS:
                value = service.read_m(bit.address)
                previous = last_bits.get(bit.address)
                last_bits[bit.address] = value
                if args.show_all or previous is None or previous != value:
                    changed_lines.append(f"{timestamp} M{bit.address:<4} {int(value)}  {bit.name}")

            for register in WATCH_REGISTERS:
                value = service.read_d(register.address)
                previous = last_registers.get(register.address)
                last_registers[register.address] = value
                if args.show_all or previous is None or previous != value:
                    changed_lines.append(f"{timestamp} D{register.address:<4} {value:g}  {register.name}")

            if changed_lines:
                print("\n".join(changed_lines), flush=True)
            time.sleep(max(0.05, args.interval))
    finally:
        print(service.disconnect())


if __name__ == "__main__":
    main()
