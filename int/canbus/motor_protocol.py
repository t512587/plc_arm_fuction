from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable


DATA_LENGTH = 8
SINGLE_MOTOR_TX_BASE_ID = 0x140
SINGLE_MOTOR_RX_BASE_ID = 0x240
MULTI_MOTOR_TX_ID = 0x280
CAN_ID_CONFIG_ID = 0x300
MOTION_CONTROL_TX_BASE_ID = 0x400
MOTION_CONTROL_RX_BASE_ID = 0x500


class ProtocolError(Exception):
    """Raised when a CAN protocol frame is malformed or unexpected."""


@dataclass(frozen=True)
class CanMotorFrame:
    arbitration_id: int
    data: bytes

    @property
    def command(self) -> int:
        return self.data[0]


PID_INDEX_NAMES = {
    0x01: "Current loop KP",
    0x02: "Current loop KI",
    0x04: "Speed loop KP",
    0x05: "Speed loop KI",
    0x07: "Position loop KP",
    0x08: "Position loop KI",
    0x09: "Position loop KD",
}


def tx_arbitration_id(motor_id: int) -> int:
    validate_motor_id(motor_id)
    return SINGLE_MOTOR_TX_BASE_ID + motor_id


def rx_arbitration_id(motor_id: int) -> int:
    validate_motor_id(motor_id)
    return SINGLE_MOTOR_RX_BASE_ID + motor_id


def motion_tx_arbitration_id(motor_id: int) -> int:
    validate_motor_id(motor_id)
    return MOTION_CONTROL_TX_BASE_ID + motor_id


def motion_rx_arbitration_id(motor_id: int) -> int:
    validate_motor_id(motor_id)
    return MOTION_CONTROL_RX_BASE_ID + motor_id


def validate_motor_id(motor_id: int) -> None:
    if not 1 <= motor_id <= 32:
        raise ValueError("motor_id must be between 1 and 32 for single motor commands.")


def make_command_data(command: int, payload: Iterable[int] = ()) -> bytes:
    values = [command, *payload]
    if len(values) > DATA_LENGTH:
        raise ValueError("Command data cannot exceed 8 bytes.")
    values.extend([0x00] * (DATA_LENGTH - len(values)))
    return bytes(value & 0xFF for value in values)


def parse_can_frame(
    arbitration_id: int,
    data: bytes,
    expected_arbitration_id: int | None = None,
    expected_command: int | None = None,
) -> CanMotorFrame:
    if expected_arbitration_id is not None and arbitration_id != expected_arbitration_id:
        raise ProtocolError(
            f"Unexpected arbitration ID: expected 0x{expected_arbitration_id:X}, "
            f"got 0x{arbitration_id:X}."
        )
    if len(data) != DATA_LENGTH:
        raise ProtocolError(f"Expected DLC 8, received {len(data)} bytes.")
    if expected_command is not None and data[0] != expected_command:
        raise ProtocolError(
            f"Unexpected command: expected 0x{expected_command:02X}, got 0x{data[0]:02X}."
        )
    return CanMotorFrame(arbitration_id=arbitration_id, data=data)


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def int8(value: int) -> int:
    return struct.unpack("<b", bytes([value & 0xFF]))[0]


def uint16_le(data: bytes) -> int:
    return int.from_bytes(data, byteorder="little", signed=False)


def int16_le(data: bytes) -> int:
    return int.from_bytes(data, byteorder="little", signed=True)


def int32_le(data: bytes) -> int:
    return int.from_bytes(data, byteorder="little", signed=True)


def float32_le(data: bytes) -> float:
    return struct.unpack("<f", data)[0]


def float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    value = max(minimum, min(maximum, value))
    span = maximum - minimum
    return int(round((value - minimum) * ((1 << bits) - 1) / span))


def uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    span = maximum - minimum
    return float(value) * span / ((1 << bits) - 1) + minimum


def make_motion_control_data(
    p_des: float,
    v_des: float,
    kp: float,
    kd: float,
    t_ff: float,
) -> bytes:
    p = float_to_uint(p_des, -12.5, 12.5, 16)
    v = float_to_uint(v_des, -45.0, 45.0, 12)
    kp_u = float_to_uint(kp, 0.0, 500.0, 12)
    kd_u = float_to_uint(kd, 0.0, 5.0, 12)
    t = float_to_uint(t_ff, -24.0, 24.0, 12)
    return bytes(
        [
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
            kp_u & 0xFF,
            (kd_u >> 4) & 0xFF,
            ((kd_u & 0x0F) << 4) | ((t >> 8) & 0x0F),
            t & 0xFF,
        ]
    )


def parse_motion_control_feedback(data: bytes) -> dict[str, float | int]:
    if len(data) != DATA_LENGTH:
        raise ProtocolError(f"Expected DLC 8, received {len(data)} bytes.")
    p_raw = (data[1] << 8) | data[2]
    v_raw = (data[3] << 4) | (data[4] >> 4)
    t_raw = ((data[4] & 0x0F) << 8) | data[5]
    return {
        "can_id": data[0],
        "p_raw": p_raw,
        "v_raw": v_raw,
        "t_raw": t_raw,
        "position_rad": uint_to_float(p_raw, -12.5, 12.5, 16),
        "velocity_rad_s": uint_to_float(v_raw, -45.0, 45.0, 12),
        "torque_nm": uint_to_float(t_raw, -24.0, 24.0, 12),
    }


def make_absolute_position_data(max_speed_dps: int, angle_degrees: float) -> bytes:
    max_speed = int(max(0, min(0xFFFF, max_speed_dps)))
    angle_lsb = int(round(angle_degrees * 100.0))
    return (
        bytes([0xA4, 0x00])
        + max_speed.to_bytes(2, "little", signed=False)
        + angle_lsb.to_bytes(4, "little", signed=True)
    )
