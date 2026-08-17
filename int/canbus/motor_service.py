from __future__ import annotations

import re
import shlex
import subprocess
import hashlib
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from motor_protocol import (
    CAN_ID_CONFIG_ID,
    PID_INDEX_NAMES,
    ProtocolError,
    float32_le,
    hex_bytes,
    int8,
    int16_le,
    int32_le,
    make_motion_control_data,
    make_absolute_position_data,
    make_command_data,
    motion_rx_arbitration_id,
    motion_tx_arbitration_id,
    parse_can_frame,
    parse_motion_control_feedback,
    rx_arbitration_id,
    tx_arbitration_id,
    uint16_le,
)

try:
    import can
except ImportError:  # pragma: no cover - handled at runtime in the UI.
    can = None

try:
    import serial
except ImportError:  # pragma: no cover - handled at runtime in the UI.
    serial = None


@dataclass(frozen=True)
class MotorConfig:
    can_interface: str = "waveshare_usbcana"
    channel: str = "COM5"
    bitrate: int = 1_000_000
    serial_baudrate: int = 2_000_000
    motor_id: int = 1
    timeout_seconds: float = 1.0
    canusb_path: str = "./canusb"
    canusb_use_sudo: bool = True
    can_kwargs: dict[str, Any] = field(default_factory=dict)


class MotorService:
    def __init__(self, config: MotorConfig) -> None:
        self.config = config
        self._bus: Any | None = None
        self._serial: Any | None = None
        self.last_tx = ""
        self.last_rx = ""
        self.connection_step = "idle"
        self._channel_lock_file: Any | None = None
        self._channel_lock_path: Path | None = None

    @property
    def is_connected(self) -> bool:
        return self._bus is not None or bool(self._serial and self._serial.is_open)

    def connect(self) -> str:
        self.connection_step = "connect start"
        if self.is_connected:
            return f"Already connected to CAN channel {self.config.channel}."
        self._acquire_channel_lock()
        try:
            if self.config.can_interface == "canusb_cli":
                return self._connect_canusb_cli()
            if self.config.can_interface == "waveshare_usbcana":
                return self._connect_waveshare_usbcana()

            if can is None:
                raise RuntimeError("python-can is not installed. Run: pip install -r requirements.txt")

            self.connection_step = "opening CAN bus"
            self._bus = can.Bus(
                interface=self.config.can_interface,
                channel=self.config.channel,
                bitrate=self.config.bitrate,
                **self.config.can_kwargs,
            )
            self.connection_step = "connected"
            return (
                f"Connected to CAN channel {self.config.channel}; "
                f"interface={self.config.can_interface}; bitrate={self.config.bitrate}."
            )
        except Exception:
            # A backend may have opened the serial/CAN handle before its
            # configuration failed. Release both the device and ownership lock
            # so a retry does not inherit a half-open connection.
            try:
                if self._bus:
                    self._bus.shutdown()
                    self._bus = None
                if self._serial:
                    self._serial.close()
                    self._serial = None
            finally:
                self._release_channel_lock()
            raise

    def disconnect(self) -> str:
        try:
            if self.config.can_interface == "canusb_cli":
                self._bus = None
            elif self._bus:
                self._bus.shutdown()
                self._bus = None
            if self._serial:
                self._serial.close()
                self._serial = None
            self.connection_step = "disconnected"
            return "Disconnected."
        finally:
            self._release_channel_lock()

    def _acquire_channel_lock(self) -> None:
        """Prevent two local processes from controlling one CAN channel."""
        if self._channel_lock_file is not None:
            return
        channel = str(self.config.channel)
        if os.name == "nt":
            channel = channel.casefold()
        identity = f"{self.config.can_interface}|{channel}".encode("utf-8")
        digest = hashlib.sha256(identity).hexdigest()[:20]
        path = Path(tempfile.gettempdir()) / f"plc_arm_can_{digest}.lock"
        handle = path.open("a+b")
        try:
            handle.seek(0, os.SEEK_END)
            if handle.tell() == 0:
                handle.write(b"0")
                handle.flush()
            handle.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl

                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, IOError) as exc:
            handle.close()
            raise RuntimeError(
                "CAN channel is already in use by another local process: "
                f"{self.config.channel}. Close 2motor_sync.py, d435_control.py, "
                "or the other action_main.py instance before retrying."
            ) from exc
        self._channel_lock_file = handle
        self._channel_lock_path = path

    def _release_channel_lock(self) -> None:
        handle = self._channel_lock_file
        if handle is None:
            return
        try:
            handle.seek(0)
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()
            self._channel_lock_file = None
            self._channel_lock_path = None

    def read_pid_parameter(self, index: int) -> dict[str, Any]:
        return self.read_pid_parameter_for_motor(self.config.motor_id, index)

    def read_pid_parameter_for_motor(self, motor_id: int, index: int) -> dict[str, Any]:
        data = make_command_data(0x30, [index])
        response = self._send_command_for_motor(motor_id, 0x30, data)
        return self._with_trace(
            {
                "command": "Read PID parameter (0x30)",
                "motor_id": motor_id,
                "index": f"0x{index:02X}",
                "name": PID_INDEX_NAMES.get(index, "Unknown PID index"),
                "value": float32_le(response.data[4:8]),
                "raw": hex_bytes(response.data),
            }
        )

    def read_multi_turn_encoder(self) -> dict[str, Any]:
        return self.read_multi_turn_encoder_for_motor(self.config.motor_id)

    def read_multi_turn_encoder_for_motor(self, motor_id: int) -> dict[str, Any]:
        response = self._send_command_for_motor(motor_id, 0x60, make_command_data(0x60))
        return self._with_trace(
            {
                "command": "Read multi-turn encoder position (0x60)",
                "motor_id": motor_id,
                "encoder_pulses": int32_le(response.data[4:8]),
                "raw": hex_bytes(response.data),
            }
        )

    def read_single_turn_encoder(self) -> dict[str, Any]:
        return self.read_single_turn_encoder_for_motor(self.config.motor_id)

    def read_single_turn_encoder_for_motor(self, motor_id: int) -> dict[str, Any]:
        response = self._send_command_for_motor(motor_id, 0x90, make_command_data(0x90))
        return self._with_trace(
            {
                "command": "Read single-turn encoder (0x90)",
                "motor_id": motor_id,
                "encoder_pulses": uint16_le(response.data[2:4]),
                "encoder_raw_pulses": uint16_le(response.data[4:6]),
                "encoder_offset_pulses": uint16_le(response.data[6:8]),
                "raw": hex_bytes(response.data),
            }
        )

    def read_single_turn_angle(self) -> dict[str, Any]:
        return self.read_single_turn_angle_for_motor(self.config.motor_id)

    def read_single_turn_angle_for_motor(self, motor_id: int) -> dict[str, Any]:
        response = self._send_command_for_motor(motor_id, 0x94, make_command_data(0x94))
        angle_lsb = int32_le(response.data[4:8])
        return self._with_trace(
            {
                "command": "Read single-turn angle (0x94)",
                "motor_id": motor_id,
                "circle_angle_lsb": angle_lsb,
                "circle_angle_degrees": angle_lsb * 0.01,
                "raw": hex_bytes(response.data),
            }
        )

    def read_multi_turn_angle(self, motor_id: int | None = None) -> dict[str, Any]:
        target_motor_id = self.config.motor_id if motor_id is None else motor_id
        response = self._send_frame(
            tx_id=tx_arbitration_id(target_motor_id),
            data=make_command_data(0x92),
            rx_id=rx_arbitration_id(target_motor_id),
            expected_command=0x92,
        )
        angle_lsb = int32_le(response.data[4:8])
        return self._with_trace(
            {
                "command": "Read multi-turn angle (0x92)",
                "motor_id": target_motor_id,
                "angle_lsb": angle_lsb,
                "angle_degrees": angle_lsb * 0.01,
                "raw": hex_bytes(response.data),
            }
        )

    def read_motor_status2(self) -> dict[str, Any]:
        return self.read_motor_status2_for_motor(self.config.motor_id)

    def read_motor_status2_for_motor(self, motor_id: int) -> dict[str, Any]:
        response = self._send_command_for_motor(motor_id, 0x9C, make_command_data(0x9C))
        iq_raw = int16_le(response.data[2:4])
        return self._with_trace(
            {
                "command": "Read motor status 2 (0x9C)",
                "motor_id": motor_id,
                "temperature_c": int8(response.data[1]),
                "iq_raw": iq_raw,
                "iq_a": iq_raw * 0.01,
                "speed_dps": int16_le(response.data[4:6]),
                "degree": int16_le(response.data[6:8]),
                "raw": hex_bytes(response.data),
            }
        )

    def absolute_position_control(
        self,
        motor_id: int,
        angle_degrees: float,
        max_speed_dps: int = 500,
    ) -> dict[str, Any]:
        data = make_absolute_position_data(max_speed_dps, angle_degrees)
        response = self._send_frame(
            tx_id=tx_arbitration_id(motor_id),
            data=data,
            rx_id=rx_arbitration_id(motor_id),
            expected_command=0xA4,
        )
        iq_raw = int16_le(response.data[2:4])
        return self._with_trace(
            {
                "command": "Absolute position closed-loop control (0xA4)",
                "motor_id": motor_id,
                "target_degrees": angle_degrees,
                "max_speed_dps": max_speed_dps,
                "temperature_c": int8(response.data[1]),
                "iq_a": iq_raw * 0.01,
                "speed_dps": int16_le(response.data[4:6]),
                "degree": int16_le(response.data[6:8]),
                "raw": hex_bytes(response.data),
            }
        )

    def stop_motor(self, motor_id: int) -> dict[str, Any]:
        """Motor stop command (0x81) — stops movement but keeps motor powered."""
        response = self._send_command_for_motor(motor_id, 0x81, make_command_data(0x81))
        return self._with_trace(
            {
                "command": "Motor stop (0x81)",
                "motor_id": motor_id,
                "raw": hex_bytes(response.data),
            }
        )

    def shutdown_motor(self, motor_id: int) -> dict[str, Any]:
        """Motor shutdown command (0x80) — turns off motor output."""
        response = self._send_command_for_motor(motor_id, 0x80, make_command_data(0x80))
        return self._with_trace(
            {
                "command": "Motor shutdown (0x80)",
                "motor_id": motor_id,
                "raw": hex_bytes(response.data),
            }
        )

    def send_absolute_position(
        self,
        motor_id: int,
        angle_degrees: float,
        max_speed_dps: int = 500,
    ) -> dict[str, Any]:
        data = make_absolute_position_data(max_speed_dps, angle_degrees)
        tx_id = tx_arbitration_id(motor_id)
        self._send_frame_no_wait(tx_id, data)
        return self._with_trace(
            {
                "command": "Send absolute position control without waiting (0xA4)",
                "motor_id": motor_id,
                "target_degrees": angle_degrees,
                "max_speed_dps": max_speed_dps,
                "raw": hex_bytes(data),
            }
        )

    def read_can_id(self) -> dict[str, Any]:
        data = make_command_data(0x79, [0x00, 0x01])
        response = self._send_frame(
            tx_id=CAN_ID_CONFIG_ID,
            data=data,
            rx_id=None,
            expected_command=0x79,
        )
        can_id = uint16_le(response.data[6:8])
        motor_id = can_id - 0x140 if can_id >= 0x140 else can_id
        return self._with_trace(
            {
                "command": "Read CANID setting (0x79)",
                "can_id": f"0x{can_id:X}",
                "motor_id": motor_id,
                "raw": hex_bytes(response.data),
            }
        )

    def set_can_id(self, motor_id: int) -> dict[str, Any]:
        if not 1 <= motor_id <= 32:
            raise ValueError("motor_id must be between 1 and 32.")
        data = make_command_data(0x79, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, motor_id])
        response = self._send_frame(
            tx_id=CAN_ID_CONFIG_ID,
            data=data,
            rx_id=None,
            expected_command=0x79,
        )
        return self._with_trace(
            {
                "command": "Set CANID setting (0x79)",
                "new_motor_id": motor_id,
                "new_tx_id": f"0x{tx_arbitration_id(motor_id):X}",
                "new_rx_id": f"0x{rx_arbitration_id(motor_id):X}",
                "raw": hex_bytes(response.data),
            }
        )

    def motion_control(
        self,
        p_des: float,
        v_des: float,
        kp: float,
        kd: float,
        t_ff: float,
        motor_id: int | None = None,
    ) -> dict[str, Any]:
        target_motor_id = self.config.motor_id if motor_id is None else motor_id
        data = make_motion_control_data(p_des, v_des, kp, kd, t_ff)
        response = self._send_frame(
            tx_id=motion_tx_arbitration_id(target_motor_id),
            data=data,
            rx_id=motion_rx_arbitration_id(target_motor_id),
            expected_command=None,
        )
        feedback = parse_motion_control_feedback(response.data)
        return self._with_trace(
            {
                "command": "Motion control CAN (0x400 + ID)",
                "motor_id": target_motor_id,
                "p_des_rad": p_des,
                "v_des_rad_s": v_des,
                "kp": kp,
                "kd": kd,
                "t_ff_nm": t_ff,
                **feedback,
                "raw": hex_bytes(response.data),
            }
        )

    def test_all_functions(self) -> dict[str, Any]:
        tests = [
            ("connection", self._test_connection),
            ("read_pid_0x30", lambda: self.read_pid_parameter(0x01)),
            ("read_multi_turn_encoder_0x60", self.read_multi_turn_encoder),
            ("read_single_turn_encoder_0x90", self.read_single_turn_encoder),
            ("read_single_turn_angle_0x94", self.read_single_turn_angle),
            ("read_motor_status2_0x9C", self.read_motor_status2),
            ("read_canid_0x79", self.read_can_id),
            ("listen_can_1s", lambda: self.listen_frames(1.0)),
        ]

        lines: list[str] = []
        passed = 0
        for name, action in tests:
            try:
                result = action()
                passed += 1
                lines.append(f"{name}: OK - {self._summarize_result(result)}")
            except Exception as exc:
                lines.append(f"{name}: ERROR - {type(exc).__name__}: {exc}")

        return {
            "command": "Run connection and function tests",
            "passed": passed,
            "total": len(tests),
            "result": "\n".join(lines),
        }

    def listen_frames(self, duration_seconds: float = 3.0) -> dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("CAN bus is not connected.")

        frames: list[str] = []
        deadline = self._monotonic() + duration_seconds
        while self._monotonic() < deadline:
            remaining = max(0.0, deadline - self._monotonic())
            if self.config.can_interface == "canusb_cli":
                raise RuntimeError("Listen CAN is not supported by canusb_cli yet; use send/read commands.")
            elif self.config.can_interface == "waveshare_usbcana":
                message = self._read_waveshare_variable_frame(min(0.2, remaining))
            else:
                message = self._bus.recv(timeout=min(0.2, remaining))
            if message is None:
                continue
            frames.append(self._format_message(message))

        return {
            "command": f"Listen CAN frames for {duration_seconds:.1f}s",
            "count": len(frames),
            "frames": " | ".join(frames) if frames else "none",
        }

    def scan_motor_ids(self) -> dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("CAN bus is not connected.")

        original_id = self.config.motor_id
        found: list[str] = []
        for motor_id in range(1, 33):
            tx_id = tx_arbitration_id(motor_id)
            data = make_command_data(0x60)
            if self.config.can_interface == "waveshare_usbcana":
                self._serial.write(self._build_waveshare_variable_frame(tx_id, data))
            elif self.config.can_interface == "canusb_cli":
                try:
                    response = self._send_frame_canusb_cli(
                        tx_id,
                        data,
                        rx_arbitration_id(motor_id),
                        0x60,
                    )
                    found.append(f"motor_id={motor_id} {self._format_message(response)}")
                except Exception:
                    pass
                continue
            else:
                message = can.Message(
                    arbitration_id=tx_id,
                    data=data,
                    is_extended_id=False,
                    is_remote_frame=False,
                    is_error_frame=False,
                )
                self._bus.send(message, timeout=self.config.timeout_seconds)
            response = self._recv_scan_response(motor_id, timeout=0.08)
            if response is not None:
                found.append(
                    f"motor_id={motor_id} {self._format_message(response)}"
                )

        self.last_tx = "scan motor_id 1..32 with command 0x60"
        self.last_rx = " | ".join(found) if found else "none"
        return {
            "command": "Scan motor IDs 1..32 with 0x60",
            "configured_motor_id": original_id,
            "found_count": len(found),
            "found": self.last_rx,
        }

    def _test_connection(self) -> dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("CAN bus is not connected.")
        return {
            "channel": self.config.channel,
            "interface": self.config.can_interface,
            "bitrate": self.config.bitrate,
            "motor_id": self.config.motor_id,
        }

    def _with_trace(self, result: dict[str, Any]) -> dict[str, Any]:
        result["tx"] = self.last_tx
        result["rx"] = self.last_rx
        return result

    def _send_command(self, expected_command: int, data: bytes):
        return self._send_command_for_motor(self.config.motor_id, expected_command, data)

    def _send_command_for_motor(self, motor_id: int, expected_command: int, data: bytes):
        return self._send_frame(
            tx_id=tx_arbitration_id(motor_id),
            data=data,
            rx_id=rx_arbitration_id(motor_id),
            expected_command=expected_command,
        )

    def _send_frame(
        self,
        tx_id: int,
        data: bytes,
        rx_id: int | None,
        expected_command: int | None,
    ):
        if not self.is_connected:
            raise RuntimeError("CAN bus is not connected.")
        if self.config.can_interface == "canusb_cli":
            return self._send_frame_canusb_cli(tx_id, data, rx_id, expected_command)
        if self.config.can_interface == "waveshare_usbcana":
            return self._send_frame_waveshare(tx_id, data, rx_id, expected_command)

        message = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False,
            is_remote_frame=False,
            is_error_frame=False,
        )
        self.last_tx = f"id=0x{tx_id:X} dlc=8 data={hex_bytes(data)}"
        self.last_rx = ""

        self._bus.send(message, timeout=self.config.timeout_seconds)
        response = self._recv_expected(rx_id, expected_command)
        self.last_rx = (
            f"id=0x{response.arbitration_id:X} dlc={len(response.data)} "
            f"data={hex_bytes(bytes(response.data))}"
        )
        return parse_can_frame(
            arbitration_id=response.arbitration_id,
            data=bytes(response.data),
            expected_arbitration_id=rx_id,
            expected_command=expected_command,
        )

    def _send_frame_no_wait(self, tx_id: int, data: bytes) -> None:
        if not self.is_connected:
            raise RuntimeError("CAN bus is not connected.")
        if self.config.can_interface == "canusb_cli":
            self._send_frame_canusb_cli(tx_id, data, None, None)
            return
        if self.config.can_interface == "waveshare_usbcana":
            packet = self._build_waveshare_variable_frame(tx_id, data)
            self.last_tx = f"serial={hex_bytes(packet)}; can_id=0x{tx_id:X} data={hex_bytes(data)}"
            self.last_rx = ""
            self._serial.write(packet)
            return

        message = can.Message(
            arbitration_id=tx_id,
            data=data,
            is_extended_id=False,
            is_remote_frame=False,
            is_error_frame=False,
        )
        self.last_tx = f"id=0x{tx_id:X} dlc=8 data={hex_bytes(data)}"
        self.last_rx = ""
        self._bus.send(message, timeout=self.config.timeout_seconds)

    def _recv_expected(self, expected_arbitration_id: int | None, expected_command: int | None):
        deadline_timeout = self.config.timeout_seconds
        ignored: list[str] = []

        while True:
            message = self._bus.recv(timeout=deadline_timeout)
            if message is None:
                ignored_text = "; ignored=" + " | ".join(ignored) if ignored else ""
                expected_id = (
                    "any" if expected_arbitration_id is None else f"0x{expected_arbitration_id:X}"
                )
                raise TimeoutError(
                    f"Timed out waiting for CAN response id={expected_id}; "
                    f"TX={self.last_tx}{ignored_text}"
                )

            data = bytes(message.data)
            summary = f"id=0x{message.arbitration_id:X} data={hex_bytes(data)}"
            if (
                (expected_arbitration_id is None or message.arbitration_id == expected_arbitration_id)
                and len(data) == 8
                and (expected_command is None or data[0] == expected_command)
            ):
                return message

            ignored.append(summary)
            if len(ignored) > 5:
                ignored.pop(0)

    def _connect_canusb_cli(self) -> str:
        command_path = self._canusb_command_path()
        self.connection_step = f"checking canusb binary {command_path}"
        if not command_path.exists():
            raise FileNotFoundError(
                f"canusb binary not found: {command_path}. "
                "Put canusb in the working directory or set canusb_path in config.json."
            )
        if not command_path.is_file():
            raise FileNotFoundError(f"canusb path is not a file: {command_path}")
        if self.config.canusb_use_sudo:
            self.connection_step = "checking sudo access"
            sudo_check = subprocess.run(
                ["sudo", "-n", "true"],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            if sudo_check.returncode != 0:
                detail = sudo_check.stderr.strip() or sudo_check.stdout.strip()
                raise RuntimeError(
                    "sudo is required for canusb_cli but passwordless/cached sudo is not available. "
                    "Run the UI with sudo, configure NOPASSWD for canusb, or set canusb_use_sudo=false "
                    f"after fixing /dev/ttyUSB0 permissions. {detail}"
                )

        self._bus = "canusb_cli"
        self.connection_step = "connected"
        return (
            f"Configured canusb CLI {command_path}; channel={self.config.channel}; "
            f"CAN={self.config.bitrate}; sudo={self.config.canusb_use_sudo}."
        )

    def _send_frame_canusb_cli(
        self,
        tx_id: int,
        data: bytes,
        rx_id: int | None,
        expected_command: int | None,
    ):
        command_path = self._canusb_command_path()
        command = [
            str(command_path),
            "-d",
            self.config.channel,
            "-s",
            str(self.config.bitrate),
            "-t",
            "-i",
            f"{tx_id:X}",
            "-j",
            data.hex().upper(),
        ]
        if self.config.canusb_use_sudo:
            command.insert(0, "sudo")

        self.last_tx = " ".join(shlex.quote(part) for part in command)
        self.last_rx = ""
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=self.config.timeout_seconds + 1.0,
            check=False,
        )
        output = "\n".join(part for part in (completed.stdout.strip(), completed.stderr.strip()) if part)
        self.last_rx = output
        if completed.returncode != 0:
            raise RuntimeError(
                f"canusb exited with code {completed.returncode}; command={self.last_tx}; output={output}"
            )

        if rx_id is None and expected_command is None:
            return parse_can_frame(tx_id, data)

        frame = self._parse_canusb_output(output, rx_id, expected_command)
        if frame is None:
            expected_id = "any" if rx_id is None else f"0x{rx_id:X}"
            raise TimeoutError(
                f"Timed out waiting for CAN response id={expected_id}; "
                f"TX={self.last_tx}; output={output or 'none'}"
            )
        self.last_rx = f"id=0x{frame.arbitration_id:X} dlc={len(frame.data)} data={hex_bytes(frame.data)}"
        return frame

    def _canusb_command_path(self) -> Path:
        command_path = Path(self.config.canusb_path)
        if command_path.is_absolute():
            return command_path
        return Path(__file__).resolve().parent / command_path

    @staticmethod
    def _parse_canusb_output(output: str, expected_arbitration_id: int | None, expected_command: int | None):
        if not output:
            return None

        for line in output.splitlines():
            id_match = re.search(r"(?:id|can id|can_id)\D*(?:0x)?([0-9A-Fa-f]+)", line, re.IGNORECASE)
            if not id_match:
                continue

            hex_values = re.findall(r"(?<![0-9A-Fa-f])(?:0x)?([0-9A-Fa-f]{2})(?![0-9A-Fa-f])", line)
            if len(hex_values) < 8:
                continue

            arbitration_id = int(id_match.group(1), 16)
            data = bytes(int(value, 16) for value in hex_values[-8:])
            try:
                return parse_can_frame(
                    arbitration_id=arbitration_id,
                    data=data,
                    expected_arbitration_id=expected_arbitration_id,
                    expected_command=expected_command,
                )
            except ProtocolError:
                continue
        return None

    def _connect_waveshare_usbcana(self) -> str:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install -r requirements.txt")
        if self.is_connected:
            return f"Already connected to Waveshare USB-CAN-A {self.config.channel}."
        if self.config.bitrate > 1_000_000:
            raise ValueError(
                "Waveshare USB-CAN-A supports CAN bitrate up to 1Mbps. "
                "The 2Mbps value is the USB serial baudrate, not CAN bitrate."
            )

        self.connection_step = f"opening serial {self.config.channel}"
        self._serial = serial.Serial(
            port=self.config.channel,
            baudrate=self.config.serial_baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.config.timeout_seconds,
            write_timeout=self.config.timeout_seconds,
        )
        self.connection_step = "configuring Waveshare USB-CAN-A"
        self._configure_waveshare_usbcana()
        self.connection_step = "connected"
        return (
            f"Connected to Waveshare USB-CAN-A {self.config.channel}; "
            f"serial={self.config.serial_baudrate}; CAN={self.config.bitrate}."
        )

    def _configure_waveshare_usbcana(self) -> None:
        baud_code = self._waveshare_baud_code(self.config.bitrate)
        packet = bytearray(
            [
                0xAA,
                0x55,
                0x12,  # variable-length protocol setting
                baud_code,
                0x01,  # standard frame
                0x00,
                0x00,
                0x00,
                0x00,  # filter ID
                0x00,
                0x00,
                0x00,
                0x00,  # mask/block ID: accept all
                0x00,  # normal mode
                0x00,  # auto retransmit enabled
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
            ]
        )
        packet[19] = sum(packet[2:19]) & 0xFF
        self.connection_step = "write config"
        self._serial.write(bytes(packet))

    def _send_frame_waveshare(
        self,
        tx_id: int,
        data: bytes,
        rx_id: int | None,
        expected_command: int | None,
    ):
        packet = self._build_waveshare_variable_frame(tx_id, data)
        self.last_tx = f"serial={hex_bytes(packet)}; can_id=0x{tx_id:X} data={hex_bytes(data)}"
        self.last_rx = ""
        self._serial.write(packet)
        frame = self._recv_waveshare_expected(rx_id, expected_command)
        self.last_rx = f"id=0x{frame.arbitration_id:X} dlc=8 data={hex_bytes(frame.data)}"
        return frame

    def _recv_waveshare_expected(self, expected_arbitration_id: int | None, expected_command: int | None):
        deadline = self._monotonic() + self.config.timeout_seconds
        ignored: list[str] = []
        while self._monotonic() < deadline:
            frame = self._read_waveshare_variable_frame(deadline - self._monotonic())
            if frame is None:
                continue
            summary = f"id=0x{frame.arbitration_id:X} data={hex_bytes(frame.data)}"
            if (
                (expected_arbitration_id is None or frame.arbitration_id == expected_arbitration_id)
                and len(frame.data) == 8
                and (expected_command is None or frame.data[0] == expected_command)
            ):
                return frame
            ignored.append(summary)
            if len(ignored) > 5:
                ignored.pop(0)
        ignored_text = "; ignored=" + " | ".join(ignored) if ignored else ""
        expected_id = "any" if expected_arbitration_id is None else f"0x{expected_arbitration_id:X}"
        raise TimeoutError(
            f"Timed out waiting for CAN response id={expected_id}; "
            f"TX={self.last_tx}{ignored_text}"
        )

    def _read_waveshare_variable_frame(self, timeout: float):
        end_time = self._monotonic() + max(0.0, timeout)
        while self._monotonic() < end_time:
            first = self._serial.read(1)
            if not first:
                return None
            if first[0] != 0xAA:
                continue
            packet_type = self._serial.read(1)
            if not packet_type:
                return None
            length = packet_type[0] & 0x0F
            is_extended = bool(packet_type[0] & 0x20)
            id_length = 4 if is_extended else 2
            rest = self._serial.read(id_length + length + 1)
            if len(rest) != id_length + length + 1 or rest[-1] != 0x55:
                continue
            can_id = int.from_bytes(rest[:id_length], "little")
            data = bytes(rest[id_length:-1])
            return parse_can_frame(can_id, data)
        return None

    @staticmethod
    def _build_waveshare_variable_frame(arbitration_id: int, data: bytes) -> bytes:
        if len(data) > 8:
            raise ValueError("CAN data cannot exceed 8 bytes.")
        packet_type = 0xC0 | len(data)
        can_id = arbitration_id.to_bytes(2, "little")
        return bytes([0xAA, packet_type]) + can_id + data + bytes([0x55])

    @staticmethod
    def _waveshare_baud_code(bitrate: int) -> int:
        codes = {
            1_000_000: 0x01,
            800_000: 0x02,
            500_000: 0x03,
            400_000: 0x04,
            250_000: 0x05,
            200_000: 0x06,
            125_000: 0x07,
            100_000: 0x08,
            50_000: 0x09,
            20_000: 0x0A,
            10_000: 0x0B,
            5_000: 0x0C,
        }
        if bitrate not in codes:
            raise ValueError("Waveshare USB-CAN-A supports CAN bitrates from 5kbps to 1Mbps.")
        return codes[bitrate]

    def _recv_scan_response(self, motor_id: int, timeout: float):
        if self.config.can_interface == "waveshare_usbcana":
            deadline = self._monotonic() + timeout
            expected_id = rx_arbitration_id(motor_id)
            while self._monotonic() < deadline:
                frame = self._read_waveshare_variable_frame(deadline - self._monotonic())
                if frame and frame.arbitration_id == expected_id and len(frame.data) == 8 and frame.data[0] == 0x60:
                    return frame
            return None

        expected_id = rx_arbitration_id(motor_id)
        deadline = self._monotonic() + timeout
        while self._monotonic() < deadline:
            message = self._bus.recv(timeout=max(0.0, deadline - self._monotonic()))
            if message is None:
                return None
            data = bytes(message.data)
            if message.arbitration_id == expected_id and len(data) == 8 and data[0] == 0x60:
                return message
        return None

    @staticmethod
    def _summarize_result(result: Any) -> str:
        if not isinstance(result, dict):
            return str(result)

        preferred_keys = [
            "channel",
            "interface",
            "bitrate",
            "motor_id",
            "value",
            "encoder_pulses",
            "circle_angle_degrees",
            "temperature_c",
            "count",
            "frames",
            "tx",
            "rx",
        ]
        parts = []
        for key in preferred_keys:
            if key in result:
                value = result[key]
                if isinstance(value, str) and len(value) > 120:
                    value = value[:117] + "..."
                parts.append(f"{key}={value}")
        return "; ".join(parts) if parts else str(result)

    @staticmethod
    def _format_message(message) -> str:
        frame_type = "ext" if getattr(message, "is_extended_id", False) else "std"
        return (
            f"id=0x{message.arbitration_id:X} {frame_type} "
            f"dlc={len(message.data)} data={hex_bytes(bytes(message.data))}"
        )

    @staticmethod
    def _monotonic() -> float:
        import time

        return time.monotonic()
