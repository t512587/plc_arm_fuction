# PyCanbusX6120 Motor CAN Tk UI

Python Tkinter CAN test utility based on `book/motor_protocol_v2.pdf`.

## Setup

1. Install Python 3.10 or newer.
2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Edit `config.json` for your CAN adapter:

```json
{
  "can_interface": "slcan",
  "channel": "COM5",
  "bitrate": 1000000,
  "motor_id": 1,
  "timeout_seconds": 1.0,
  "can_kwargs": {}
}
```

`can_interface` is the adapter backend name. `waveshare_usbcana` is built into
this project for Waveshare USB-CAN-A, which uses a custom serial-to-CAN protocol
instead of standard SLCAN.

For Waveshare USB-CAN-A, `serial_baudrate` is the USB virtual COM port speed.
Its factory default is 2 Mbps. `bitrate` is the actual CAN bus speed, and the
USB-CAN-A supports CAN up to 1 Mbps.

On Linux, the UI can also call the external `canusb` command-line program. This
matches commands like:

```bash
sudo ./canusb -d /dev/ttyUSB0 -s 1000000 -t -i 5 -j BEEE
```

Use this config for that mode:

```json
{
  "can_interface": "canusb_cli",
  "channel": "/dev/ttyUSB0",
  "bitrate": 1000000,
  "serial_baudrate": 2000000,
  "motor_id": 4,
  "timeout_seconds": 1.0,
  "canusb_path": "./canusb",
  "canusb_use_sudo": true,
  "can_kwargs": {}
}
```

The program replaces `-i` with the target CAN ID, for example `144`, and `-j`
with the 8-byte command data, for example `6000000000000000`.

Examples:

```json
{
  "can_interface": "waveshare_usbcana",
  "channel": "/dev/ttyUSB0",
  "bitrate": 1000000,
  "serial_baudrate": 2000000,
  "motor_id": 1,
  "timeout_seconds": 1.0,
  "can_kwargs": {}
}
```

For other 2 Mbps-capable adapters, use python-can backend names such as `pcan`,
`kvaser`, or `vector`:

```json
{
  "can_interface": "pcan",
  "channel": "PCAN_USBBUS1",
  "bitrate": 2000000,
  "motor_id": 1,
  "timeout_seconds": 1.0,
  "can_kwargs": {}
}
```

```json
{
  "can_interface": "vector",
  "channel": 0,
  "bitrate": 2000000,
  "motor_id": 1,
  "timeout_seconds": 1.0,
  "can_kwargs": {
    "app_name": "CANalyzer"
  }
}
```

4. Start the Tk UI:

```bash
cd /home/admin/int-amr/canbus
python3 app.py
```

If dependencies are not installed system-wide, this workspace can also run it
with uv on Linux:

```bash
cd /home/admin/int-amr/canbus
uv run --with "python-can[serial]" --with pyserial python3 app.py
```

To run the same connection and command checks without the UI:

```bash
cd /home/admin/int-amr/canbus
uv run --with "python-can[serial]" --with pyserial python3 test_can_functions.py
```

## CAN Format

- Bus interface: CAN
- Bitrate: 1 Mbps
- Single motor command TX ID: `0x140 + motor_id`
- Single motor response RX ID: `0x240 + motor_id`
- Frame format: data frame
- Frame type: standard frame
- DLC: 8 bytes

## Implemented Commands

- Connect / disconnect CAN bus.
- Read PID parameter command `0x30`.
- Read multi-turn encoder position command `0x60`.
- Read single-turn encoder command `0x90`.
- Read single-turn angle command `0x94`.
- Read motor status 2 command `0x9C`.
- Read / set CANID command `0x79` on arbitration ID `0x300`.
- Motion control command on arbitration ID `0x400 + motor_id`.
- Listen for any CAN frames for 3 seconds.
- Scan motor IDs 1 through 32 with command `0x60`.
- Test all connection and read commands in one run.
- Two-motor absolute position control Tk UI in `2motor.py`.
- Four-motor synchronized absolute position Tk UI in `2motor_sync.py`.

## Two-Motor UI

Start the dedicated two-motor controller:

```powershell
uv run --with "python-can[serial]" --with pyserial python 2motor.py
```

## Four-Motor Sync UI

Start the four-motor synchronized controller:

```powershell
uv run --with "python-can[serial]" --with pyserial python 2motor_sync.py
```

It controls:

- CAN command ID `0x141` (`motor_id=1`)
- CAN command ID `0x142` (`motor_id=2`)

`Read Current Positions` sends `0xA4` with speed `0`. The left/right buttons
move the displayed position by `-1` or `+1` degree with fixed speed `0x01F4`.

## Motion Control Ranges

The UI packs the motion command fields into the 8-byte MIT-style CAN frame:

- `p(rad)`: `-12.5` to `12.5`
- `v(rad/s)`: `-45` to `45`
- `kp`: `0` to `500`
- `kd`: `0` to `5`
- `t(Nm)`: `-24` to `24`

Values outside these ranges are clipped before packing.

## Timeout Checklist

If connection succeeds but every command times out:

- Confirm the motor is powered and CANH/CANL/GND are wired correctly.
- Confirm the bus has 120 ohm termination at both physical ends.
- Run `Listen CAN 3s`; if it shows no frames, the adapter is not receiving bus traffic.
- Run `Scan motor IDs 1-32`; if it finds a different ID, update `motor_id` in `config.json`.
- Confirm your USB-CAN adapter really uses the configured `can_interface`.
