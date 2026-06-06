#!/usr/bin/env python3
"""
CubeMars UART status/debug polling script.

This script can:
- Optionally send the debug terminal command "run" first.
- Poll COMM_GET_VALUES.
- Verify CRC before accepting packets.
- Decode COMM_PRINT text packets.
- Parse COMM_GET_VALUES status packets.

Typical usage:

    python3 cubemars_uart_status_poll.py \
        --port /dev/tty.usbmodem34B7DA5F92CC2 \
        --baud 921600 \
        --hz 2 \
        --send-run

If you only want pure read-only COMM_GET_VALUES polling, omit --send-run.

Wiring with USB-to-TTL adapter:
    Adapter TX  -> CubeMars RX
    Adapter RX  <- CubeMars TX
    Adapter GND -> CubeMars GND

Do not power the motor from the USB-to-TTL adapter.
Use the motor's proper power supply.
"""

import argparse
import time
import struct
from typing import Optional, Tuple, Dict, Any

import serial


# CubeMars / VESC-style packet IDs
COMM_FW_VERSION = 0x00
COMM_GET_VALUES = 0x04
COMM_TERMINAL_CMD = 0x14
COMM_PRINT = 0x15
COMM_ROTOR_POSITION = 0x16
COMM_GET_VALUES_SETUP = 0x32

FRAME_START = 0x02
FRAME_END = 0x03


ERROR_CODES = {
    0: "FAULT_CODE_NONE",
    1: "FAULT_CODE_OVER_VOLTAGE",
    2: "FAULT_CODE_UNDER_VOLTAGE",
    3: "FAULT_CODE_DRV",
    4: "FAULT_CODE_ABS_OVER_CURRENT",
    5: "FAULT_CODE_OVER_TEMP_FET",
    6: "FAULT_CODE_OVER_TEMP_MOTOR",
    7: "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE",
    8: "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE",
    9: "FAULT_CODE_MCU_UNDER_VOLTAGE",
    10: "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET",
    11: "FAULT_CODE_ENCODER_SPI",
    12: "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    14: "FAULT_CODE_FLASH_CORRUPTION",
    15: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1",
    16: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3",
    18: "FAULT_CODE_UNBALANCED_CURRENTS",
}


def crc16_ccitt(data: bytes) -> int:
    """
    CRC16 used by the CubeMars/VESC-style packet format.
    CRC is calculated over payload only.

    Self-test:
        crc16_ccitt(bytes([0x04])) == 0x4084
    """
    crc = 0x0000
    poly = 0x1021

    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc & 0xFFFF


def make_packet(payload: bytes) -> bytes:
    """
    Build packet:

        0x02 LEN PAYLOAD CRC_H CRC_L 0x03
    """
    if len(payload) > 255:
        raise ValueError("Payload too long for this simple packet format")

    crc = crc16_ccitt(payload)

    return bytes([
        FRAME_START,
        len(payload),
        *payload,
        (crc >> 8) & 0xFF,
        crc & 0xFF,
        FRAME_END,
        ])


def make_terminal_cmd(text: str) -> bytes:
    """
    Build a COMM_TERMINAL_CMD packet.

    Examples:
        run
        exit
        encoder
        origin
    """
    return make_packet(bytes([COMM_TERMINAL_CMD]) + text.encode("ascii"))


def read_packet(ser: serial.Serial, timeout_s: float = 0.2) -> Optional[bytes]:
    """
    Read one CubeMars-style packet:

        0x02 LEN PAYLOAD CRC_H CRC_L 0x03

    Returns full packet as bytes, or None on timeout/malformed frame.
    """
    deadline = time.monotonic() + timeout_s

    # Find frame start byte.
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue

        if b[0] == FRAME_START:
            break
    else:
        return None

    # Read length byte.
    length_b = ser.read(1)
    if len(length_b) != 1:
        return None

    payload_len = length_b[0]

    # Read payload + CRC_H + CRC_L + frame end.
    rest = ser.read(payload_len + 3)
    if len(rest) != payload_len + 3:
        return None

    packet = bytes([FRAME_START, payload_len]) + rest

    if packet[-1] != FRAME_END:
        return None

    return packet


def verify_packet(packet: bytes) -> Tuple[bool, bytes, int, int]:
    """
    Verify CRC of a received packet.

    Returns:
        ok, payload, received_crc, computed_crc
    """
    if len(packet) < 5:
        return False, b"", 0, 0

    if packet[0] != FRAME_START or packet[-1] != FRAME_END:
        return False, b"", 0, 0

    payload_len = packet[1]
    expected_len = 1 + 1 + payload_len + 2 + 1

    if len(packet) != expected_len:
        return False, b"", 0, 0

    payload = packet[2:2 + payload_len]
    received_crc = (packet[2 + payload_len] << 8) | packet[2 + payload_len + 1]
    computed_crc = crc16_ccitt(payload)

    return received_crc == computed_crc, payload, received_crc, computed_crc


def get_i16(data: bytes, idx: int) -> int:
    return struct.unpack(">h", data[idx:idx + 2])[0]


def get_u8(data: bytes, idx: int) -> int:
    return data[idx]


def get_i32(data: bytes, idx: int) -> int:
    return struct.unpack(">i", data[idx:idx + 4])[0]


def parse_get_values(payload: bytes) -> Optional[Dict[str, Any]]:
    """
    Parse COMM_GET_VALUES response.

    Expected payload format:

        payload[0] = 0x04

    Then:
        MOS Temperature:              int16 / 10
        Motor Temperature:            int16 / 10
        Output Current:               int32 / 100
        Input Current:                int32 / 100
        Id Current:                   int32 / 100
        Iq Current:                   int32 / 100
        Duty:                         int16 / 1000
        Speed:                        int32
        Input Voltage:                int16 / 10
        Reserved:                     24 bytes
        Error/status code:            uint8
        Motor Outer Loop Position:    int32 / 1000 or / 1000000 depending firmware/manual section
        Motor Control ID:             uint8
        Temperature reserved:         6 bytes
        Vd:                           int32 / 1000
        Vq:                           int32 / 1000
    """
    if not payload or payload[0] != COMM_GET_VALUES:
        return None

    # Full response in your logs/manual has len 0x49 = 73 bytes.
    if len(payload) < 73:
        return {
            "parse_error": f"COMM_GET_VALUES payload too short: {len(payload)} bytes",
            "raw_payload_hex": payload.hex(" "),
        }

    i = 1

    mos_temp = get_i16(payload, i) / 10.0
    i += 2

    motor_temp = get_i16(payload, i) / 10.0
    i += 2

    output_current = get_i32(payload, i) / 100.0
    i += 4

    input_current = get_i32(payload, i) / 100.0
    i += 4

    id_current = get_i32(payload, i) / 100.0
    i += 4

    iq_current = get_i32(payload, i) / 100.0
    i += 4

    duty = get_i16(payload, i) / 1000.0
    i += 2

    speed_erpm = get_i32(payload, i)
    i += 4

    input_voltage = get_i16(payload, i) / 10.0
    i += 2

    reserved_24 = payload[i:i + 24]
    i += 24

    error_code = get_u8(payload, i)
    i += 1

    # Manual versions disagree slightly. Your class used /1000000. The newer manual text says /1000.
    # Print both so you can see which one makes sense for your firmware.
    raw_position = get_i32(payload, i)
    position_div_1000 = raw_position / 1000.0
    position_div_1000000 = raw_position / 1000000.0
    i += 4

    control_id = get_u8(payload, i)
    i += 1

    temp_reserved_6 = payload[i:i + 6]
    i += 6

    vd = get_i32(payload, i) / 1000.0
    i += 4

    vq = get_i32(payload, i) / 1000.0
    i += 4

    return {
        "mos_temperature_C": mos_temp,
        "motor_temperature_C": motor_temp,
        "output_current_A": output_current,
        "input_current_A": input_current,
        "id_current_A": id_current,
        "iq_current_A": iq_current,
        "duty": duty,
        "speed_ERPM": speed_erpm,
        "input_voltage_V": input_voltage,
        "reserved_24_hex": reserved_24.hex(" "),
        "error_code": error_code,
        "error": ERROR_CODES.get(error_code, f"UNKNOWN_ERROR_{error_code}"),
        "raw_position": raw_position,
        "position_div_1000": position_div_1000,
        "position_div_1000000": position_div_1000000,
        "control_id_or_motor_id": control_id,
        "temp_reserved_6_hex": temp_reserved_6.hex(" "),
        "Vd_V": vd,
        "Vq_V": vq,
    }


def print_status(status: Dict[str, Any]) -> None:
    if "parse_error" in status:
        print(f"Status parse error: {status['parse_error']}")
        print(f"Raw payload: {status['raw_payload_hex']}")
        return

    print("COMM_GET_VALUES status:")
    print(f"  MOS temp:       {status['mos_temperature_C']:.1f} C")
    print(f"  Motor temp:     {status['motor_temperature_C']:.1f} C")
    print(f"  Input voltage:  {status['input_voltage_V']:.2f} V")
    print(f"  Input current:  {status['input_current_A']:.3f} A")
    print(f"  Output current: {status['output_current_A']:.3f} A")
    print(f"  Id current:     {status['id_current_A']:.3f} A")
    print(f"  Iq current:     {status['iq_current_A']:.3f} A")
    print(f"  Duty:           {status['duty']:.4f}")
    print(f"  Speed:          {status['speed_ERPM']} ERPM")
    print(f"  Raw position:   {status['raw_position']}")
    print(f"  Position /1000: {status['position_div_1000']:.6f}")
    print(f"  Position /1e6:  {status['position_div_1000000']:.6f}")
    print(f"  Control/Motor ID: {status['control_id_or_motor_id']}")
    print(f"  Vd:             {status['Vd_V']:.3f} V")
    print(f"  Vq:             {status['Vq_V']:.3f} V")
    print(f"  Error:          {status['error_code']} ({status['error']})")


def handle_packet(packet: bytes, poll_num: int, verbose_raw: bool) -> bool:
    """
    Returns True if this packet was a COMM_GET_VALUES status packet.
    """
    ok, payload, received_crc, computed_crc = verify_packet(packet)

    if verbose_raw:
        print(f"[{poll_num}] RX: {packet.hex(' ')}")
        print(f"[{poll_num}] Payload: {payload.hex(' ')}")
        print(
            f"[{poll_num}] CRC received=0x{received_crc:04X}, "
            f"computed=0x{computed_crc:04X}, ok={ok}"
        )

    if not ok:
        print(f"[{poll_num}] CRC mismatch. Ignoring packet.")
        return False

    if not payload:
        print(f"[{poll_num}] Empty payload.")
        return False

    packet_id = payload[0]

    if packet_id == COMM_GET_VALUES:
        status = parse_get_values(payload)
        print(f"[{poll_num}] Got COMM_GET_VALUES / status frame.")
        if status is not None:
            print_status(status)
        return True

    if packet_id == COMM_PRINT:
        text = payload[1:].decode("ascii", errors="replace")
        print(f"[{poll_num}] COMM_PRINT/debug text: {text!r}")
        return False

    if packet_id == COMM_FW_VERSION:
        print(f"[{poll_num}] COMM_FW_VERSION-ish payload: {payload.hex(' ')}")
        maybe_text = payload[1:].decode("ascii", errors="replace")
        print(f"[{poll_num}] Decoded-ish: {maybe_text!r}")
        return False

    if packet_id == COMM_ROTOR_POSITION:
        if len(payload) >= 5:
            pos_raw = get_i32(payload, 1)
            print(f"[{poll_num}] COMM_ROTOR_POSITION raw={pos_raw}, /10000={pos_raw / 10000.0}")
        else:
            print(f"[{poll_num}] COMM_ROTOR_POSITION but too short: {payload.hex(' ')}")
        return False

    print(f"[{poll_num}] Valid packet but not status: id=0x{packet_id:02X}, payload={payload.hex(' ')}")
    return False


def drain_packets(ser: serial.Serial, duration_s: float, verbose_raw: bool = False) -> None:
    """
    Drain and print packets for a short time.
    Useful after sending 'run' because the motor may print menu/debug text.
    """
    deadline = time.monotonic() + duration_s
    n = 0

    while time.monotonic() < deadline:
        packet = read_packet(ser, timeout_s=0.05)
        if packet is None:
            continue

        n += 1
        handle_packet(packet, poll_num=-n, verbose_raw=verbose_raw)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="CubeMars UART COMM_GET_VALUES status polling/debug test"
    )

    parser.add_argument(
        "--port",
        required=True,
        help="Serial port, e.g. /dev/ttyUSB0, /dev/tty.usbserial-XXXX, /dev/tty.usbmodemXXXX, or COM3",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=921600,
        help="UART baud rate. Must match CubeMars setting. Default: 921600",
    )

    parser.add_argument(
        "--hz",
        type=float,
        default=2.0,
        help="Polling frequency in Hz. Default: 2 Hz",
    )

    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of COMM_GET_VALUES polls to send. 0 means run forever. Default: 0",
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=0.2,
        help="Response timeout in seconds for each read. Default: 0.2",
    )

    parser.add_argument(
        "--send-run",
        action="store_true",
        help="Send debug terminal command 'run' before polling.",
    )

    parser.add_argument(
        "--send-encoder",
        action="store_true",
        help="Send debug terminal command 'encoder' before polling.",
    )

    parser.add_argument(
        "--send-exit",
        action="store_true",
        help="Send debug terminal command 'exit' before polling. Useful if stuck in a submenu.",
    )

    parser.add_argument(
        "--startup-drain",
        type=float,
        default=0.5,
        help="Seconds to drain startup/debug packets after opening serial. Default: 0.5",
    )

    parser.add_argument(
        "--post-command-drain",
        type=float,
        default=0.5,
        help="Seconds to drain packets after sending --send-run or --send-exit. Default: 0.5",
    )

    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print raw RX/payload/CRC for every received packet.",
    )

    parser.add_argument(
        "--quiet-tx",
        action="store_true",
        help="Do not print TX line every poll.",
    )

    args = parser.parse_args()

    if args.hz <= 0:
        raise ValueError("--hz must be greater than 0")

    request = make_packet(bytes([COMM_GET_VALUES]))

    expected_test_crc = crc16_ccitt(bytes([COMM_GET_VALUES]))
    print(f"CRC test: CRC16([0x04]) = 0x{expected_test_crc:04X}")

    if expected_test_crc != 0x4084:
        print("WARNING: CRC self-test failed. Expected 0x4084.")
        print("Do not continue until CRC implementation is fixed.")
        return

    print()
    print("CubeMars UART status/debug poll")
    print("-------------------------------")
    print(f"Port:        {args.port}")
    print(f"Baud:        {args.baud}")
    print(f"Poll rate:   {args.hz} Hz")
    print(f"TX request:  {request.hex(' ')}")
    print(f"Send run:    {args.send_run}")
    print(f"Send exit:   {args.send_exit}")
    print()

    period = 1.0 / args.hz
    polls_sent = 0
    status_frames = 0

    with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01,
            write_timeout=0.1,
    ) as ser:

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("Serial port opened.")
        print(f"Draining startup/debug packets for {args.startup_drain:.2f} s...")
        drain_packets(ser, duration_s=args.startup_drain, verbose_raw=args.raw)

        if args.send_exit:
            cmd = make_terminal_cmd("exit")
            print(f"TX terminal command 'exit': {cmd.hex(' ')}")
            ser.write(cmd)
            ser.flush()
            drain_packets(ser, duration_s=args.post_command_drain, verbose_raw=args.raw)

        if args.send_run:
            cmd = make_terminal_cmd("run")
            print(f"TX terminal command 'run': {cmd.hex(' ')}")
            ser.write(cmd)
            ser.flush()
            drain_packets(ser, duration_s=args.post_command_drain, verbose_raw=args.raw)
        if args.send_encoder:
            cmd = make_terminal_cmd("encoder")
            print(f"TX terminal command 'encoder': {cmd.hex(' ')}")
            ser.write(cmd)
            ser.flush()
            drain_packets(ser, duration_s=2.0, verbose_raw=True)

        # Clear leftovers before status polling.
        ser.reset_input_buffer()

        print()
        print("Starting COMM_GET_VALUES polling...")
        print()

        next_poll = time.monotonic()

        while True:
            if args.count > 0 and polls_sent >= args.count:
                print("Done.")
                print(f"Polls sent:    {polls_sent}")
                print(f"Status frames: {status_frames}")
                break

            now = time.monotonic()

            if now >= next_poll:
                polls_sent += 1

                if not args.quiet_tx:
                    print(f"[{polls_sent}] TX: {request.hex(' ')}")

                ser.write(request)
                ser.flush()

                got_status_this_poll = False

                # Read at least one packet. There may be queued COMM_PRINT packets before status.
                # Keep reading briefly until timeout window expires.
                read_deadline = time.monotonic() + args.timeout

                while time.monotonic() < read_deadline:
                    packet = read_packet(ser, timeout_s=0.03)

                    if packet is None:
                        continue

                    is_status = handle_packet(
                        packet,
                        poll_num=polls_sent,
                        verbose_raw=args.raw,
                    )

                    if is_status:
                        status_frames += 1
                        got_status_this_poll = True
                        break

                if not got_status_this_poll:
                    print(f"[{polls_sent}] No COMM_GET_VALUES status response in timeout window.")

                print()
                next_poll += period

            time.sleep(0.001)


if __name__ == "__main__":
    main()

"""
Raw output after running command `python3 cubemars_uart_status_poll.py --port /dev/tty.usbmodem34B7DA5F92CC2 --baud 921600 --hz 2 --count 20 --raw`: 

[4] TX: 02 01 04 40 84 03
[4] RX: 02 49 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0a 79 61 40 68 00 00 00 00 00 00 00 00 00 00 00 00 00 00 b9 96 03
[4] Payload: 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0a 79 61 40 68 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[4] CRC received=0xB996, computed=0xB996, ok=True
[4] Got COMM_GET_VALUES / status frame.
COMM_GET_VALUES status:
  MOS temp:       0.0 C
  Motor temp:     0.0 C
  Input voltage:  0.00 V
  Input current:  0.000 A
  Output current: 0.000 A
  Id current:     0.000 A
  Iq current:     0.000 A
  Duty:           0.0000
  Speed:          0 ERPM
  Raw position:   175726912
  Position /1000: 175726.912000
  Position /1e6:  175.726912
  Control/Motor ID: 104
  Vd:             0.000 V
  Vq:             0.000 V
  Error:          0 (FAULT_CODE_NONE)

[5] TX: 02 01 04 40 84 03
[5] RX: 02 17 15 0a 0d 0a 0d 20 43 75 62 65 4d 61 72 73 2d 36 30 2d 36 0a 0d 0a 0d 52 a3 03
[5] Payload: 15 0a 0d 0a 0d 20 43 75 62 65 4d 61 72 73 2d 36 30 2d 36 0a 0d 0a 0d
[5] CRC received=0x52A3, computed=0x52A3, ok=True
[5] COMM_PRINT/debug text: '\n\r\n\r CubeMars-60-6\n\r\n\r'
[5] RX: 02 17 15 0a 0d 0a 0d 20 43 75 62 65 4d 61 72 73 2d 41 4b 36 30 0a 0d 0a 0d fe 5a 03
[5] Payload: 15 0a 0d 0a 0d 20 43 75 62 65 4d 61 72 73 2d 41 4b 36 30 0a 0d 0a 0d
[5] CRC received=0xFE5A, computed=0xFE5A, ok=True
[5] COMM_PRINT/debug text: '\n\r\n\r CubeMars-AK60\n\r\n\r'
[5] RX: 02 11 15 0a 0d 20 44 65 62 75 67 20 49 6e 66 6f 3a 0a 0d 5c 88 03
[5] Payload: 15 0a 0d 20 44 65 62 75 67 20 49 6e 66 6f 3a 0a 0d
[5] CRC received=0x5C88, computed=0x5C88, ok=True
[5] COMM_PRINT/debug text: '\n\r Debug Info:\n\r'
[5] RX: 02 19 15 20 48 61 72 64 77 61 72 65 20 56 65 72 73 69 6f 6e 3a 20 32 2e 32 0a 0d 46 7a 03
[5] Payload: 15 20 48 61 72 64 77 61 72 65 20 56 65 72 73 69 6f 6e 3a 20 32 2e 32 0a 0d
[5] CRC received=0x467A, computed=0x467A, ok=True
[5] COMM_PRINT/debug text: ' Hardware Version: 2.2\n\r'
[5] RX: 02 19 15 20 46 69 72 6d 77 61 72 65 20 56 65 72 73 69 6f 6e 3a 20 31 2e 32 0a 0d 28 21 03
[5] Payload: 15 20 46 69 72 6d 77 61 72 65 20 56 65 72 73 69 6f 6e 3a 20 31 2e 32 0a 0d
[5] CRC received=0x2821, computed=0x2821, ok=True
[5] COMM_PRINT/debug text: ' Firmware Version: 1.2\n\r'
[5] RX: 02 2a 15 20 41 44 43 31 20 4f 66 66 73 65 74 3a 20 32 30 39 30 20 20 20 20 41 44 43 32 20 4f 66 66 73 65 74 3a 20 32 30 33 32 0a 0d 9e d0 03
[5] Payload: 15 20 41 44 43 31 20 4f 66 66 73 65 74 3a 20 32 30 39 30 20 20 20 20 41 44 43 32 20 4f 66 66 73 65 74 3a 20 32 30 33 32 0a 0d
[5] CRC received=0x9ED0, computed=0x9ED0, ok=True
[5] COMM_PRINT/debug text: ' ADC1 Offset: 2090    ADC2 Offset: 2032\n\r'
[5] RX: 02 30 15 20 50 6f 73 69 74 69 6f 6e 20 53 65 6e 73 6f 72 20 45 6c 65 63 74 72 69 63 61 6c 20 4f 66 66 73 65 74 3a 20 20 20 2d 34 2e 36 32 34 35 0a 0d b4 9a 03
[5] Payload: 15 20 50 6f 73 69 74 69 6f 6e 20 53 65 6e 73 6f 72 20 45 6c 65 63 74 72 69 63 61 6c 20 4f 66 66 73 65 74 3a 20 20 20 2d 34 2e 36 32 34 35 0a 0d
[5] CRC received=0xB49A, computed=0xB49A, ok=True
[5] COMM_PRINT/debug text: ' Position Sensor Electrical Offset:   -4.6245\n\r'
[5] RX: 02 21 15 20 4f 75 74 70 75 74 20 5a 65 72 6f 20 50 6f 73 69 74 69 6f 6e 3a 20 20 30 2e 30 30 30 30 0a 0d 93 c4 03
[5] Payload: 15 20 4f 75 74 70 75 74 20 5a 65 72 6f 20 50 6f 73 69 74 69 6f 6e 3a 20 20 30 2e 30 30 30 30 0a 0d
[5] CRC received=0x93C4, computed=0x93C4, ok=True
[5] COMM_PRINT/debug text: ' Output Zero Position:  0.0000\n\r'
[5] RX: 02 0e 15 20 43 41 4e 20 49 44 3a 20 20 31 0a 0d 2c 7e 03
[5] Payload: 15 20 43 41 4e 20 49 44 3a 20 20 31 0a 0d
[5] CRC received=0x2C7E, computed=0x2C7E, ok=True
[5] COMM_PRINT/debug text: ' CAN ID:  1\n\r'
[5] RX: 02 07 15 0a 0d 0a 0d 0a 0d fd 38 03
[5] Payload: 15 0a 0d 0a 0d 0a 0d
[5] CRC received=0xFD38, computed=0xFD38, ok=True
[5] COMM_PRINT/debug text: '\n\r\n\r\n\r'
[5] RX: 02 0d 15 20 43 6f 6d 6d 61 6e 64 73 3a 0a 0d c5 28 03
[5] Payload: 15 20 43 6f 6d 6d 61 6e 64 73 3a 0a 0d
[5] CRC received=0xC528, computed=0xC528, ok=True
[5] COMM_PRINT/debug text: ' Commands:\n\r'
[5] RX: 02 14 15 20 72 75 6e 20 2d 20 4d 6f 74 6f 72 20 4d 6f 64 65 0a 0d dd fd 03
[5] Payload: 15 20 72 75 6e 20 2d 20 4d 6f 74 6f 72 20 4d 6f 64 65 0a 0d
[5] CRC received=0xDDFD, computed=0xDDFD, ok=True
[5] COMM_PRINT/debug text: ' run - Motor Mode\n\r'
[5] RX: 02 21 15 20 63 61 6c 69 62 72 61 74 65 20 2d 20 43 61 6c 69 62 72 61 74 65 20 45 6e 63 6f 64 65 72 0a 0d c5 48 03
[5] Payload: 15 20 63 61 6c 69 62 72 61 74 65 20 2d 20 43 61 6c 69 62 72 61 74 65 20 45 6e 63 6f 64 65 72 0a 0d
[5] CRC received=0xC548, computed=0xC548, ok=True
[5] COMM_PRINT/debug text: ' calibrate - Calibrate Encoder\n\r'
[5] RX: 02 1b 15 20 73 65 74 75 70 20 2d 20 53 65 74 75 70 20 74 68 65 20 6d 6f 74 6f 72 0a 0d 5c 47 03
[5] Payload: 15 20 73 65 74 75 70 20 2d 20 53 65 74 75 70 20 74 68 65 20 6d 6f 74 6f 72 0a 0d
[5] CRC received=0x5C47, computed=0x5C47, ok=True
[5] COMM_PRINT/debug text: ' setup - Setup the motor\n\r'
[5] RX: 02 24 15 20 65 6e 63 6f 64 65 72 20 2d 20 53 68 6f 77 20 65 6e 63 6f 64 65 72 20 76 61 6c 75 65 20 6e 6f 77 0a 0d 81 ea 03
[5] Payload: 15 20 65 6e 63 6f 64 65 72 20 2d 20 53 68 6f 77 20 65 6e 63 6f 64 65 72 20 76 61 6c 75 65 20 6e 6f 77 0a 0d
[5] CRC received=0x81EA, computed=0x81EA, ok=True
[5] COMM_PRINT/debug text: ' encoder - Show encoder value now\n\r'
[5] RX: 02 1e 15 20 6f 72 69 67 69 6e 20 2d 20 53 65 74 20 5a 65 72 6f 20 50 6f 73 69 74 69 6f 6e 0a 0d c4 af 03
[5] Payload: 15 20 6f 72 69 67 69 6e 20 2d 20 53 65 74 20 5a 65 72 6f 20 50 6f 73 69 74 69 6f 6e 0a 0d
[5] CRC received=0xC4AF, computed=0xC4AF, ok=True
[5] COMM_PRINT/debug text: ' origin - Set Zero Position\n\r'
[5] RX: 02 17 15 20 65 78 69 74 20 2d 20 45 78 69 74 20 74 6f 20 4d 65 6e 75 0a 0d c5 44 03
[5] Payload: 15 20 65 78 69 74 20 2d 20 45 78 69 74 20 74 6f 20 4d 65 6e 75 0a 0d
[5] CRC received=0xC544, computed=0xC544, ok=True
[5] COMM_PRINT/debug text: ' exit - Exit to Menu\n\r'
[5] RX: 02 17 15 20 65 78 69 74 20 2d 20 45 78 69 74 20 74 6f 20 4d 65 6e 75 0a 0d c5 44 03
[5] Payload: 15 20 65 78 69 74 20 2d 20 45 78 69 74 20 74 6f 20 4d 65 6e 75 0a 0d
[5] CRC received=0xC544, computed=0xC544, ok=True
[5] COMM_PRINT/debug text: ' exit - Exit to Menu\n\r'
[5] RX: 02 1e 00 05 01 0a 4d 49 54 20 54 45 53 54 20 56 32 00 41 00 52 00 01 51 33 32 36 36 32 39 01 00 25 d4 03
[5] Payload: 00 05 01 0a 4d 49 54 20 54 45 53 54 20 56 32 00 41 00 52 00 01 51 33 32 36 36 32 39 01 00
[5] CRC received=0x25D4, computed=0x25D4, ok=True
[5] COMM_FW_VERSION-ish payload: 00 05 01 0a 4d 49 54 20 54 45 53 54 20 56 32 00 41 00 52 00 01 51 33 32 36 36 32 39 01 00
[5] Decoded-ish: '\x05\x01\nMIT TEST V2\x00A\x00R\x00\x01Q326629\x01\x00'
[5] No COMM_GET_VALUES status response in timeout window.
"""

"""
by running `python3 -m serial.tools.miniterm /dev/tty.usbmodem34B7DA5F92CC2 921600 --raw`
python3 -m serial.tools.miniterm /dev/tty.usbserial-AV0KL7N0  921600 --raw

CubeMars-60-6
CubeMars-AK60

Debug Info:
Hardware Version: 2.2
Firmware Version: 1.2
ADC1 Offset: 2090    ADC2 Offset: 2032
Position Sensor Electrical Offset:  -4.6245
Output Zero Position:  0.0000
CAN ID:  1



Commands:
run - Motor Mode
calibrate - Calibrate Encoder
setup - Setup the motor
encoder - Show encoder value now
origin - Set Zero Position
exit - Exit to Menu

MIT TEST V2
"""
"""
The frames sent have payload ID "0x15" (decimal 21) which seems to correspond to debugging ? 
I get one frame with payload ID "0x04" and this does in fact give me the running parameter values (once)
"""

# How to switch to Servo Mode ?


# alternatives : use cmd = bytes.fromhex("02 08 14 65 6e 63 6f 64 65 72 b0 4c 03") to send HEX commands directly