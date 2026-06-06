#!/usr/bin/env python3
"""
Interactive CubeMars framed UART terminal.

This is NOT a normal ASCII serial terminal.

The CubeMars debug terminal expects commands wrapped in packets:

    0x02 LEN 0x14 ASCII_COMMAND CRC_H CRC_L 0x03

where:
    0x14 = COMM_TERMINAL_CMD
    0x15 = COMM_PRINT response

Example:
    encoder

is sent as:

    02 08 14 65 6e 63 6f 64 65 72 b0 4c 03

Usage:
    python3 cubemars_terminal.py --port /dev/tty.usbmodem34B7DA5F92CC2 --baud 921600

Then type:
    encoder
    run
    exit
    origin

Exit this script with:
    /quit
"""

import argparse
import serial
import threading
import time
import sys
from typing import Optional, Tuple


FRAME_START = 0x02
FRAME_END = 0x03

COMM_FW_VERSION = 0x00
COMM_TERMINAL_CMD = 0x14
COMM_PRINT = 0x15
COMM_GET_VALUES = 0x04
COMM_ROTOR_POSITION = 0x16


def crc16_ccitt(data: bytes) -> int:
    """
    CubeMars/VESC-style CRC16-CCITT.
    CRC is computed over the payload only.

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
    Build CubeMars packet:

        0x02 LEN PAYLOAD CRC_H CRC_L 0x03
    """
    if len(payload) > 255:
        raise ValueError("Payload too long for simple 1-byte length packet")

    crc = crc16_ccitt(payload)

    return bytes([
        FRAME_START,
        len(payload),
        *payload,
        (crc >> 8) & 0xFF,
        crc & 0xFF,
        FRAME_END,
        ])


def make_terminal_packet(command: str) -> bytes:
    """
    Wrap a terminal command string as COMM_TERMINAL_CMD.
    """
    command = command.strip()
    return make_packet(bytes([COMM_TERMINAL_CMD]) + command.encode("ascii"))


def make_get_values_packet() -> bytes:
    """
    COMM_GET_VALUES packet:
        02 01 04 40 84 03
    """
    return make_packet(bytes([COMM_GET_VALUES]))


def read_packet(ser: serial.Serial, timeout_s: float = 0.1) -> Optional[bytes]:
    """
    Read one CubeMars packet:

        0x02 LEN PAYLOAD CRC_H CRC_L 0x03

    Returns:
        full packet bytes, or None on timeout/malformed frame.
    """
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue

        if b[0] == FRAME_START:
            break
    else:
        return None

    length_b = ser.read(1)
    if len(length_b) != 1:
        return None

    payload_len = length_b[0]
    rest = ser.read(payload_len + 3)

    if len(rest) != payload_len + 3:
        return None

    packet = bytes([FRAME_START, payload_len]) + rest

    if packet[-1] != FRAME_END:
        return None

    return packet


def verify_packet(packet: bytes) -> Tuple[bool, bytes, int, int]:
    """
    Verify CRC.

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


def decode_payload(payload: bytes, show_packets: bool = False) -> None:
    """
    Decode known packet payloads.
    """
    if not payload:
        print("\n[empty payload]")
        return

    packet_id = payload[0]

    if packet_id == COMM_PRINT:
        text = payload[1:].decode("ascii", errors="replace")
        print(text, end="", flush=True)
        return

    if packet_id == COMM_FW_VERSION:
        # This packet is not purely text, but often contains firmware string.
        text = payload[1:].decode("ascii", errors="replace")
        print(f"\n[COMM_FW_VERSION / id=0x00] {payload.hex(' ')}")
        print(f"[decoded-ish] {text!r}")
        return

    if packet_id == COMM_GET_VALUES:
        print(f"\n[COMM_GET_VALUES reply] {payload.hex(' ')}")
        return

    if packet_id == COMM_ROTOR_POSITION:
        print(f"\n[COMM_ROTOR_POSITION] {payload.hex(' ')}")
        return

    if show_packets:
        text = payload[1:].decode("ascii", errors="replace")
        print(f"\n[packet id=0x{packet_id:02X}] {payload.hex(' ')}")
        if text:
            print(f"[decoded-ish] {text!r}")
    else:
        print(f"\n[packet id=0x{packet_id:02X}, len={len(payload)}]")


def reader_loop(ser: serial.Serial, stop_flag: dict, show_packets: bool, show_crc: bool) -> None:
    """
    Background thread: read packets and print decoded responses.
    """
    while not stop_flag["stop"]:
        packet = read_packet(ser, timeout_s=0.1)

        if packet is None:
            continue

        ok, payload, rx_crc, calc_crc = verify_packet(packet)

        if show_crc:
            print(
                f"\n[rx packet] {packet.hex(' ')} "
                f"crc_rx=0x{rx_crc:04X} crc_calc=0x{calc_crc:04X} ok={ok}"
            )

        if not ok:
            print(f"\n[bad crc] {packet.hex(' ')}")
            continue

        decode_payload(payload, show_packets=show_packets)


def print_help() -> None:
    print()
    print("Local commands:")
    print("  /help          show this help")
    print("  /quit          quit")
    print("  /raw HEX       send raw hex bytes, e.g. /raw 02 01 04 40 84 03")
    print("  /get           send COMM_GET_VALUES")
    print("  /packet CMD    send CMD as framed terminal command")
    print()
    print("CubeMars terminal commands to try:")
    print("  encoder")
    print("  run")
    print("  exit")
    print("  origin")
    print("  calibrate")
    print("  setup")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Interactive CubeMars framed UART terminal"
    )

    parser.add_argument(
        "--port",
        required=True,
        help="Serial port, e.g. /dev/ttyUSB0, /dev/tty.usbmodemXXXX, COM3",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=921600,
        help="UART baud rate. Default: 921600",
    )

    parser.add_argument(
        "--show-packets",
        action="store_true",
        help="Print unknown packet payloads in hex.",
    )

    parser.add_argument(
        "--show-crc",
        action="store_true",
        help="Print every received packet with CRC verification.",
    )

    parser.add_argument(
        "--no-reset-buffers",
        action="store_true",
        help="Do not reset input/output buffers on open.",
    )

    parser.add_argument(
        "--boot-wait",
        type=float,
        default=0.0,
        help="Wait this many seconds after opening before accepting input.",
    )

    args = parser.parse_args()

    crc_test = crc16_ccitt(bytes([COMM_GET_VALUES]))
    if crc_test != 0x4084:
        print(f"CRC self-test failed: expected 0x4084, got 0x{crc_test:04X}")
        sys.exit(1)

    print("Opening serial port...")
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")

    with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.2,
    ) as ser:

        if not args.no_reset_buffers:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

        stop_flag = {"stop": False}

        reader = threading.Thread(
            target=reader_loop,
            args=(ser, stop_flag, args.show_packets, args.show_crc),
            daemon=True,
        )
        reader.start()

        if args.boot_wait > 0:
            print(f"Waiting {args.boot_wait:.2f}s for boot/menu output...")
            time.sleep(args.boot_wait)

        print()
        print("CubeMars framed terminal ready.")
        print("Type /help for commands.")
        print("Type CubeMars commands directly, e.g. encoder, run, exit.")
        print()

        try:
            while True:
                try:
                    line = input("> ")
                except EOFError:
                    break

                line = line.strip()

                if not line:
                    continue

                if line in ["/quit", "/q"]:
                    break

                if line == "/help":
                    print_help()
                    continue

                if line == "/get":
                    packet = make_get_values_packet()
                    print(f"[tx COMM_GET_VALUES] {packet.hex(' ')}")
                    ser.write(packet)
                    ser.flush()
                    continue

                if line.startswith("/raw "):
                    hex_part = line[len("/raw "):].strip()
                    try:
                        packet = bytes.fromhex(hex_part)
                    except ValueError as exc:
                        print(f"Invalid hex: {exc}")
                        continue

                    print(f"[tx raw] {packet.hex(' ')}")
                    ser.write(packet)
                    ser.flush()
                    continue

                if line.startswith("/packet "):
                    command = line[len("/packet "):].strip()
                    packet = make_terminal_packet(command)
                    print(f"[tx terminal] {command!r} -> {packet.hex(' ')}")
                    ser.write(packet)
                    ser.flush()
                    continue

                if line.startswith("/nl "):
                    command = line[len("/nl "):]
                    packet = make_packet(bytes([COMM_TERMINAL_CMD]) + command.encode("ascii") + b"\n")
                    print(f"[tx terminal+LF] {command!r} -> {packet.hex(' ')}")
                    ser.write(packet)
                    ser.flush()
                    continue

                if line.startswith("/crlf "):
                    command = line[len("/crlf "):]
                    packet = make_packet(bytes([COMM_TERMINAL_CMD]) + command.encode("ascii") + b"\r\n")
                    print(f"[tx terminal+CRLF] {command!r} -> {packet.hex(' ')}")
                    ser.write(packet)
                    ser.flush()
                    continue

                # Default: treat typed line as CubeMars terminal command.
                packet = make_terminal_packet(line)
                print(f"[tx terminal] {line!r} -> {packet.hex(' ')}")
                ser.write(packet)
                ser.flush()

        except KeyboardInterrupt:
            print("\nInterrupted.")

        finally:
            stop_flag["stop"] = True
            time.sleep(0.15)

    print("Closed.")


if __name__ == "__main__":
    main()