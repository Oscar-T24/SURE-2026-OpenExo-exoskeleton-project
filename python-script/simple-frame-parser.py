#!/usr/bin/env python3
import argparse
import serial
from dataclasses import dataclass
import time


FRAME_START = 0x02
FRAME_END = 0x03


@dataclass
class Frame:
    payload: bytes
    checksum: int
    raw: bytes

    @property
    def command(self) -> int | None:
        return self.payload[0] if self.payload else None


def read_exact(ser: serial.Serial, n: int) -> bytes:
    data = ser.read(n)
    if len(data) != n:
        raise TimeoutError(f"Expected {n} bytes, got {len(data)}")
    return data


def read_frame(ser: serial.Serial) -> Frame:
    # Wait for frame header 0x02
    while True:
        b = read_exact(ser, 1)[0]
        if b == FRAME_START:
            break

    length = read_exact(ser, 1)[0]
    payload = read_exact(ser, length)

    checksum_bytes = read_exact(ser, 2)
    checksum = int.from_bytes(checksum_bytes, byteorder="big")

    end = read_exact(ser, 1)[0]
    if end != FRAME_END:
        raise ValueError(f"Invalid end byte: 0x{end:02X}")

    raw = bytes([FRAME_START, length]) + payload + checksum_bytes + bytes([end])

    return Frame(
        payload=payload,
        checksum=checksum,
        raw=raw,
    )

def crc16_ccitt(data: bytes) -> int:
    crc = 0x0000

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1

            crc &= 0xFFFF

    return crc


def build_frame(payload: bytes) -> bytes:
    crc = crc16_ccitt(payload)

    return bytes([
        0x02,                  # frame start
        len(payload),          # payload length
        *payload,              # command/data bytes
        (crc >> 8) & 0xFF,     # CRC high byte
        crc & 0xFF,            # CRC low byte
        0x03,                  # frame end
    ])



def main() -> None:
    parser = argparse.ArgumentParser(description="Read and parse custom serial frames")
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0 or COM3")
    parser.add_argument("baudrate", type=int, help="Baudrate, e.g. 115200")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout in seconds")

    args = parser.parse_args()

    with serial.Serial(args.port, args.baudrate, timeout=args.timeout) as ser:
        print(f"Listening on {args.port} at {args.baudrate} baud...")

        while True:
            try:
                frame = read_frame(ser)
                """
                print()
                print("Frame received")
                print(f"  Raw:      {frame.raw.hex(' ')}")
                print(f"  Length:   {len(frame.payload)}")
                print(f"  Payload:  {frame.payload.hex(' ')}")
                """
                data = bytes.fromhex(frame.payload.hex(' '))
                text = data.decode("ascii", errors="replace")
                print(repr(text))
                print(text)

                if frame.command is not None:
                    print(f"  Command:  0x{frame.command:02X}")

                print(f"  Checksum: 0x{frame.checksum:04X}")

            except TimeoutError:
                continue

            except KeyboardInterrupt:
                print("\nExiting.")
                break

            except Exception as e:
                print(f"Parse error: {e}")


if __name__ == "__main__":
    main()