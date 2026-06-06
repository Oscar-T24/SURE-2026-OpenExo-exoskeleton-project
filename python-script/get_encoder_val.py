import serial
import time

PORT = "/dev/tty.usbserial-AV0KL7N0"
BAUD = 921600

def send_line(ser, text):
    print(f"TX: {text}")
    ser.write((text + "\r\n").encode("ascii"))
    ser.flush()
    time.sleep(0.5)

    data = ser.read(4096)
    if data:
        print("RX hex:  ", data.hex(" "))
        print("RX ascii:", data.decode("ascii", errors="replace"))

with serial.Serial(PORT, BAUD, timeout=0.2, write_timeout=1) as ser:
    time.sleep(0.5)
    ser.reset_input_buffer()

    send_line(ser, "exit")
    send_line(ser, "run")