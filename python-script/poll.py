import time
from main import TMotorManager_servo_serial

PORT = ("/dev/tty.usbmodem34B7DA5F92CC2")     # Linux Uno R4 WiFi USB serial, adjust if needed
BAUD = 921600             # use the baud rate your motor/firmware expects

with TMotorManager_servo_serial(port=PORT, baud=BAUD, max_mosfett_temp=100) as motor:
    while True:
        state = motor.poll_values(timeout=0.5)

        if state is None:
            print("No response")
        else:
            s = motor.status_dict()
            print(
                f"ID={s['control_id_or_motor_id']} "
                f"Vin={s['input_voltage_V']:.2f}V "
                f"MOS={s['mos_temperature_C']:.1f}C "
                f"Motor={s['motor_temperature_C']:.1f}C "
                f"Iq={s['iq_current_A']:.3f}A "
                f"speed={s['speed_ERPM']:.1f}ERPM "
                f"err={s['error']}"
            )

        time.sleep(5)