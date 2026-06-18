"""
This code plots all CubeMars motors data from the following format:

    motor id: 2 pos (rad?): 0.1234 vel(rad/s?): 0.5678 trq(N*m): 1.2345 temp (C): 32 err: 0

Type in CLion terminal:
python software\src\cubemars_CAN_arduino\plotMotorMIT.py

"""

import serial
import time
import matplotlib.pyplot as plt

# Arduino Connection
port = "COM5" # Change port as necessary
baud = 115200
time_window = 10  # Plot x-axis window in seconds

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)

# Clear old data
ser.reset_input_buffer()
ser.reset_output_buffer()

running = {"in_progress": True}

def stop_motor():
    running["in_progress"] = False

    if ser.is_open:
        ser.flush()
        ser.close()
        print("Motor Plotting Stopped")

def close_plot(event):
    stop_motor()

time.sleep(1)

# Prepare 4 plots data
time_data = []
position_data = []
velocity_data = []
torque_data = []
temperature_data = []

start_time = time.perf_counter()

# Create plot
plt.ion()
print("Figure created")

fig, ax = plt.subplots()


# Customize all 4 plots

posline, = ax.plot(
    time_data,
    position_data,
    linewidth=1.5,
    color="#A8E6CF",
    label="Motor Position in Radians"
)

velline, = ax.plot(
    time_data,
    velocity_data,
    linewidth=1.5,
    color="#A7C7E7",
    label="Motor velocity in Radians per Second"
)

torqline, = ax.plot(
    time_data,
    torque_data,
    linewidth=1.5,
    color="purple",
    label="Motor Torque in Newtons Meter"
)

templine, = ax.plot(
    time_data,
    temperature_data,
    linewidth=1.5,
    color="pink",
    label="Motor Temperature in Degrees Celsius"
)

ax.set_title("CubeMars Motor Data Over Time")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Voltage (V)")
ax.grid(True)
ax.legend()

fig.canvas.mpl_connect("close_event", close_plot)

# Show figure window
plt.show(block=False)

# Start Load Cells Test
ser.flush()
print("Load cells test starts now")
