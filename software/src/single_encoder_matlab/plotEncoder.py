"""
This code is used to plot the relative angle of one encoder in degrees
relative to the original position the ankle is at when the test just starts

python src/single_encoder_matlab/plotEncoder.py

Arduino Pin Connection:
SPI Clock (SCK): Pin 13
SPI MOSI:        Pin 11
SPI MISO:        Pin 12
SPI Chip Select: Pin 10

"""

import serial
import time
import matplotlib.pyplot as plt

# Arduino Connection
port = "COM7"
baud = 115200
time_window = 15 # For plot x-axis; plot will move horizontally after 15 seconds

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)
# Clear old data
ser.reset_input_buffer()
ser.reset_output_buffer()

running = {"in_progress": True} # Define dictionary to determine when the encoder is running

# Define function to stop encoder test
def stop_encoder():

    running["in_progress"] = False

    if ser.is_open:
        ser.write(b"n")
        ser.flush()
        ser.close()
        print("Encoder Test Stopped")


def close_plot(event):
    stop_encoder()

# Set current position at zero
ser.write(b"z")
print("Current Position Set to Zero Successfully")
time.sleep(1)

# Prepare plot

time_data = []
angle_data = []

start_time = time.perf_counter()

# Create plot and define chart elements
plt.ion()
print("Figure created")

fig, ax = plt.subplots()
line, = ax.plot(time_data, angle_data, linewidth=1.5)

ax.set_title("AMT20 Encoder Relative Angle vs Time")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Encoder Position (Deg)")
ax.grid(True)

fig.canvas.mpl_connect("close_event", close_plot)

# Start encoder test
ser.write(b"y")
ser.flush()
print("Encoder Test Starting Now")

try:
    while (running["in_progress"] == True) and (plt.fignum_exists(fig.number) == True):
        # y-axis data
        value = ser.readline().decode().strip()
        print("Angle: " + value)

        try:
            angle = float(value)
            angle_data.append(angle)

        except ValueError:
            continue

        # x-axis data
        current_time = time.perf_counter() - start_time
        time_data.append(current_time)

        # keep only last 30 seconds
        while time_data and (current_time - time_data[0]) > time_window:
            time_data.pop(0)
            angle_data.pop(0)

        # update plot
        line.set_data(time_data, angle_data)

        ax.set_xlim(max(0, current_time - time_window), current_time)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    print("Ctrl+C pressed, closing figure")
    ser.close()

finally:
    stop_encoder()
    plt.close("all")
