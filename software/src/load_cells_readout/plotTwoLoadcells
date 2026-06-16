"""
This code plots the voltage output of 2 load cells connected to the analog amplifier board connected to an Arduino board
The analog count value is converted to voltage on a 5V basis

The left and right load cells need to be carefully defined to avoid confusion

Type in CLion terminal: python src/load_cells_readout/plotLoadcell.py
"""

import serial
import time
import matplotlib.pyplot as plt

# Arduino Connection
port = "COM3"
baud = 115200
time_window = 15 # For plot x-axis; plot will move horizontally after 15 seconds

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)
# Clear old data
ser.reset_input_buffer()
ser.reset_output_buffer()

running = {"in_progress": True} # Define dictionary to determine when the load cells are running

# Define function to stop load cells test
def stop_loadcells():

    running["in_progress"] = False

    if ser.is_open:
        ser.flush()
        ser.close()
        print("Load Cells Test Stopped")


def close_plot(event):
    stop_loadcells()

time.sleep(1)

# Prepare plot

time_data = []
left_data = []
right_data = []

start_time = time.perf_counter()

# Create plot and define chart elements
plt.ion()
print("Figure created")

fig, ax = plt.subplots()
line1, = ax.plot(time_data, left_data, linewidth=1.5, color="blue", label="Left Load Cell")
line2, = ax.plot(time_data, right_data, linewidth=1.5, color="orange", label="Right Load Cell")

ax.set_title("Load Cells Voltage vs Time")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Encoder Position (Deg)")
ax.grid(True)
ax.legend()

fig.canvas.mpl_connect("close_event", close_plot)

# Start Load Cells Test
ser.flush()
print("Load cells test starts now")

try:
    while (running["in_progress"] == True) and (plt.fignum_exists(fig.number) == True):
        # y-axis data
        raw = ser.readline().decode().strip()
        arr = raw.split(" ")

        try:
            left_data = float(arr[3])
            right_data.append(arr[-1])

        except ValueError:
            continue

        # x-axis data
        current_time = time.perf_counter() - start_time
        time_data.append(current_time)

        # keep only a certain time window
        while time_data and (current_time - time_data[0]) > time_window:
            time_data.pop(0)
            left_data.pop(0)
            right_data.pop(0)

        # update plot
        line1.set_data(time_data, left_data)
        line2.set_data(time_data, right_data)

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
