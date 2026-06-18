"""
This code plots the voltage output of 2 load cells connected to the analog amplifier board connected to an Arduino board.
The analog count value is converted to voltage on a 5V basis.

The left and right load cells need to be carefully defined to avoid confusion.

Type in CLion terminal:
python software\src\load_cells_readout\plotTwoLoadcellsPolished.py
"""

import serial
import time
import matplotlib.pyplot as plt

# Arduino Connection
port = "COM5"
baud = 115200
time_window = 10  # Plot x-axis window in seconds

ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)

# Clear old data
ser.reset_input_buffer()
ser.reset_output_buffer()

running = {"in_progress": True}


def stop_loadcells():
    """Stop load cell test and close serial port."""
    running["in_progress"] = False

    if ser.is_open:
        ser.flush()
        ser.close()
        print("Load Cells Test Stopped")


def close_plot(event):
    stop_loadcells()


time.sleep(1)

# Prepare plot data
time_data = []
left_data = []
right_data = []

start_time = time.perf_counter()

# Create plot
plt.ion()
print("Figure created")

fig, ax = plt.subplots()

line1, = ax.plot(
    time_data,
    left_data,
    linewidth=1.5,
    color="blue",
    label="Left Load Cell"
)

line2, = ax.plot(
    time_data,
    right_data,
    linewidth=1.5,
    color="orange",
    label="Right Load Cell"
)

ax.set_title("Load Cells Voltage vs Time")
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

try:
    while running["in_progress"] and plt.fignum_exists(fig.number):

        # Read serial data
        raw = ser.readline().decode(errors="ignore").strip()

        if not raw:
            plt.pause(0.001)
            continue

        arr = raw.split()

        try:
            # Find positions of "left" and "right"
            left_index = next(
                i for i, x in enumerate(arr)
                if x.lower().rstrip(":") == "left"
            )

            right_index = next(
                i for i, x in enumerate(arr)
                if x.lower().rstrip(":") == "right"
            )

            # Find first numeric value after LEFT
            left_value = None
            for token in arr[left_index + 1:right_index]:
                try:
                    left_value = float(token)
                    break
                except ValueError:
                    pass

            # Find first numeric value after RIGHT
            right_value = None
            for token in arr[right_index + 1:]:
                try:
                    right_value = float(token)
                    break
                except ValueError:
                    pass

            if left_value is None or right_value is None:
                continue

        except (StopIteration, IndexError):
            continue

        # Store data
        left_data.append(left_value)
        right_data.append(right_value)

        current_time = time.perf_counter() - start_time
        time_data.append(current_time)

        # Keep only data within time window
        while time_data and (current_time - time_data[0]) > time_window:
            time_data.pop(0)
            left_data.pop(0)
            right_data.pop(0)

        # Safety check to avoid shape mismatch
        min_len = min(len(time_data), len(left_data), len(right_data))

        line1.set_data(time_data[-min_len:], left_data[-min_len:])
        line2.set_data(time_data[-min_len:], right_data[-min_len:])

        ax.set_xlim(max(0, current_time - time_window), current_time)

        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

        fig.canvas.draw_idle()
        plt.pause(0.001)

except KeyboardInterrupt:
    print("Ctrl+C pressed, closing figure")

finally:
    stop_loadcells()
    plt.close("all")