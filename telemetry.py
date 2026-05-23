import serial
import csv
from datetime import datetime
import matplotlib.pyplot as plt

# --- Configuration ---
PORT = 'COM4'  # Windows: 'COM3', 'COM4' etc. Mac/Linux: '/dev/ttyUSB0' or similar
BAUD_RATE = 115200
FILENAME = "imu_pid_log.csv"

# Lists to store data for live plotting when the script finishes
time_history = []
angle_history = []
setpoint_history = []

start_time = None
print(f"Attempting to connect to Arduino on {PORT}...")

try:
    # Initialize serial connection
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    ser.flushInput()  # Clear old data sitting in the buffer

    # Create/Open CSV file and write the header column labels
    with open(FILENAME, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["PC_Timestamp", "Arduino_Millis", "Setpoint", "Roll_Angle", "PID_Output", "Motor_PWM"])

        print(f"Connected! Recording data to '{FILENAME}'.")
        print("Press Ctrl+C in this terminal window to stop recording and generate the plot.\n")

        while True:
            if ser.in_waiting > 0:
                # Read line, decode byte string to text, and strip whitespace
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if not line:
                    continue
 
                # Split line by commas
                data_split = line.split(',')

                # Check if the line has exactly 5 metrics (our telemetry format)
                if len(data_split) == 5:
                    try:
                        # Attempt to parse all data points as floats
                        numeric_data = [float(val) for val in data_split]

                        # Unpack variables for easier usage
                        arduino_millis = numeric_data[0]
                        setpoint = numeric_data[1]
                        roll_angle = numeric_data[2]
                        pid_output = numeric_data[3]
                        motor_pwm = numeric_data[4]

                        # Generate relative time in seconds for a clean X-axis plot
                        if start_time == None:
                            start_time = arduino_millis
                        relative_time_seconds = (arduino_millis - start_time) / 1000.0

                        # Store in memory for plotting later
                        time_history.append(relative_time_seconds)
                        angle_history.append(roll_angle)
                        setpoint_history.append(setpoint)

                        # Generate precise PC side timestamp for CSV
                        pc_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                        # Save row to CSV
                        row = [pc_time, arduino_millis, setpoint, roll_angle, pid_output, motor_pwm]
                        writer.writerow(row)

                        # Print live output cleanly to the console
                        print(
                            f"[LOGGED] Time: {relative_time_seconds:.2f}s | Roll: {roll_angle:.2f}° | Setpoint: {setpoint:.2f}°")

                    except ValueError:
                        # If conversion fails, it's an initialization string or limit flag
                        print(f"[Arduino Info]: {line}")
                else:
                    # Catch text updates from the setup sequence
                    print(f"[Arduino Info]: {line}")

except serial.SerialException as e:
    print(f"\nSerial Port Error: Could not open {PORT}. Is your Serial Monitor/Plotter closed?")
except KeyboardInterrupt:
    print("\nRecording stopped by user. CSV data saved successfully.")

    # --- Plotting Generation ---
    if len(time_history) > 0:
        print("Generating data plot...")

        # Configure layout and style
        plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
        fig, ax = plt.subplots(figsize=(10, 5))

        # Plot Roll Angle vs Time
        ax.plot(time_history, angle_history, label='Actual Roll Angle', color='#1f77b4', linewidth=2)

        # Plot Setpoint vs Time (useful reference for tracking performance)
        ax.plot(time_history, setpoint_history, label='Target Setpoint', color='#d62728', linestyle='--', linewidth=1.5)

        # Labels and formatting
        ax.set_title('IMU Roll Angle & Setpoint over Time', fontsize=14, fontweight='bold', pad=15)
        ax.set_xlabel('Elapsed Time (seconds)', fontsize=12)
        ax.set_ylabel('Angle (degrees)', fontsize=12)
        ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='none')

        # Adjust margins to prevent label truncation
        plt.tight_layout()

        print("Plot window opened. Close the graph window to exit the Python script completely.")
        plt.show()
    else:
        print("No valid numeric data was collected, skipping plot generation.")
