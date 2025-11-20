# import serial
# import time
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from collections import deque

# # --- Configuration ---
# # !!! UPDATE THIS to your Arduino's COM port !!!
# # Windows: 'COM3', 'COM4', etc.
# # macOS: '/dev/cu.usbmodemXXXX'
# # Linux: '/dev/ttyUSB0', '/dev/ttyACM0', etc.
# SERIAL_PORT = '/dev/ttyACM0'  
# BAUD_RATE = 115200     # Must match your Arduino's Serial.begin()
# MAX_DATA_POINTS = 300  # How many points to show on the scrolling plot

# # --- Global Variables ---
# ser = None
# start_time = time.time()

# # Deques are efficient for adding/removing from both ends
# time_data = deque(maxlen=MAX_DATA_POINTS)
# x_data = deque(maxlen=MAX_DATA_POINTS)
# y_data = deque(maxlen=MAX_DATA_POINTS)
# z_data = deque(maxlen=MAX_DATA_POINTS)

# # Buffer for incomplete data blocks
# current_data = {'X': None, 'Y': None, 'Z': None}

# # --- Plot Setup ---
# fig, ax = plt.subplots(figsize=(10, 6))
# line_x, = ax.plot([], [], 'r-', label='X')
# line_y, = ax.plot([], [], 'g-', label='Y')
# line_z, = ax.plot([], [], 'b-', label='Z')
# ax.set_title('Real-time IMU Kinematics (X, Y, Z)')
# ax.set_xlabel('Time (s)')
# ax.set_ylabel('Position (m)')
# ax.legend()
# ax.grid(True)

# def init_plot():
#     """Initializes the plot axes."""
#     # Based on your segmentLength=0.14m, Z max is 0.14, X/Y max is ~0.09
#     ax.set_xlim(0, 30) # Initial 30-second window
#     ax.set_ylim(-0.1, 0.15) # Set reasonable initial Y-limits
#     line_x.set_data([], [])
#     line_y.set_data([], [])
#     line_z.set_data([], [])
#     return line_x, line_y, line_z,

# def parse_serial():
#     """Reads from serial, parses data, and updates data deques."""
#     global current_data
    
#     try:
#         while ser.in_waiting > 0:
#             # Read one line from the serial buffer
#             line = ser.readline().decode('utf-8').strip()

#             # Attempt to parse the line
#             if line.startswith("X: "):
#                 current_data['X'] = float(line.split(": ")[1])
#             elif line.startswith("Y: "):
#                 current_data['Y'] = float(line.split(": ")[1])
#             elif line.startswith("Z: "):
#                 current_data['Z'] = float(line.split(": ")[1])
#             elif line.startswith("-------------"):
#                 # This is the end-of-block delimiter
#                 # Check if we have a complete X,Y,Z block
#                 if all(v is not None for v in current_data.values()):
#                     # Add data to our deques
#                     current_time = time.time() - start_time
#                     time_data.append(current_time)
#                     x_data.append(current_data['X'])
#                     y_data.append(current_data['Y'])
#                     z_data.append(current_data['Z'])
                    
#                     # Reset buffer for the next block
#                     current_data = {'X': None, 'Y': None, 'Z': None}
#                 else:
#                     # Incomplete block, just reset
#                     current_data = {'X': None, 'Y': None, 'Z': None}

#     except (serial.SerialException, ValueError, OSError) as e:
#         print(f"Error reading or parsing serial data: {e}")
#     except Exception as e:
#         print(f"An unexpected error occurred: {e}")


# def update_plot(frame):
#     """Called by FuncAnimation to update the plot."""
#     # First, process any available serial data
#     parse_serial()
    
#     # Update the plot line data
#     line_x.set_data(time_data, x_data)
#     line_y.set_data(time_data, y_data)
#     line_z.set_data(time_data, z_data)
    
#     # Adjust plot limits
#     if time_data:
#         # Set X-axis to scroll
#         ax.set_xlim(time_data[0], time_data[-1] + 1) # +1 for some padding
        
#         # Auto-scale Y-axis (optional, can be slow)
#         # ax.relim()
#         # ax.autoscale_view(scalex=False, scaley=True) 

#     return line_x, line_y, line_z,

# # --- Main Execution ---
# def main():
#     global ser
#     try:
#         print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE}...")
#         ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
#         time.sleep(2) # Wait for Arduino to reset
#         ser.flushInput() # Clear any old data in the buffer
#         print("Connection successful. Starting plot...")

#         # Start the animation
#         # interval=100 means try to update every 100ms
#         # cache_frame_data=False prevents memory leaks on long runs
#         ani = animation.FuncAnimation(
#             fig, 
#             update_plot, 
#             init_func=init_plot, 
#             blit=True, 
#             interval=100, 
#             cache_frame_data=False
#         )
        
#         plt.show() # Display the plot

#     except serial.SerialException as e:
#         print(f"Error: Could not open serial port {SERIAL_PORT}.")
#         print(f"Details: {e}")
#         print("Please check the port name and ensure the Arduino is connected.")
#     except Exception as e:
#         print(f"An error occurred: {e}")
#     finally:
#         if ser and ser.is_open:
#             ser.close()
#             print("Serial port closed.")

# if __name__ == "__main__":
#     main()




# import serial
# import time
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from mpl_toolkits.mplot3d import Axes3D  # <-- Import for 3D plotting
# from collections import deque

# # --- Configuration ---
# # !!! UPDATE THIS to your Arduino's COM port !!!
# SERIAL_PORT = '/dev/ttyACM0'  
# BAUD_RATE = 115200     # Must match your Arduino's Serial.begin()
# MAX_DATA_POINTS = 300  # How many trail points to show

# # --- Global Variables ---
# ser = None

# # Deques store the (x, y, z) coordinate history
# x_data = deque(maxlen=MAX_DATA_POINTS)
# y_data = deque(maxlen=MAX_DATA_POINTS)
# z_data = deque(maxlen=MAX_DATA_POINTS)

# # Buffer for incomplete data blocks
# current_data = {'X': None, 'Y': None, 'Z': None}

# # --- Plot Setup ---
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d') # <-- Create a 3D axes

# # These plot objects will be updated
# # 'line' is the historical trail
# # 'point' is the current position
# line, = ax.plot([], [], [], 'b-', label='Tip Path')
# point, = ax.plot([], [], [], 'ro', markersize=8, label='Current Tip Position')

# # Based on your Arduino code: segmentLength = 0.14m
# # Z will be between 0 and 0.14
# # X and Y will be between roughly -0.09 and +0.09
# # We set limits slightly larger to see everything clearly.
# ax.set_title('Real-time IMU Tip Position (Constant Curvature Model)')
# ax.set_xlabel('X (m)')
# ax.set_ylabel('Y (m)')
# ax.set_zlabel('Z (m)')
# ax.set_xlim([-0.15, 0.15])
# ax.set_ylim([-0.15, 0.15])
# ax.set_zlim([0, 0.15])
# ax.legend()
# ax.grid(True)

# def init_plot():
#     """Initializes the plot objects."""
#     line.set_data_3d([], [], [])
#     point.set_data_3d([], [], [])
#     return line, point,

# def parse_serial():
#     """Reads from serial, parses data, and updates data deques."""
#     global current_data
    
#     try:
#         while ser.in_waiting > 0:
#             # Read one line from the serial buffer
#             line = ser.readline().decode('utf-8').strip()

#             if line.startswith("X: "):
#                 current_data['X'] = float(line.split(": ")[1])
#             elif line.startswith("Y: "):
#                 current_data['Y'] = float(line.split(": ")[1])
#             elif line.startswith("Z: "):
#                 current_data['Z'] = float(line.split(": ")[1])
#             elif line.startswith("-------------"):
#                 # End of data block
#                 if all(v is not None for v in current_data.values()):
#                     # Add data to our deques
#                     x_data.append(current_data['X'])
#                     y_data.append(current_data['Y'])
#                     z_data.append(current_data['Z'])
                    
#                 # Reset buffer for the next block
#                 current_data = {'X': None, 'Y': None, 'Z': None}

#     except (serial.SerialException, ValueError, OSError) as e:
#         print(f"Error reading or parsing serial data: {e}")
#     except Exception as e:
#         print(f"An unexpected error occurred: {e}")


# def update_plot(frame):
#     """Called by FuncAnimation to update the plot."""
#     # First, process any available serial data
#     parse_serial()
    
#     # Update the trail line
#     line.set_data_3d(x_data, y_data, z_data)
    
#     # Update the current position marker
#     if x_data: # Only if we have data
#         point.set_data_3d([x_data[-1]], [y_data[-1]], [z_data[-1]])

#     return line, point,

# # --- Main Execution ---
# def main():
#     global ser
#     try:
#         print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE}...")
#         ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
#         time.sleep(2) # Wait for Arduino to reset
#         ser.flushInput() # Clear any old data
#         print("Connection successful. Starting 3D plot...")

#         # Start the animation
#         # NOTE: blit=False is often more reliable for 3D plots
#         ani = FuncAnimation(
#             fig, 
#             update_plot, 
#             init_func=init_plot, 
#             blit=False,  # <-- Set blit=False
#             interval=50, # Update interval in ms
#             cache_frame_data=False
#         )
        
#         plt.show() # Display the plot

#     except serial.SerialException as e:
#         print(f"Error: Could not open serial port {SERIAL_PORT}.")
#         print(f"Details: {e}")
#         print("Please check the port and ensure the Arduino is connected.")
#     except Exception as e:
#         print(f"An error occurred: {e}")
#     finally:
#         if ser and ser.is_open:
#             ser.close()
#             print("Serial port closed.")

# if __name__ == "__main__":
#     main()
import serial
import time
import csv
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque

# --- Configuration ---
# !!! UPDATE THIS to your Arduino's COM port !!!
SERIAL_PORT = '/dev/ttyACM0'  # Example: 'COM3' on Windows or '/dev/ttyACM0' on Linux
BAUD_RATE = 115200     # Must match your Arduino's Serial.begin()
MAX_DATA_POINTS = 300  # How many trail points to show on the plot

# --- Global Variables ---
ser = None

# Deques store the (x, y, z) coordinate history for the plot
x_data = deque(maxlen=MAX_DATA_POINTS)
y_data = deque(maxlen=MAX_DATA_POINTS)
z_data = deque(maxlen=MAX_DATA_POINTS)

# --- CSV Logging ---
csv_writer = None
csv_file = None
# Define the header for our CSV file
data_header = [
    'Timestamp', 
    'State', 'S1_Angle', 'S2_Angle', 'S3_Angle', 
    'Kappa', 'Phi', 
    'X', 'Y', 'Z'
]

# Buffer for incomplete data blocks
# This dictionary will hold all data points for one block
current_data = {
    'State': None, 
    'S1': None, 'S2': None, 'S3': None, 
    'Kappa': None, 'Phi': None, 
    'X': None, 'Y': None, 'Z': None
}

# --- Plot Setup ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d') # Create a 3D axes

# These plot objects will be updated
line, = ax.plot([], [], [], 'b-', label='Tip Path')
point, = ax.plot([], [], [], 'ro', markersize=8, label='Current Tip Position')

# Set plot limits based on your segment length (0.14m)
ax.set_title('Real-time IMU Tip Position and Data Logger')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_xlim([-0.15, 0.15])
ax.set_ylim([-0.15, 0.15])
ax.set_zlim([0, 0.15])
ax.legend()
ax.grid(True)

def init_plot():
    """Initializes the plot objects."""
    line.set_data_3d([], [], [])
    point.set_data_3d([], [], [])
    return line, point,

def parse_serial():
    """Reads from serial, parses all data, updates plot deques, and writes to CSV."""
    global current_data, csv_writer
    
    try:
        while ser.in_waiting > 0:
            # Read one line from the serial buffer
            line = ser.readline().decode('utf-8').strip()

            # --- Parse all data fields ---
            if line.startswith("Servo State: "):
                current_data['State'] = int(line.split(": ")[1])
            
            elif line.startswith("Servo Angles (1,2,3): "):
                try:
                    # Parse the "S1, S2, S3" string
                    angles = line.split(": ")[1].split(",")
                    if len(angles) == 3:
                        current_data['S1'] = int(angles[0].strip())
                        current_data['S2'] = int(angles[1].strip())
                        current_data['S3'] = int(angles[2].strip())
                except Exception as e:
                    print(f"Error parsing servo angles: {e} | Line: {line}")
            
            elif line.startswith("Kappa: "):
                current_data['Kappa'] = float(line.split(": ")[1])
            elif line.startswith("Phi: "):
                current_data['Phi'] = float(line.split(": ")[1])
            elif line.startswith("X: "):
                current_data['X'] = float(line.split(": ")[1])
            elif line.startswith("Y: "):
                current_data['Y'] = float(line.split(": ")[1])
            elif line.startswith("Z: "):
                current_data['Z'] = float(line.split(": ")[1])
            
            elif line.startswith("-------------"):
                # End of data block. Now we process and save.
                
                # Check if we have received all data points for this block
                if all(v is not None for v in current_data.values()):
                    
                    # 1. Add (X, Y, Z) to our plot deques
                    x_data.append(current_data['X'])
                    y_data.append(current_data['Y'])
                    z_data.append(current_data['Z'])
                    
                    # 2. Get current timestamp
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    
                    # 3. Write all data to the CSV file
                    if csv_writer:
                        csv_writer.writerow([
                            timestamp,
                            current_data['State'],
                            current_data['S1'],
                            current_data['S2'],
                            current_data['S3'],
                            current_data['Kappa'],
                            current_data['Phi'],
                            current_data['X'],
                            current_data['Y'],
                            current_data['Z']
                        ])
                    
                # 4. Reset buffer for the next block
                current_data = {k: None for k in current_data}

    except (serial.SerialException, ValueError, OSError) as e:
        print(f"Error reading or parsing serial data: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


def update_plot(frame):
    """Called by FuncAnimation to update the plot."""
    # First, process any available serial data (which also handles CSV logging)
    parse_serial()
    
    # Update the trail line
    line.set_data_3d(x_data, y_data, z_data)
    
    # Update the current position marker
    if x_data: # Only if we have data
        point.set_data_3d([x_data[-1]], [y_data[-1]], [z_data[-1]])

    return line, point,

# --- Main Execution ---
def main():
    global ser, csv_writer, csv_file
    
    # --- Setup CSV File ---
    try:
        # Create a unique filename with a timestamp
        filename = f"imu_data_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        print(f"Opening CSV file for writing: {filename}")
        
        # Open the file and create a CSV writer object
        # newline='' is important to prevent blank rows
        csv_file = open(filename, 'w', newline='', encoding='utf-8')
        csv_writer = csv.writer(csv_file)
        
        # Write the header row
        csv_writer.writerow(data_header)
        
    except Exception as e:
        print(f"FATAL ERROR: Could not open CSV file: {e}")
        return # Exit if we can't create the log file

    # --- Setup Serial Connection ---
    try:
        print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) # Wait for Arduino to reset
        ser.flushInput() # Clear any old data
        print("Connection successful. Starting 3D plot and data logging...")

        # Start the animation
        ani = FuncAnimation(
            fig, 
            update_plot, 
            init_func=init_plot, 
            blit=False,
            interval=50, # Update plot every 50ms
            cache_frame_data=False
        )
        
        plt.show() # Display the plot (this blocks until the window is closed)

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}.")
        print(f"Details: {e}")
        print("Please check the port and ensure the Arduino is connected.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # This code runs when the script ends (e.g., plot window is closed)
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")
        
        if csv_file:
            csv_file.close()
            print(f"CSV file closed. Data saved to {filename}")

if __name__ == "__main__":
    main()
