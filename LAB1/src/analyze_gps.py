import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import argparse
import os
from pathlib import Path
import numpy as np

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Analyze GPS data from ROS2 bag')
parser.add_argument('-f', '--file', required=True, help='Path to rosbag folder')
parser.add_argument('-m', '--motion', choices=['yes', 'no'], required=True, 
                    help='Is this stationary (no) or moving (yes) data?')
args = parser.parse_args()

# Create results folder
bag_name = Path(args.file).stem  # Get folder name without path
results_dir = Path(args.file) / 'results'
results_dir.mkdir(exist_ok=True)
print(f"Results will be saved to: {results_dir}")

# Open the bag
storage_options = rosbag2_py.StorageOptions(uri=args.file, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# Read messages
easting = []
northing = []
altitude = []
timestamps = []

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic == '/gps':
        msg_type = get_message('gps_driver/msg/GpsMsg')
        msg = deserialize_message(data, msg_type)
        easting.append(msg.utm_easting)
        northing.append(msg.utm_northing)
        altitude.append(msg.altitude)
        timestamps.append(timestamp)

print(f"Read {len(easting)} GPS points")

# Subtract first values for relative positioning
easting_relative = np.array([e - easting[0] for e in easting])
northing_relative = np.array([n - northing[0] for n in northing])
altitude = np.array(altitude)

# Convert timestamps to seconds from start
time_seconds = np.array([(t - timestamps[0]) / 1e9 for t in timestamps])

# PLOT 1: Northing vs Easting
plt.figure(figsize=(10, 8))
plt.scatter(easting_relative, northing_relative, c='blue', s=20, alpha=0.6)

# If moving data, add best fit line (first to last point)
if args.motion == 'yes':
    # Draw line from first to last point
    plt.plot([easting_relative[0], easting_relative[-1]], 
             [northing_relative[0], northing_relative[-1]], 
             'r-', linewidth=2, label='Best Fit Line (First-Last)')
    plt.legend()

plt.xlabel('Easting (m) - Relative to First Point')
plt.ylabel('Northing (m) - Relative to First Point')
if args.motion == 'no':
    plt.title('GPS Position: Northing vs Easting (Stationary Data)')
    plot_name = 'stationary_northing_vs_easting.png'
else:
    plt.title('GPS Position: Northing vs Easting (Moving Data)')
    plot_name = 'moving_northing_vs_easting.png'
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.tight_layout()
plt.savefig(results_dir / plot_name, dpi=300)
print(f"Saved: {plot_name}")
plt.close()

# PLOT 2: Altitude vs Time
plt.figure(figsize=(10, 6))
plt.plot(time_seconds, altitude, 'b-', linewidth=1)
plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (m)')
if args.motion == 'no':
    plt.title('Altitude vs Time (Stationary Data)')
    plot_name = 'stationary_altitude_vs_time.png'
else:
    plt.title('Altitude vs Time (Moving Data)')
    plot_name = 'moving_altitude_vs_time.png'
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(results_dir / plot_name, dpi=300)
print(f"Saved: {plot_name}")
plt.close()

# PLOT 3: Histogram of position error (ONLY for stationary data)
if args.motion == 'no':
    # Calculate 2D position error from mean position
    mean_easting = np.mean(easting_relative)
    mean_northing = np.mean(northing_relative)
    position_error = np.sqrt((easting_relative - mean_easting)**2 + 
                             (northing_relative - mean_northing)**2)
    
    plt.figure(figsize=(10, 6))
    plt.hist(position_error, bins=20, color='blue', alpha=0.7, edgecolor='black')
    plt.xlabel('Position Error (m)')
    plt.ylabel('Frequency')
    plt.title('Histogram of Position Error from Mean Position')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(results_dir / 'stationary_error_histogram.png', dpi=300)
    print(f"Saved: stationary_error_histogram.png")
    plt.close()
    
    print(f"\n=== Stationary Error Statistics ===")
    print(f"Mean error: {np.mean(position_error):.4f} m")
    print(f"Std error: {np.std(position_error):.4f} m")
    print(f"Max error: {np.max(position_error):.4f} m")

# Calculate perpendicular distance from best fit line (ONLY for moving data)
if args.motion == 'yes':
    # Line from first to last point
    x1, y1 = easting_relative[0], northing_relative[0]
    x2, y2 = easting_relative[-1], northing_relative[-1]
    
    # Calculate perpendicular distance from each point to the line
    # Using formula: |ax + by + c| / sqrt(a^2 + b^2)
    # Line equation: (y2-y1)x - (x2-x1)y + (x2-x1)y1 - (y2-y1)x1 = 0
    a = y2 - y1
    b = -(x2 - x1)
    c = (x2 - x1) * y1 - (y2 - y1) * x1
    
    # Perpendicular distance for each point
    distances = np.abs(a * easting_relative + b * northing_relative + c) / np.sqrt(a**2 + b**2)
    
    print(f"\n=== Moving Data - Error from Best Fit Line ===")
    print(f"Mean perpendicular error: {np.mean(distances):.4f} m")
    print(f"Std perpendicular error: {np.std(distances):.4f} m")
    print(f"Max perpendicular error: {np.max(distances):.4f} m")

print(f"\nAll plots saved to: {results_dir}")