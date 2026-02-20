import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import numpy as np

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Compare GPS vs RTK data for same scenario')
parser.add_argument('--gps', required=True, help='Path to GPS (Lab 1) rosbag folder')
parser.add_argument('--rtk', required=True, help='Path to RTK (Lab 2) rosbag folder')
parser.add_argument('--scenario', required=True, choices=['open', 'occluded', 'walking'], 
                    help='Scenario type: open, occluded, or walking')
parser.add_argument('--output', help='Output directory name (default: scenario_comparison)')
args = parser.parse_args()

if args.output is None:
    args.output = f'{args.scenario}_comparison'

def read_bag(bag_path, topic, msg_type_name):
    """Read a bag file and return data arrays"""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    easting = []
    northing = []
    altitude = []
    timestamps = []
    hdop = []
    fix_quality = []
    
    msg_type = get_message(f'gps_driver/msg/{msg_type_name}')
    
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, msg_type)
            easting.append(msg.utm_easting)
            northing.append(msg.utm_northing)
            altitude.append(msg.altitude)
            timestamps.append(timestamp)
            
            # Try to get hdop and fix_quality if available (RTK data)
            if hasattr(msg, 'hdop'):
                hdop.append(msg.hdop)
            if hasattr(msg, 'fix_quality'):
                fix_quality.append(msg.fix_quality)
    
    return {
        'easting': np.array(easting),
        'northing': np.array(northing),
        'altitude': np.array(altitude),
        'timestamps': np.array(timestamps),
        'hdop': np.array(hdop) if hdop else None,
        'fix_quality': np.array(fix_quality) if fix_quality else None
    }

def process_stationary_data(data):
    """Calculate centroid and relative positions for stationary data"""
    centroid_easting = np.mean(data['easting'])
    centroid_northing = np.mean(data['northing'])
    
    easting_rel = data['easting'] - centroid_easting
    northing_rel = data['northing'] - centroid_northing
    
    # Calculate position error from centroid
    position_error = np.sqrt(easting_rel**2 + northing_rel**2)
    
    # Time in seconds from start
    time_seconds = (data['timestamps'] - data['timestamps'][0]) / 1e9
    
    return {
        'easting_rel': easting_rel,
        'northing_rel': northing_rel,
        'centroid_easting': centroid_easting,
        'centroid_northing': centroid_northing,
        'position_error': position_error,
        'time_seconds': time_seconds
    }

def process_walking_data(data):
    """Calculate relative positions and best fit line for walking data"""
    # Use centroid for consistency
    centroid_easting = np.mean(data['easting'])
    centroid_northing = np.mean(data['northing'])
    
    easting_rel = data['easting'] - centroid_easting
    northing_rel = data['northing'] - centroid_northing
    
    # Time in seconds from start
    time_seconds = (data['timestamps'] - data['timestamps'][0]) / 1e9
    
    # Fit best fit line using polyfit (linear regression)
    coefficients = np.polyfit(easting_rel, northing_rel, 1)
    poly_func = np.poly1d(coefficients)
    
    # Calculate perpendicular distance from best fit line
    # Line equation: y = mx + c, rewrite as: mx - y + c = 0
    m = coefficients[0]
    c = coefficients[1]
    
    # Perpendicular distance formula: |mx - y + c| / sqrt(m^2 + 1)
    distances = np.abs(m * easting_rel - northing_rel + c) / np.sqrt(m**2 + 1)
    
    return {
        'easting_rel': easting_rel,
        'northing_rel': northing_rel,
        'centroid_easting': centroid_easting,
        'centroid_northing': centroid_northing,
        'time_seconds': time_seconds,
        'coefficients': coefficients,
        'poly_func': poly_func,
        'distances': distances
    }

# Read both bags
print(f"Comparing {args.scenario.upper()} scenario: GPS vs RTK")
print("="*70)

print("\nReading GPS (Lab 1) bag...")
gps_data = read_bag(args.gps, '/gps', 'GpsMsg')
print(f"  Read {len(gps_data['easting'])} points")

print("Reading RTK (Lab 2) bag...")
rtk_data = read_bag(args.rtk, '/rtk', 'RtkMsg')
print(f"  Read {len(rtk_data['easting'])} points")

# Process data based on scenario
is_walking = (args.scenario == 'walking')

if is_walking:
    gps_proc = process_walking_data(gps_data)
    rtk_proc = process_walking_data(rtk_data)
else:
    gps_proc = process_stationary_data(gps_data)
    rtk_proc = process_stationary_data(rtk_data)

# Create output directory
output_dir = Path(args.output)
output_dir.mkdir(exist_ok=True)
print(f"\nResults will be saved to: {output_dir}")

# ============================================================================
# PLOT 1: Northing vs Easting - GPS vs RTK Side by Side
# ============================================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

# GPS
ax1.scatter(gps_proc['easting_rel'], gps_proc['northing_rel'], 
            c='blue', s=20, alpha=0.6, label='GPS (Lab 1)')
if is_walking:
    # Plot best fit line
    x_line = np.linspace(gps_proc['easting_rel'].min(), gps_proc['easting_rel'].max(), 100)
    y_line = gps_proc['poly_func'](x_line)
    ax1.plot(x_line, y_line, 'r-', linewidth=2, label='Best Fit Line')
else:
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
    ax1.axvline(x=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)

ax1.set_xlabel('Easting (m) - Relative to Centroid')
ax1.set_ylabel('Northing (m) - Relative to Centroid')
ax1.set_title(f'GPS (Lab 1): {args.scenario.capitalize()} - Northing vs Easting')
ax1.grid(True, alpha=0.3)
ax1.axis('equal')
ax1.legend()

# RTK
ax2.scatter(rtk_proc['easting_rel'], rtk_proc['northing_rel'], 
            c='green', s=20, alpha=0.6, label='RTK (Lab 2)')
if is_walking:
    # Plot best fit line
    x_line = np.linspace(rtk_proc['easting_rel'].min(), rtk_proc['easting_rel'].max(), 100)
    y_line = rtk_proc['poly_func'](x_line)
    ax2.plot(x_line, y_line, 'r-', linewidth=2, label='Best Fit Line')
else:
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)
    ax2.axvline(x=0, color='k', linestyle='--', linewidth=0.5, alpha=0.3)

ax2.set_xlabel('Easting (m) - Relative to Centroid')
ax2.set_ylabel('Northing (m) - Relative to Centroid')
ax2.set_title(f'RTK (Lab 2): {args.scenario.capitalize()} - Northing vs Easting')
ax2.grid(True, alpha=0.3)
ax2.axis('equal')
ax2.legend()

plt.tight_layout()
plt.savefig(output_dir / f'{args.scenario}_northing_vs_easting_comparison.png', dpi=300)
print(f"Saved: {args.scenario}_northing_vs_easting_comparison.png")
plt.close()

# ============================================================================
# PLOT 2: Altitude vs Time - GPS vs RTK Side by Side
# ============================================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

# GPS
ax1.plot(gps_proc['time_seconds'], gps_data['altitude'], 'b-', linewidth=1)
ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Altitude (m)')
ax1.set_title(f'GPS (Lab 1): {args.scenario.capitalize()} - Altitude vs Time')
ax1.grid(True, alpha=0.3)

# RTK
ax2.plot(rtk_proc['time_seconds'], rtk_data['altitude'], 'g-', linewidth=1)
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Altitude (m)')
ax2.set_title(f'RTK (Lab 2): {args.scenario.capitalize()} - Altitude vs Time')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(output_dir / f'{args.scenario}_altitude_vs_time_comparison.png', dpi=300)
print(f"Saved: {args.scenario}_altitude_vs_time_comparison.png")
plt.close()

# ============================================================================
# PLOT 3: Error Analysis - Different for stationary vs walking
# ============================================================================
if not is_walking:
    # Stationary: Error histograms
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # GPS
    ax1.hist(gps_proc['position_error'], bins=20, color='blue', alpha=0.7, edgecolor='black')
    ax1.set_xlabel('Position Error from Centroid (m)')
    ax1.set_ylabel('Frequency')
    ax1.set_title(f'GPS (Lab 1): {args.scenario.capitalize()} - Error Distribution')
    ax1.grid(True, alpha=0.3)
    
    # RTK
    ax2.hist(rtk_proc['position_error'], bins=20, color='green', alpha=0.7, edgecolor='black')
    ax2.set_xlabel('Position Error from Centroid (m)')
    ax2.set_ylabel('Frequency')
    ax2.set_title(f'RTK (Lab 2): {args.scenario.capitalize()} - Error Distribution')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / f'{args.scenario}_error_histogram_comparison.png', dpi=300)
    print(f"Saved: {args.scenario}_error_histogram_comparison.png")
    plt.close()

# ============================================================================
# QUANTITATIVE STATISTICS
# ============================================================================
print("\n" + "="*70)
print(f"QUANTITATIVE RESULTS - {args.scenario.upper()} SCENARIO")
print("="*70)

print("\n--- GPS (LAB 1) STATISTICS ---")
print(f"Number of points: {len(gps_data['easting'])}")
print(f"Centroid offset - Easting: {gps_proc['centroid_easting']:.2f} m")
print(f"Centroid offset - Northing: {gps_proc['centroid_northing']:.2f} m")

if is_walking:
    print(f"\nError from Best Fit Line:")
    print(f"  Mean perpendicular error: {np.mean(gps_proc['distances']):.4f} m")
    print(f"  Std perpendicular error:  {np.std(gps_proc['distances']):.4f} m")
    print(f"  Max perpendicular error:  {np.max(gps_proc['distances']):.4f} m")
    print(f"  Min perpendicular error:  {np.min(gps_proc['distances']):.4f} m")
    print(f"\nBest fit line equation: y = {gps_proc['coefficients'][0]:.4f}x + {gps_proc['coefficients'][1]:.4f}")
else:
    print(f"\nPosition Error from Centroid:")
    print(f"  Mean error: {np.mean(gps_proc['position_error']):.4f} m")
    print(f"  Std error:  {np.std(gps_proc['position_error']):.4f} m")
    print(f"  Max error:  {np.max(gps_proc['position_error']):.4f} m")
    print(f"  Min error:  {np.min(gps_proc['position_error']):.4f} m")

print("\n--- RTK (LAB 2) STATISTICS ---")
print(f"Number of points: {len(rtk_data['easting'])}")
print(f"Centroid offset - Easting: {rtk_proc['centroid_easting']:.2f} m")
print(f"Centroid offset - Northing: {rtk_proc['centroid_northing']:.2f} m")

if is_walking:
    print(f"\nError from Best Fit Line:")
    print(f"  Mean perpendicular error: {np.mean(rtk_proc['distances']):.4f} m")
    print(f"  Std perpendicular error:  {np.std(rtk_proc['distances']):.4f} m")
    print(f"  Max perpendicular error:  {np.max(rtk_proc['distances']):.4f} m")
    print(f"  Min perpendicular error:  {np.min(rtk_proc['distances']):.4f} m")
    print(f"\nBest fit line equation: y = {rtk_proc['coefficients'][0]:.4f}x + {rtk_proc['coefficients'][1]:.4f}")
else:
    print(f"\nPosition Error from Centroid:")
    print(f"  Mean error: {np.mean(rtk_proc['position_error']):.4f} m")
    print(f"  Std error:  {np.std(rtk_proc['position_error']):.4f} m")
    print(f"  Max error:  {np.max(rtk_proc['position_error']):.4f} m")
    print(f"  Min error:  {np.min(rtk_proc['position_error']):.4f} m")

if rtk_data['hdop'] is not None:
    print(f"\nHDOP Statistics:")
    print(f"  Mean HDOP: {np.mean(rtk_data['hdop']):.2f}")
    print(f"  Min HDOP:  {np.min(rtk_data['hdop']):.2f}")
    print(f"  Max HDOP:  {np.max(rtk_data['hdop']):.2f}")

if rtk_data['fix_quality'] is not None:
    print(f"\nFix Quality Statistics:")
    print(f"  Mean Fix Quality: {np.mean(rtk_data['fix_quality']):.2f}")
    print(f"  Fix Quality Range: {int(np.min(rtk_data['fix_quality']))} to {int(np.max(rtk_data['fix_quality']))}")

print("\n--- COMPARISON (RTK vs GPS) ---")
if is_walking:
    error_diff = np.mean(rtk_proc['distances']) - np.mean(gps_proc['distances'])
    improvement_pct = (1 - np.mean(rtk_proc['distances'])/np.mean(gps_proc['distances'])) * 100
    print(f"Error difference (RTK - GPS): {error_diff:.4f} m")
    print(f"RTK improvement over GPS: {improvement_pct:.1f}%")
else:
    error_diff = np.mean(rtk_proc['position_error']) - np.mean(gps_proc['position_error'])
    improvement_pct = (1 - np.mean(rtk_proc['position_error'])/np.mean(gps_proc['position_error'])) * 100
    print(f"Error difference (RTK - GPS): {error_diff:.4f} m")
    print(f"RTK improvement over GPS: {improvement_pct:.1f}%")

print("\n" + "="*70)
print(f"All results saved to: {output_dir}")
print("="*70)