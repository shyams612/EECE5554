#!/usr/bin/env python3
"""
run_yaw.py — load data, run calibration, then run yaw estimation.
Run from the analysis/ directory:
    python run_yaw.py
"""

import json
import numpy as np
from pathlib import Path
from src.data_loader import load_bag
from src import mag_calibration, yaw_estimation

with open("config.json") as f:
    cfg = json.load(f)

DATA_DIR    = Path(cfg["data_dir"])
RESULTS_DIR = Path(cfg["results_dir"])
FC_HZ       = cfg["yaw"]["complementary_filter_fc_hz"]
BIAS_WIN    = cfg["yaw"]["gyro_bias_window_s"]
RESULTS_DIR.mkdir(exist_ok=True)

# --- Load bags ---
print("Loading circle bag...")
circle = load_bag(str(DATA_DIR / "circle_bag/circle_bag_0.db3"), topics=["/imu"])

print("Loading adventure bag...")
driving = load_bag(str(DATA_DIR / "adventure_bag/adventure_bag_0.db3"), topics=["/gps", "/imu"])

print(f"  Circle  IMU : {len(circle['/imu'])} messages")
print(f"  Driving IMU : {len(driving['/imu'])} messages")
print(f"  Driving GPS : {len(driving['/gps'])} messages")

# --- Calibration from circle bag ---
print("\nRunning magnetometer calibration...")
imu_c  = circle["/imu"]
mx_raw = np.array([r["mag_x"] for r in imu_c])
my_raw = np.array([r["mag_y"] for r in imu_c])

offset_x, offset_y, W = mag_calibration.calibrate(mx_raw, my_raw, RESULTS_DIR)

# --- Yaw estimation on adventure bag ---
print("\nRunning yaw estimation...")
results = yaw_estimation.run(
    driving_imu        = driving["/imu"],
    results_dir        = RESULTS_DIR,
    offset_x           = offset_x,
    offset_y           = offset_y,
    W                  = W,
    fc_hz              = FC_HZ,
    gyro_bias_window_s = BIAS_WIN,
)

print("\n--- Yaw summary (first and last values) ---")
for key in ["yaw_mag", "yaw_gyro", "yaw_cf", "yaw_imu"]:
    arr = results[key]
    print(f"  {key:15s}: start={arr[0]:8.2f}°  end={arr[-1]:8.2f}°  range={arr.max()-arr.min():.1f}°")

print("\nDone.")
