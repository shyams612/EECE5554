#!/usr/bin/env python3
"""
run_calibration.py — load data and run magnetometer calibration only.
Run from the analysis/ directory:
    python run_calibration.py
"""

import numpy as np
from pathlib import Path
from src.data_loader import load_bag, to_numpy
from src import mag_calibration

DATA_DIR    = Path("../data")
RESULTS_DIR = Path("results")
RESULTS_DIR.mkdir(exist_ok=True)

# --- Load circle bag ---
print("Loading circle bag...")
circle = load_bag(str(DATA_DIR / "circle_bag/circle_bag_0.db3"), topics=["/gps", "/imu"])

imu = circle["/imu"]
gps = circle["/gps"]
print(f"  IMU messages : {len(imu)}")
print(f"  GPS messages : {len(gps)}")

# --- Sanity check: print first GPS and IMU records ---
print("\n--- First GPS record ---")
for k, v in gps[0].items():
    print(f"  {k}: {v}")

print("\n--- First IMU record ---")
for k, v in imu[0].items():
    print(f"  {k}: {v}")

# --- Extract mag arrays ---
imu_np = to_numpy(imu, ["sec", "nanosec", "mag_x", "mag_y", "mag_z",
                         "gyro_x", "gyro_y", "gyro_z",
                         "accel_x", "accel_y", "accel_z",
                         "roll", "pitch", "yaw"])

mx_raw = imu_np["mag_x"]
my_raw = imu_np["mag_y"]
t      = imu_np["sec"] + imu_np["nanosec"] * 1e-9

print(f"\n--- Mag X range: [{mx_raw.min():.4f}, {mx_raw.max():.4f}] Gauss")
print(f"--- Mag Y range: [{my_raw.min():.4f}, {my_raw.max():.4f}] Gauss")

# --- Run calibration ---
print("\nRunning calibration...")
offset_x, offset_y, W = mag_calibration.calibrate(mx_raw, my_raw, RESULTS_DIR)
mx_cal, my_cal = mag_calibration.apply_calibration(mx_raw, my_raw, offset_x, offset_y, W)

# --- Validation report ---
mag_calibration.validation_report(mx_raw, my_raw, mx_cal, my_cal, offset_x, offset_y)

# --- Plots ---
mag_calibration.plot_xy(
    mx_raw, my_raw, mx_cal, my_cal,
    save_path=RESULTS_DIR / "plot_mag_xy_calibration.png",
)
mag_calibration.plot_timeseries(
    t, mx_raw, my_raw, mx_cal, my_cal,
    save_path=RESULTS_DIR / "plot_mag_timeseries.png",
)

print("\nDone.")
