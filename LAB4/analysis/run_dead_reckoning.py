#!/usr/bin/env python3
"""
run_dead_reckoning.py — Chapter 3: dead reckoning trajectory.
Run from the analysis/ directory:
    python run_dead_reckoning.py
"""

import json
import numpy as np
from pathlib import Path
from src.data_loader import load_bag
from src import mag_calibration, yaw_estimation, velocity, dead_reckoning

with open("config.json") as f:
    cfg = json.load(f)

DATA_DIR    = Path(cfg["data_dir"])
RESULTS_DIR = Path(cfg["results_dir"])
RESULTS_DIR.mkdir(exist_ok=True)

# --- Load bags ---
print("Loading circle bag...")
circle = load_bag(str(DATA_DIR / cfg["bags"]["circle"]), topics=["/imu"])

print("Loading adventure bag...")
driving = load_bag(str(DATA_DIR / cfg["bags"]["driving"]), topics=["/gps", "/imu"])

# --- Calibration ---
mx_raw = np.array([r["mag_x"] for r in circle["/imu"]])
my_raw = np.array([r["mag_y"] for r in circle["/imu"]])
offset_x, offset_y, W = mag_calibration.calibrate(mx_raw, my_raw, RESULTS_DIR)

# --- Yaw estimation ---
print("\nRunning yaw estimation...")
yaw_res = yaw_estimation.run(
    driving_imu        = driving["/imu"],
    results_dir        = RESULTS_DIR,
    offset_x           = offset_x,
    offset_y           = offset_y,
    W                  = W,
    fc_hz              = cfg["yaw"]["complementary_filter_fc_hz"],
    gyro_bias_window_s = cfg["yaw"]["gyro_bias_window_s"],
)

# --- Velocity estimation ---
print("\nRunning velocity estimation...")
vel_res = velocity.run(
    driving_imu   = driving["/imu"],
    driving_gps   = driving["/gps"],
    results_dir   = RESULTS_DIR,
    bias_window_s = cfg["yaw"]["gyro_bias_window_s"],
)

# --- Dead reckoning ---
print("\nRunning dead reckoning...")
dr_res = dead_reckoning.run(
    driving_imu = driving["/imu"],
    driving_gps = driving["/gps"],
    v_adj       = vel_res["v_adj"],
    yaw_cf_deg  = yaw_res["yaw_cf"],
    results_dir = RESULTS_DIR,
)

print("\nDone.")
