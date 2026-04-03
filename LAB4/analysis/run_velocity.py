#!/usr/bin/env python3
"""
run_velocity.py — Chapter 2: forward velocity from accelerometer and GPS.
Run from the analysis/ directory:
    python run_velocity.py
"""

import json
import numpy as np
from pathlib import Path
from src.data_loader import load_bag
from src import velocity

with open("config.json") as f:
    cfg = json.load(f)

DATA_DIR    = Path(cfg["data_dir"])
RESULTS_DIR = Path(cfg["results_dir"])
RESULTS_DIR.mkdir(exist_ok=True)

print("Loading adventure bag...")
driving = load_bag(
    str(DATA_DIR / cfg["bags"]["driving"]),
    topics=["/gps", "/imu"],
)
print(f"  IMU: {len(driving['/imu'])}  GPS: {len(driving['/gps'])}")

print("\nRunning velocity estimation...")
results = velocity.run(
    driving_imu  = driving["/imu"],
    driving_gps  = driving["/gps"],
    results_dir  = RESULTS_DIR,
    bias_window_s= cfg["yaw"]["gyro_bias_window_s"],
)

print("\nDone.")
