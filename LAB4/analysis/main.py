#!/usr/bin/env python3
"""
main.py — entry point for LAB4 analysis pipeline.

Run from the analysis/ directory:
    python main.py

Each chapter maps to a module under src/:
    Chapter 1 (Yaw)      → src/yaw_estimation.py
    Chapter 2 (Velocity) → src/velocity.py
    Chapter 3 (Dead Reckoning) → src/dead_reckoning.py
"""

import json
import numpy as np
from pathlib import Path

from src.data_loader import load_bag
from src import mag_calibration, yaw_estimation


def load_config(config_path: str = "config.json") -> dict:
    with open(config_path) as f:
        return json.load(f)


def main():
    cfg = load_config()

    data_dir = Path(cfg["data_dir"])
    results_dir = Path(cfg["results_dir"])
    results_dir.mkdir(exist_ok=True)

    gps_topic = cfg["topics"]["gps"]
    imu_topic = cfg["topics"]["imu"]

    # --- Load bags ---
    print("Loading circle bag...")
    circle_data = load_bag(str(data_dir / cfg["bags"]["circle"]), topics=[gps_topic, imu_topic])
    print("Loading driving bag...")
    driving_data = load_bag(str(data_dir / cfg["bags"]["driving"]), topics=[gps_topic, imu_topic])

    print(f"Circle  — GPS: {len(circle_data[gps_topic])}  IMU: {len(circle_data[imu_topic])}")
    print(f"Driving — GPS: {len(driving_data[gps_topic])}  IMU: {len(driving_data[imu_topic])}")

    # -----------------------------------------------------------------------
    # Chapter 1a: Magnetometer calibration (circle bag)
    # -----------------------------------------------------------------------
    circle_imu = circle_data[imu_topic]
    mx_raw = np.array([r["mag_x"] for r in circle_imu])
    my_raw = np.array([r["mag_y"] for r in circle_imu])
    t_circ = np.array([r["sec"] + r["nanosec"] * 1e-9 for r in circle_imu])

    offset_x, offset_y, W = mag_calibration.calibrate(mx_raw, my_raw, results_dir)
    mx_cal, my_cal = mag_calibration.apply_calibration(mx_raw, my_raw, offset_x, offset_y, W)

    mag_calibration.validation_report(mx_raw, my_raw, mx_cal, my_cal, offset_x, offset_y)

    mag_calibration.plot_xy(
        mx_raw, my_raw, mx_cal, my_cal,
        save_path=results_dir / "plot_mag_xy_calibration.png",
    )
    mag_calibration.plot_timeseries(
        t_circ, mx_raw, my_raw, mx_cal, my_cal,
        save_path=results_dir / "plot_mag_timeseries.png",
    )

    # -----------------------------------------------------------------------
    # Chapter 1b: Yaw estimation (driving bag)
    # -----------------------------------------------------------------------
    yaw_cfg = cfg["yaw"]
    yaw_results = yaw_estimation.run(
        driving_imu=driving_data[imu_topic],
        results_dir=results_dir,
        offset_x=offset_x,
        offset_y=offset_y,
        W=W,
        tau_s=yaw_cfg["complementary_filter_tau_s"],
        gyro_bias_window_s=yaw_cfg["gyro_bias_window_s"],
    )

    # TODO: Chapter 2  — velocity.run(driving_data, results_dir)
    # TODO: Chapter 3  — dead_reckoning.run(driving_data, results_dir)


if __name__ == "__main__":
    main()
