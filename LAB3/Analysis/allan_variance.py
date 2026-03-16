#!/usr/bin/env python3
"""
Allan Variance Analysis for VectorNav VN-100 IMU
EECE5554 Lab 3

Usage:
    /usr/bin/python3 allan_variance.py --bag <path_to_bag_folder>

Example:
    /usr/bin/python3 allan_variance.py --bag ../imu_bags/5_hour_imu_data
"""

import argparse
import sqlite3
import os
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import allantools


def parse_vnymr(raw_bytes):
    """
    Extract and parse the $VNYMR string embedded in the raw CDR message.
    Format: $VNYMR,Yaw,Pitch,Roll,MagX,MagY,MagZ,AccX,AccY,AccZ,GyroX,GyroY,GyroZ*checksum
    Returns dict of values or None if parsing/checksum fails.
    """
    try:
        # Decode bytes and find the VNYMR sentence
        text = raw_bytes.decode('utf-8', errors='ignore')
        match = re.search(r'\$VNYMR,([^\r\n*]+)\*([0-9A-Fa-f]{2})', text)
        if not match:
            return None

        # Validate checksum
        body = match.group(1)
        given = int(match.group(2), 16)
        computed = 0
        for ch in ('VNYMR,' + body):
            computed ^= ord(ch)
        if computed != given:
            return None

        fields = body.split(',')
        if len(fields) != 12:
            return None

        return {
            'yaw':   float(fields[0]),
            'pitch': float(fields[1]),
            'roll':  float(fields[2]),
            'mag_x': float(fields[3]),
            'mag_y': float(fields[4]),
            'mag_z': float(fields[5]),
            'acc_x': float(fields[6]),
            'acc_y': float(fields[7]),
            'acc_z': float(fields[8]),
            'gyr_x': float(fields[9]),
            'gyr_y': float(fields[10]),
            'gyr_z': float(fields[11]),
        }
    except Exception:
        return None


def read_bag(bag_path):
    """Read IMU data by extracting VNYMR strings from raw CDR messages."""
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        raise FileNotFoundError(f"No .db3 file found in {bag_path}")

    db_path = os.path.join(bag_path, db_files[0])
    print(f"Opening: {db_path}")

    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT COUNT(*) FROM messages")
    total = cur.fetchone()[0]
    print(f"Total messages: {total}")

    cur.execute("SELECT data, timestamp FROM messages")

    data = {k: [] for k in ['gyro_x','gyro_y','gyro_z',
                             'accel_x','accel_y','accel_z',
                             'timestamps']}
    count, errors = 0, 0

    for raw, t in cur:
        parsed = parse_vnymr(raw)
        if parsed is None:
            errors += 1
            continue
        data['gyro_x'].append(parsed['gyr_x'])
        data['gyro_y'].append(parsed['gyr_y'])
        data['gyro_z'].append(parsed['gyr_z'])
        data['accel_x'].append(parsed['acc_x'])
        data['accel_y'].append(parsed['acc_y'])
        data['accel_z'].append(parsed['acc_z'])
        data['timestamps'].append(t)
        count += 1
        if count % 50000 == 0:
            print(f"  Parsed {count}/{total} messages...")

    conn.close()
    print(f"Done. Parsed {count}, skipped {errors} corrupted.")
    return {k: np.array(v) for k, v in data.items()}


def compute_sample_rate(timestamps):
    """Estimate sample rate from timestamps (nanoseconds)."""
    diffs = np.diff(timestamps) * 1e-9
    rate = 1.0 / np.median(diffs)
    print(f"Estimated sample rate: {rate:.2f} Hz")
    return rate


def extract_parameters(taus, adev):
    """Extract N, B, K from Allan deviation curve."""
    taus, adev = np.array(taus), np.array(adev)

    # N: Angle Random Walk at tau = 1s
    idx_1s = np.argmin(np.abs(taus - 1.0))
    N = adev[idx_1s]

    # B: Bias instability at local minimum
    idx_min = np.argmin(adev)
    B = adev[idx_min]
    tau_min = taus[idx_min]

    # K: Rate Random Walk at 3 * tau_min
    idx_k = np.argmin(np.abs(taus - 3.0 * tau_min))
    K = adev[idx_k] / np.sqrt(taus[idx_k])

    return N, B, K, tau_min


def plot_allan_variance(taus, adev, axis_label, sensor, N, B, K, tau_min, ax):
    """Plot Allan deviation on log-log scale with parameter annotations."""
    ax.loglog(taus, adev, linewidth=1.5, label=f'{axis_label} axis')

    # Reference slope lines
    t_ref = np.array([taus[0], taus[-1]])
    ax.loglog(t_ref, N / np.sqrt(t_ref), 'b--', lw=0.8, alpha=0.5, label='N slope (-0.5)')
    ax.loglog(t_ref, K * np.sqrt(t_ref), 'r--', lw=0.8, alpha=0.5, label='K slope (+0.5)')

    # Mark N at tau=1
    ax.axvline(x=1.0, color='gray', linestyle='--', lw=0.8, alpha=0.5)
    ax.plot(1.0, N, 'o', color='blue', markersize=6)
    ax.annotate(f'N={N:.2e}', xy=(1.0, N), xytext=(2.0, N * 2.0),
                fontsize=8, color='blue',
                arrowprops=dict(arrowstyle='->', color='blue', lw=0.8))

    # Mark B at minimum
    ax.plot(tau_min, B, 's', color='green', markersize=6)
    ax.annotate(f'B={B:.2e}', xy=(tau_min, B), xytext=(tau_min * 2, B * 0.5),
                fontsize=8, color='green',
                arrowprops=dict(arrowstyle='->', color='green', lw=0.8))

    ax.set_xlabel('Averaging time τ (s)')
    ax.set_ylabel('Allan deviation')
    ax.set_title(f'{sensor} — {axis_label}\nN={N:.2e}, B={B:.2e}, K={K:.2e}')
    ax.legend(fontsize=8)
    ax.grid(True, which='both', alpha=0.3)


def run_allan_variance(data, rate, keys, sensor_name, units):
    """Run Allan variance analysis for X, Y, Z axes."""
    print(f"\n--- {sensor_name} Allan Variance ---")
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(f'{sensor_name} Allan Variance', fontsize=13)

    results = {}
    for i, (key, ax) in enumerate(zip(keys, axes)):
        axis_label = ['X', 'Y', 'Z'][i]
        print(f"  Computing {axis_label} axis...")
        (taus_out, adev, _, _) = allantools.oadev(
            data[key], rate=rate, data_type='freq', taus='all'
        )
        N, B, K, tau_min = extract_parameters(taus_out, adev)
        print(f"    N (ARW)            = {N:.4e} {units}/sqrt(s)")
        print(f"    B (Bias stability) = {B:.4e} {units}")
        print(f"    K (RRW)            = {K:.4e} {units}*sqrt(s)")
        plot_allan_variance(taus_out, adev, axis_label,
                           sensor_name, N, B, K, tau_min, ax)
        results[axis_label] = {'N': N, 'B': B, 'K': K}

    plt.tight_layout()
    fname = f"{sensor_name.lower().replace(' ', '_')}_allan_variance.png"
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    print(f"  Saved: {fname}")
    return results


def print_summary(gyro_results, accel_results):
    """Print summary table of all extracted parameters."""
    print("\n" + "="*62)
    print("PARAMETER SUMMARY")
    print("="*62)
    print(f"{'Sensor':<12} {'Axis':<6} {'N (ARW)':<16} {'B (Bias)':<16} {'K (RRW)':<16}")
    print("-"*62)
    for ax, v in gyro_results.items():
        print(f"{'Gyro':<12} {ax:<6} {v['N']:<16.4e} {v['B']:<16.4e} {v['K']:<16.4e}")
    for ax, v in accel_results.items():
        print(f"{'Accel':<12} {ax:<6} {v['N']:<16.4e} {v['B']:<16.4e} {v['K']:<16.4e}")
    print("="*62)


def main():
    parser = argparse.ArgumentParser(description='Allan Variance Analysis')
    parser.add_argument('--bag', required=True, help='Path to ROS2 bag folder')
    args = parser.parse_args()

    data = read_bag(args.bag)
    rate = compute_sample_rate(data['timestamps'])

    gyro_results = run_allan_variance(
        data, rate,
        keys=['gyro_x', 'gyro_y', 'gyro_z'],
        sensor_name='Gyroscope', units='rad/s'
    )
    accel_results = run_allan_variance(
        data, rate,
        keys=['accel_x', 'accel_y', 'accel_z'],
        sensor_name='Accelerometer', units='m/s^2'
    )
    print_summary(gyro_results, accel_results)


if __name__ == '__main__':
    main()