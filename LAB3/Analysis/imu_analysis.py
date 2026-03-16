#!/usr/bin/env python3
"""
IMU Data Analysis — Stationary and Motion
EECE5554 Lab 3

Usage:
    /usr/bin/python3 imu_analysis.py --bag <path> --mode stationary
    /usr/bin/python3 imu_analysis.py --bag <path> --mode motion

Examples:
    /usr/bin/python3 imu_analysis.py --bag ../Data/stationary_data --mode stationary
    /usr/bin/python3 imu_analysis.py --bag ../Data/motion_data --mode motion
"""

import argparse
import sqlite3
import os
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ─────────────────────────────────────────────
# Bag reading (VNYMR string extraction)
# ─────────────────────────────────────────────

def parse_vnymr(raw_bytes):
    """
    Extract and parse the $VNYMR string embedded in raw CDR message bytes.
    Format: $VNYMR,Yaw,Pitch,Roll,MagX,MagY,MagZ,AccX,AccY,AccZ,GyroX,GyroY,GyroZ*checksum
    Returns dict or None on failure.
    """
    try:
        text = raw_bytes.decode('utf-8', errors='ignore')
        match = re.search(r'\$VNYMR,([^\r\n*]+)\*([0-9A-Fa-f]{2})', text)
        if not match:
            return None
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
    """Read all IMU fields from a db3 rosbag."""
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

    keys = ['yaw','pitch','roll',
            'mag_x','mag_y','mag_z',
            'acc_x','acc_y','acc_z',
            'gyr_x','gyr_y','gyr_z',
            'timestamps']
    data = {k: [] for k in keys}
    count, errors = 0, 0

    for raw, t in cur:
        parsed = parse_vnymr(raw)
        if parsed is None:
            errors += 1
            continue
        for k in keys[:-1]:
            data[k].append(parsed[k])
        data['timestamps'].append(t)
        count += 1
        if count % 10000 == 0:
            print(f"  Parsed {count}/{total}...")

    conn.close()
    print(f"Done. Parsed {count}, skipped {errors} corrupted.")
    return {k: np.array(v) for k, v in data.items()}


def make_time_axis(timestamps):
    """Convert nanosecond timestamps to seconds from start."""
    t = (timestamps - timestamps[0]) * 1e-9
    return t


# ─────────────────────────────────────────────
# Stationary analysis
# ─────────────────────────────────────────────

def plot_time_series(t, data, output_dir):
    """Plot time series for gyro, accel, magnetometer, and orientation."""

    sensors = [
        ('Gyroscope (rad/s)',       ['gyr_x', 'gyr_y', 'gyr_z'],   ['X', 'Y', 'Z']),
        ('Accelerometer (m/s²)',    ['acc_x', 'acc_y', 'acc_z'],   ['X', 'Y', 'Z']),
        ('Magnetometer (Gauss)',    ['mag_x', 'mag_y', 'mag_z'],   ['X', 'Y', 'Z']),
        ('Orientation (degrees)',   ['roll',  'pitch', 'yaw'],      ['Roll', 'Pitch', 'Yaw']),
    ]

    fnames = []
    for title, keys, labels in sensors:
        fig, ax = plt.subplots(figsize=(12, 4))
        for key, label in zip(keys, labels):
            ax.plot(t, data[key], linewidth=0.6, label=label)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(title.split('(')[1].replace(')', ''))
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        fname = os.path.join(output_dir,
                             f"timeseries_{title.split(' ')[0].lower()}.png")
        plt.savefig(fname, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"  Saved: {fname}")
        fnames.append(fname)
    return fnames


def plot_orientation_histograms(data, output_dir):
    """Plot histograms of orientation (roll, pitch, yaw) around their medians."""
    axes_info = [
        ('roll',  'Roll'),
        ('pitch', 'Pitch'),
        ('yaw',   'Yaw'),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle('Orientation Distribution Around Median', fontsize=13)

    for ax, (key, label) in zip(axes, axes_info):
        values = data[key]
        median = np.median(values)
        mean   = np.mean(values)
        centered = values - median

        ax.hist(centered, bins=60, edgecolor='none', alpha=0.75, color='steelblue')
        ax.axvline(0,          color='green', linestyle='--', lw=1.2, label=f'Median: {median:.4f}°')
        ax.axvline(mean - median, color='red', linestyle='--', lw=1.2, label=f'Mean: {mean:.4f}°')
        ax.set_xlabel(f'{label} deviation from median (°)')
        ax.set_ylabel('Count')
        ax.set_title(f'{label}\nMean={mean:.4f}°  Median={median:.4f}°')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fname = os.path.join(output_dir, 'orientation_histograms.png')
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {fname}")
    return fname


def print_stationary_stats(data):
    """Print mean and median for all orientation axes."""
    print("\n--- Orientation Statistics ---")
    print(f"{'Axis':<8} {'Mean (°)':<14} {'Median (°)':<14} {'Std Dev (°)':<14}")
    print("-" * 50)
    for key, label in [('roll','Roll'), ('pitch','Pitch'), ('yaw','Yaw')]:
        mean   = np.mean(data[key])
        median = np.median(data[key])
        std    = np.std(data[key])
        print(f"{label:<8} {mean:<14.4f} {median:<14.4f} {std:<14.4f}")


def run_stationary(data, output_dir):
    """Run full stationary analysis."""
    print("\n=== Stationary Analysis ===")
    t = make_time_axis(data['timestamps'])
    print(f"Duration: {t[-1]:.1f} s  |  Samples: {len(t)}")

    print("\nGenerating time series plots...")
    plot_time_series(t, data, output_dir)

    print("\nGenerating orientation histograms...")
    plot_orientation_histograms(data, output_dir)

    print_stationary_stats(data)


# ─────────────────────────────────────────────
# Motion analysis
# ─────────────────────────────────────────────

def plot_motion_clip(t, data, t_start, t_end, clip_num, output_dir):
    """
    Plot a 5-second clip of motion data.
    Plots gyro X/Y/Z and accel X/Y/Z on two subplots.
    """
    mask = (t >= t_start) & (t <= t_end)
    t_clip = t[mask]

    if len(t_clip) == 0:
        print(f"  WARNING: No data in clip {clip_num} ({t_start}–{t_end}s)")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(f'Motion Clip {clip_num}  ({t_start:.1f}s – {t_end:.1f}s)', fontsize=12)

    # Gyro
    for key, label, color in zip(['gyr_x','gyr_y','gyr_z'],
                                   ['X','Y','Z'],
                                   ['tab:blue','tab:orange','tab:green']):
        ax1.plot(t_clip, data[key][mask], label=label, color=color, linewidth=1.0)
    ax1.set_ylabel('Angular velocity (rad/s)')
    ax1.set_title('Gyroscope')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Accel
    for key, label, color in zip(['acc_x','acc_y','acc_z'],
                                   ['X','Y','Z'],
                                   ['tab:blue','tab:orange','tab:green']):
        ax2.plot(t_clip, data[key][mask], label=label, color=color, linewidth=1.0)
    ax2.set_ylabel('Acceleration (m/s²)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Accelerometer')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    fname = os.path.join(output_dir, f'motion_clip_{clip_num}.png')
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {fname}")


def plot_motion_overview(t, data, output_dir):
    """Plot full motion time series overview for all sensors."""
    sensors = [
        ('Gyroscope (rad/s)',     ['gyr_x','gyr_y','gyr_z'],  ['X','Y','Z']),
        ('Accelerometer (m/s²)', ['acc_x','acc_y','acc_z'],  ['X','Y','Z']),
        ('Orientation (degrees)', ['roll','pitch','yaw'],      ['Roll','Pitch','Yaw']),
    ]

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Motion Data Overview', fontsize=13)

    for ax, (title, keys, labels) in zip(axes, sensors):
        for key, label in zip(keys, labels):
            ax.plot(t, data[key], linewidth=0.5, label=label)
        ax.set_ylabel(title)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')

    plt.tight_layout()
    fname = os.path.join(output_dir, 'motion_overview.png')
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {fname}")


def run_motion(data, output_dir, clips):
    """Run motion analysis with specified clip times."""
    print("\n=== Motion Analysis ===")
    t = make_time_axis(data['timestamps'])
    print(f"Duration: {t[-1]:.1f} s  |  Samples: {len(t)}")

    print("\nGenerating motion overview plot...")
    plot_motion_overview(t, data, output_dir)

    print("\nGenerating motion clips...")
    for i, (t_start, t_end) in enumerate(clips, start=1):
        print(f"  Clip {i}: {t_start:.1f}s – {t_end:.1f}s")
        plot_motion_clip(t, data, t_start, t_end, i, output_dir)


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='IMU Stationary and Motion Analysis')
    parser.add_argument('--bag',  required=True,  help='Path to ROS2 bag folder')
    parser.add_argument('--mode', required=True,  choices=['stationary', 'motion'],
                        help='Analysis mode: stationary or motion')
    parser.add_argument('--clips', nargs='+', type=float, default=None,
                        help='Motion clip start times in seconds (e.g. --clips 10 45 120). '
                             'Each clip covers [t, t+5]s. Required for motion mode.')
    parser.add_argument('--out',  default='.',
                        help='Output directory for plots (default: current dir)')
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    data = read_bag(args.bag)

    if args.mode == 'stationary':
        run_stationary(data, args.out)

    elif args.mode == 'motion':
        if args.clips is None:
            # Default: auto-pick 3 clips evenly spaced through recording
            t = make_time_axis(data['timestamps'])
            duration = t[-1]
            starts = [duration * 0.2, duration * 0.5, duration * 0.8]
            clips = [(s, s + 5.0) for s in starts]
            print(f"No clips specified. Auto-selecting at: "
                  f"{[f'{s:.1f}s' for s,_ in clips]}")
        else:
            clips = [(s, s + 5.0) for s in args.clips]
        run_motion(data, args.out, clips)


if __name__ == '__main__':
    main()