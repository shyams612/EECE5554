# velocity.py
# Chapter 2 — Forward velocity estimation from accelerometer and GPS.
#
# Two methods:
#   1. Integrate accel_x (forward axis) after bias removal
#   2. Differentiate GPS UTM position (speed magnitude)
#
# Cross-check:
#   Compute omega * v_forward and compare to accel_y_obs  (Q7)
#
# Assignment plots produced:
#   plot_velocity_accel.png     — accel velocity before/after bias adjustment
#   plot_velocity_comparison.png — adjusted accel velocity vs GPS velocity

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


# ---------------------------------------------------------------------------
# Method 1 — accelerometer velocity
# ---------------------------------------------------------------------------

def accel_velocity(
    accel_x: np.ndarray,
    pitch_deg: np.ndarray,
    t: np.ndarray,
    bias_window_s: float = 10.0,
    g: float = 9.81,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Integrate accel_x to get forward velocity.

    The IMU is tilted in pitch (dashboard mount), so gravity leaks into accel_x:
        g * sin(pitch) ≈ 0.549 m/s² at 3.2° tilt
    This is removed using the VectorNav pitch angle before integration.
    A residual constant bias (noise floor) is then removed using the
    stationary period at the start of the bag.

    Returns
    -------
    v_raw       : velocity from raw accel_x (no correction)
    v_adj       : velocity after gravity + bias removal
    accel_adj   : corrected accel_x signal
    bias        : residual constant bias after gravity removal (m/s²)
    """
    dt = np.diff(t)

    # Remove time-varying gravity component from pitch tilt
    gravity_component = g * np.sin(np.radians(pitch_deg))
    accel_no_gravity  = accel_x - gravity_component

    # Estimate and remove residual constant bias from stationary period
    mask = (t - t[0]) < bias_window_s
    bias = accel_no_gravity[mask].mean()
    accel_adj = accel_no_gravity - bias

    # Raw integration (no correction at all)
    increments_raw = np.concatenate([[0.0], 0.5 * (accel_x[:-1] + accel_x[1:]) * dt])
    v_raw = np.cumsum(increments_raw)

    # Adjusted integration (gravity + bias removed)
    increments_adj = np.concatenate([[0.0], 0.5 * (accel_adj[:-1] + accel_adj[1:]) * dt])
    v_adj = np.cumsum(increments_adj)

    # Linear drift correction: vehicle starts and ends at rest so v[end] should be 0.
    # Subtract a linear ramp from v[0]=0 to v[end]=v_adj[-1] to enforce this.
    drift_ramp = np.linspace(0, v_adj[-1], len(v_adj))
    v_adj = v_adj - drift_ramp

    return v_raw, v_adj, accel_adj, float(bias)


# ---------------------------------------------------------------------------
# Method 2 — GPS velocity
# ---------------------------------------------------------------------------

def gps_velocity(
    utm_easting: np.ndarray,
    utm_northing: np.ndarray,
    t_gps: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Estimate speed from GPS UTM positions by finite differencing.

    Returns
    -------
    t_mid   : midpoint timestamps between consecutive GPS fixes
    v_gps   : speed magnitude (m/s) at each midpoint
    """
    dt   = np.diff(t_gps)
    de   = np.diff(utm_easting)
    dn   = np.diff(utm_northing)
    speed = np.sqrt(de**2 + dn**2) / dt
    t_mid = 0.5 * (t_gps[:-1] + t_gps[1:])
    return t_mid, speed


# ---------------------------------------------------------------------------
# Cross-check: omega * v_forward vs accel_y  (Q7)
# ---------------------------------------------------------------------------

def lateral_crosscheck(
    v_adj: np.ndarray,
    gyro_z: np.ndarray,
    accel_y: np.ndarray,
    t: np.ndarray,
) -> dict:
    """
    Compute omega * X_dot and compare to accel_y_obs.

    Under the model assumptions (no skid, IMU at CM):
        accel_y_obs = omega * X_dot

    Returns dict with arrays for plotting and correlation metric.
    """
    omega_vx = gyro_z * v_adj   # predicted accel_y from model

    # Correlation over the full drive
    corr = float(np.corrcoef(omega_vx, accel_y)[0, 1])

    return {
        "omega_vx": omega_vx,
        "accel_y":  accel_y,
        "t":        t - t[0],
        "correlation": corr,
    }


# ---------------------------------------------------------------------------
# Top-level runner
# ---------------------------------------------------------------------------

def run(
    driving_imu: list[dict],
    driving_gps: list[dict],
    results_dir: Path,
    bias_window_s: float = 10.0,
) -> dict:
    """
    Compute forward velocity from accelerometer and GPS.
    Saves both assignment plots to results_dir.
    Returns dict of arrays.
    """
    # IMU arrays
    t_imu     = np.array([r["sec"] + r["nanosec"] * 1e-9 for r in driving_imu])
    accel_x   = np.array([r["accel_x"] for r in driving_imu])
    accel_y   = np.array([r["accel_y"] for r in driving_imu])
    gyro_z    = np.array([r["gyro_z"]  for r in driving_imu])
    pitch_deg = np.array([r["pitch"]   for r in driving_imu])

    # GPS arrays
    t_gps    = np.array([r["sec"] + r["nanosec"] * 1e-9 for r in driving_gps])
    easting  = np.array([r["utm_easting"]  for r in driving_gps])
    northing = np.array([r["utm_northing"] for r in driving_gps])

    # Method 1
    v_raw, v_adj, accel_adj, bias = accel_velocity(accel_x, pitch_deg, t_imu, bias_window_s)
    print(f"[velocity] pitch mean  : {pitch_deg[:int(bias_window_s/0.025)].mean():.3f} deg")
    print(f"[velocity] residual bias after gravity removal: {bias:.5f} m/s²")
    print(f"[velocity] v_adj range : {v_adj.min():.2f} to {v_adj.max():.2f} m/s")

    # Method 2
    t_gps_mid, v_gps = gps_velocity(easting, northing, t_gps)

    # Cross-check
    cc = lateral_crosscheck(v_adj, gyro_z, accel_y, t_imu)
    print(f"[velocity] omega*vx vs accel_y correlation: {cc['correlation']:.4f}")

    # Plots
    _plot_accel_velocity(t_imu - t_imu[0], v_raw, v_adj, bias, results_dir)
    _plot_velocity_comparison(t_imu - t_imu[0], v_adj,
                              t_gps_mid - t_imu[0], v_gps, results_dir)
    _plot_crosscheck(cc, results_dir)

    return {
        "t_imu":      t_imu - t_imu[0],
        "v_raw":      v_raw,
        "v_adj":      v_adj,
        "accel_adj":  accel_adj,
        "t_gps":      t_gps_mid - t_imu[0],
        "v_gps":      v_gps,
        "accel_y":    accel_y,
        "gyro_z":     gyro_z,
        "bias":       bias,
        "crosscheck": cc,
    }


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def _plot_accel_velocity(
    t: np.ndarray,
    v_raw: np.ndarray,
    v_adj: np.ndarray,
    bias: float,
    results_dir: Path,
):
    """Assignment plot 1: accel velocity before and after bias adjustment."""
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(t, v_raw, label="Raw (no bias removal)",      color="steelblue",  linewidth=0.8)
    ax.plot(t, v_adj, label=f"Adjusted (bias={bias:.4f} m/s²)", color="darkorange", linewidth=0.9)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Forward Velocity (m/s)")
    ax.set_title("Forward Velocity from Accelerometer: Before and After Bias Adjustment")
    ax.legend()
    ax.grid(True, linewidth=0.4)
    fig.tight_layout()
    path = results_dir / "plot_velocity_accel.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[velocity] Saved → {path}")


def _plot_velocity_comparison(
    t_imu: np.ndarray,
    v_adj: np.ndarray,
    t_gps: np.ndarray,
    v_gps: np.ndarray,
    results_dir: Path,
):
    """Assignment plot 2: adjusted accel velocity vs GPS speed."""
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(t_imu, v_adj, label="Accelerometer (adjusted)", color="darkorange", linewidth=0.9)
    ax.plot(t_gps, v_gps, label="GPS speed",                color="steelblue",  linewidth=1.2,
            marker=".", markersize=3)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Forward Velocity: Accelerometer vs GPS")
    ax.legend()
    ax.grid(True, linewidth=0.4)
    fig.tight_layout()
    path = results_dir / "plot_velocity_comparison.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[velocity] Saved → {path}")


def _plot_crosscheck(cc: dict, results_dir: Path):
    """Q7 cross-check plot: omega*v_forward vs accel_y."""
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(cc["t"], cc["omega_vx"], label=r"$\omega \cdot \dot{X}$ (model)",
            color="green",      linewidth=0.9)
    ax.plot(cc["t"], cc["accel_y"],  label=r"$\ddot{y}_{obs}$ (measured accel_y)",
            color="steelblue",  linewidth=0.8, alpha=0.8)
    ax.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title(r"Cross-check: $\omega \cdot \dot{X}$ vs $\ddot{y}_{obs}$"
                 f"  (correlation={cc['correlation']:.3f})")
    ax.legend()
    ax.grid(True, linewidth=0.4)
    fig.tight_layout()
    path = results_dir / "plot_velocity_crosscheck.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[velocity] Saved → {path}")
