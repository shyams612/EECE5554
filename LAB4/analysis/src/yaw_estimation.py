# yaw_estimation.py
# Chapter 1 — Yaw angle estimation from magnetometer and gyroscope.
#
# Four methods computed and compared:
#   1. Magnetometer yaw        — atan2(my_cal, mx_cal), uses calibration from circle bag
#   2. Gyro-integrated yaw     — cumulative trapezoid integral of gyro_z (rad/s), debiased
#   3. Complementary filter    — LPF(mag) + HPF(gyro), removes gyro drift + mag noise
#   4. IMU Euler yaw           — onboard VectorNav estimate (reference only, not used in fusion)
#
# Key design note — wrapped vs unwrapped:
#   The CF operates in the wrapped [-180, 180] domain.
#   Innovation is computed as wrap(mag - CF_prev), NOT (mag - prediction).
#   This prevents the correction from silently becoming zero when the CF
#   accumulates a drift near a multiple of 360°.
#   All signals are unwrapped at the end purely for plotting.
#
# Assignment plots produced:
#   plot_yaw_lpf_hpf_cf.png    — LPF mag, HPF gyro, CF together
#   plot_yaw_four_methods.png  — all four yaw estimates on one figure

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.signal import butter, filtfilt


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wrap(x: np.ndarray | float) -> np.ndarray | float:
    """Wrap angle(s) in degrees to [-180, 180]."""
    return ((np.asarray(x) + 180.0) % 360.0) - 180.0


def _safe_unwrap(deg: np.ndarray) -> np.ndarray:
    """Unwrap a degree sequence using numpy, converting through radians."""
    return np.degrees(np.unwrap(np.radians(deg)))


# ---------------------------------------------------------------------------
# Gyro integration
# ---------------------------------------------------------------------------

def integrate_gyro(
    gyro_z: np.ndarray,
    t: np.ndarray,
    bias_window_s: float = 10.0,
) -> np.ndarray:
    """
    Integrate gyro_z (rad/s) to cumulative yaw in degrees starting at 0.

    Bias is estimated from the first `bias_window_s` seconds (car stationary).
    Uses trapezoidal rule.
    """
    dt = np.diff(t)
    bias_mask = t[:-1] - t[0] < bias_window_s
    bias = gyro_z[:-1][bias_mask].mean()

    gz_deb = gyro_z.copy()
    gz_deb[:-1] -= bias

    increments = np.concatenate([[0.0], 0.5 * (gz_deb[:-1] + gz_deb[1:]) * dt])
    return np.degrees(np.cumsum(increments))


# ---------------------------------------------------------------------------
# Magnetometer yaw
# ---------------------------------------------------------------------------

def mag_yaw(mx_cal: np.ndarray, my_cal: np.ndarray) -> np.ndarray:
    """Compute wrapped yaw in degrees from calibrated magnetometer readings."""
    return np.degrees(np.arctan2(my_cal, mx_cal))


# ---------------------------------------------------------------------------
# Complementary filter  (operates in wrapped domain)
# ---------------------------------------------------------------------------

def complementary_filter(
    yaw_mag_wrapped: np.ndarray,
    gyro_z_rads: np.ndarray,
    t: np.ndarray,
    fc_hz: float = 0.1,
    bias_window_s: float = 10.0,
    order: int = 4,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Complementary filter using Butterworth LP + HP filters.

    Both yaw signals are unwrapped before filtering to remove discontinuities.
    A low-pass Butterworth filter is applied to magnetometer yaw (slow, drift-free).
    A high-pass Butterworth filter is applied to integrated gyro yaw (fast, no drift).
    CF = LPF(yaw_mag) + HPF(yaw_gyro)

    Parameters
    ----------
    fc_hz : cutoff frequency in Hz (same for LP and HP)
    order : Butterworth filter order

    Returns
    -------
    yaw_cf       : complementary filter output (degrees, unwrapped)
    yaw_lpf_mag  : low-pass filtered magnetometer yaw (degrees, unwrapped)
    yaw_hpf_gyro : high-pass filtered integrated gyro yaw (degrees)
    """
    fs   = 1.0 / np.median(np.diff(t))   # sample rate Hz
    nyq  = 0.5 * fs
    wn   = fc_hz / nyq

    b_lp, a_lp = butter(order, wn, btype='low')
    b_hp, a_hp = butter(order, wn, btype='high')

    # Unwrap magnetometer yaw before filtering
    yaw_mag_uw = _safe_unwrap(yaw_mag_wrapped)

    # Integrated gyro yaw (debiased, starts at 0, then aligned to mag start)
    yaw_gyro = integrate_gyro(gyro_z_rads, t, bias_window_s)
    yaw_gyro += yaw_mag_uw[0]   # align starting heading to mag

    # Apply filters
    yaw_lpf_mag  = filtfilt(b_lp, a_lp, yaw_mag_uw)
    yaw_hpf_gyro = filtfilt(b_hp, a_hp, yaw_gyro)

    yaw_cf = yaw_lpf_mag + yaw_hpf_gyro

    return yaw_cf, yaw_lpf_mag, yaw_hpf_gyro


# ---------------------------------------------------------------------------
# Top-level runner
# ---------------------------------------------------------------------------

def run(
    driving_imu: list[dict],
    results_dir: Path,
    offset_x: float,
    offset_y: float,
    W: np.ndarray,
    fc_hz: float = 0.1,
    gyro_bias_window_s: float = 10.0,
) -> dict:
    """
    Compute all four yaw estimates from the driving bag IMU data.
    Saves both assignment plots to results_dir.
    Returns dict of arrays.
    """
    from .mag_calibration import apply_calibration

    t        = np.array([r["sec"] + r["nanosec"] * 1e-9 for r in driving_imu])
    gyro_z   = np.array([r["gyro_z"] for r in driving_imu])
    mx_raw   = np.array([r["mag_x"] for r in driving_imu])
    my_raw   = np.array([r["mag_y"] for r in driving_imu])
    yaw_imu  = np.array([r["yaw"] for r in driving_imu])   # VectorNav Euler yaw

    # 1. Magnetometer yaw (calibrated, wrapped)
    mx_cal, my_cal     = apply_calibration(mx_raw, my_raw, offset_x, offset_y, W)
    yaw_mag_wrapped    = mag_yaw(mx_cal, my_cal)

    # 2. Gyro-integrated yaw (starts at 0, then aligned to mag start)
    yaw_gyro = integrate_gyro(gyro_z, t, bias_window_s=gyro_bias_window_s)
    yaw_gyro += yaw_mag_wrapped[0]

    # 3. Complementary filter
    yaw_cf, yaw_lpf_mag, yaw_hpf_gyro = complementary_filter(
        yaw_mag_wrapped, gyro_z, t, fc_hz=fc_hz, bias_window_s=gyro_bias_window_s
    )

    # Unwrap magnetometer and IMU
    yaw_mag_uw = _safe_unwrap(yaw_mag_wrapped)
    yaw_imu_uw = _safe_unwrap(yaw_imu)

    # Align IMU to same starting value as mag
    yaw_imu_uw += (yaw_mag_uw[0] - yaw_imu_uw[0])

    print(f"[yaw_estimation] Butterworth CF: order=4  f_c={fc_hz:.4f} Hz")

    results = {
        "t":             t - t[0],
        "yaw_mag":       yaw_mag_uw,
        "yaw_gyro":      yaw_gyro,
        "yaw_cf":        yaw_cf,
        "yaw_lpf_mag":   yaw_lpf_mag,
        "yaw_hpf_gyro":  yaw_hpf_gyro,
        "yaw_imu":       yaw_imu_uw,
        "yaw_cf_wrapped": np.degrees(np.arctan2(
            np.sin(np.radians(yaw_cf)), np.cos(np.radians(yaw_cf))
        )),
        "f_c_hz":        fc_hz,
    }

    _plot_lpf_hpf_cf(results, results_dir)
    _plot_four_methods(results, results_dir)

    return results


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def _plot_lpf_hpf_cf(res: dict, results_dir: Path):
    """
    Assignment plot 3: LPF(mag) + HPF(gyro) + complementary filter.

    LPF and CF are shown on the heading scale (degrees).
    HPF is shown on its own axis (right side, zero-mean short-term dynamics).
    """
    t   = res["t"]
    f_c = res["f_c_hz"]

    fig, ax1 = plt.subplots(figsize=(14, 5))
    ax2 = ax1.twinx()

    ax1.plot(t, res["yaw_lpf_mag"], label=f"LPF Magnetometer  (f_c={f_c:.4f} Hz)",
             color="steelblue", linewidth=1.1, alpha=0.9)
    ax1.plot(t, res["yaw_cf"],      label=f"Complementary Filter  (f_c={f_c:.4f} Hz)",
             color="green",     linewidth=1.3)
    ax2.plot(t, res["yaw_hpf_gyro"], label=f"HPF Integrated Gyro  (f_c={f_c:.4f} Hz)",
             color="darkorange", linewidth=0.9, alpha=0.85)
    ax2.axhline(0, color="darkorange", linewidth=0.5, linestyle=":")

    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Heading (degrees)")
    ax2.set_ylabel("HPF Gyro — short-term dynamics (degrees)", color="darkorange")
    ax2.tick_params(axis="y", labelcolor="darkorange")

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="lower left")

    ax1.set_title("Yaw Estimation: LPF Magnetometer, HPF Integrated Gyro, Complementary Filter")
    ax1.grid(True, linewidth=0.4)
    fig.tight_layout()

    path = results_dir / "plot_yaw_lpf_hpf_cf.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[yaw_estimation] Saved → {path}")


def _plot_four_methods(res: dict, results_dir: Path):
    """Assignment plot 4: all four yaw methods compared (unwrapped)."""
    t = res["t"]

    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)

    axes[0].plot(t, res["yaw_mag"],  label="Magnetometer (calibrated)",
                 color="steelblue",  linewidth=0.9)
    axes[0].plot(t, res["yaw_gyro"], label="Gyro integrated (debiased)",
                 color="darkorange", linewidth=0.9)
    axes[0].plot(t, res["yaw_cf"],   label="Complementary filter",
                 color="green",      linewidth=1.2)
    axes[0].plot(t, res["yaw_imu"],  label="IMU Euler (VectorNav)",
                 color="red",        linewidth=0.9, linestyle="--")
    axes[0].set_ylabel("Yaw — unwrapped (degrees)")
    axes[0].set_title("Yaw Estimation: Four Methods Compared")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, linewidth=0.4)

    axes[1].plot(t, res["yaw_mag"]  - res["yaw_cf"], label="Mag − CF",
                 color="steelblue",  linewidth=0.8)
    axes[1].plot(t, res["yaw_gyro"] - res["yaw_cf"], label="Gyro − CF",
                 color="darkorange", linewidth=0.8)
    axes[1].plot(t, res["yaw_imu"]  - res["yaw_cf"], label="IMU Euler − CF",
                 color="red",        linewidth=0.8, linestyle="--")
    axes[1].axhline(0, color="green", linewidth=1.0, linestyle="--",
                    label="Complementary filter (reference)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Deviation from CF (degrees)")
    axes[1].set_title("Deviation of Each Method from Complementary Filter")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, linewidth=0.4)

    fig.tight_layout()
    path = results_dir / "plot_yaw_four_methods.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[yaw_estimation] Saved → {path}")
