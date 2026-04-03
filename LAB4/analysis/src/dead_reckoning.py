# dead_reckoning.py
# Chapter 3 — Dead reckoning trajectory from forward velocity + heading.
#
# Steps:
#   1. Rotate forward velocity into East-North frame using CF yaw
#   2. Integrate to get (x_east, x_north) trajectory
#   3. Compute GPS trajectory from UTM positions
#   4. Align both tracks at origin with same initial heading
#   5. Plot overlay
#
# Assignment plot:
#   plot_dead_reckoning.png — DR trajectory overlaid on GPS track

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


# ---------------------------------------------------------------------------
# Core computation
# ---------------------------------------------------------------------------

def compute_trajectory(
    v_adj: np.ndarray,
    yaw_cf_deg: np.ndarray,
    t_imu: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Rotate forward velocity into East-North frame and integrate.

    Parameters
    ----------
    v_adj      : forward velocity (m/s), body frame
    yaw_cf_deg : heading from complementary filter (degrees, unwrapped)
    t_imu      : IMU timestamps (s, relative)

    Returns
    -------
    x_east, x_north : dead reckoning position (m), starting at origin
    """
    yaw_rad  = np.radians(yaw_cf_deg)
    v_east   = v_adj * np.cos(yaw_rad)
    v_north  = v_adj * np.sin(yaw_rad)

    dt = np.diff(t_imu)
    x_east  = np.concatenate([[0.0], np.cumsum(0.5 * (v_east[:-1]  + v_east[1:])  * dt)])
    x_north = np.concatenate([[0.0], np.cumsum(0.5 * (v_north[:-1] + v_north[1:]) * dt)])

    return x_east, x_north


def compute_gps_track(
    utm_easting: np.ndarray,
    utm_northing: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Shift GPS UTM track so it starts at origin.

    Returns
    -------
    gps_east, gps_north : GPS positions relative to first fix (m)
    """
    gps_east  = utm_easting  - utm_easting[0]
    gps_north = utm_northing - utm_northing[0]
    return gps_east, gps_north


def align_tracks(
    dr_east: np.ndarray,
    dr_north: np.ndarray,
    gps_east: np.ndarray,
    gps_north: np.ndarray,
    t_imu: np.ndarray,
    t_gps: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float]:
    """
    Rotate dead reckoning track so its initial heading matches GPS.

    Initial GPS bearing is computed from the first two GPS fixes.
    Initial DR heading is computed from the first few seconds of DR positions.
    The DR track is rotated by (gps_bearing - dr_bearing).

    Returns
    -------
    dr_east_rot, dr_north_rot : rotated DR track
    rot_deg                   : rotation angle applied (degrees)
    """
    # GPS initial bearing from first two fixes
    de = gps_east[1]  - gps_east[0]
    dn = gps_north[1] - gps_north[0]
    gps_bearing = np.degrees(np.arctan2(dn, de))

    # DR initial bearing from first few seconds (~10s worth of points)
    n_pts = min(400, len(dr_east) - 1)
    de_dr = dr_east[n_pts]  - dr_east[0]
    dn_dr = dr_north[n_pts] - dr_north[0]
    dr_bearing = np.degrees(np.arctan2(dn_dr, de_dr))

    rot_deg = gps_bearing - dr_bearing
    rot_rad = np.radians(rot_deg)

    c, s = np.cos(rot_rad), np.sin(rot_rad)
    dr_east_rot  = c * dr_east  - s * dr_north
    dr_north_rot = s * dr_east  + c * dr_north

    return dr_east_rot, dr_north_rot, float(rot_deg)


# ---------------------------------------------------------------------------
# Top-level runner
# ---------------------------------------------------------------------------

def compute_scale_factor(
    v_adj: np.ndarray,
    t_imu: np.ndarray,
    gps_east: np.ndarray,
    gps_north: np.ndarray,
    t_gps: np.ndarray,
) -> float:
    """
    Compute the scale factor to match DR distance to GPS distance.
    GPS total distance is the ground truth odometry.
    """
    gps_dist = np.sum(np.sqrt(np.diff(gps_east)**2 + np.diff(gps_north)**2))
    dr_dist  = np.trapz(np.abs(v_adj), t_imu)
    return float(gps_dist / dr_dist) if dr_dist > 0 else 1.0


def run(
    driving_imu: list[dict],
    driving_gps: list[dict],
    v_adj: np.ndarray,
    yaw_cf_deg: np.ndarray,
    results_dir: Path,
) -> dict:
    """
    Compute and plot dead reckoning trajectory vs GPS.

    Parameters
    ----------
    v_adj      : corrected forward velocity from velocity.py
    yaw_cf_deg : CF yaw from yaw_estimation.py (unwrapped degrees)
    """
    # IMU time
    t_imu = np.array([r["sec"] + r["nanosec"] * 1e-9 for r in driving_imu])
    t_imu = t_imu - t_imu[0]

    # GPS arrays
    t_gps    = np.array([r["sec"] + r["nanosec"] * 1e-9 for r in driving_gps])
    easting  = np.array([r["utm_easting"]  for r in driving_gps])
    northing = np.array([r["utm_northing"] for r in driving_gps])
    t_gps    = t_gps - t_gps[0]

    # GPS trajectory (origin-relative)
    gps_east, gps_north = compute_gps_track(easting, northing)

    # Scale factor to match GPS total distance
    scale = compute_scale_factor(v_adj, t_imu, gps_east, gps_north, t_gps)
    print(f"[dead_reckoning] Scale factor: {scale:.3f}")

    # Shift CF yaw so it starts at the VectorNav geographic heading at t=0
    # Our mag yaw reference is arbitrary; VN yaw is calibrated to geographic North
    yaw_imu_start = np.array([r["yaw"] for r in driving_imu])[0]
    yaw_offset    = yaw_imu_start - yaw_cf_deg[0]
    yaw_cf_geo    = yaw_cf_deg + yaw_offset
    print(f"[dead_reckoning] Yaw offset applied (mag→geo): {yaw_offset:.1f} deg")

    # DR trajectory (with scale applied to velocity)
    dr_east, dr_north = compute_trajectory(v_adj * scale, yaw_cf_geo, t_imu)

    # No additional rotation needed — yaw_cf_geo is already in geographic frame
    dr_east_rot, dr_north_rot, rot_deg = dr_east, dr_north, 0.0

    print(f"[dead_reckoning] Heading alignment rotation: {rot_deg:.1f} deg")
    print(f"[dead_reckoning] DR extent  — E: {dr_east_rot.min():.0f} to {dr_east_rot.max():.0f} m"
          f"   N: {dr_north_rot.min():.0f} to {dr_north_rot.max():.0f} m")
    print(f"[dead_reckoning] GPS extent — E: {gps_east.min():.0f} to {gps_east.max():.0f} m"
          f"   N: {gps_north.min():.0f} to {gps_north.max():.0f} m")

    _plot_trajectory(dr_east_rot, dr_north_rot, gps_east, gps_north,
                     rot_deg, scale, results_dir)

    return {
        "dr_east":   dr_east_rot,
        "dr_north":  dr_north_rot,
        "gps_east":  gps_east,
        "gps_north": gps_north,
        "t_imu":     t_imu,
        "t_gps":     t_gps,
        "rot_deg":   rot_deg,
        "scale":     scale,
    }


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def _plot_trajectory(
    dr_east: np.ndarray,
    dr_north: np.ndarray,
    gps_east: np.ndarray,
    gps_north: np.ndarray,
    rot_deg: float,
    scale: float,
    results_dir: Path,
):
    """Assignment plot: DR trajectory overlaid on GPS track."""
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.plot(gps_east,  gps_north,  color="steelblue", linewidth=1.5,
            label="GPS track (ground truth)", zorder=3)
    ax.plot(dr_east,   dr_north,   color="darkorange", linewidth=1.0,
            label=f"Dead reckoning (scale={scale:.3f}, rot={rot_deg:.1f}°)",
            alpha=0.85, zorder=2)

    # Mark start
    ax.scatter([0], [0], color="black", s=60, zorder=5, label="Start")

    ax.set_xlabel("Easting (m)")
    ax.set_ylabel("Northing (m)")
    ax.set_title("Dead Reckoning Trajectory vs GPS Ground Truth")
    ax.legend(loc="best")
    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.4)
    fig.tight_layout()

    path = results_dir / "plot_dead_reckoning.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[dead_reckoning] Saved → {path}")
