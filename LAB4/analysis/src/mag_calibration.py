# mag_calibration.py
# Magnetometer hard-iron + soft-iron calibration via algebraic least-squares ellipse fit.
#
# Method (2-D, X-Y plane):
#   1. Fit general conic  ax² + bxy + cy² + dx + ey = 1  via least squares.
#   2. Hard-iron offset  = ellipse center derived analytically from conic coefficients.
#   3. Soft-iron matrix  W = V @ sqrt(Λ) @ V^T  (eigendecomp of the centered ellipse matrix).
#      Applying W to centered data maps the ellipse onto a unit circle.
#   4. Calibration params saved to results/mag_calibration.npz for reuse.

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


# ---------------------------------------------------------------------------
# Core calibration
# ---------------------------------------------------------------------------

def fit_ellipse_lstsq(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Fit  ax² + bxy + cy² + dx + ey = 1  via ordinary least squares.

    Returns coefficient vector [a, b, c, d, e].
    Assumes data describes a full or near-full ellipse sweep (360°).
    """
    D = np.column_stack([x**2, x * y, y**2, x, y])   # N×5 design matrix
    ones = np.ones(len(x))
    coeffs, _, _, _ = np.linalg.lstsq(D, ones, rcond=None)
    return coeffs  # [a, b, c, d, e]


def ellipse_center(coeffs: np.ndarray) -> tuple[float, float]:
    """
    Derive ellipse center (hard-iron offset) from conic coefficients.

    From  ax² + bxy + cy² + dx + ey + f = 0  (f = -1):
        ∂F/∂x = 2ax + by + d = 0
        ∂F/∂y = bx + 2cy + e = 0
    → center = -0.5 * [[2a, b], [b, 2c]]^{-1} @ [d, e]
    """
    a, b, c, d, e = coeffs
    # Gradient conditions at center:  [[2a, b], [b, 2c]] @ [cx, cy] = [-d, -e]
    M = np.array([[2*a, b],
                  [b,  2*c]])
    center = np.linalg.solve(M, np.array([-d, -e]))
    return float(center[0]), float(center[1])


def soft_iron_matrix(coeffs: np.ndarray, cx: float, cy: float) -> np.ndarray:
    """
    Build 2×2 soft-iron correction matrix W.

    In centered coords the ellipse is  x^T · Q · x = 1  where
        Q = [[a, b/2], [b/2, c]] / k,   k = 1 - (d*cx + e*cy + a*cx² + b*cx*cy + c*cy²)

    Eigendecompose Q = V Λ V^T, then  W = V @ sqrt(Λ) @ V^T.
    Applying W maps any point on the ellipse to the unit circle:
        (Wx)^T (Wx) = x^T W^T W x = x^T Q x = 1.
    """
    a, b, c, d, e = coeffs
    f = -1.0
    k = -(a*cx**2 + b*cx*cy + c*cy**2 + d*cx + e*cy + f)
    Q = np.array([[a/k,   b/(2*k)],
                  [b/(2*k), c/k  ]])
    eigenvalues, eigenvectors = np.linalg.eigh(Q)   # eigenvalues sorted ascending
    W = eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T
    return W


def calibrate(
    mag_x: np.ndarray,
    mag_y: np.ndarray,
    results_dir: Path,
) -> tuple[float, float, np.ndarray]:
    """
    Full calibration pipeline.

    Parameters
    ----------
    mag_x, mag_y  : raw magnetometer readings (Gauss) from circle drive
    results_dir   : directory where mag_calibration.npz will be saved

    Returns
    -------
    (offset_x, offset_y, W)  — hard-iron offsets and 2×2 soft-iron matrix
    """
    coeffs = fit_ellipse_lstsq(mag_x, mag_y)
    cx, cy = ellipse_center(coeffs)
    W = soft_iron_matrix(coeffs, cx, cy)

    results_dir.mkdir(parents=True, exist_ok=True)
    np.savez(
        results_dir / "mag_calibration.npz",
        offset_x=cx,
        offset_y=cy,
        W=W,
        coeffs=coeffs,
    )
    print(f"[mag_calibration] hard-iron offset: ({cx:.5f}, {cy:.5f})")
    print(f"[mag_calibration] soft-iron W:\n{W}")
    return cx, cy, W


def load_calibration(results_dir: Path) -> tuple[float, float, np.ndarray]:
    """Load previously saved calibration parameters."""
    data = np.load(results_dir / "mag_calibration.npz")
    return float(data["offset_x"]), float(data["offset_y"]), data["W"]


def apply_calibration(
    mag_x: np.ndarray,
    mag_y: np.ndarray,
    offset_x: float,
    offset_y: float,
    W: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply hard-iron + soft-iron correction to any mag_x, mag_y arrays."""
    centered = np.column_stack([mag_x - offset_x, mag_y - offset_y])
    corrected = (W @ centered.T).T
    return corrected[:, 0], corrected[:, 1]


# ---------------------------------------------------------------------------
# Validation metrics
# ---------------------------------------------------------------------------

def validation_report(
    mx_raw: np.ndarray,
    my_raw: np.ndarray,
    mx_cal: np.ndarray,
    my_cal: np.ndarray,
    offset_x: float,
    offset_y: float,
) -> dict:
    """
    Compute quantitative validation metrics and print a summary.

    Key checks:
      - Radius from ellipse center: std should drop after calibration
      - Calibrated radius std/mean %: closeness to a perfect circle
      - Axis ratio of covariance (corrected for distribution bias)
    """
    # Radius from the fitted ellipse center (correct reference for raw data)
    r_raw_centered = np.sqrt((mx_raw - offset_x)**2 + (my_raw - offset_y)**2)
    # Calibrated data should lie on a circle centered at origin
    r_cal = np.sqrt(mx_cal**2 + my_cal**2)

    cov_cal = np.cov(np.column_stack([mx_cal, my_cal]).T)
    eig_cal = np.linalg.eigvalsh(cov_cal)
    axis_ratio_cal = eig_cal.max() / eig_cal.min()

    metrics = {
        "hard_iron_offset":       (offset_x, offset_y),
        "radius_std_raw":         float(np.std(r_raw_centered)),
        "radius_mean_cal":        float(np.mean(r_cal)),
        "radius_std_cal":         float(np.std(r_cal)),
        "radius_pct_variation":   float(np.std(r_cal) / np.mean(r_cal) * 100),
        "axis_ratio_cal":         float(axis_ratio_cal),
    }

    print("\n[mag_calibration] Validation report")
    print(f"  Hard-iron offset:          ({offset_x:+.5f}, {offset_y:+.5f}) Gauss")
    print(f"  Radius std (raw, centered):{metrics['radius_std_raw']:.5f}  Gauss")
    print(f"  Radius mean (calibrated):  {metrics['radius_mean_cal']:.5f}  Gauss")
    print(f"  Radius std  (calibrated):  {metrics['radius_std_cal']:.5f}  Gauss")
    print(f"  Radius variation:          {metrics['radius_pct_variation']:.2f}%  (lower → better circle)")
    print(f"  Axis ratio (calibrated):   {metrics['axis_ratio_cal']:.4f}  (1.0 = perfect circle)")

    return metrics


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def plot_xy(
    mx_raw: np.ndarray,
    my_raw: np.ndarray,
    mx_cal: np.ndarray,
    my_cal: np.ndarray,
    save_path: Path,
):
    """Assignment plot 1: X-Y scatter before and after calibration."""
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    for ax, x, y, title, color in [
        (axes[0], mx_raw, my_raw, "Raw (before calibration)", "steelblue"),
        (axes[1], mx_cal, my_cal, "Calibrated (after correction)", "darkorange"),
    ]:
        ax.scatter(x, y, s=2, alpha=0.5, color=color)
        ax.axhline(0, color="k", linewidth=0.6, linestyle="--")
        ax.axvline(0, color="k", linewidth=0.6, linestyle="--")
        ax.set_aspect("equal")
        ax.set_title(title)
        ax.set_xlabel("Mag X (Gauss)")
        ax.set_ylabel("Mag Y (Gauss)")
        ax.grid(True, linewidth=0.4)

    fig.suptitle("Magnetometer X-Y: Hard-Iron & Soft-Iron Calibration")
    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"[mag_calibration] Saved → {save_path}")


def plot_timeseries(
    t: np.ndarray,
    mx_raw: np.ndarray,
    my_raw: np.ndarray,
    mx_cal: np.ndarray,
    my_cal: np.ndarray,
    save_path: Path,
):
    """Assignment plot 2: time-series of mag X and Y before vs. after calibration."""
    t0 = t[0]
    t_rel = t - t0

    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    axes[0].plot(t_rel, mx_raw,  label="Raw",        color="steelblue",  linewidth=0.8)
    axes[0].plot(t_rel, mx_cal,  label="Calibrated", color="darkorange", linewidth=0.8)
    axes[0].set_ylabel("Mag X (Gauss)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, linewidth=0.4)
    axes[0].set_title("Magnetometer Time Series: Before vs. After Calibration")

    axes[1].plot(t_rel, my_raw,  label="Raw",        color="steelblue",  linewidth=0.8)
    axes[1].plot(t_rel, my_cal,  label="Calibrated", color="darkorange", linewidth=0.8)
    axes[1].set_ylabel("Mag Y (Gauss)")
    axes[1].set_xlabel("Time (s)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, linewidth=0.4)

    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"[mag_calibration] Saved → {save_path}")
