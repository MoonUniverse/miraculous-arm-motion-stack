#!/usr/bin/env python3
"""
plot_trajectory.py — Plot and analyse trajectory tracking test results.

Usage:
    python3 plot_trajectory.py <csv_file> [--save <output.png>] [--no-show]

CSV format (produced by trajectory_tracking_test_node):
    timestamp,command_rad,actual_rad,error_rad

Output:
    - Three-panel figure: command vs actual, error, and frequency response
    - Printed metrics: RMSE, MAE, max error, correlation, estimated lag
"""

import argparse
import math
import os
import sys

import numpy as np


def load_csv(path: str) -> dict:
    """Load the tracking test CSV into a dict of numpy arrays."""
    data = np.genfromtxt(path, delimiter=",", names=True)
    return {
        "t": data["timestamp"],
        "cmd": data["command_rad"],
        "act": data["actual_rad"],
        "err": data["error_rad"],
    }


def compute_metrics(d: dict) -> dict:
    """Compute tracking accuracy metrics."""
    n = len(d["err"])
    rmse = math.sqrt(np.mean(d["err"] ** 2))
    mae = np.mean(np.abs(d["err"]))
    max_err = np.max(np.abs(d["err"]))
    # Pearson correlation
    corr = np.corrcoef(d["cmd"], d["act"])[0, 1]
    # Bode-like: compute amplitude ratio and phase shift from FFT
    dt = d["t"][1] - d["t"][0] if len(d["t"]) > 1 else 0.01
    fft_cmd = np.fft.rfft(d["cmd"] - np.mean(d["cmd"]))
    fft_act = np.fft.rfft(d["act"] - np.mean(d["act"]))
    freqs = np.fft.rfftfreq(len(d["t"]), d=dt)

    # Find the dominant frequency peak in command
    peak_idx = np.argmax(np.abs(fft_cmd[1:])) + 1  # skip DC
    peak_freq = freqs[peak_idx]

    # Amplitude ratio at dominant frequency
    amp_cmd = np.abs(fft_cmd[peak_idx])
    amp_act = np.abs(fft_act[peak_idx])
    gain = amp_act / amp_cmd if amp_cmd > 0 else 0.0

    # Phase shift at dominant frequency
    phase_cmd = np.angle(fft_cmd[peak_idx])
    phase_act = np.angle(fft_act[peak_idx])
    phase_diff = phase_act - phase_cmd  # radians (negative = lag)
    phase_deg = math.degrees(phase_diff)

    return {
        "n": n,
        "duration": d["t"][-1] if len(d["t"]) > 0 else 0,
        "rmse": rmse,
        "rmse_deg": rmse * 180 / math.pi,
        "mae": mae,
        "mae_deg": mae * 180 / math.pi,
        "max_err": max_err,
        "max_err_deg": max_err * 180 / math.pi,
        "correlation": corr,
        "peak_freq_hz": peak_freq,
        "gain": gain,
        "phase_deg": phase_deg,
        "phase_ms": phase_deg / (360 * peak_freq) * 1000 if peak_freq > 0 else 0,
    }


def plot(d: dict, metrics: dict, save_path: str = None, show: bool = True):
    """Generate the three-panel plot."""
    try:
        import matplotlib
        matplotlib.use("Agg" if not show else "TkAgg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARNING: matplotlib not installed. Skipping plot generation.")
        print("Install with: pip3 install matplotlib")
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    # ---- Panel 1: Command vs Actual ----
    ax1 = axes[0]
    ax1.plot(d["t"], d["cmd"], "b-", linewidth=1.8, label="Command (target)")
    ax1.plot(d["t"], d["act"], "r-", linewidth=1.2, alpha=0.85, label="Actual (encoder)")
    ax1.set_ylabel("Position [rad]")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)
    ax1.set_title(
        f"Trajectory Tracking  |  RMSE={metrics['rmse_deg']:.3f}°  "
        f"Corr={metrics['correlation']:.4f}  Lag={metrics['phase_ms']:.1f}ms"
    )

    # ---- Panel 2: Tracking Error ----
    ax2 = axes[1]
    ax2.plot(d["t"], d["err"] * 1000, "g-", linewidth=0.8)
    ax2.axhline(y=0, color="k", linestyle=":", linewidth=0.5)
    ax2.axhline(y=metrics["rmse"] * 1000, color="orange", linestyle="--",
                linewidth=0.7, label=f"RMSE = {metrics['rmse']*1000:.2f} mrad")
    ax2.axhline(y=-metrics["rmse"] * 1000, color="orange", linestyle="--",
                linewidth=0.7)
    ax2.set_ylabel("Error [mrad]")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)
    ax2.set_title("Tracking Error (command − actual)")

    # ---- Panel 3: Error Histogram ----
    ax3 = axes[2]
    err_mrad = d["err"] * 1000
    bins = np.linspace(err_mrad.min(), err_mrad.max(), 50)
    ax3.hist(err_mrad, bins=bins, color="steelblue", edgecolor="white", alpha=0.8)
    ax3.axvline(x=metrics["mae"] * 1000, color="red", linestyle="--",
                linewidth=1.2, label=f"MAE = {metrics['mae']*1000:.2f} mrad")
    ax3.set_ylabel("Count")
    ax3.set_xlabel("Error [mrad]")
    ax3.legend(loc="upper right")
    ax3.grid(True, alpha=0.3)
    ax3.set_title("Error Distribution")

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Plot saved to: {save_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def print_metrics(metrics: dict):
    """Pretty-print the computed metrics."""
    print("=" * 50)
    print("  Trajectory Tracking Analysis Results")
    print("=" * 50)
    print(f"  Samples:         {metrics['n']}")
    print(f"  Duration:        {metrics['duration']:.3f} s")
    print(f"  RMSE:            {metrics['rmse']:.6f} rad  ({metrics['rmse_deg']:.3f} deg)")
    print(f"  MAE:             {metrics['mae']:.6f} rad  ({metrics['mae_deg']:.3f} deg)")
    print(f"  Max |error|:     {metrics['max_err']:.6f} rad  ({metrics['max_err_deg']:.3f} deg)")
    print(f"  Correlation:     {metrics['correlation']:.6f}")
    print(f"  Dominant freq:   {metrics['peak_freq_hz']:.2f} Hz")
    print(f"  Gain:            {metrics['gain']:.4f}")
    print(f"  Phase shift:     {metrics['phase_deg']:.2f} deg  ({metrics['phase_ms']:.1f} ms)")
    print("=" * 50)


def main():
    parser = argparse.ArgumentParser(
        description="Plot and analyse trajectory tracking test results.")
    parser.add_argument("csv_file", help="Path to the tracking test CSV file")
    parser.add_argument("--save", default=None,
                        help="Save plot to this path (e.g. result.png)")
    parser.add_argument("--no-show", action="store_true",
                        help="Do not display the plot (only save/print)")
    args = parser.parse_args()

    if not os.path.isfile(args.csv_file):
        print(f"ERROR: file not found: {args.csv_file}")
        sys.exit(1)

    d = load_csv(args.csv_file)
    if len(d["t"]) == 0:
        print("ERROR: CSV file is empty or has no data rows.")
        sys.exit(1)

    metrics = compute_metrics(d)
    print_metrics(metrics)

    save_path = args.save
    if save_path is None:
        # Auto-generate a save path next to the CSV file.
        base = os.path.splitext(args.csv_file)[0]
        save_path = base + "_plot.png"

    plot(d, metrics, save_path=save_path, show=not args.no_show)


if __name__ == "__main__":
    main()
