#!/usr/bin/env python3
"""
plot_trajectory.py — Plot and analyse trajectory tracking test results.

Usage:
    python3 plot_trajectory.py <csv_file> [--save <output.png>] [--no-show]

CSV format (produced by trajectory_tracking_test_node):
  Single joint (legacy-compatible):
    timestamp,command_rad,actual_rad,error_rad
  Multiple joints:
    timestamp,J1_command_rad,J1_actual_rad,J1_error_rad,J2_command_rad,...

Output:
    - Three-panel figure: command vs actual, error, and error distribution
    - Per-joint metrics: RMSE, MAE, max error, correlation, gain, and phase
"""

import argparse
import math
import os
import sys

import numpy as np


def load_csv(path: str) -> dict:
    """Load either the legacy single-joint or multi-joint CSV format."""
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    names = data.dtype.names
    if not names or "timestamp" not in names:
        raise ValueError("CSV is missing the timestamp column")

    timestamp = np.asarray(data["timestamp"], dtype=float)
    series = {}

    legacy_columns = {"command_rad", "actual_rad", "error_rad"}
    if legacy_columns.issubset(names):
        series["joint"] = {
            "cmd": np.asarray(data["command_rad"], dtype=float),
            "act": np.asarray(data["actual_rad"], dtype=float),
            "err": np.asarray(data["error_rad"], dtype=float),
        }
    else:
        suffix = "_command_rad"
        for name in names:
            if not name.endswith(suffix):
                continue
            joint = name[:-len(suffix)]
            actual_name = f"{joint}_actual_rad"
            error_name = f"{joint}_error_rad"
            if actual_name not in names or error_name not in names:
                raise ValueError(f"CSV has incomplete columns for {joint}")
            series[joint] = {
                "cmd": np.asarray(data[name], dtype=float),
                "act": np.asarray(data[actual_name], dtype=float),
                "err": np.asarray(data[error_name], dtype=float),
            }

    if not series:
        raise ValueError("CSV contains no trajectory columns")
    return {"t": timestamp, "series": series}


def compute_metrics(d: dict) -> dict:
    """Compute tracking accuracy metrics."""
    n = len(d["err"])
    rmse = math.sqrt(np.mean(d["err"] ** 2))
    mae = np.mean(np.abs(d["err"]))
    max_err = np.max(np.abs(d["err"]))

    # Pearson correlation
    if n > 1 and np.std(d["cmd"]) > 0 and np.std(d["act"]) > 0:
        corr = np.corrcoef(d["cmd"], d["act"])[0, 1]
    else:
        corr = 0.0

    # Bode-like: compute amplitude ratio and phase shift from FFT
    peak_freq = 0.0
    gain = 0.0
    phase_deg = 0.0
    if n > 1:
        dt = float(np.mean(np.diff(d["t"])))
        if dt > 0:
            fft_cmd = np.fft.rfft(d["cmd"] - np.mean(d["cmd"]))
            fft_act = np.fft.rfft(d["act"] - np.mean(d["act"]))
            freqs = np.fft.rfftfreq(n, d=dt)

            if len(fft_cmd) > 1:
                peak_idx = np.argmax(np.abs(fft_cmd[1:])) + 1
                peak_freq = freqs[peak_idx]
                amp_cmd = np.abs(fft_cmd[peak_idx])
                amp_act = np.abs(fft_act[peak_idx])
                gain = amp_act / amp_cmd if amp_cmd > 0 else 0.0
                phase_diff = np.angle(fft_act[peak_idx]) - np.angle(fft_cmd[peak_idx])
                phase_diff = (phase_diff + math.pi) % (2 * math.pi) - math.pi
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


def plot(dataset: dict, metrics: dict, save_path: str = None, show: bool = True):
    """Generate a shared three-panel plot for all recorded joints."""
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARNING: matplotlib not installed. Skipping plot generation.")
        print("Install with: pip3 install matplotlib")
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    colors = plt.get_cmap("tab10").colors
    timestamp = dataset["t"]

    # ---- Panel 1: Command vs Actual ----
    ax1 = axes[0]
    for index, (joint, d) in enumerate(dataset["series"].items()):
        color = colors[index % len(colors)]
        label = "" if joint == "joint" else f"{joint} "
        ax1.plot(timestamp, d["cmd"], linestyle="--", color=color, linewidth=1.5,
                 label=f"{label}command")
        ax1.plot(timestamp, d["act"], linestyle="-", color=color, linewidth=1.2,
                 alpha=0.85, label=f"{label}actual")
    ax1.set_ylabel("Position [rad]")
    ax1.legend(loc="upper right", ncol=2)
    ax1.grid(True, alpha=0.3)
    summary = " | ".join(
        f"{joint}: RMSE={values['rmse_deg']:.3f} deg"
        for joint, values in metrics.items()
    )
    ax1.set_title(f"Trajectory Tracking | {summary}")

    # ---- Panel 2: Tracking Error ----
    ax2 = axes[1]
    for index, (joint, d) in enumerate(dataset["series"].items()):
        color = colors[index % len(colors)]
        ax2.plot(timestamp, d["err"] * 1000, color=color, linewidth=0.9,
                 label=joint)
    ax2.axhline(y=0, color="k", linestyle=":", linewidth=0.5)
    ax2.set_ylabel("Error [mrad]")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)
    ax2.set_title("Tracking Error (command - actual)")

    # ---- Panel 3: Error Histogram ----
    ax3 = axes[2]
    for index, (joint, d) in enumerate(dataset["series"].items()):
        color = colors[index % len(colors)]
        ax3.hist(d["err"] * 1000, bins=50, color=color, alpha=0.45, label=joint)
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


def print_metrics(metrics_by_joint: dict):
    """Pretty-print the computed metrics for each recorded joint."""
    print("=" * 50)
    print("  Trajectory Tracking Analysis Results")
    print("=" * 50)
    for joint, metrics in metrics_by_joint.items():
        print(f"  -- {joint} --")
        print(f"  Samples:         {metrics['n']}")
        print(f"  Duration:        {metrics['duration']:.3f} s")
        print(f"  RMSE:            {metrics['rmse']:.6f} rad  ({metrics['rmse_deg']:.3f} deg)")
        print(f"  MAE:             {metrics['mae']:.6f} rad  ({metrics['mae_deg']:.3f} deg)")
        print(f"  Max |error|:     {metrics['max_err']:.6f} rad  "
              f"({metrics['max_err_deg']:.3f} deg)")
        print(f"  Correlation:     {metrics['correlation']:.6f}")
        print(f"  Dominant freq:   {metrics['peak_freq_hz']:.2f} Hz")
        print(f"  Gain:            {metrics['gain']:.4f}")
        print(f"  Phase shift:     {metrics['phase_deg']:.2f} deg  "
              f"({metrics['phase_ms']:.1f} ms)")
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

    try:
        dataset = load_csv(args.csv_file)
    except (ValueError, OSError) as exc:
        print(f"ERROR: cannot load CSV: {exc}")
        sys.exit(1)

    if len(dataset["t"]) == 0:
        print("ERROR: CSV file is empty or has no data rows.")
        sys.exit(1)

    metrics = {
        joint: compute_metrics({"t": dataset["t"], **series})
        for joint, series in dataset["series"].items()
    }
    print_metrics(metrics)

    save_path = args.save
    if save_path is None:
        # Auto-generate a save path next to the CSV file.
        base = os.path.splitext(args.csv_file)[0]
        save_path = base + "_plot.png"

    plot(dataset, metrics, save_path=save_path, show=not args.no_show)


if __name__ == "__main__":
    main()
