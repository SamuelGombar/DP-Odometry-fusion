#!/usr/bin/env python3
"""
Plot results from trajectory_eval.py output CSV.

Usage:
    python3 plot_trajectory_eval.py results.csv [--save output.png]

Produces three subplots:
  1. Trajectories — estimated vs. ground truth paths in 2D
  2. Per-pose error over time
  3. Error distribution histogram
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    expected = {"timestamp_s", "x_est", "y_est", "x_gt", "y_gt", "error_m"}
    missing = expected - set(df.columns)
    if missing:
        print(f"ERROR: CSV is missing columns: {missing}", file=sys.stderr)
        sys.exit(1)
    return df


def plot(df: pd.DataFrame, csv_path: str, save_path: str | None, separate: bool = False) -> None:
    t = df["timestamp_s"].values
    t_rel = t - t[0]  # seconds from start

    rmse = float(np.sqrt(np.mean(df["error_m"] ** 2)))
    mean_e = float(df["error_m"].mean())
    max_e = float(df["error_m"].max())
    n = len(df)

    suptitle = (
        f"{os.path.basename(csv_path)}  —  ATE RMSE: {rmse:.4f} m  |  "
        f"Mean: {mean_e:.4f} m  |  Max: {max_e:.4f} m  |  n={n}"
    )

    if separate:
        figs_axes = [
            plt.subplots(1, 1, figsize=(8, 7)),
            plt.subplots(1, 1, figsize=(8, 7)),
            plt.subplots(1, 1, figsize=(8, 7)),
        ]
        axes = [fa[1] for fa in figs_axes]
        for fa in figs_axes:
            fa[0].suptitle(suptitle, fontsize=10)
        fig = figs_axes[0][0]  # used only for colorbar reference below
    else:
        fig, _axes = plt.subplots(1, 3, figsize=(16, 5))
        axes = list(_axes)
        fig.suptitle(suptitle, fontsize=11)

    # --- 1. Trajectories (estimated coloured by error magnitude) ---
    ax = axes[0]
    ax.plot(df["x_gt"], df["y_gt"], color="tab:green", linewidth=1.5, label="Ground truth", zorder=2)
    sc = ax.scatter(
        df["x_est"], df["y_est"],
        c=df["error_m"], cmap="RdYlGn_r",
        s=4, linewidths=0, zorder=3,
        label="Estimated (colour = error)",
    )
    cbar = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Error (m)", fontsize=8)
    cbar.ax.tick_params(labelsize=7)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Trajectories (error coloured)")
    ax.legend(fontsize=8, markerscale=3)
    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.4)

    # --- 2. Per-pose error over time ---
    ax = axes[1]
    ax.plot(t_rel, df["error_m"], color="tab:orange", linewidth=0.8)
    ax.axhline(rmse, color="tab:red", linestyle="--", linewidth=1.2, label=f"RMSE {rmse:.4f} m")
    ax.axhline(mean_e, color="tab:purple", linestyle=":", linewidth=1.2, label=f"Mean {mean_e:.4f} m")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.set_title("Per-pose Error over Time")
    ax.legend(fontsize=9)
    ax.grid(True, linewidth=0.4)

    # --- 3. Error distribution ---
    ax = axes[2]
    ax.hist(df["error_m"], bins=40, color="tab:blue", alpha=0.75, edgecolor="white", linewidth=0.4)
    ax.axvline(rmse, color="tab:red", linestyle="--", linewidth=1.2, label=f"RMSE {rmse:.4f} m")
    ax.axvline(mean_e, color="tab:purple", linestyle=":", linewidth=1.2, label=f"Mean {mean_e:.4f} m")
    ax.set_xlabel("Error (m)")
    ax.set_ylabel("Count")
    ax.set_title("Error Distribution")
    ax.legend(fontsize=9)
    ax.grid(True, linewidth=0.4, axis="y")

    plt.tight_layout()

    if save_path:
        if separate:
            base, ext = os.path.splitext(save_path)
            ext = ext or ".png"
            for i, (f, _) in enumerate(figs_axes):
                out = f"{base}_{i+1}{ext}"
                f.savefig(out, dpi=150, bbox_inches="tight")
                print(f"Saved to: {os.path.abspath(out)}")
        else:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"Saved to: {os.path.abspath(save_path)}")
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot trajectory evaluation CSV produced by trajectory_eval.py"
    )
    parser.add_argument("csv_path", help="Path to the results CSV file")
    parser.add_argument(
        "--separate",
        action="store_true",
        help="Plot each graph in its own larger window instead of a single combined figure.",
    )
    parser.add_argument(
        "--save",
        metavar="OUTPUT_PNG",
        help="Save figure to this path instead of displaying interactively",
    )
    args = parser.parse_args()

    csv_path = os.path.abspath(args.csv_path)
    if not os.path.isfile(csv_path):
        print(f"ERROR: file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    df = load_csv(csv_path)
    plot(df, csv_path, args.save, separate=args.separate)


if __name__ == "__main__":
    main()