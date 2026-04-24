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
import matplotlib.collections as mc
import numpy as np
import pandas as pd


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    expected = {"timestamp_s", "x_est", "y_est", "timestamp_gt_s", "x_gt", "y_gt", "error_m"}
    missing = expected - set(df.columns)
    if missing:
        print(f"ERROR: CSV is missing columns: {missing}", file=sys.stderr)
        sys.exit(1)
    return df


def rotate_coords(x: np.ndarray, y: np.ndarray, deg: float):
    """Rotate 2-D points by *deg* degrees counter-clockwise."""
    rad = np.deg2rad(deg)
    c, s = np.cos(rad), np.sin(rad)
    return c * x - s * y, s * x + c * y


def plot(df: pd.DataFrame, csv_path: str, save_path: str | None, separate: bool = False, match_lines: bool = False, rotate_deg: float = 0.0) -> None:
    matched = df[df["error_m"].notna()].reset_index(drop=True)
    t = matched["timestamp_s"].values
    t_rel = t - t[0]  # seconds from start

    rmse = float(np.sqrt(np.mean(matched["error_m"] ** 2)))
    mean_e = float(matched["error_m"].mean())
    max_e = float(matched["error_m"].max())
    n = len(matched)

    suptitle = (
        f"{os.path.basename(csv_path)}  —  ATE RMSE: {rmse:.4f} m  |  "
        f"Mean: {mean_e:.4f} m  |  Max: {max_e:.4f} m  |  n={n}"
    )

    if separate:
        figs_axes = [
            plt.subplots(1, 1, figsize=(14, 8)),
            plt.subplots(1, 1, figsize=(14, 8)),
            plt.subplots(1, 1, figsize=(14, 8)),
        ]
        axes = [fa[1] for fa in figs_axes]
        for fa in figs_axes:
            fa[0].suptitle(suptitle, fontsize=10)
        fig = figs_axes[0][0]  # used only for colorbar reference below
    else:
        fig, _axes = plt.subplots(1, 3, figsize=(16, 5))
        axes = list(_axes)
        fig.suptitle(suptitle, fontsize=11)

    # --- apply optional rotation ---
    if rotate_deg != 0.0:
        matched = matched.copy()
        matched["x_est"], matched["y_est"] = rotate_coords(
            matched["x_est"].values, matched["y_est"].values, rotate_deg
        )
        matched["x_gt"], matched["y_gt"] = rotate_coords(
            matched["x_gt"].values, matched["y_gt"].values, rotate_deg
        )
        df = df.copy()
        df["x_gt"], df["y_gt"] = rotate_coords(
            df["x_gt"].values, df["y_gt"].values, rotate_deg
        )

    # --- 1. Trajectories (estimated coloured by error magnitude) ---
    ax = axes[0]
    # Sort GT by its own timestamps so the line follows the actual GT path order
    gt_line = df[df["timestamp_gt_s"].notna()].drop_duplicates(subset="timestamp_gt_s").sort_values("timestamp_gt_s")
    ax.plot(gt_line["x_gt"], gt_line["y_gt"], color=plt.cm.plasma(0.0), linewidth=3.0, label="Ground truth", zorder=2)
    if match_lines:
        segments = np.stack(
            [np.column_stack([matched["x_est"], matched["y_est"]]),
             np.column_stack([matched["x_gt"],  matched["y_gt"]])],
            axis=1,
        )
        ax.add_collection(mc.LineCollection(segments, colors="gray", linewidths=0.4, alpha=0.4, zorder=1))
    est_pts = np.column_stack([matched["x_est"].values, matched["y_est"].values])
    est_segments = np.stack([est_pts[:-1], est_pts[1:]], axis=1)
    norm = plt.Normalize(matched["error_m"].min(), matched["error_m"].max())
    est_lc = mc.LineCollection(est_segments, cmap="plasma", norm=norm, linewidths=3.0, zorder=3)
    est_lc.set_array((matched["error_m"].values[:-1] + matched["error_m"].values[1:]) / 2)
    ax.add_collection(est_lc)
    sc = ax.scatter(
        matched["x_est"], matched["y_est"],
        c=matched["error_m"], cmap="plasma", norm=norm,
        s=4, linewidths=0, zorder=4,
        label="Estimated (colour = error)",
    )
    cbar = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Error (m)", fontsize=21)
    cbar.ax.tick_params(labelsize=19)
    ax.set_xlabel("x (m)", fontsize=26)
    ax.set_ylabel("y (m)", fontsize=26)
    ax.set_title("Trajectories (error coloured)", fontsize=24)
    ax.legend(fontsize=15, markerscale=3)
    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.4)
    ax.tick_params(labelsize=24)

    # --- 2. Per-pose error over time ---
    ax = axes[1]
    ax.plot(t_rel, matched["error_m"], color="tab:orange", linewidth=1.8)
    ax.axhline(rmse, color="tab:red", linestyle="--", linewidth=1.2, label=f"RMSE {rmse:.4f} m")
    ax.axhline(mean_e, color="tab:purple", linestyle=":", linewidth=1.2, label=f"Mean {mean_e:.4f} m")
    ax.set_xlabel("Time (s)", fontsize=26)
    ax.set_ylabel("Error (m)", fontsize=26)
    ax.set_title("Per-pose Error over Time", fontsize=24)
    ax.legend(fontsize=15)
    ax.grid(True, linewidth=0.4)
    ax.tick_params(labelsize=24)

    # --- 3. Error distribution ---
    ax = axes[2]
    ax.hist(matched["error_m"], bins=40, color="tab:blue", alpha=0.75, edgecolor="white", linewidth=0.4)
    ax.axvline(rmse, color="tab:red", linestyle="--", linewidth=1.2, label=f"RMSE {rmse:.4f} m")
    ax.axvline(mean_e, color="tab:purple", linestyle=":", linewidth=1.2, label=f"Mean {mean_e:.4f} m")
    ax.set_xlabel("Error (m)", fontsize=26)
    ax.set_ylabel("Count", fontsize=26)
    ax.set_title("Error Distribution", fontsize=24)
    ax.legend(fontsize=15)
    ax.grid(True, linewidth=0.4, axis="y")
    ax.tick_params(labelsize=24)

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
    parser.add_argument(
        "--match-lines",
        action="store_true",
        help="Draw gray lines connecting each matched estimated/GT pose pair in the trajectory plot.",
    )
    parser.add_argument(
        "--rotate",
        metavar="DEG",
        type=float,
        default=0.0,
        help="Rotate both trajectories counter-clockwise by this many degrees before plotting.",
    )
    args = parser.parse_args()

    csv_path = os.path.abspath(args.csv_path)
    if not os.path.isfile(csv_path):
        print(f"ERROR: file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    df = load_csv(csv_path)
    plot(df, csv_path, args.save, separate=args.separate, match_lines=args.match_lines, rotate_deg=args.rotate)


if __name__ == "__main__":
    main()