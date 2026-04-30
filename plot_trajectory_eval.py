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
from matplotlib.legend_handler import HandlerTuple
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


def plot(df: pd.DataFrame, csv_path: str, save_path: str | None, separate: bool = False, match_lines: bool = False, rotate_deg: float = 0.0, traj_title: str = "Trajectories (error coloured)", est_label: str = "Estimated (colour = error)", colormap: str = "copper") -> None:
    matched = df[df["error_m"].notna()].reset_index(drop=True)
    t = matched["timestamp_s"].values
    t_gt = matched["timestamp_gt_s"].values
    common_t0 = min(t[0], t_gt[0])
    t_est_rel = t - common_t0       # est timestamps relative to common origin
    t_gt_rel  = t_gt - common_t0   # GT  timestamps relative to common origin
    t_rel = t_est_rel              # used for error-over-time plot (est axis)

    rmse = float(np.sqrt(np.mean(matched["error_m"] ** 2)))
    mean_e = float(matched["error_m"].mean())
    std_e = float(matched["error_m"].std())
    max_e = float(matched["error_m"].max())
    n = len(matched)

    suptitle = (
        f"{os.path.basename(csv_path)}  —  ATE RMSE: {rmse:.4f} m  |  "
        f"Aritm. priemer: {mean_e:.4f} m  |  Smerodajná odch.: {std_e:.4f} m  |  Max: {max_e:.4f} m  |  n={n}"
    )

    if separate:
        fig_xy, (ax_x, ax_y) = plt.subplots(2, 1, figsize=(14, 8))  # x/y vs time combined
        figs_axes = [
            plt.subplots(1, 1, figsize=(11, 8)), #(11, 8) pre Frodo, (6, 8) 
            plt.subplots(1, 1, figsize=(14, 8)),
            plt.subplots(1, 1, figsize=(14, 8)),
            (fig_xy, None),  # placeholder — axes handled separately
        ]
        axes = [figs_axes[0][1], figs_axes[1][1], figs_axes[2][1], ax_x, ax_y]
        for fa in figs_axes[:3]:
            pass  # fa[0].suptitle(suptitle, fontsize=10)
        fig = figs_axes[0][0]  # used only for colorbar reference below
    else:
        fig, _axes = plt.subplots(1, 5, figsize=(26, 5))
        axes = list(_axes)
        # fig.suptitle(suptitle, fontsize=11)

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
    # Rows where x_est is present but error_m is absent = post-cutoff estimated tail
    tail = df[df["x_est"].notna() & df["error_m"].isna()].sort_values("timestamp_s")
    # Sort GT by its own timestamps so the line follows the actual GT path order
    gt_line = df[df["timestamp_gt_s"].notna()].drop_duplicates(subset="timestamp_gt_s").sort_values("timestamp_gt_s")
    cmap_obj = plt.get_cmap(colormap)
    gt_handle, = ax.plot(gt_line["x_gt"], gt_line["y_gt"], color=cmap_obj(0.0), linewidth=3.0, zorder=2)
    if match_lines:
        segments = np.stack(
            [np.column_stack([matched["x_est"], matched["y_est"]]),
             np.column_stack([matched["x_gt"],  matched["y_gt"]])],
            axis=1,
        )
        ax.add_collection(mc.LineCollection(segments, colors="gray", linewidths=0.2, alpha=0.4, zorder=1))
    est_pts = np.column_stack([matched["x_est"].values, matched["y_est"].values])
    est_segments = np.stack([est_pts[:-1], est_pts[1:]], axis=1)
    norm = plt.Normalize(matched["error_m"].min(), matched["error_m"].max())
    est_lc = mc.LineCollection(est_segments, cmap=colormap, norm=norm, linewidths=3.0, zorder=3)
    est_lc.set_array((matched["error_m"].values[:-1] + matched["error_m"].values[1:]) / 2)
    ax.add_collection(est_lc)
    sc = ax.scatter(
        matched["x_est"], matched["y_est"],
        c=matched["error_m"], cmap=colormap, norm=norm,
        s=4, linewidths=0, zorder=4,
    )    # Draw post-cutoff estimated tail (no pairing) in gray
    if not tail.empty:
        # Connect to the last matched point so the line is continuous
        last_matched = matched.iloc[-1]
        tail_x = np.concatenate([[last_matched["x_est"]], tail["x_est"].values])
        tail_y = np.concatenate([[last_matched["y_est"]], tail["y_est"].values])
        ax.plot(tail_x, tail_y, color="gray", linewidth=2.0, linestyle="--", zorder=3, label="Po cutoff (nepárované)")
    cbar = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Error (m)", fontsize=21)
    cbar.ax.tick_params(labelsize=19)
    ax.set_xlabel("x (m)", fontsize=26)
    ax.set_ylabel("y (m)", fontsize=26)
    ax.set_title(f"{est_label} - {traj_title}", fontsize=24)
    n_dots = 6
    est_dots = tuple(
        plt.Line2D([], [], marker="o", linestyle="None", markersize=7,
                   color=cmap_obj(i / (n_dots - 1)))
        for i in range(n_dots)
    )
    ax.legend(
        handles=[gt_handle, est_dots],
        labels=["Referenčná trajektória", est_label],
        handler_map={tuple: HandlerTuple(ndivide=None, pad=0.2)},
        fontsize=15,
    )
    ax.set_aspect("equal")
    ax.set_xlim(-20, 20) #only for ralf
    # ax.set_xlim(-2, 2) #only for eight kobuki map
    ax.grid(True, linewidth=0.4)
    ax.tick_params(labelsize=24)

    # --- 2. Per-pose error over time ---
    ax = axes[1]
    ax.plot(t_rel, matched["error_m"], color="tab:orange", linewidth=1.8)
    ax.axhline(rmse, color="tab:red", linestyle="--", linewidth=2.5, label=f"RMSE {rmse:.4f} m")
    ax.axhline(mean_e, color="tab:blue", linestyle=":", linewidth=2.5, label=f"Aritm. priemer {mean_e:.4f} m")
    ax.axhline(std_e, color="tab:green", linestyle="-.", linewidth=2.5, label=f"Smerodajná odch. {std_e:.4f} m")
    ax.set_xlabel("Čas (s)", fontsize=26)
    ax.set_ylabel("Error (m)", fontsize=26)
    ax.set_title(f"Chyba polôh v čase: {est_label} - {traj_title}", fontsize=24)
    ax.legend(fontsize=15)
    ax.grid(True, linewidth=0.4)
    ax.tick_params(labelsize=24)

    # --- 3. Error distribution ---
    ax = axes[2]
    ax.hist(matched["error_m"], bins=40, color="tab:blue", alpha=0.75, edgecolor="white", linewidth=0.4)
    ax.axvline(rmse, color="tab:red", linestyle="--", linewidth=1.8, label=f"RMSE {rmse:.4f} m")
    ax.axvline(mean_e, color="tab:purple", linestyle=":", linewidth=2.5, label=f"Mean {mean_e:.4f} m")
    ax.axvline(std_e, color="tab:green", linestyle="-.", linewidth=2.5, label=f"Smerodajná odch. {std_e:.4f} m")
    ax.set_xlabel("Error (m)", fontsize=26)
    ax.set_ylabel("Počet", fontsize=26)
    ax.set_title(f"Distribúcia chyby: {est_label} - {traj_title}", fontsize=24)
    ax.legend(fontsize=15)
    ax.grid(True, linewidth=0.4, axis="y")
    ax.tick_params(labelsize=24)

    # --- 4. x vs time ---
    # Each trajectory is plotted at its own timestamps.
    ax = axes[3]
    ax.plot(t_est_rel, matched["x_est"].values, color="red", linewidth=3.6, label=est_label)
    ax.plot(t_gt_rel,  matched["x_gt"].values,  color=cmap_obj(0.0), linewidth=3.6, label="Referenčná trajektória")
    ax.set_ylabel("x (m)", fontsize=26)
    ax.set_title(f"{est_label} - {traj_title}", fontsize=24)
    ax.legend(fontsize=15)
    ax.grid(True, linewidth=0.4)
    ax.tick_params(labelsize=24)
    ax.tick_params(axis="x", labelbottom=False)

    # --- 5. y vs time ---
    ax = axes[4]
    ax.plot(t_est_rel, matched["y_est"].values, color="#00e000", linewidth=3.6, label=est_label)
    ax.plot(t_gt_rel,  matched["y_gt"].values,  color=cmap_obj(0.0), linewidth=3.6, label="Referenčná trajektória")
    ax.set_xlabel("Čas (s)", fontsize=26)
    ax.set_ylabel("y (m)", fontsize=26)
    ax.legend(fontsize=15)
    ax.grid(True, linewidth=0.4)
    ax.tick_params(labelsize=24)

    if separate:
        fig_xy.tight_layout()
    plt.tight_layout()

    if save_path:
        if separate:
            base, ext = os.path.splitext(save_path)
            ext = ext or ".png"
            # figs_axes[3] is the combined x/y figure
            for i, suffix in enumerate(["", "_err", "_dist"]):
                out = f"{base}{suffix}{ext}"
                figs_axes[i][0].savefig(out, dpi=150, bbox_inches="tight")
                print(f"Saved to: {os.path.abspath(out)}")
            out_xy = f"{base}_xy{ext}"
            fig_xy.savefig(out_xy, dpi=150, bbox_inches="tight")
            print(f"Saved to: {os.path.abspath(out_xy)}")
        else:
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"Saved to: {os.path.abspath(save_path)}")
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
    parser.add_argument(
        "--traj-title",
        metavar="TITLE",
        default="Trajectories (error coloured)",
        help="Title for the trajectory subplot (default: 'Trajectories (error coloured)').",
    )
    parser.add_argument(
        "--est-label",
        metavar="LABEL",
        default="Estimated (colour = error)",
        help="Legend label for the estimated trajectory (default: 'Estimated (colour = error)').",
    )
    parser.add_argument(
        "--colormap",
        metavar="CMAP",
        default="copper",
        help="Matplotlib colormap for the estimated trajectory (default: copper).",
    )
    args = parser.parse_args()

    csv_path = os.path.abspath(args.csv_path)
    if not os.path.isfile(csv_path):
        print(f"ERROR: file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    df = load_csv(csv_path)
    plot(df, csv_path, args.save, separate=args.separate, match_lines=args.match_lines, rotate_deg=args.rotate, traj_title=args.traj_title, est_label=args.est_label, colormap=args.colormap)


if __name__ == "__main__":
    main()