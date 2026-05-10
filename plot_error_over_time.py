#!/usr/bin/env python3
"""
Plot per-pose error over time from multiple trajectory_eval.py output CSVs.

Usage:
    python3 plot_error_over_time.py bag1.csv bag2.csv ... [--labels L1 L2 ...] [--save output.png] [--title TITLE]

Each CSV produces one coloured line on a shared axes. RMSE is shown as a
horizontal dashed line in the matching colour.
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Colour cycle — extend if more than len(COLORS) datasets are passed
COLORS = [
    "tab:orange", "tab:blue", "#C9A800", "#3B1F0A",
    "tab:red", "tab:purple", "tab:brown", "tab:pink", "tab:cyan",
]


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    expected = {"timestamp_s", "error_m"}
    missing = expected - set(df.columns)
    if missing:
        print(f"ERROR: {path} is missing columns: {missing}", file=sys.stderr)
        sys.exit(1)
    return df


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot chyba polôh v čase from multiple trajectory_eval.py CSVs."
    )
    parser.add_argument(
        "csv_paths",
        nargs="+",
        help="One or more result CSV files (one per bag/run).",
    )
    parser.add_argument(
        "--labels",
        nargs="+",
        metavar="LABEL",
        help="Legend labels for each CSV (must match number of csv_paths). "
             "Defaults to the CSV filename without extension.",
    )
    parser.add_argument(
        "--title",
        default="Chyba polôh v čase",
        help="Plot title (default: 'Chyba polôh v čase').",
    )
    parser.add_argument(
        "--save",
        metavar="OUTPUT_PNG",
        help="Save figure to this path instead of displaying interactively.",
    )
    args = parser.parse_args()

    csv_paths = [os.path.abspath(p) for p in args.csv_paths]
    for p in csv_paths:
        if not os.path.isfile(p):
            print(f"ERROR: file not found: {p}", file=sys.stderr)
            sys.exit(1)

    if args.labels:
        if len(args.labels) != len(csv_paths):
            print(
                f"ERROR: --labels count ({len(args.labels)}) must match number of CSVs ({len(csv_paths)}).",
                file=sys.stderr,
            )
            sys.exit(1)
        labels = args.labels
    else:
        labels = [os.path.splitext(os.path.basename(p))[0] for p in csv_paths]

    fig, ax = plt.subplots(figsize=(14, 7))

    for i, (csv_path, label) in enumerate(zip(csv_paths, labels)):
        color = COLORS[i % len(COLORS)]
        df = load_csv(csv_path)
        matched = df[df["error_m"].notna()].copy()

        t = matched["timestamp_s"].values
        t_rel = t - t[0]

        errors = matched["error_m"].values
        rmse = float(np.sqrt(np.mean(errors ** 2)))

        ax.plot(t_rel, errors, color=color, linewidth=1.8, label=f"{label}  (RMSE {rmse:.4f} m)")
        ax.axhline(
            rmse,
            color=color,
            linestyle="--",
            linewidth=2.0,
            alpha=0.75,
        )

    ax.set_xlabel("Čas (s)", fontsize=24)
    ax.set_ylabel("Error (m)", fontsize=24)
    ax.set_title(args.title, fontsize=26)
    ax.legend(fontsize=18)
    ax.grid(True, linewidth=0.4)
    ax.tick_params(labelsize=22)

    plt.tight_layout()

    if args.save:
        save_path = os.path.abspath(args.save)
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved to: {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
