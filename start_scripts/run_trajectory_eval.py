#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--workspace",   default=os.path.expanduser("~/ros2_ws_host"))
parser.add_argument("--bag-name",    required=True)
parser.add_argument("--kobuki",      default="false")
parser.add_argument("--mode",        default="temporal")
parser.add_argument("--save-dir",    default="~/Desktop")
parser.add_argument("--save-prefix", default="")
args = parser.parse_args()

BAG_NAME    = args.bag_name
KOBUKI      = args.kobuki.lower() == "true"
MODE        = args.mode
SAVE_DIR    = os.path.expanduser(args.save_dir)
SAVE_PREFIX = args.save_prefix
if SAVE_PREFIX and not SAVE_PREFIX.endswith("_"):
    SAVE_PREFIX += "_"

WORKSPACE       = os.path.expanduser(args.workspace)
RECORDINGS_BASE = os.path.join(WORKSPACE, "recordings", "output")

# Hardcoded ground-truth lookup
# Columns: (base_name, distance_suffix, gt_bag_relative, cutoff_ts)
GT_LOOKUP = [
    ("Tetragon", "4m", "autopicker/genz/Tetragon_7m", 1777204630),
    ("Tetragon", "7m", "autopicker/genz/Tetragon_7m", 1777204630),
    ("Lambda",   "4m", "autopicker/genz/Lambda_4m",   1777206864),
    ("Lambda",   "7m", "autopicker/genz/Lambda_4m",   1777206864),
    ("Sigma",    "4m", "autopicker/genz/Sigma_4m",    1777213240),
    ("Sigma",    "7m", "autopicker/genz/Sigma_4m",    1777213240),
]


def lookup_gt(bag_name: str) -> tuple[str, int]:
    """Return (gt_bag_absolute_path, cutoff_ts) for the given bag_name.

    Matches by extracting the base trajectory name and distance suffix
    (e.g. 'Tetragon_4m' → base='Tetragon', dist='4m').
    Falls back to ('', 9999999999) if no match is found.
    """
    parts = bag_name.rsplit("_", 1)
    if len(parts) == 2:
        base, dist = parts
    else:
        base, dist = bag_name, ""

    for entry_base, entry_dist, gt_rel, cutoff in GT_LOOKUP:
        if entry_base.lower() == base.lower() and entry_dist == dist:
            return os.path.join(RECORDINGS_BASE, gt_rel), cutoff

    print(f"WARNING: No ground-truth entry found for '{bag_name}'. "
          "Using no GT bag and no cutoff.", file=sys.stderr)
    return "", 9999999999


PIPELINE_OPTIONS = {
    "1": dict(odom_type="csm",  odom_topic="/fusion_odometry",              est_label=r"P$_L$ICP",         colormap="copper"),
    "2": dict(odom_type="genz", odom_topic="/odometry/filtered",            est_label="GenZ-ICP",           colormap="plasma"),   # /genz/odometry
    "3": dict(odom_type="kin",  odom_topic="/kinematic_icp/lidar_odometry", est_label="Kinematic ICP",      colormap="viridis"),
    "4": dict(odom_type="csm",  odom_topic="/odom_icp",                     est_label=r"pôvodný P$_L$ICP",  colormap="cividis"),
}

print("Select odometry pipeline:")
print("  1) CSM")
print("  2) GenZ-ICP + EKF")
print("  3) Kinematic ICP")
print("  4) Pure CSM")
odom_choice = input("Enter choice [1-4]: ").strip()

if odom_choice not in PIPELINE_OPTIONS:
    print("Invalid choice. Exiting.")
    sys.exit(1)

pipeline = PIPELINE_OPTIONS[odom_choice]
odom_type  = pipeline["odom_type"]
odom_topic = pipeline["odom_topic"]
est_label  = pipeline["est_label"]
colormap   = pipeline["colormap"]

traj_title = input("Enter trajectory plot title: ").strip()

os.makedirs(SAVE_DIR, exist_ok=True)
bag_short = "_".join(BAG_NAME.split("_")[:2])
file_stem = f"{SAVE_PREFIX}{odom_type}_{bag_short}_{MODE}"
csv_path  = os.path.join(SAVE_DIR, f"{file_stem}.csv")
save_path = os.path.join(SAVE_DIR, f"{file_stem}.png")

# --- Run trajectory_eval.py ---

trajectory_eval = os.path.join(WORKSPACE, "trajectory_eval.py")

if KOBUKI:
    bag_path = os.path.join(RECORDINGS_BASE, "kobuki", odom_type, BAG_NAME)
    cmd = [
        sys.executable, trajectory_eval,
        bag_path,
        "--align",
        "--kobuki",
        "--mode", MODE,
        "--output-csv", csv_path,
        "--odom-topic", odom_topic,
        "--timestamp-offset", "0.0",
    ]
else:
    bag_path = os.path.join(RECORDINGS_BASE, "autopicker", odom_type, BAG_NAME)
    gt_bag, cutoff_ts = lookup_gt(BAG_NAME)
    cmd = [
        sys.executable, trajectory_eval,
        bag_path,
        "--align",
        "--odom-topic", odom_topic,
        "--output-csv", csv_path,
        "--mode", MODE,
        "--cutoff-timestamp", str(cutoff_ts),
    ]
    if gt_bag:
        cmd += ["--gt-bag", gt_bag]
    # --timestamp-offset -6.5
    # --hybrid-fraction 1 0.09    # 0 = temporal first

subprocess.run(cmd, check=True)

# --- Run plot_trajectory_eval.py ---

plot_eval = os.path.join(WORKSPACE, "plot_trajectory_eval.py")
cmd = [
    sys.executable, plot_eval,
    csv_path,
    "--separate",
    "--rotate", "-180",
    "--traj-title", traj_title,
    "--est-label", est_label,
    "--colormap", colormap,
    "--match-lines",
    "--save", save_path,
]

subprocess.run(cmd, check=True)
