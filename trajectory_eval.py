#!/usr/bin/env python3
"""
Offline Trajectory Error Evaluator — Computes ATE between odometry and ground truth
from a recorded ROS 2 bag file.

Usage:
    python3 trajectory_eval.py <bag_path> [options]

Requires a sourced ROS 2 environment:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

Both topics must publish nav_msgs/Odometry.
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np


# ---------------------------------------------------------------------------
# Bag helpers
# ---------------------------------------------------------------------------

def _detect_storage_id(bag_path: str) -> str:
    if glob.glob(os.path.join(bag_path, "*.db3")):
        return "sqlite3"
    if glob.glob(os.path.join(bag_path, "*.mcap")):
        return "mcap"
    return "sqlite3"  # fallback


def read_poses_from_bag(
    bag_path: str, topics: list[str], pose_stamped_topics: set[str] | None = None
) -> dict[str, list[tuple[int, float, float]]]:
    """
    Open the bag once, read all messages on the given topics, and return:
        { topic: [(timestamp_ns, x, y), ...] }
    timestamp_ns is taken from msg.header.stamp.
    Topics in pose_stamped_topics are read as geometry_msgs/PoseStamped
    (msg.pose.position) rather than nav_msgs/Odometry (msg.pose.pose.position).
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id=_detect_storage_id(bag_path)
    )
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # Validate topics
    missing = [t for t in topics if t not in topic_type_map]
    if missing:
        available = sorted(topic_type_map.keys())
        print("ERROR: The following topics were not found in the bag:", file=sys.stderr)
        for t in missing:
            print(f"  {t}", file=sys.stderr)
        print("Available topics:", file=sys.stderr)
        for t in available:
            print(f"  {t}", file=sys.stderr)
        sys.exit(1)

    # Cache message type objects
    msg_types: dict[str, type] = {}
    for t in topics:
        msg_types[t] = get_message(topic_type_map[t])

    storage_filter = rosbag2_py.StorageFilter(topics=topics)
    reader.set_filter(storage_filter)

    results: dict[str, list[tuple[int, float, float]]] = {t: [] for t in topics}
    pose_stamped_topics = pose_stamped_topics or set()
    # Per-topic state for OptiTrack origin subtraction
    _optitrack_first: dict[str, tuple[float, float, float]] = {}

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        )
        if topic in pose_stamped_topics:
            # OptiTrack Y-up: x→-x, z→y, then subtract first pose and rotate
            import math
            raw_x = -msg.pose.position.x
            raw_y =  msg.pose.position.z
            raw_yaw = math.atan2(
                2.0 * (msg.pose.orientation.w * msg.pose.orientation.z +
                       msg.pose.orientation.x * msg.pose.orientation.y),
                1.0 - 2.0 * (msg.pose.orientation.y ** 2 +
                              msg.pose.orientation.z ** 2)
            )
            if topic not in _optitrack_first:
                _optitrack_first[topic] = (raw_x, raw_y, raw_yaw)
            ox, oy, oyaw = _optitrack_first[topic]
            dx = raw_x - ox
            dy = raw_y - oy
            cx =  math.cos(oyaw) * dx + math.sin(oyaw) * dy
            cy = -math.sin(oyaw) * dx + math.cos(oyaw) * dy
            # Additional 90° CCW rotation (matches odom_to_path)
            x = -cy
            y =  cx
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
        results[topic].append((stamp_ns, x, y))

    return results


# ---------------------------------------------------------------------------
# GT interpolation
# ---------------------------------------------------------------------------

def interpolate_gt_poses(
    gt_poses: list[tuple[int, float, float]],
    max_gap_ns: int,
) -> tuple[list[tuple[int, float, float]], list[tuple[int, int]]]:
    """
    Fill time gaps larger than max_gap_ns in the GT trajectory with linearly
    interpolated poses at ~10 Hz.  Returns (poses, gap_ranges) where gap_ranges
    is a list of (start_ns, end_ns) for each gap that was filled.
    """
    result: list[tuple[int, float, float]] = []
    gap_ranges: list[tuple[int, int]] = []
    total_inserted = 0

    for i, p1 in enumerate(gt_poses):
        result.append(p1)
        if i + 1 >= len(gt_poses):
            break
        p0 = p1
        p1 = gt_poses[i + 1]
        dt = p1[0] - p0[0]
        if dt > max_gap_ns:
            gap_ranges.append((p0[0], p1[0]))
            n_steps = max(1, int(dt / 100_000_000))  # ~10 Hz
            for k in range(1, n_steps):
                alpha = k / n_steps
                t_ns = int(p0[0] + alpha * dt)
                x = p0[1] + alpha * (p1[1] - p0[1])
                y = p0[2] + alpha * (p1[2] - p0[2])
                result.append((t_ns, x, y))
                total_inserted += 1

    result.sort(key=lambda p: p[0])
    print(f"  Interpolated {total_inserted} GT poses to fill {len(gap_ranges)} gap(s) > {max_gap_ns / 1e9:.3f} s")
    return result, gap_ranges


# ---------------------------------------------------------------------------
# Matching strategies
# ---------------------------------------------------------------------------

def match_spatial(
    est_poses: list[tuple[int, float, float]],
    gt_poses: list[tuple[int, float, float]],
    deduplicate: bool,
    gap_ranges: list[tuple[int, int]] | None = None,
) -> list[tuple[int, float, float, float, float, float]]:
    """
    Match estimated poses to the nearest GT pose in 2D (x, y) using a KD-tree.
    When gap_ranges is provided (from interpolate_gt_poses), estimated poses
    whose timestamp falls inside a gap are matched against the full GT (including
    interpolated points), while poses outside gaps are matched against real-only GT.
    Returns rows of (timestamp_ns, x_est, y_est, x_gt, y_gt, error_m).
    """
    from scipy.spatial import KDTree

    gt_arr = np.array([[p[1], p[2]] for p in gt_poses])
    est_arr = np.array([[p[1], p[2]] for p in est_poses])
    tree_all = KDTree(gt_arr)

    # Build a real-only tree if gap_ranges provided
    if gap_ranges:
        gap_set: set[int] = set()
        for idx, gp in enumerate(gt_poses):
            for gs, ge in gap_ranges:
                if gs < gp[0] < ge:
                    gap_set.add(idx)
                    break
        real_indices = [i for i in range(len(gt_poses)) if i not in gap_set]
        real_arr = np.array([[gt_poses[i][1], gt_poses[i][2]] for i in real_indices])
        tree_real = KDTree(real_arr)

        def _in_gap(t_ns: int) -> bool:
            return any(gs <= t_ns <= ge for gs, ge in gap_ranges)
    else:
        tree_real = tree_all
        real_indices = list(range(len(gt_poses)))

        def _in_gap(t_ns: int) -> bool:
            return False

    rows = []

    if not deduplicate:
        for ep in est_poses:
            if _in_gap(ep[0]):
                dist, idx = tree_all.query([ep[1], ep[2]])
                gp = gt_poses[int(idx)]
            else:
                dist, ridx = tree_real.query([ep[1], ep[2]])
                gp = gt_poses[real_indices[int(ridx)]]
            rows.append((ep[0], ep[1], ep[2], gp[0], gp[1], gp[2], float(dist)))
        return rows

    # Deduplicate: each GT pose matched at most once, iterating over estimated poses.
    k = min(len(gt_arr), 20)
    used: set[int] = set()
    fallback_count = 0

    for ep in est_poses:
        if _in_gap(ep[0]):
            active_tree = tree_all
            active_idx_map = list(range(len(gt_poses)))
        else:
            active_tree = tree_real
            active_idx_map = real_indices
        kk = min(len(active_idx_map), k)
        dists, idxs = active_tree.query([ep[1], ep[2]], k=kk)
        # k=1 returns scalars — normalise to arrays
        if kk == 1:
            dists = [float(dists)]
            idxs = [int(idxs)]

        matched = False
        for dist, local_idx in zip(dists, idxs):
            global_idx = active_idx_map[int(local_idx)]
            if global_idx not in used:
                used.add(global_idx)
                gp = gt_poses[global_idx]
                rows.append((ep[0], ep[1], ep[2], gp[0], gp[1], gp[2], float(dist)))
                matched = True
                break

        if not matched:
            # All k candidates were already taken; fall back to nearest regardless.
            dist0, idx0 = active_tree.query([ep[1], ep[2]], k=1)
            global_idx0 = active_idx_map[int(idx0)]
            gp = gt_poses[global_idx0]
            rows.append((ep[0], ep[1], ep[2], gp[0], gp[1], gp[2], float(dist0)))
            fallback_count += 1

    if fallback_count:
        print(
            f"  Warning: {fallback_count} poses fell back to nearest GT (dedup pool "
            f"of k={k} exhausted). Consider a denser GT trajectory.",
            file=sys.stderr,
        )

    return rows


def match_temporal(
    est_poses: list[tuple[int, float, float]],
    gt_poses: list[tuple[int, float, float]],
    max_dt_ns: int,
) -> list[tuple[int, float, float, float, float, float]]:
    """
    Match estimated poses to GT by nearest timestamp.
    Pairs where |t_est - t_gt| > max_dt_ns are dropped.
    Returns rows of (timestamp_ns, x_est, y_est, x_gt, y_gt, error_m).
    """
    gt_times = np.array([p[0] for p in gt_poses], dtype=np.int64)
    sorted_idx = np.argsort(gt_times)
    gt_sorted = [gt_poses[i] for i in sorted_idx]
    gt_times_sorted = gt_times[sorted_idx]

    rows = []
    dropped = 0

    for ep in est_poses:
        t = ep[0]
        idx = int(np.searchsorted(gt_times_sorted, t))

        # Pick the closer of the two bracketing timestamps
        candidates = []
        if idx < len(gt_sorted):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)

        best = min(candidates, key=lambda i: abs(int(gt_times_sorted[i]) - t))
        dt = abs(int(gt_times_sorted[best]) - t)

        if dt > max_dt_ns:
            dropped += 1
            continue

        gp = gt_sorted[best]
        error = float(np.hypot(ep[1] - gp[1], ep[2] - gp[2]))
        rows.append((ep[0], ep[1], ep[2], gp[0], gp[1], gp[2], error))

    if dropped:
        print(
            f"  Warning: {dropped} estimated poses dropped (no GT match within "
            f"{max_dt_ns / 1e9:.3f} s). Adjust --max-dt if needed.",
            file=sys.stderr,
        )

    return rows


# ---------------------------------------------------------------------------
# ATE computation
# ---------------------------------------------------------------------------

def compute_se2_transform(
    rows: list[tuple[int, float, float, float, float, float]],
) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute the optimal SE(2) transform (R, t) that maps estimated poses onto GT
    using SVD (Umeyama method). Returns (R, t).
    """
    est = np.array([[r[1], r[2]] for r in rows])
    gt  = np.array([[r[4], r[5]] for r in rows])

    mu_est = est.mean(axis=0)
    mu_gt  = gt.mean(axis=0)

    est_c = est - mu_est
    gt_c  = gt  - mu_gt

    H = est_c.T @ gt_c
    U, _, Vt = np.linalg.svd(H)
    # Correct reflection
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, d])
    R = Vt.T @ D @ U.T
    t = mu_gt - R @ mu_est
    return R, t


def apply_se2_transform(
    poses: list[tuple[int, float, float]],
    R: np.ndarray,
    t: np.ndarray,
) -> list[tuple[int, float, float]]:
    """Apply SE(2) transform (R, t) to a list of (timestamp_ns, x, y) poses."""
    result = []
    for ts, x, y in poses:
        p = R @ np.array([x, y]) + t
        result.append((ts, float(p[0]), float(p[1])))
    return result


def align_se2(
    rows: list[tuple[int, float, float, float, float, float]],
) -> list[tuple[int, float, float, float, float, float]]:
    """
    Find the optimal SE(2) transform (rotation + translation, no scale) that
    maps estimated poses onto GT poses using SVD (Umeyama method), then
    recompute errors with the aligned estimated poses.
    Returns new rows with updated x_est, y_est, error_m.
    """
    R, t = compute_se2_transform(rows)
    est = np.array([[r[1], r[2]] for r in rows])
    aligned = (R @ est.T).T + t

    new_rows = []
    for i, r in enumerate(rows):
        ax, ay = float(aligned[i, 0]), float(aligned[i, 1])
        err = float(np.hypot(ax - r[4], ay - r[5]))
        new_rows.append((r[0], ax, ay, r[3], r[4], r[5], err))
    return new_rows

def compute_ate(errors: list[float]) -> tuple[float, float, float, float]:
    e = np.array(errors)
    return (
        float(np.sqrt(np.mean(e ** 2))),  # RMSE
        float(np.mean(e)),                # mean
        float(np.std(e)),                 # std dev
        float(np.max(e)),                 # max
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Compute ATE (Absolute Trajectory Error) between an odometry topic "
            "and a ground truth topic stored in a ROS 2 bag file."
        )
    )
    parser.add_argument("bag_path", help="Path to the ROS 2 bag directory")
    parser.add_argument(
        "--odom-topic",
        default="/odometry/filtered",
        metavar="TOPIC",
        help="Estimated odometry topic (nav_msgs/Odometry). Default: /odometry/filtered",
    )
    parser.add_argument(
        "--gt-topic",
        default="/ground_truth_wrapper",
        metavar="TOPIC",
        help="Ground truth topic (nav_msgs/Odometry). Default: /ground_truth_wrapper",
    )
    parser.add_argument(
        "--kobuki",
        action="store_true",
        help=(
            "Use Kobuki/OptiTrack mode: GT topic is /vrpn_mocap/RigidBody_002/pose "
            "(geometry_msgs/PoseStamped). Overrides --gt-topic default."
        ),
    )
    parser.add_argument(
        "--mode",
        choices=["spatial", "temporal"],
        default="spatial",
        help=(
            "Matching mode. 'spatial': nearest-neighbour on 2D position (KD-tree) — "
            "robust when timestamps are on different time bases (e.g. sped-up GT bag). "
            "'temporal': nearest-neighbour on header timestamp. Default: spatial"
        ),
    )
    parser.add_argument(
        "--deduplicate",
        action="store_true",
        help=(
            "[spatial mode only] Each GT pose is matched at most once. "
            "Prevents trajectory loops or dense clusters from pulling many estimated "
            "poses to the same GT point."
        ),
    )
    parser.add_argument(
        "--align",
        action="store_true",
        help=(
            "Align estimated trajectory to GT using an optimal SE(2) transform "
            "(rotation + translation via SVD) before computing ATE."
        ),
    )
    parser.add_argument(
        "--max-dt",
        type=float,
        default=0.1,
        metavar="SECONDS",
        help="[temporal mode only] Max allowed time difference in seconds for a valid match. Default: 0.1",
    )
    parser.add_argument(
        "--interpolate-gt",
        type=float,
        default=None,
        metavar="SECONDS",
        help=(
            "[spatial mode only] Fill GT time gaps larger than SECONDS with linearly "
            "interpolated poses (~10 Hz). Interpolated points are only used to match "
            "estimated poses whose timestamp falls inside the gap."
        ),
    )
    parser.add_argument(
        "--output-csv",
        metavar="CSV_PATH",
        help="Write per-pose results to this CSV file (timestamp_s, x_est, y_est, timestamp_gt_s, x_gt, y_gt, error_m)",
    )
    args = parser.parse_args()

    bag_path = os.path.abspath(args.bag_path)
    if not os.path.isdir(bag_path):
        print(f"ERROR: bag path not found or not a directory: {bag_path}", file=sys.stderr)
        sys.exit(1)

    if args.kobuki and args.gt_topic == "/ground_truth_wrapper":
        args.gt_topic = "/vrpn_mocap/RigidBody_002/pose"

    pose_stamped_topics: set[str] = set()
    if args.kobuki:
        pose_stamped_topics.add(args.gt_topic)

    # --- Print header ---
    print()
    print(f"Bag        : {bag_path}")
    print(f"Odom topic : {args.odom_topic}")
    print(f"GT topic   : {args.gt_topic}")
    mode_label = args.mode
    if args.mode == "spatial" and args.deduplicate:
        mode_label += " (deduplicate ON)"
    print(f"Mode       : {mode_label}")
    print()

    # --- Read bag (single pass) ---
    print("Reading bag...")
    poses = read_poses_from_bag(bag_path, [args.odom_topic, args.gt_topic], pose_stamped_topics)
    est_poses = poses[args.odom_topic]
    gt_poses = poses[args.gt_topic]

    print(f"  Estimated poses : {len(est_poses)}")
    print(f"  GT poses        : {len(gt_poses)}")

    if not est_poses:
        print(f'ERROR: No messages found on topic "{args.odom_topic}"', file=sys.stderr)
        sys.exit(1)
    if not gt_poses:
        print(f'ERROR: No messages found on topic "{args.gt_topic}"', file=sys.stderr)
        sys.exit(1)

    if args.interpolate_gt is not None:
        max_gap_ns = int(args.interpolate_gt * 1_000_000_000)
        print(f"Interpolating GT gaps > {args.interpolate_gt:.3f} s...")
        gt_poses, gap_ranges = interpolate_gt_poses(gt_poses, max_gap_ns)
    else:
        gap_ranges = None

    # --- Match ---
    if args.align and args.mode == "spatial":
        # Iterative ICP-style alignment: match → SE(2) → apply → repeat until convergence.
        MAX_ITER = 50
        CONV_THRESH = 1e-6  # metres RMSE change
        prev_rmse = float("inf")
        rows = []
        print("Aligning estimated trajectory (iterative SE(2) / SVD)...")
        for iteration in range(1, MAX_ITER + 1):
            rows = match_spatial(est_poses, gt_poses, args.deduplicate, gap_ranges)
            if not rows:
                print(
                    "ERROR: No pose pairs matched. Check topic names.",
                    file=sys.stderr,
                )
                sys.exit(1)
            errors_iter = [r[6] for r in rows]
            rmse_iter = float(np.sqrt(np.mean(np.array(errors_iter) ** 2)))
            print(f"  iter {iteration:2d}  RMSE = {rmse_iter:.6f} m")
            if abs(prev_rmse - rmse_iter) < CONV_THRESH:
                print(f"  Converged after {iteration} iteration(s).")
                break
            prev_rmse = rmse_iter
            R, t = compute_se2_transform(rows)
            est_poses = apply_se2_transform(est_poses, R, t)
        else:
            print(f"  Reached max iterations ({MAX_ITER}).")
    else:
        print("Matching poses...")
        if args.mode == "spatial":
            rows = match_spatial(est_poses, gt_poses, args.deduplicate, gap_ranges)
        else:
            max_dt_ns = int(args.max_dt * 1_000_000_000)
            rows = match_temporal(est_poses, gt_poses, max_dt_ns)

        if args.align:
            print("Aligning trajectories (SE(2) / SVD)...")
            rows = align_se2(rows)

    if not rows:
        print(
            "ERROR: No pose pairs matched. Check topic names and, for temporal mode, --max-dt.",
            file=sys.stderr,
        )
        sys.exit(1)

    # --- ATE ---
    errors = [r[6] for r in rows]
    rmse, mean, std, max_e = compute_ate(errors)

    print()
    print("=" * 47)
    print("  Absolute Trajectory Error (ATE) — 2D")
    print("=" * 47)
    print(f"  Matched pairs : {len(rows):>8}")
    print(f"  RMSE          : {rmse:>10.4f} m")
    print(f"  Mean          : {mean:>10.4f} m")
    print(f"  Std dev       : {std:>10.4f} m")
    print(f"  Max           : {max_e:>10.4f} m")
    print("=" * 47)

    # --- Optional CSV ---
    if args.output_csv:
        out_path = os.path.abspath(args.output_csv)
        with open(out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp_s", "x_est", "y_est", "timestamp_gt_s", "x_gt", "y_gt", "error_m"])
            for r in rows:
                writer.writerow([
                    f"{r[0] / 1e9:.9f}",
                    f"{r[1]:.6f}",
                    f"{r[2]:.6f}",
                    f"{r[3] / 1e9:.9f}",
                    f"{r[4]:.6f}",
                    f"{r[5]:.6f}",
                    f"{r[6]:.6f}",
                ])
            # Write all GT poses (NaN for est columns) so the plotter can draw the full GT trajectory
            for gp in gt_poses:
                writer.writerow(["", "", "", f"{gp[0] / 1e9:.9f}", f"{gp[1]:.6f}", f"{gp[2]:.6f}", ""])
        print(f"\nPer-pose results written to: {out_path}")


if __name__ == "__main__":
    main()
