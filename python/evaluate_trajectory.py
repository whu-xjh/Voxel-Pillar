#!/usr/bin/env python3
"""
Trajectory Evaluation Script
Evaluates SLAM trajectory estimates against ground truth using EVO library.
Supports both CSV and TXT files with or without headers.

Features:
- Auto-detection of file format (CSV/TXT, with/without header)
- Nanosecond timestamp conversion
- Command-line interface with argument parsing
- Default execution mode for quick evaluation
"""

import numpy as np
from evo.core import metrics
from evo.core.trajectory import PoseTrajectory3D
from evo.core.transformations import quaternion_matrix, quaternion_slerp
import matplotlib.pyplot as plt
import os
import argparse
import sys


def quat2rotm(q):
    """Convert quaternion to rotation matrix."""
    return quaternion_matrix(q)[0:3, 0:3]


def lerp(a, b, t):
    """Linear interpolation."""
    return (1.0 - t) * a + t * b


def cv_interp(traj, ref_timestamps, *, extrapolate_past_end=False):
    """Constant velocity interpolation for trajectory."""
    if hasattr(ref_timestamps, 'timestamps'):
        ref_timestamps = ref_timestamps.timestamps

    ref_timestamps = np.array(ref_timestamps, copy=True)
    est_tran = traj.positions_xyz
    est_quat = traj.orientations_quat_wxyz

    i = 1
    for t_ref in ref_timestamps:
        while i < traj.num_poses and traj.timestamps[i] <= t_ref:
            i += 1
        if not extrapolate_past_end and traj.num_poses <= i:
            break
        t_prev = traj.timestamps[i - 1]
        t_next = traj.timestamps[i]
        td = (t_ref - t_prev) / (t_next - t_prev)
        quat_j = quaternion_slerp(est_quat[i - 1], est_quat[i], td)
        tran_j = lerp(est_tran[i - 1], est_tran[i], td)
        yield np.r_[t_ref, tran_j, quat_j]


def trajectory_interpolation(traj, timestamps):
    """Interpolate trajectory at given timestamps."""
    poses = np.array(list(cv_interp(traj, timestamps)))
    times = poses[:, 0]
    trans = poses[:, 1:4]
    quats = poses[:, 4:8]
    return PoseTrajectory3D(trans, quats, times, meta=traj.meta)


def load_txt_trajectory(txt_path, t_min=None, t_max=None, t_B_prism=None, skip_header=True):
    """
    Load trajectory from TXT/CSV file with auto-detection.

    Args:
        txt_path: Path to trajectory file
        t_min: Minimum timestamp filter
        t_max: Maximum timestamp filter
        t_B_prism: Sensor to body transformation [x, y, z]
        skip_header: Whether to skip first row (header)

    Returns:
        t: Timestamps
        P: Positions (N x 3)
        Q: Quaternions (N x 4) in wxyz format
    """
    # Auto-detect delimiter and header
    try:
        with open(txt_path, 'r') as f:
            first_line = f.readline().strip()
            # Check if first line contains non-numeric characters (header)
            has_header = any(
                not c.replace(',', '').replace('.', '').replace('-', '').replace('+', '').replace('e', '').replace(' ', '').isdigit()
                for c in first_line if c.isalpha() or c in '_,'
            )
            # Detect delimiter: comma if contains multiple commas, else whitespace
            if ',' in first_line and first_line.count(',') >= 6:
                delimiter = ','
            else:
                delimiter = None  # Default to whitespace
    except Exception:
        has_header = False
        delimiter = ','

    # Load data, skipping header if present
    skip_rows = 1 if (has_header and skip_header) else 0
    try:
        data = np.loadtxt(txt_path, skiprows=skip_rows, delimiter=delimiter)
    except ValueError:
        # Fallback: try without delimiter (whitespace)
        data = np.loadtxt(txt_path, skiprows=skip_rows, delimiter=None)

    # Validate column count
    if data.shape[1] != 8:
        raise ValueError(f"Expected 8 columns (timestamp, x, y, z, qx, qy, qz, qw), got {data.shape[1]}")

    # Extract data: timestamp, x, y, z, qx, qy, qz, qw
    t = data[:, 0]
    P = data[:, 1:4]
    Q = data[:, 4:8]

    # Convert quaternion from xyzw to wxyz format
    Q_wxyz = np.column_stack([Q[:, 3], Q[:, 0], Q[:, 1], Q[:, 2]])

    # Detect and convert nanosecond timestamps to seconds
    # Nanosecond timestamps are typically > 1e12 (after year 2000)
    if np.mean(np.abs(t)) > 1e12:
        t = t / 1e9
        print(f"  [Detected nanosecond timestamps, converted to seconds]")

    # Apply time filtering
    if t_min is not None and t_max is not None:
        idx_intime = [idx for idx in range(len(t)) if t[idx] >= t_min and t[idx] <= t_max]
        t = t[idx_intime]
        P = P[idx_intime, :]
        Q_wxyz = Q_wxyz[idx_intime, :]

    # Apply sensor to body transformation
    if t_B_prism is not None:
        for n in range(len(P)):
            P[n, :] = P[n, :] + (quat2rotm(Q_wxyz[n, :]).dot(t_B_prism.reshape(3, 1))).transpose()

    return t, P, Q_wxyz


def calculate_metric(traj_gtr, traj_est):
    """Calculate Absolute Trajectory Error (ATE)."""
    metric = metrics.APE(pose_relation=metrics.PoseRelation.translation_part)
    metric.process_data((traj_gtr, traj_est))
    return float(metric.get_result(ref_name='reference', est_name='estimate').stats['rmse'])


def _set_axes_equal_3d(ax):
    """Make 3D plot axes have equal scale (1 unit in x/y/z is the same length)."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass


def plot_trajectory_alignment(traj_gt, traj_est, save_path=None):
    """Plot trajectory alignment in multiple views (3D, top-down, side)."""
    fig = plt.figure(figsize=(18, 6))

    # 3D view
    ax3d = fig.add_subplot(131, projection='3d')
    ax3d.plot(
        traj_gt.positions_xyz[:, 0], traj_gt.positions_xyz[:, 1], traj_gt.positions_xyz[:, 2],
        "g-", label="Ground Truth", linewidth=2
    )
    ax3d.plot(
        traj_est.positions_xyz[:, 0], traj_est.positions_xyz[:, 1], traj_est.positions_xyz[:, 2],
        "r--", label="Estimated", linewidth=2
    )
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title("3D (equal scale)")
    ax3d.legend()
    ax3d.grid(True)
    _set_axes_equal_3d(ax3d)

    # Top-down view (XY)
    ax_xy = fig.add_subplot(132)
    ax_xy.plot(
        traj_gt.positions_xyz[:, 0], traj_gt.positions_xyz[:, 1],
        "g-", label="GT", linewidth=2
    )
    ax_xy.plot(
        traj_est.positions_xyz[:, 0], traj_est.positions_xyz[:, 1],
        "r--", label="EST", linewidth=2
    )
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_title("Top-down (XY)")
    ax_xy.grid(True)
    ax_xy.set_aspect("equal", adjustable="box")

    # Side view (XZ)
    ax_xz = fig.add_subplot(133)
    ax_xz.plot(
        traj_gt.positions_xyz[:, 0], traj_gt.positions_xyz[:, 2],
        "g-", label="GT", linewidth=2
    )
    ax_xz.plot(
        traj_est.positions_xyz[:, 0], traj_est.positions_xyz[:, 2],
        "r--", label="EST", linewidth=2
    )
    ax_xz.set_xlabel("X (m)")
    ax_xz.set_ylabel("Z (m)")
    ax_xz.set_title("Side (XZ)")
    ax_xz.grid(True)
    ax_xz.set_aspect("equal", adjustable="box")

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  [Plot saved to {save_path}]")
        plt.close()
    else:
        plt.show()


def evaluate_txt_trajectory(
    txt_path,
    gt_path,
    t_B_prism=None,
    min_completeness=90.0,
    visualize=False,
    plot_save_path=None,
    *,
    align_fraction=1.0,
):
    """
    Evaluate trajectory against ground truth.

    Args:
        txt_path: Path to estimated trajectory file
        gt_path: Path to ground truth file
        t_B_prism: Sensor to body transformation [x, y, z]
        min_completeness: Minimum completeness percentage required
        visualize: Whether to show visualization
        plot_save_path: Path to save plot (if None, shows interactive plot)
        align_fraction: Fraction of trajectory to use for alignment (0-1]

    Returns:
        Dictionary containing evaluation results
    """
    # Validate align_fraction
    if not (0.0 < float(align_fraction) <= 1.0):
        raise ValueError(f"align_fraction must be in (0, 1], got {align_fraction}")

    print(f"Loading ground truth: {gt_path}")
    t_gt, P_gt, Q_gt = load_txt_trajectory(gt_path)

    print(f"Loading estimated trajectory: {txt_path}")
    t_est_full, P_est_full, Q_est_full = load_txt_trajectory(txt_path, t_B_prism=t_B_prism)

    # Find time overlap
    t_min = max(t_gt[0], t_est_full[0])
    t_max = min(t_gt[-1], t_est_full[-1])

    t_est, P_est, Q_est = load_txt_trajectory(txt_path, t_min, t_max, t_B_prism)

    # Find valid poses (timestamp within 50ms of ground truth)
    valid = []
    for n in range(len(t_est)):
        min_diff = np.min(np.abs(t_gt - t_est[n]))
        if min_diff < 0.05:
            valid.append(n)

    t_valid = t_est[valid]
    P_valid = P_est[valid, :]
    Q_valid = Q_est[valid, :]

    print(f"  Valid poses: {len(t_valid)}/{len(t_est)}")

    traj_est = PoseTrajectory3D(P_valid, Q_valid, t_valid)
    traj_gt = PoseTrajectory3D(P_gt, Q_gt, t_gt)
    traj_gt_interp = trajectory_interpolation(traj_gt, traj_est.timestamps)

    # Ensure both trajectories have matching timestamps after interpolation
    if traj_gt_interp.num_poses != traj_est.num_poses:
        valid_indices = []
        gt_timestamps = traj_gt_interp.timestamps
        for i, ts in enumerate(traj_est.timestamps):
            if ts in gt_timestamps:
                valid_indices.append(i)
        traj_est = PoseTrajectory3D(
            traj_est.positions_xyz[valid_indices],
            traj_est.orientations_quat_wxyz[valid_indices],
            traj_est.timestamps[valid_indices]
        )

    # Calculate alignment poses
    n_total = traj_est.num_poses
    n_align = int(np.ceil(n_total * float(align_fraction)))
    n_align = max(2, min(n_align, n_total))

    print(f"  Aligning using first {n_align}/{n_total} poses ({align_fraction*100:.0f}%)")

    # Align trajectories
    try:
        traj_est.align(traj_gt_interp, n=n_align)
    except TypeError:
        traj_est.align(traj_gt_interp)

    # Re-interpolate GT after alignment
    traj_gt_interp = trajectory_interpolation(traj_gt, traj_est.timestamps)

    # Calculate metrics
    ate = calculate_metric(traj_gt_interp, traj_est)
    completeness = round((t_max - t_min) / (t_gt[-1] - t_gt[0]) * 100, 2)

    # Apply completeness threshold
    if ate < 20 and completeness < min_completeness:
        ate = float("inf")

    # Generate visualization
    if visualize or plot_save_path:
        plot_trajectory_alignment(traj_gt_interp, traj_est, save_path=plot_save_path)

    results = {
        "ate": ate,
        "completeness": completeness,
        "num_poses": len(t_valid),
        "time_range": (t_min, t_max),
        "align_fraction": float(align_fraction),
        "num_poses_align": int(n_align),
    }

    return results


def print_results(result):
    """Print evaluation results in formatted output."""
    print("\n" + "=" * 50)
    print("EVALUATION RESULTS")
    print("=" * 50)
    print(f"  Sequence      : {result.get('sequence', 'N/A')}")
    print(f"  ATE (RMSE)    : {result['ate']:.4f} m")
    print(f"  Completeness  : {result['completeness']:.2f}%")
    print(f"  Valid poses   : {result['num_poses']}")
    print(f"  Time range    : {result['time_range'][0]:.2f} - {result['time_range'][1]:.2f} s")
    print("=" * 50)


def find_default_files():
    """Search for default trajectory and ground truth files."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    catkin_ws = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))

    # Common result directories
    result_dirs = [
        os.path.join(catkin_ws, "Log", "result"),
        os.path.join(script_dir, "..", "Log", "result"),
    ]

    # Common ground truth directories
    gt_dirs = [
        os.path.join(script_dir, "ground_truth"),
        os.path.join(script_dir, "..", "ground_truth"),
    ]

    # Find trajectory files
    pred_path = None
    gt_path = None

    for result_dir in result_dirs:
        if os.path.exists(result_dir):
            for file in os.listdir(result_dir):
                if file.endswith('.txt') and not file.startswith('.'):
                    pred_path = os.path.join(result_dir, file)
                    break
            if pred_path:
                break

    # Find ground truth file matching prediction
    if pred_path:
        pred_name = os.path.splitext(os.path.basename(pred_path))[0]
        for gt_dir in gt_dirs:
            if os.path.exists(gt_dir):
                for file in os.listdir(gt_dir):
                    if pred_name in file or file.endswith('.csv') or file.endswith('.txt'):
                        gt_path = os.path.join(gt_dir, file)
                        break
                if gt_path:
                    break

    return pred_path, gt_path


def main():
    """Main entry point with command-line argument parsing."""
    parser = argparse.ArgumentParser(
        description="Evaluate SLAM trajectory against ground truth",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # With command-line arguments
  python %(prog)s --pred result.txt --gt ground_truth.csv

  # With visualization
  python %(prog)s --pred result.txt --gt ground_truth.csv --visualize

  # Custom alignment fraction
  python %(prog)s --pred result.txt --gt ground_truth.csv --align-fraction 0.5

  # No arguments: auto-detect and evaluate default files
  python %(prog)s
        """
    )
    parser.add_argument("--pred", help="Path to predicted trajectory file")
    parser.add_argument("--gt", help="Path to ground truth file")
    parser.add_argument("--t-b-prism", nargs=3, type=float, default=[0, 0, 0],
                       help="Sensor to body transformation (x y z), default: 0 0 0")
    parser.add_argument("--align-fraction", type=float, default=1.0,
                       help="Fraction of trajectory for alignment (0-1], default: 1.0")
    parser.add_argument("--min-completeness", type=float, default=90.0,
                       help="Minimum completeness percentage, default: 90.0")
    parser.add_argument("--visualize", action="store_true",
                       help="Show interactive visualization plots")
    parser.add_argument("--save-plot", type=str, default=None,
                       help="Path to save visualization plot (PNG)")

    args = parser.parse_args()

    # Auto-detect files if not provided
    pred_path = args.pred
    gt_path = args.gt

    if pred_path is None or gt_path is None:
        print("[No arguments provided - searching for default files...]")
        auto_pred, auto_gt = find_default_files()
        if auto_pred and auto_gt:
            pred_path = auto_pred if args.pred is None else args.pred
            gt_path = auto_gt if args.gt is None else args.gt
            print(f"  [Auto-detected: {pred_path}]")
            print(f"  [Auto-detected: {gt_path}]")
        else:
            parser.print_help()
            print("\n[ERROR] Could not auto-detect trajectory files.")
            print("Please specify --pred and --gt arguments.")
            sys.exit(1)

    # Setup plot save path
    plot_save_path = args.save_plot
    if plot_save_path is None and args.visualize is False:
        # Default: save plot in script directory
        sequence_name = os.path.splitext(os.path.basename(pred_path))[0]
        script_dir = os.path.dirname(os.path.abspath(__file__))
        plot_save_path = os.path.join(script_dir, f"{sequence_name}_trajectory.png")

    # Run evaluation
    t_B_prism = np.array(args.t_b_prism)

    result = evaluate_txt_trajectory(
        pred_path,
        gt_path,
        t_B_prism,
        min_completeness=args.min_completeness,
        visualize=args.visualize,
        plot_save_path=plot_save_path,
        align_fraction=args.align_fraction,
    )

    # Add metadata
    result["sequence"] = os.path.splitext(os.path.basename(pred_path))[0]
    result["pred_path"] = pred_path
    result["gt_path"] = gt_path

    # Print results
    print_results(result)

    return 0


if __name__ == "__main__":
    sys.exit(main())
