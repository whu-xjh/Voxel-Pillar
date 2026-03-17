import numpy as np
import pandas as pd
from evo.core import metrics
from evo.core.trajectory import PoseTrajectory3D
from evo.core.transformations import quaternion_matrix, quaternion_slerp
import matplotlib.pyplot as plt
import os


def quat2rotm(q):
    return quaternion_matrix(q)[0:3, 0:3]


def lerp(a, b, t):
    return (1.0-t)*a + t*b


def cv_interp(traj, ref_timestamps, *, extrapolate_past_end=False):
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
        t_prev = traj.timestamps[i-1]
        t_next = traj.timestamps[i]
        td = (t_ref - t_prev) / (t_next - t_prev)
        quat_j = quaternion_slerp(est_quat[i-1], est_quat[i], td)
        tran_j = lerp(est_tran[i-1], est_tran[i], td)
        yield np.r_[t_ref, tran_j, quat_j]


def trajectory_interpolation(traj, timestamps):
    poses = np.array(list(cv_interp(traj, timestamps)))
    times = poses[:, 0]
    trans = poses[:, 1:4]
    quats = poses[:, 4:8]
    return PoseTrajectory3D(trans, quats, times, meta=traj.meta)


def load_txt_trajectory(txt_path, t_min=None, t_max=None, t_B_prism=None):
    data = np.loadtxt(txt_path)
    
    t = data[:, 0]
    P = data[:, 1:4]
    Q = data[:, 4:8]
    
    if t_min is not None and t_max is not None:
        idx_intime = [idx for idx in range(len(t)) if t[idx] >= t_min and t[idx] <= t_max]
        t = t[idx_intime]
        P = P[idx_intime, :]
        Q = Q[idx_intime, :]
    
    if t_B_prism is not None:
        for n in range(len(P)):
            P[n, :] = P[n, :] + (quat2rotm(Q[n, :]).dot(t_B_prism.reshape(3, 1))).transpose()
    
    return t, P, Q


def calculate_metric(traj_gtr, traj_est):
    metric = metrics.APE(pose_relation=metrics.PoseRelation.translation_part)
    metric.process_data((traj_gtr, traj_est))
    return float(metric.get_result(ref_name='reference', est_name='estimate').stats['rmse'])


def _set_axes_equal_3d(ax):
    """
    Make 3D plot axes have equal scale so that 1 unit in x/y/z is the same length.
    """
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

    # If available, also set box aspect to 1:1:1 (keeps equal scaling under resize)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass


def plot_trajectory_alignment(traj_gt, traj_est):
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
    ax3d.set_title("3D (equal scale: 1m=1m=1m)")
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
    ax_xz.set_title("Side (XZ) (equal scale)")
    ax_xz.grid(True)
    ax_xz.set_aspect("equal", adjustable="box")

    plt.tight_layout()
    plt.show()


def evaluate_txt_trajectory(
    txt_path,
    gt_path,
    t_B_prism=None,
    min_completeness=90.0,
    visualize=False,
    *,
    align_fraction=1.0,
):
    if not (0.0 < float(align_fraction) <= 1.0):
        raise ValueError(f"align_fraction must be in (0, 1], got {align_fraction}")

    t_gt, P_gt, Q_gt = load_txt_trajectory(gt_path)
    t_est_full, P_est_full, Q_est_full = load_txt_trajectory(txt_path, t_B_prism=t_B_prism)

    t_min = max(t_gt[0], t_est_full[0])
    t_max = min(t_gt[-1], t_est_full[-1])

    t_est, P_est, Q_est = load_txt_trajectory(txt_path, t_min, t_max, t_B_prism)

    valid = []
    for n in range(len(t_est)):
        min_diff = np.min(np.abs(t_gt - t_est[n]))
        if min_diff < 0.05:
            valid.append(n)

    t_valid = t_est[valid]
    P_valid = P_est[valid, :]
    Q_valid = Q_est[valid, :]

    traj_est = PoseTrajectory3D(P_valid, Q_valid, t_valid)
    traj_gt = PoseTrajectory3D(P_gt, Q_gt, t_gt)
    traj_gt_interp = trajectory_interpolation(traj_gt, traj_est.timestamps)

    # Use only the first align_fraction of common data for least-squares alignment
    n_total = traj_est.num_poses
    n_align = int(np.ceil(n_total * float(align_fraction)))
    n_align = max(2, min(n_align, n_total))  # need at least 2, cannot exceed total

    try:
        traj_est.align(traj_gt_interp, n=n_align)
    except TypeError:
        # Backward-compat for evo versions without 'n' parameter
        traj_est.align(traj_gt_interp)

    ate = calculate_metric(traj_gt_interp, traj_est)
    completeness = round((t_max - t_min) / (t_gt[-1] - t_gt[0]) * 100, 2)

    if ate < 20 and completeness < min_completeness:
        ate = float("inf")

    if visualize:
        plot_trajectory_alignment(traj_gt_interp, traj_est)

    results = {
        "ate": ate,
        "completeness": completeness,
        "num_poses": len(t_valid),
        "time_range": (t_min, t_max),
        "align_fraction": float(align_fraction),
        "num_poses_align": int(n_align),
    }

    return results

if __name__ == "__main__":
    pred_path = "../Log/result/Laurel_Cavern_enhanced_pillar.txt"
    gt_path = "/home/whuxjh/data/SubT_MRS/ground_truth/Laurel_Cavern/ground_truth_path.csv"
    t_B_prism = np.array([0, 0, 0])

    # Example: only use first 50% of common data to estimate alignment
    result = evaluate_txt_trajectory(pred_path, gt_path, t_B_prism, visualize=True, align_fraction=0.5)
    result["sequence"] = os.path.splitext(os.path.basename(pred_path))[0]
    result["pred_path"] = pred_path
    result["gt_path"] = gt_path

    results_df = pd.DataFrame([result])
    results_df.to_csv("evaluation_results.csv", index=False)
    print(f"Results saved to evaluation_results.csv")
    print(results_df[["sequence", "ate", "completeness"]])
