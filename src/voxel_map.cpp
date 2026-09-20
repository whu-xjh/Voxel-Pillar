
#include "voxel_map.h"
#include <limits>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Sequential id generator for voxel planes (file-local, was a header static)
static int voxel_plane_id = 0;

// Squared intensity measurement noise; overwritten by the online estimator
// (LIVMapper::estimateIntensityNoise) during the init window
double VoxelPlane::intensity_meas_var_ = 1.0;

// EMA alpha for intensity statistics; overwritten from lio/intensity_ema_alpha
double VoxelPlane::intensity_ema_alpha_ = 0.5;

// Calculate point covariance from sensor measurement errors (range and angle)
// Input: point coordinate pb in sensor frame, range error range_inc, angle error degree_inc
// Output: covariance matrix cov in sensor frame
void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov)
{
  if (pb[2] == 0) pb[2] = 0.0001; // Prevent division by zero
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;

  // Build 2x2 diagonal matrix for angular variance in orthogonal directions
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);

  // Build unit direction vector and skew-symmetric matrix
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;

  // Build two orthogonal base vectors
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();

  // Calculate covariance: range error along direction + angle error perpendicular
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config)
{
  nh.param<bool>("publish/pub_plane_en", voxel_config.is_pub_plane_map_, false);

  nh.param<int>("lio/max_layer", voxel_config.max_layer_, 1);
  nh.param<double>("lio/voxel_size", voxel_config.max_voxel_size_, 0.5);
  nh.param<double>("lio/min_eigen_value", voxel_config.planner_threshold_, 0.01);
  nh.param<double>("lio/sigma_num", voxel_config.sigma_num_, 3);
  nh.param<double>("lio/beam_err", voxel_config.beam_err_, 0.02);
  nh.param<double>("lio/dept_err", voxel_config.dept_err_, 0.05);
  nh.param<vector<int>>("lio/layer_init_num", voxel_config.layer_init_num_, vector<int>{5,5,5,5,5});
  nh.param<int>("lio/max_points_num", voxel_config.max_points_num_, 50);
  nh.param<bool>("lio/plane_refine_en", voxel_config.plane_refine_en_, true);
  nh.param<double>("lio/init_distance_threshold", voxel_config.init_distance_threshold_, 0.1);
  nh.param<bool>("lio/plane_valid_check_en", voxel_config.plane_valid_check_en_, true);
  nh.param<int>("lio/valid_check_max_layer", voxel_config.valid_check_max_layer_, 0);
  nh.param<int>("lio/valid_check_min_points_size", voxel_config.valid_check_min_points_size_, 10);
  nh.param<int>("lio/valid_check_resolution", voxel_config.valid_check_resolution_, 5);
  nh.param<double>("lio/valid_check_p_threshold", voxel_config.valid_check_p_threshold_, 0.8);
  if (voxel_config.valid_check_resolution_ < 1) voxel_config.valid_check_resolution_ = 1;  // 0 would degenerate the projection grid
  nh.param<int>("lio/max_iterations", voxel_config.max_iterations_, 5);
  nh.param<int>("lio/capacity", voxel_config.capacity_, 100000);
  nh.param<bool>("lio/intensity_fusion_en", voxel_config.intensity_fusion_en_, false);
  nh.param<bool>("lio/intensity_gate_en", voxel_config.intensity_gate_en_, false);
  nh.param<double>("lio/intensity_gate_k", voxel_config.intensity_gate_k_, 3.0);
  nh.param<double>("lio/intensity_fusion_weight", voxel_config.intensity_fusion_weight_, 0.8);
  nh.param<bool>("lio/degeneracy_adaptive_en", voxel_config.degeneracy_adaptive_en_, false);
  nh.param<double>("lio/degeneracy_on_threshold", voxel_config.degeneracy_on_threshold_, 0.1);
  nh.param<double>("lio/degeneracy_off_threshold", voxel_config.degeneracy_off_threshold_, 0.2);
  nh.param<int>("lio/degeneracy_min_frames", voxel_config.degeneracy_min_frames_, 2);
  double intensity_ema_alpha = 0.5;
  nh.param<double>("lio/intensity_ema_alpha", intensity_ema_alpha, 0.5);
  // Keep alpha in (0, 1]: 0 would freeze the statistics, >1 diverges
  VoxelPlane::intensity_ema_alpha_ = std::min(std::max(intensity_ema_alpha, 1e-3), 1.0);
  nh.param<bool>("lio/intensity_angle_comp_en", voxel_config.intensity_angle_comp_en_, false);
  nh.param<double>("lio/intensity_angle_exp", voxel_config.intensity_angle_exp_, 1.0);
  nh.param<double>("lio/intensity_angle_cos_floor", voxel_config.intensity_angle_cos_floor_, 0.2);
  nh.param<bool>("lio/intensity_angle_debug_en", voxel_config.intensity_angle_debug_en_, false);

  nh.param<bool>("local_map/map_sliding_en", voxel_config.map_sliding_en_, false);
  nh.param<int>("local_map/half_map_size", voxel_config.half_map_size_, 100);
  nh.param<double>("local_map/sliding_thresh", voxel_config.sliding_thresh_, 8);
  }

void loadPillarMapConfig(ros::NodeHandle &nh, PillarMapConfig &config)
{
  nh.param<bool>("pillar_map/pillar_map_en", config.pillar_map_en_, false);
  nh.param<double>("pillar_map/voxel_size", config.voxel_size_, 1.0);
  nh.param<int>("pillar_map/adjacent_redundant_threshold", config.adjacent_redundant_threshold_, 3);
  nh.param<bool>("pillar_map/delete_redundant", config.delete_redundant_, false);
  nh.param<bool>("pillar_map/delete_isolated", config.delete_isolated_, false);
  nh.param<int>("pillar_map/adjacent_isolated_threshold", config.adjacent_isolated_threshold_, 3);
  nh.param<int>("pillar_map/neighbor_ring_num", config.neighbor_ring_num_, 1);
  nh.param<bool>("pillar_map/dyn_bridge_display", config.dyn_bridge_display_, false);
  nh.param<int>("pillar_map/dyn_buffer_max_age", config.dyn_buffer_max_age_, 3);
  nh.param<double>("pillar_map/height_consistency_ratio", config.height_consistency_ratio_, 0.25);
  nh.param<int>("pillar_map/min_num", config.min_num_, 5);  // redundant voxel needs > this many points, isolated needs < this
  nh.param<bool>("pillar_map/dyn_detect_en", config.dyn_detect_en_, false);
  nh.param<int>("pillar_map/pillar_buffer", config.pillar_buffer_, 10);
  nh.param<bool>("pillar_map/delete_dyn", config.delete_dyn_, false);
  nh.param<int>("pillar_map/adjacent_dyn_threshold", config.adjacent_dyn_threshold_, 0);
  nh.param<bool>("pillar_map/dyn_cluster_en", config.dyn_cluster_en_, false);
  nh.param<int>("pillar_map/dyn_cluster_min_num", config.dyn_cluster_min_num_, 5);
  nh.param<bool>("pillar_map/dyn_cluster_expansion", config.dyn_cluster_expansion_, false);
  nh.param<bool>("pillar_map/dyn_flat_filter_en", config.dyn_flat_filter_en_, false);
  nh.param<double>("pillar_map/dyn_flat_band", config.dyn_flat_band_, 0.2);

  if (config.dyn_bridge_display_ && config.dyn_buffer_max_age_ <= 0)
    ROS_WARN("[pillar_map] dyn_bridge_display needs dyn_buffer_max_age > 0 (buffer off) — display will be a no-op");
}

namespace
{
// Rotation snapshot consumed by the map-side incidence-angle gain (init_plane
// batch stats and the UpdateOctoTree EMA): refreshed at the BuildVoxelMap /
// UpdateVoxelMap entry from the manager state (frame 1: gravity-aligned init;
// afterwards the previous frame's optimized pose — matching when each path
// actually runs). The residual side (build_single_residual) passes the live
// iteration rotation instead. Map-side paths run only on the sequential
// per-frame pipeline, so no synchronization is needed
Eigen::Matrix3d g_intensity_rot = Eigen::Matrix3d::Identity();

// Incidence-angle gain of the intensity channel: gain = max(|cosθ|, floor)^k
// with θ the angle between the laser beam and the plane normal. Corrected
// intensity = raw / gain, corrected variance = raw variance / gain² — plane
// statistics then express material (normal-incidence reflectivity) instead of
// viewing geometry (Intensity-SLAM-style cosine compensation). Deliberately
// free of the comp switch: callers gate on intensity_angle_comp_en_ so the
// debug dump can record the raw geometric gain with compensation disabled.
// Returns 1.0 (no-op) when the exponent is <= 0, the normal is unset
// (non-plane voxel), or the body-frame beam direction is degenerate
double intensityIncGain(const Eigen::Vector3d &point_b, const Eigen::Vector3d &normal_w,
                        const Eigen::Matrix3d &rot_w, const VoxelMapConfig &config)
{
  const double k = config.intensity_angle_exp_;
  if (k <= 0.0) return 1.0;
  if (normal_w.squaredNorm() < 0.5) return 1.0;  // normal not set (non-plane voxel)
  const double norm_b = point_b.norm();
  if (norm_b < 1e-6) return 1.0;                 // beam direction unavailable
  const Eigen::Vector3d n_b = rot_w.transpose() * normal_w;
  const double cos_inc = std::fabs(n_b.dot(point_b) / norm_b);
  return std::pow(std::max(cos_inc, config.intensity_angle_cos_floor_), k);
}

// Plain PCA over a point set: centroid, covariance, eigen decomposition
// (ascending eigenvalues, col(0) = min-eigenvalue direction). Shared by the
// multi-pass plane fitting in init_plane
void pcaFit(const std::vector<pointWithVar> &points, Eigen::Vector3d &center, Eigen::Matrix3d &cov,
            Eigen::Matrix3d &evecs, Eigen::Vector3d &evals)
{
  center.setZero();
  cov.setZero();
  for (const auto &pv : points)
  {
    cov += pv.point_w * pv.point_w.transpose();
    center += pv.point_w;
  }
  const double inv_n = 1.0 / static_cast<double>(points.size());
  center *= inv_n;
  cov = cov * inv_n - center * center.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
  evecs = saes.eigenvectors();
  evals = saes.eigenvalues();
}
} // namespace

void VoxelOctoTree::init_plane(std::vector<pointWithVar> &points, VoxelPlane *plane)
{
  // 1. Initialize plane parameters
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
  plane->covariance_ = Eigen::Matrix3d::Zero();
  plane->sum_ppt_ = Eigen::Matrix3d::Zero();
  plane->center_ = Eigen::Vector3d::Zero();
  plane->normal_ = Eigen::Vector3d::Zero();
  plane->points_size_ = points.size();
  plane->radius_ = 0;

  // 2. Pass 1: PCA over ALL points for a first plane estimate (ascending
  // eigenvalues; col(0) is the plane normal)
  Eigen::Vector3d center;
  Eigen::Matrix3d cov, evecs;
  Eigen::Vector3d evals;
  if (config_ptr_->plane_refine_en_)
  {
    // 2. Pass 1: PCA over ALL points for a first plane estimate (ascending
    // eigenvalues; col(0) is the plane normal)
    pcaFit(points, center, cov, evecs, evals);
    const Eigen::Vector3d first_normal = evecs.col(0);

    // 3. Outlier removal (R-VoxelMap-style): drop points farther than
    // init_distance_threshold_ from the first fit — the in-place erase releases
    // their storage — then re-fit on the retained set
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&](const pointWithVar &pv) {
                                  return std::abs((pv.point_w - center).dot(first_normal)) >
                                         config_ptr_->init_distance_threshold_;
                                }),
                 points.end());
    plane->points_size_ = points.size();

    // Degenerate guard: too few points left to fit a plane — treat as non-plane
    // (the caller subdivides whatever remains). is_update_ stays false: these
    // voxels are not planes and must not enter the pubVoxelMap plane list
    if (points.size() < 3)
    {
      plane->is_plane_ = false;
      return;
    }

    // 4. Pass 2: PCA over the retained points
    pcaFit(points, center, cov, evecs, evals);
    Eigen::Matrix3f::Index first_evals_min, first_evals_max;
    evals.rowwise().sum().minCoeff(&first_evals_min);
    evals.rowwise().sum().maxCoeff(&first_evals_max);
    int first_evals_mid = 3 - static_cast<int>(first_evals_min) - static_cast<int>(first_evals_max);

    // 5. Coplanar-disjoint-surface guard (ported from R-VoxelMap): on the
    // configured layers, project the retained points onto the fitted plane and
    // keep only the largest 4-connected cluster — smaller clusters belong to
    // different physical surfaces that happen to be coplanar, and are dropped
    if (config_ptr_->plane_valid_check_en_ && layer_ <= config_ptr_->valid_check_max_layer_ &&
        static_cast<int>(points.size()) >= config_ptr_->valid_check_min_points_size_)
    {
      if (!plane_valid_check(points, center, evecs.col(first_evals_max), evecs.col(first_evals_mid)))
      {
        // is_update_ stays false: not a trustworthy plane, must not enter the
        // pubVoxelMap plane list; the caller subdivides the remaining points
        plane->is_plane_ = false;
        return;
      }
      // Re-fit on the pruned set
      pcaFit(points, center, cov, evecs, evals);
      evals.rowwise().sum().minCoeff(&first_evals_min);
      evals.rowwise().sum().maxCoeff(&first_evals_max);
      first_evals_mid = 3 - static_cast<int>(first_evals_min) - static_cast<int>(first_evals_max);
    }
  }
  else
  {
    // Original behavior: single PCA over all points
    pcaFit(points, center, cov, evecs, evals);
  }

  // 6. Write the final fit back; the eigen decomposition and plane_var
  // propagation below run on the filtered (and possibly pruned) set
  plane->center_ = center;
  plane->covariance_ = cov;
  plane->points_size_ = points.size();

  // 7. Batch-initialize intensity statistics only on the first init. On periodic
  // re-inits every point has already been folded into the EMA state by
  // UpdateOctoTree, so overwriting from the recent temp_points_ window would
  // discard the accumulated history. No hard std floor here: the measurement
  // noise term (intensity_meas_var_) is added at the use sites instead.
  // Points are folded in as incidence-angle-corrected reflectivity (raw / gain)
  // so the statistics match the EMA and residual-side semantics; the fitted
  // normal comes from the local pcaFit result (ascending eigenvalues → col(0))
  if (!points.empty() && !plane->intensity_init_)
  {
    const bool batch_comp = config_ptr_->intensity_angle_comp_en_;
    Eigen::Matrix3f::Index fit_min_idx = 0;
    evals.minCoeff(&fit_min_idx);
    const Eigen::Vector3d fit_normal = evecs.col(fit_min_idx);

    double intensity_sum = 0.0;
    for (auto pv : points)
    {
      const double gain = batch_comp ? intensityIncGain(pv.point_b, fit_normal, g_intensity_rot, *config_ptr_) : 1.0;
      intensity_sum += static_cast<double>(pv.intensity) / gain;
    }
    double intensity_mean = intensity_sum / static_cast<double>(plane->points_size_);
    double intensity_variance = 0.0;
    for (auto pv : points)
    {
      const double gain = batch_comp ? intensityIncGain(pv.point_b, fit_normal, g_intensity_rot, *config_ptr_) : 1.0;
      double diff = static_cast<double>(pv.intensity) / gain - intensity_mean;
      intensity_variance += diff * diff;
    }
    plane->mean_intensity_ = intensity_mean;
    plane->intensity_std_ = sqrt(intensity_variance / static_cast<double>(plane->points_size_));
    plane->intensity_obs_count_ = static_cast<int>(points.size());
    plane->intensity_init_ = true;
  }

  // 8. Eigenvalue decomposition to extract plane normal and other parameters
  // Eigenvalues represent variance in three principal directions:
  // - λ1 (max): variance in first principal direction
  // - λ2 (mid): variance in second principal direction
  // - λ3 (min): variance in third principal direction

  // Original code using EigenSolver (general matrix, produces complex results)
  // Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
  // Eigen::Matrix3cd evecs = es.eigenvectors();
  // Eigen::Vector3cd evals = es.eigenvalues();
  // Eigen::Vector3d evalsReal;
  // evalsReal = evals.real();

  // Optimized: SelfAdjointEigenSolver for symmetric covariance matrix
  // - Faster (3-5x speedup for 3x3 matrices)
  // - Guarantees real eigenvalues/vectors (no complex conversion needed)
  // - Better numerical stability for symmetric matrices
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(plane->covariance_);
  Eigen::Matrix3d evecsReal = saes.eigenvectors();
  Eigen::Vector3d evalsReal = saes.eigenvalues();

  // Find indices of min, mid, max eigenvalues
  Eigen::Matrix3f::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  int evalsMid = 3 - evalsMin - evalsMax;

  // Get eigenvectors for min, mid, max eigenvalues
  // - v1: direction of max variance (primary distribution)
  // - v2: direction of mid variance (secondary distribution)
  // - v3: direction of min variance (normal for planar surfaces)

  // Original: evecs.real().col(...)
  // Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
  // Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
  // Eigen::Vector3d evecMax = evecs.real().col(evalsMax);

  Eigen::Vector3d evecMin = evecsReal.col(evalsMin);
  Eigen::Vector3d evecMid = evecsReal.col(evalsMid);
  Eigen::Vector3d evecMax = evecsReal.col(evalsMax);
  Eigen::Matrix3d J_Q;
  J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_;

  // 9. Check if points form a plane based on min eigenvalue
  if (evalsReal(evalsMin) < planner_threshold_)
  {
    // Points form a plane when variance in third direction is significantly smaller
    for (int i = 0; i < points.size(); i++)
    {
      Eigen::Matrix<double, 6, 3> J;
      Eigen::Matrix3d F;
      // Compute F matrix: sensitivity of eigenvalues to point positions
      for (int m = 0; m < 3; m++)
      {
        if (m != (int)evalsMin)
        {
          // Original: (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() + ...)
          // Eigen::Matrix<double, 1, 3> F_m =
          //   (points[i].point_w - plane->center_).transpose() / ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
          //   (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() + evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
        
          Eigen::Matrix<double, 1, 3> F_m =
              (points[i].point_w - plane->center_).transpose() / ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
              (evecsReal.col(m) * evecsReal.col(evalsMin).transpose() + evecsReal.col(evalsMin) * evecsReal.col(m).transpose());
          F.row(m) = F_m;
        }
        else
        {
          Eigen::Matrix<double, 1, 3> F_m;
          F_m << 0, 0, 0;
          F.row(m) = F_m;
        }
      }
      // Compute Jacobian J: sensitivity of plane parameters to point positions
      // Original: evecs.real() * F
      J.block<3, 3>(0, 0) = evecsReal * F;  // Normal sensitivity
      J.block<3, 3>(3, 0) = J_Q;                // Center sensitivity
      // Error propagation: from point covariance to plane parameter covariance
      plane->plane_var_ += J * points[i].var * J.transpose();
    }

    // Set plane properties
    // Original: evecs.real()(i, evalsMin/Mid/Max)
    plane->normal_ << evecsReal(0, evalsMin), evecsReal(1, evalsMin), evecsReal(2, evalsMin);
    plane->y_normal_ << evecsReal(0, evalsMid), evecsReal(1, evalsMid), evecsReal(2, evalsMid);
    plane->x_normal_ << evecsReal(0, evalsMax), evecsReal(1, evalsMax), evecsReal(2, evalsMax);
    plane->min_eigen_value_ = evalsReal(evalsMin);
    plane->mid_eigen_value_ = evalsReal(evalsMid);
    plane->max_eigen_value_ = evalsReal(evalsMax);
    plane->radius_ = sqrt(evalsReal(evalsMax));
    plane->d_ = -(plane->normal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) + plane->normal_(2) * plane->center_(2));
    plane->is_plane_ = true;
    plane->is_update_ = true;
    if (!plane->is_init_)
    {
      plane->id_ = voxel_plane_id;
      voxel_plane_id++;
      plane->is_init_ = true;
    }

    // Incremental statistics for check_and_update (reset after every refit so
    // the cached mean/sum_ppt/cov match the stored points exactly)
    plane->sum_ppt_.setZero();
    for (const auto &pv : points)
    {
      plane->sum_ppt_ += pv.point_w * pv.point_w.transpose();
    }
    plane->points_size_ = points.size();
    plane->cov_need_update_ = false;
  }
  else
  {
    plane->is_update_ = true;
    plane->is_plane_ = false;
  }
}

// R-VoxelMap check_and_update: O(1) rank-1 trial before accepting a new point
// into a plane voxel. Returns false (point rejected, not stored) when the
// insertion would push the min eigenvalue past the planarity threshold; on
// acceptance the incremental statistics and plane parameters are updated in
// place (plane_var_ refresh deferred — cov_need_update_)
bool VoxelOctoTree::check_and_update(const pointWithVar &pv)
{
  VoxelPlane *plane = plane_ptr_;
  const int curr_points_num = plane->points_size_;
  if (curr_points_num < 3) return true;  // too few points to judge planarity: accept

  const Eigen::Vector3d p_vec(pv.point_w[0], pv.point_w[1], pv.point_w[2]);
  const Eigen::Vector3d new_mean = (plane->center_ * curr_points_num + p_vec) / (curr_points_num + 1);
  const Eigen::Matrix3d new_ppt = plane->sum_ppt_ + p_vec * p_vec.transpose();
  const Eigen::Matrix3d new_cov = new_ppt / (curr_points_num + 1) - new_mean * new_mean.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(new_cov);
  if (es.eigenvalues()(0) >= planner_threshold_) return false;  // would break planarity: reject

  // Accept: refresh the cached statistics and plane parameters in place
  plane->center_ = new_mean;
  plane->sum_ppt_ = new_ppt;
  plane->covariance_ = new_cov;
  plane->normal_ = es.eigenvectors().col(0);
  plane->y_normal_ = es.eigenvectors().col(1);
  plane->x_normal_ = plane->y_normal_.cross(plane->normal_);
  plane->min_eigen_value_ = es.eigenvalues()(0);
  plane->mid_eigen_value_ = es.eigenvalues()(1);
  plane->max_eigen_value_ = es.eigenvalues()(2);
  plane->radius_ = sqrt(es.eigenvalues()(2));
  plane->d_ = -plane->normal_.dot(plane->center_);
  plane->points_size_ = curr_points_num + 1;
  plane->cov_need_update_ = true;
  return true;
}

// Coplanar-disjoint-surface guard (ported from R-VoxelMap): project the
// retained points onto the fitted plane, rasterize into a 2D grid of
// resolution = quater_length * 4 / valid_check_resolution, cluster occupied
// grids with 4-connected DFS, and keep only the largest cluster (by point
// count) — smaller clusters belong to different physical surfaces that happen
// to be coplanar. Returns false when even the largest cluster holds less than
// valid_check_p_threshold_ of the points (the plane is untrustworthy and the
// caller treats the voxel as non-planar)
bool VoxelOctoTree::plane_valid_check(std::vector<pointWithVar> &points, const Eigen::Vector3d &center,
                                      const Eigen::Vector3d &x_normal, const Eigen::Vector3d &y_normal)
{
  const double resolution = quater_length_ * 4 / config_ptr_->valid_check_resolution_;

  std::map<std::pair<int, int>, std::vector<size_t>> grid_map;
  for (size_t i = 0; i < points.size(); ++i)
  {
    const Eigen::Vector3d relative_point = points[i].point_w - center;
    double x_coord = relative_point.dot(x_normal);
    double y_coord = relative_point.dot(y_normal);
    double grid_x_f = x_coord / resolution;
    if (x_coord < 0) grid_x_f -= 1;
    double grid_y_f = y_coord / resolution;
    if (y_coord < 0) grid_y_f -= 1;
    grid_map[{static_cast<int>(grid_x_f), static_cast<int>(grid_y_f)}].push_back(i);
  }

  // 4-connected DFS clustering over occupied grids
  std::map<std::pair<int, int>, bool> visited;
  std::vector<std::vector<std::pair<int, int>>> clusters;
  std::function<void(int, int, std::vector<std::pair<int, int>> &)> dfs =
      [&](int x, int y, std::vector<std::pair<int, int>> &cluster) {
        std::pair<int, int> key(x, y);
        if (visited[key] || grid_map.find(key) == grid_map.end()) return;
        visited[key] = true;
        cluster.push_back(key);
        dfs(x + 1, y, cluster);
        dfs(x - 1, y, cluster);
        dfs(x, y + 1, cluster);
        dfs(x, y - 1, cluster);
      };
  for (const auto &grid_pair : grid_map)
  {
    if (!visited[grid_pair.first])
    {
      std::vector<std::pair<int, int>> cluster;
      dfs(grid_pair.first.first, grid_pair.first.second, cluster);
      clusters.push_back(cluster);
    }
  }

  // Largest cluster by point count
  size_t max_cluster_size = 0;
  size_t max_cluster_idx = 0;
  for (size_t i = 0; i < clusters.size(); ++i)
  {
    size_t cluster_size = 0;
    for (const auto &grid_pair : clusters[i]) cluster_size += grid_map[grid_pair].size();
    if (cluster_size > max_cluster_size)
    {
      max_cluster_size = cluster_size;
      max_cluster_idx = i;
    }
  }

  if (static_cast<double>(max_cluster_size) / points.size() <= config_ptr_->valid_check_p_threshold_)
    return false;

  // Keep only the largest cluster: mark its points, erase the rest in place
  std::unordered_set<size_t> keep;
  for (const auto &grid_pair : clusters[max_cluster_idx])
  {
    for (size_t idx : grid_map[grid_pair]) keep.insert(idx);
  }
  size_t write = 0;
  for (size_t i = 0; i < points.size(); ++i)
  {
    if (keep.count(i)) points[write++] = points[i];
  }
  points.resize(write);
  return true;
}

void VoxelOctoTree::init_octo_tree()
{
  if (temp_points_.size() > points_size_threshold_)
  {
    init_plane(temp_points_, plane_ptr_);
    if (plane_ptr_->is_plane_ == true)
    {
      octo_state_ = 0;  // Current voxel is plane, set as leaf node
      // Release memory when the retained (post-filter) point count reaches max
      if (temp_points_.size() >= max_points_num_)
      {
        update_enable_ = false;
        std::vector<pointWithVar>().swap(temp_points_);
        new_points_ = 0;
      }
    }
    else
    {
      octo_state_ = 1;  // Current voxel is non-planar, set as parent node and subdivide
      cut_octo_tree();
    }
    init_octo_ = true;
    new_points_ = 0;
  }
}

// Subdivide non-planar voxel into 8 child voxels for adaptive multi-resolution map
void VoxelOctoTree::cut_octo_tree()
{
  // Stop if max layer reached
  if (layer_ >= max_layer_)
  {
    octo_state_ = 0;
    return;
  }
  // Distribute points to 8 child voxels
  for (size_t i = 0; i < temp_points_.size(); i++)
  {
    // 2*2*2 = 8 child voxels
    int xyz[3] = {0, 0, 0};
    if (temp_points_[i].point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
    if (temp_points_[i].point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
    if (temp_points_[i].point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
    /*
      Index mapping: leafnum = 4*x + 2*y + z
      Index: [0, 1, 2, 3, 4, 5, 6, 7]
      Maps:  [000, 001, 010, 011, 100, 101, 110, 111]
    */
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    // Create child octree node if not exists
    if (leaves_[leafnum] == nullptr)
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planner_threshold_, config_ptr_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      // Compute child voxel center
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;  // Child size is half of parent
    }
    // Add point to corresponding child voxel
    leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
    leaves_[leafnum]->new_points_++;
  }
  // Recursively build octree for child voxels
  for (uint i = 0; i < 8; i++)
  {
    if (leaves_[i] != nullptr)
    {
      if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_)
      {
        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
        if (leaves_[i]->plane_ptr_->is_plane_)
        {
          leaves_[i]->octo_state_ = 0;
          if (leaves_[i]->temp_points_.size() >= leaves_[i]->max_points_num_)
          {
            leaves_[i]->update_enable_ = false;
            std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
            new_points_ = 0;
          }
        }
        else
        {
          leaves_[i]->octo_state_ = 1;
          leaves_[i]->cut_octo_tree();
        }
        leaves_[i]->init_octo_ = true;
        leaves_[i]->new_points_ = 0;
      }
    }
  }
}

// Update octree and insert new point
void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv)
{
  if (!init_octo_)  // OctoTree not initialized, add point directly
  {
    new_points_++;
    temp_points_.push_back(pv);
    if (temp_points_.size() > points_size_threshold_) { init_octo_tree(); }
  }
  else
  {
    if (plane_ptr_->is_plane_) // Current voxel is already a plane, incremental update, periodically update plane parameters
    {
      // EMA update for intensity statistics (always runs, even when update_enable_=false).
      // No hard std floor: the measurement noise term added at the use sites
      // (intensity_meas_var_) keeps the effective variance from collapsing.
      // The folded value is the incidence-angle-corrected reflectivity
      // (raw / gain, gain = 1 when compensation is off) so the plane statistics
      // stay viewing-geometry-invariant and consistent with init_plane / residuals
      {
        const double alpha = VoxelPlane::intensity_ema_alpha_;
        const double gain = config_ptr_->intensity_angle_comp_en_
                                ? intensityIncGain(pv.point_b, plane_ptr_->normal_, g_intensity_rot, *config_ptr_)
                                : 1.0;
        double val = static_cast<double>(pv.intensity) / gain;
        double delta = val - plane_ptr_->mean_intensity_;
        plane_ptr_->mean_intensity_ += alpha * delta;
        double variance = (1.0 - alpha) * plane_ptr_->intensity_std_ * plane_ptr_->intensity_std_ + alpha * delta * delta;
        plane_ptr_->intensity_std_ = sqrt(variance);
        plane_ptr_->intensity_obs_count_++;
      }

      if (update_enable_)
      {
        // R-VoxelMap check_and_update: reject points whose insertion would
        // push the min eigenvalue past the planarity threshold (O(1) rank-1
        // trial; accepted points refresh the incremental plane statistics).
        // Skipped entirely when plane_refine_en_ is off (original behavior)
        if (config_ptr_->plane_refine_en_ && !check_and_update(pv)) return;
        new_points_++;
        temp_points_.push_back(pv);
        if (new_points_ > update_size_threshold_)
        {
          init_plane(temp_points_, plane_ptr_);
          new_points_ = 0;
        }
        if (temp_points_.size() >= max_points_num_) // Reached max points, stop updating
        {
          update_enable_ = false;
          std::vector<pointWithVar>().swap(temp_points_);
          new_points_ = 0;
        }
      }
    }
    else // Current voxel is not a plane, continue subdividing octree
    {
      if (layer_ < max_layer_) // Not reached max layer, can continue subdividing
      {
        int xyz[3] = {0, 0, 0};
        if (pv.point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
        if (pv.point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
        if (pv.point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] != nullptr) { leaves_[leafnum]->UpdateOctoTree(pv); }
        else
        {
          leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planner_threshold_, config_ptr_);
          leaves_[leafnum]->layer_init_num_ = layer_init_num_;
          leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
          leaves_[leafnum]->quater_length_ = quater_length_ / 2;
          leaves_[leafnum]->UpdateOctoTree(pv);
        }
      }
      else // Reached max layer, treat as leaf node
      {
        // EMA update for intensity statistics (always runs).
        // No hard std floor: the measurement noise term added at the use sites
        // (intensity_meas_var_) keeps the effective variance from collapsing.
        // Incidence-angle-corrected reflectivity, same as the plane branch;
        // non-plane voxels have an unset normal → gain 1
        {
          const double alpha = VoxelPlane::intensity_ema_alpha_;
          const double gain = config_ptr_->intensity_angle_comp_en_
                                  ? intensityIncGain(pv.point_b, plane_ptr_->normal_, g_intensity_rot, *config_ptr_)
                                  : 1.0;
          double val = static_cast<double>(pv.intensity) / gain;
          double delta = val - plane_ptr_->mean_intensity_;
          plane_ptr_->mean_intensity_ += alpha * delta;
          double variance = (1.0 - alpha) * plane_ptr_->intensity_std_ * plane_ptr_->intensity_std_ + alpha * delta * delta;
          plane_ptr_->intensity_std_ = sqrt(variance);
          plane_ptr_->intensity_obs_count_++;
        }

        if (update_enable_)
        {
          new_points_++;
          temp_points_.push_back(pv);
          if (new_points_ > update_size_threshold_)
          {
            init_plane(temp_points_, plane_ptr_);
            new_points_ = 0;
          }
          if (temp_points_.size() >= max_points_num_)
          {
            update_enable_ = false;
            std::vector<pointWithVar>().swap(temp_points_);
            new_points_ = 0;
          }
        }
      }
    }
  }
}

VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw)
{
  if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

  int xyz[3] = {0, 0, 0};
  xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
  xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
  xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
  int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

  // printf("leafnum: %d. \n", leafnum);

  return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

VoxelOctoTree *VoxelOctoTree::Insert(const pointWithVar &pv)
{
  if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) || (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_)))
  {
    new_points_++;
    temp_points_.push_back(pv);
    return this;
  }

  if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_))
  {
    int xyz[3] = {0, 0, 0};
    xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] != nullptr) { return leaves_[leafnum]->Insert(pv); }
    else
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planner_threshold_, config_ptr_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      return leaves_[leafnum]->Insert(pv);
    }
  }
  return nullptr;
}

void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
{
  cross_mat_list_.clear();
  cross_mat_list_.reserve(feats_down_size_);
  body_cov_list_.clear();
  body_cov_list_.reserve(feats_down_size_);

  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    if (point_this[2] == 0) { point_this[2] = 0.001; }
    // Compute point cloud covariance matrix in sensor coordinate system
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    body_cov_list_.push_back(var);
    // Transform point cloud to IMU coordinate system
    point_this = extR_ * point_this + extT_;
    // Compute skew-symmetric matrix
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRIX(point_this);
    cross_mat_list_.push_back(point_crossmat);
  }

  vector<pointWithVar>().swap(pv_list_);
  pv_list_.resize(feats_down_size_);

  // Initialize Kalman filter related matrices
  int rematch_num = 0; // Rematch counter
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
  G.setZero(); // Kalman filter gain matrix
  H_T_H.setZero(); // Weighted sum of observation matrix
  I_STATE.setIdentity(); // State identity matrix

  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;

  for (int iterCount = 0; iterCount < config_setting_.max_iterations_; iterCount++)
  {
    double total_residual = 0.0;

    // Transform point cloud from sensor coordinate system to world coordinate system
    PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
    M3D rot_var = state_.cov.block<3, 3>(0, 0);
    M3D t_var = state_.cov.block<3, 3>(3, 3);

    // Compute point cloud covariance
    for (size_t i = 0; i < feats_down_body_->size(); i++)
    {
      // Build pointWithVar structure for each point, including position and covariance in sensor and world coordinate systems
      pointWithVar &pv = pv_list_[i];
      pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
      pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
      pv.intensity = feats_down_body_->points[i].intensity;  // Extract point intensity

      // Compute total covariance of point in world coordinate system: measurement noise + rotation error + translation error
      M3D cov = body_cov_list_[i];
      M3D point_crossmat = cross_mat_list_[i];
      cov = state_.rot_end * cov * state_.rot_end.transpose() + (-point_crossmat) * rot_var * (-point_crossmat.transpose()) + t_var;
      pv.var = cov;
      pv.body_var = body_cov_list_[i];
    }
    ptpl_list_.clear();

    double t1 = omp_get_wtime();
    BuildResidualListOMP(pv_list_, ptpl_list_); // Find corresponding planes for points and compute residuals
    double t2 = omp_get_wtime();

    // Compute total residual
    for (int i = 0; i < ptpl_list_.size(); i++)
    {
      total_residual += fabs(ptpl_list_[i].dis_to_plane_);
    }
    effect_feat_num_ = ptpl_list_.size();
    double current_average_residual = total_residual / effect_feat_num_;

    cout << "[ LIO ] Raw feature num: " << feats_undistort_->size() << ", downsampled feature num:" << feats_down_size_
         << ", effective feature num: " << effect_feat_num_ << ", average residual: " << current_average_residual
         << ", time: " << t2 - t1 << "s"<< endl;

    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
     * ***/
    // Build observation model, initialize observation matrix
    MatrixXd Hsub(effect_feat_num_, 6); // Observation Jacobian matrix
    MatrixXd Hsub_T_R_inv(6, effect_feat_num_); // Weighted observation Jacobian matrix
    VectorXd R_inv(effect_feat_num_); // Inverse covariance matrix of observation noise
    VectorXd meas_vec(effect_feat_num_); // Observation residual vector
    meas_vec.setZero();

    // Compute observation Jacobian matrix and observation noise covariance for each effective feature point
    for (int i = 0; i < effect_feat_num_; i++)
    {
      auto &ptpl = ptpl_list_[i];
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_;
      V3D point_body(ptpl.point_b_);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRIX(point_this);

      /*** get the normal vector of closest surface/corner ***/
      // Compute Jacobian matrix of plane parameters J_nq
      V3D point_world = state_propagat.rot_end * point_this + state_propagat.pos_end;
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_; // Position component
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_; // Normal vector component

      M3D var;
      // V3D normal_b = state_.rot_end.inverse() * ptpl_list_[i].normal_;
      // V3D point_b = ptpl_list_[i].point_b_;
      // double cos_theta = fabs(normal_b.dot(point_b) / point_b.norm());
      // ptpl_list_[i].body_cov_ = ptpl_list_[i].body_cov_ * (1.0 / cos_theta) * (1.0 / cos_theta);

      // point_w cov
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) + (-point_crossmat) * state_propagat.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose();

      // point_w cov (another_version)
      // var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose() +
      //       state_propagat.cov.block<3, 3>(3, 3) - point_crossmat * state_propagat.cov.block<3, 3>(0, 0) * point_crossmat;

      // point_body cov
      // Compute measurement covariance matrix R
      var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose();
      double sigma_l = J_nq * ptpl_list_[i].plane_var_ * J_nq.transpose();
      R_inv(i) = 1.0 / (0.001 + sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);
      // R_inv(i) = 1.0 / (sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);

      /*** calculate the Measuremnt Jacobian matrix H ***/
      // Compute observation Jacobian matrix H
      V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;
    }

    // Kalman filter iterative update
    EKF_stop_flg = false;
    flg_EKF_converged = false;
    /*** Iterative Kalman Filter Update ***/
    MatrixXd K(DIM_STATE, effect_feat_num_);
    // auto &&Hsub_T = Hsub.transpose();
    auto &&HTz = Hsub_T_R_inv * meas_vec;
    // fout_dbg<<"HTz: "<<HTz<<endl;
    H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
    if (config_setting_.degeneracy_adaptive_en_)
    {
      // Degeneracy detection: λ_min/λ_max of the geometric observation
      // information block — near zero means a weakly-constrained direction
      // (e.g. the along-axis direction in corridors and tunnels)
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H_T_H.block<6, 6>(0, 0));
      if (es.eigenvalues()(5) > 1e-10)  // guard: fully unobservable system
        degeneracy_factor_ = es.eigenvalues()(0) / es.eigenvalues()(5);
    }
    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
    MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H.block<DIM_STATE, DIM_STATE>(0, 0) + state_.cov.block<DIM_STATE, DIM_STATE>(0, 0).inverse()).inverse();
    G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
    auto vec = state_propagat - state_;
    VD(DIM_STATE)
    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec.block<DIM_STATE, 1>(0, 0) - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
    int minRow, minCol;

    // Update state (pose)
    state_ += solution;

    // Check convergence
    auto rot_add = solution.block<3, 1>(0, 0);
    auto t_add = solution.block<3, 1>(3, 0);
    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; }

    /*** Rematch Judgement ***/
    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (config_setting_.max_iterations_ - 2)))) { rematch_num++; }

    /*** Convergence Judgements and Covariance Update ***/
    if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == config_setting_.max_iterations_ - 1)))
    {
      /*** Covariance Update ***/
      state_.cov.block<DIM_STATE, DIM_STATE>(0, 0) =
          (I_STATE.block<DIM_STATE, DIM_STATE>(0, 0) - G.block<DIM_STATE, DIM_STATE>(0, 0)) * state_.cov.block<DIM_STATE, DIM_STATE>(0, 0);
      position_last_ = state_.pos_end;

      EKF_stop_flg = true;
    }
    if (EKF_stop_flg) break;
  }

  // Incidence-angle validation dump: per-frame matched (raw intensity, geometric
  // gain) pairs from the last iteration. Scatter intensity vs gain to decide
  // whether to enable compensation: slope ≈ 0 → the sensor intensity is already
  // angle-calibrated (keep comp off); slope ≈ I·k → Lambertian, enable comp.
  // Written after the iteration loop so earlier iterations don't duplicate rows
  if (config_setting_.intensity_angle_debug_en_ && !angle_dbg_gain_.empty())
  {
    std::ofstream fout(DEBUG_FILE_DIR("intensity_angle.txt"), std::ios::app);
    for (size_t i = 0; i < angle_dbg_gain_.size(); ++i)
    {
      fout << angle_dbg_intensity_[i] << " " << angle_dbg_gain_[i] << "\n";
    }
  }

  // Degeneracy-adaptive intensity fusion: frame-end hysteresis state machine
  // on the last iteration's degeneracy factor. The resulting
  // intensity_fusion_active_ governs the NEXT frame's residual association
  // (one-frame lag; degeneracy is a spatially continuous state)
  if (config_setting_.degeneracy_adaptive_en_)
  {
    if (degeneracy_factor_ < config_setting_.degeneracy_on_threshold_)
    {
      degenerate_streak_++;
      normal_streak_ = 0;
    }
    else if (degeneracy_factor_ > config_setting_.degeneracy_off_threshold_)
    {
      normal_streak_++;
      degenerate_streak_ = 0;
    }
    intensity_fusion_active_ = degenerate_streak_ >= config_setting_.degeneracy_min_frames_;
  }
}

void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                     PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR_ * p + extT_) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

// LRU
// Execute only on first system run to build initial voxel map for ICP registration reference
void VoxelMapManager::BuildVoxelMap()
{
  // 1. Parameter initialization phase
  float voxel_size = config_setting_.max_voxel_size_; // Voxel size of the map
  float planer_threshold = config_setting_.planner_threshold_; // Plane feature eigenvalue threshold for plane points
  int max_layer = config_setting_.max_layer_; // Maximum number of layers
  int max_points_num = config_setting_.max_points_num_; // Maximum points per voxel
  std::vector<int> layer_init_num = config_setting_.layer_init_num_; // Initial point threshold for each layer subdivision
  g_intensity_rot = state_.rot_end; // refresh the map-side incidence-angle rotation snapshot

  // 2. Data preparation phase
  // Optimization: pre-allocate input_points capacity to avoid runtime reallocation
  std::vector<pointWithVar> input_points;
  input_points.reserve(feats_down_world_->size());

  for (size_t i = 0; i < feats_down_world_->size(); i++)
  {
    pointWithVar pv;
    pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
    pv.intensity = feats_down_world_->points[i].intensity;  // Extract point intensity
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    pv.point_b = point_this; // beam direction in the sensor frame, consumed by the incidence-angle gain
    M3D var;
    // Compute point covariance matrix in sensor coordinate system
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRIX(point_this);
    // Error propagation to get point covariance matrix in world coordinate system
    var = (state_.rot_end * extR_) * var * (state_.rot_end * extR_).transpose() +
          (-point_crossmat) * state_.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + state_.cov.block<3, 3>(3, 3);
    pv.var = var;
    input_points.push_back(pv);
  }

  // 3. Voxelization phase
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar p_v = input_points[i];

    // Compute voxel coordinates using std::floor for consistent boundary handling
    int64_t loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = static_cast<int64_t>(std::floor(p_v.point_w[j] / voxel_size));
    }
    VoxelLocation position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      // If voxel already exists, add point directly to that voxel
      iter->second->second->temp_points_.push_back(p_v);
      iter->second->second->new_points_++;
    }
    else
    {
      // If voxel does not exist, create new voxel and add point
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold, &config_setting_);
      octo_tree->quater_length_ = voxel_size / 4;
      octo_tree->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      octo_tree->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      octo_tree->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      octo_tree->temp_points_.push_back(p_v);
      octo_tree->new_points_++;
      octo_tree->layer_init_num_ = layer_init_num;

      // Insert new node at head (most recently used)
      voxel_map_cache_.emplace_front(position, octo_tree);
      voxel_map_.insert({position, voxel_map_cache_.begin()});
    }
  }

  // 4. Octree initialization phase, traverse all voxels and initialize octree for each voxel
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter)
  {
    iter->second->second->init_octo_tree();
  }

  enforceCapacity();
}

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)
{
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = config_setting_.layer_init_num_;
  g_intensity_rot = state_.rot_end; // refresh the map-side incidence-angle rotation snapshot

  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar p_v = input_points[i];
    // Compute voxel coordinates using std::floor for consistent boundary handling
    int64_t loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = static_cast<int64_t>(std::floor(p_v.point_w[j] / voxel_size));
    }
    VoxelLocation position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      iter->second->second->UpdateOctoTree(p_v);
      voxel_map_cache_.splice(voxel_map_cache_.begin(), voxel_map_cache_, iter->second); // Update value and move to head
    }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold, &config_setting_);
      octo_tree->quater_length_ = voxel_size / 4;
      octo_tree->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      octo_tree->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      octo_tree->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      octo_tree->temp_points_.push_back(p_v);
      octo_tree->new_points_++;
      octo_tree->layer_init_num_ = layer_init_num;

      // Insert new node at head (most recently used)
      voxel_map_cache_.emplace_front(position, octo_tree);
      voxel_map_.insert({position, voxel_map_cache_.begin()});
    }
  }

  enforceCapacity();
}

// Evict least-recently-updated voxels from the LRU tail until the cache is
// within capacity. No-op when capacity is disabled (<= 1: 0 means off, and 1
// would degenerate into evicting the entry right after inserting it).
size_t VoxelMapManager::enforceCapacity()
{
  if (config_setting_.capacity_ <= 1) return 0; // LRU cache disabled

  size_t evicted = 0;
  while (voxel_map_cache_.size() > config_setting_.capacity_)
  {
    delete voxel_map_cache_.back().second;
    auto last_key = voxel_map_cache_.back().first;
    voxel_map_.erase(last_key);
    voxel_map_cache_.pop_back();
    ++evicted;
  }
  if (evicted > 0)
  {
    evicted_voxel_count_ += evicted;
    std::cout << YELLOW << "[ LRU ]: evicted " << evicted << " voxels, cache size " << voxel_map_cache_.size()
              << ", total evicted " << evicted_voxel_count_ << RESET << std::endl;
  }
  return evicted;
}

// Find corresponding plane for each point and compute point-to-plane distance for ICP registration
void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
{
  int max_layer = config_setting_.max_layer_;
  double voxel_size = config_setting_.max_voxel_size_;
  double sigma_num = config_setting_.sigma_num_;
  // std::mutex mylock;
  ptpl_list.clear();
  if (config_setting_.intensity_angle_debug_en_)
  {
    angle_dbg_intensity_.clear();
    angle_dbg_gain_.clear();
  }
  std::vector<PointToPlane> all_ptpl_list(pv_list.size());
  // NOT vector<bool>: per-index proxy writes inside the parallel loop below
  // would be non-atomic word-level RMW races
  std::vector<uint8_t> useful_ptpl(pv_list.size());
  std::vector<size_t> index(pv_list.size());

  for (size_t i = 0; i < index.size(); ++i)
  {
    index[i] = i;
    useful_ptpl[i] = false;
  }

  // Multi-threaded processing for each point
  #ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM); // Set number of threads to use
    #pragma omp parallel for // Start parallel for loop
  #endif

  // Main loop: find corresponding plane for each point
  for (int i = 0; i < index.size(); i++)
  {
    pointWithVar &pv = pv_list[i]; // Get current point

    if (!skip_list_.empty() && skip_list_[i]) {
      // Skip points that don't need processing
      continue;
    }

    // Compute voxel location using std::floor for consistent boundary handling
    int64_t loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = static_cast<int64_t>(std::floor(pv.point_w[j] / voxel_size));
    }
    VoxelLocation position(loc_xyz[0], loc_xyz[1], loc_xyz[2]); // Create voxel for current point

    // Find corresponding voxel in voxel map
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) // If voxel found
    {
      // VoxelOctoTree *current_octo = iter->second; // Get octree node corresponding to voxel
      VoxelOctoTree *current_octo = iter->second->second; // Get octree node corresponding to voxel

      PointToPlane single_ptpl{}; // Store plane information for current point
      bool is_success = false; // Mark whether plane was successfully found
      double prob = 0; // Store probability value of point to plane

      // Find corresponding plane in octree of current voxel, build point-to-plane residual if found
      build_single_residual(pv, current_octo, 0, is_success, prob, single_ptpl);
      if (!is_success)
      {
        // If no valid plane found in current voxel, check adjacent voxels
        VoxelLocation near_position = position;

        // Helper: single-axis offset, +1/-1 when the point sits in the outer
        // half of its voxel along that axis
        auto calc_offset = [&](double coord, double center, double quater_len) -> int {
          if (coord > center + quater_len) {
            return 1;
          } else if (coord < center - quater_len) {
            return -1;
          }
          return 0;
        };

        near_position.x += calc_offset(loc_xyz[0], current_octo->voxel_center_[0], current_octo->quater_length_);
        near_position.y += calc_offset(loc_xyz[1], current_octo->voxel_center_[1], current_octo->quater_length_);
        near_position.z += calc_offset(loc_xyz[2], current_octo->voxel_center_[2], current_octo->quater_length_);

        // Find plane in adjacent voxels, build residual if found
        auto iter_near = voxel_map_.find(near_position);
        if (iter_near != voxel_map_.end()) { build_single_residual(pv, (*(iter_near->second)).second, 0, is_success, prob, single_ptpl); }
      }

      if (is_success)
      {
        useful_ptpl[i] = true;
        all_ptpl_list[i] = single_ptpl;
      }
      else
      {
        useful_ptpl[i] = false;
      }
    }
  }
  for (size_t i = 0; i < useful_ptpl.size(); i++)
  {
    if (useful_ptpl[i])
    {
      ptpl_list.push_back(all_ptpl_list[i]);
      if (config_setting_.intensity_angle_debug_en_)
      {
        angle_dbg_intensity_.push_back(all_ptpl_list[i].intensity_);
        angle_dbg_gain_.push_back(all_ptpl_list[i].intensity_gain_);
      }
    }
  }
}

void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_success,
                                            double &prob, PointToPlane &single_ptpl)
{
  int max_layer = config_setting_.max_layer_;
  double sigma_num = config_setting_.sigma_num_;

  double radius_k = 3.0;
  Eigen::Vector3d p_w = pv.point_w;
  if (current_octo->plane_ptr_->is_plane_) // Check if current voxel contains valid plane
  {
    VoxelPlane &plane = *current_octo->plane_ptr_;

    float dis_to_plane = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_); // Point-to-plane distance
    float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) + (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                          (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2)); // Squared distance from point to plane center
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane); // Projection distance from point to plane center within plane

    if (range_dis <= radius_k * plane.radius_) // If point is within plane influence range, compute point-to-plane distance (3x plane radius)
    {
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
      J_nq.block<1, 3>(0, 3) = -plane.normal_;
      double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
      sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
      if (dis_to_plane < sigma_num * sqrt(sigma_l))
      {
        // Effective intensity variance: plane spread + squared measurement noise
        // (auto-estimated during the init window; 0 if estimation failed, i.e.
        // intensity noise is not considered). Floored at 1e-3 so it is always
        // strictly positive: every candidate plane is scored with the same 2D
        // likelihood form and none falls back to geometry-only scoring while
        // fusion/gate are enabled.
        // Query-side incidence-angle correction: the query intensity enters as
        // normal-incidence-equivalent reflectivity (raw / gain) to match the
        // plane statistics' semantics; the variance (plane spread + raw-space
        // measurement noise) is scaled by 1/gain² accordingly. Uses the live
        // iteration rotation, unlike the map-side snapshot. Skipped entirely
        // when neither compensation nor the validation dump needs it
        const bool angle_needed = config_setting_.intensity_angle_comp_en_ || config_setting_.intensity_angle_debug_en_;
        const double raw_gain = angle_needed ? intensityIncGain(pv.point_b, plane.normal_, state_.rot_end, config_setting_) : 1.0;
        const double intensity_gain = config_setting_.intensity_angle_comp_en_ ? raw_gain : 1.0;
        double intensity_diff = static_cast<double>(pv.intensity) / intensity_gain - plane.mean_intensity_;
        double sigma_int_sq = std::max(plane.intensity_std_ * plane.intensity_std_ + VoxelPlane::intensity_meas_var_, 1e-3) /
                              (intensity_gain * intensity_gain);

        // Intensity gate: reject associations whose intensity profile is inconsistent
        // with the plane (e.g. dynamic objects in front of static surfaces). Requires
        // mature statistics to be meaningful. On rejection is_success stays false,
        // so the caller falls back to searching neighbor voxels.
        if (config_setting_.intensity_gate_en_ && plane.intensity_obs_count_ >= 20)
        {
          double m_int = intensity_diff / sqrt(sigma_int_sq);
          if (std::fabs(m_int) > config_setting_.intensity_gate_k_) { return; }
        }

        is_success = true;

        double this_prob;
        // Adaptive engage: with degeneracy_adaptive_en, fusion engages only
        // when the previous frame's observation Hessian was degenerate — weak
        // geometry is exactly where independent intensity evidence pays off
        const bool fusion_on = config_setting_.intensity_fusion_en_ &&
                               (!config_setting_.degeneracy_adaptive_en_ || intensity_fusion_active_);
        if (fusion_on)
        {
          // Weighted Mahalanobis scoring: squared normalized distances of the
          // two modalities combined with an explicit trust weight — no 1/√σ
          // prefactor, so tight planes no longer win by construction and the
          // weight genuinely expresses per-modality trust
          double m_geo = dis_to_plane * dis_to_plane / sigma_l;
          double m_int = intensity_diff * intensity_diff / sigma_int_sq;
          const double w_i = config_setting_.intensity_fusion_weight_;
          const double w_g = 1.0 - w_i;
          this_prob = exp(-(w_g * m_geo + w_i * m_int));
        }
        else
        {
          // Geometry-only scoring (intensity fusion disabled)
          this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
        }

        if (this_prob > prob) // When point may match multiple planes, select the one with highest probability
        {
          prob = this_prob;
          pv.normal = plane.normal_;
          single_ptpl.body_cov_ = pv.body_var;
          single_ptpl.point_b_ = pv.point_b;
          single_ptpl.point_w_ = pv.point_w;
          single_ptpl.intensity_ = pv.intensity;  // Pass intensity information
          single_ptpl.intensity_gain_ = raw_gain; // geometric gain, for the validation dump
          single_ptpl.plane_var_ = plane.plane_var_;
          single_ptpl.normal_ = plane.normal_;
          single_ptpl.center_ = plane.center_;
          single_ptpl.d_ = plane.d_;
          single_ptpl.layer_ = current_layer;
          single_ptpl.dis_to_plane_ = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
        }
        return;
      }
      else
      {
        return;
      }
    }
    else
    {
      return;
    }
  }
  else
  {
    if (current_layer < max_layer) // If current layer hasn't reached max layer, continue traversing octree downward
    {
      for (size_t leafnum = 0; leafnum < 8; leafnum++)
      {
        if (current_octo->leaves_[leafnum] != nullptr)
        {
          VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
          build_single_residual(pv, leaf_octo, current_layer + 1, is_success, prob, single_ptpl);
        }
      }
      return;
    }
    else { return; }
  }
}

void VoxelMapManager::pubVoxelMap()
{
  double max_trace = 0.25;
  double pow_num = 0.2;
  ros::Rate loop(500);
  float use_alpha = 0.8;
  visualization_msgs::MarkerArray voxel_plane;
  voxel_plane.markers.reserve(1000000);
  std::vector<VoxelPlane> pub_plane_list;
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++)
  {
    // GetUpdatePlane(iter->second, config_setting_.max_layer_, pub_plane_list);
    GetUpdatePlane(iter->second->second, config_setting_.max_layer_, pub_plane_list);
  }
  for (size_t i = 0; i < pub_plane_list.size(); i++)
  {
    V3D plane_cov = pub_plane_list[i].plane_var_.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) { trace = max_trace; }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
    double alpha;
    if (pub_plane_list[i].is_plane_) { alpha = use_alpha; }
    else { alpha = 0; }
    pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
  }
  voxel_map_pub_.publish(voxel_plane);
  loop.sleep();
}

void VoxelMapManager::GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list)
{
  if (current_octo->layer_ > pub_max_voxel_layer) { return; }
  if (current_octo->plane_ptr_->is_update_) { plane_list.push_back(*current_octo->plane_ptr_); }
  if (current_octo->layer_ < current_octo->max_layer_)
  {
    if (!current_octo->plane_ptr_->is_plane_)
    {
      for (size_t i = 0; i < 8; i++)
      {
        if (current_octo->leaves_[i] != nullptr) { GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list); }
      }
    }
  }
  return;
}

void VoxelMapManager::pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane,
                                     const float alpha, const Eigen::Vector3d rgb)
{
  visualization_msgs::Marker plane;
  plane.header.frame_id = "world";
  plane.header.stamp = ros::Time();
  plane.ns = plane_ns;
  plane.id = single_plane.id_;
  plane.type = visualization_msgs::Marker::CYLINDER;
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = single_plane.center_[0];
  plane.pose.position.y = single_plane.center_[1];
  plane.pose.position.z = single_plane.center_[2];
  geometry_msgs::Quaternion q;
  CalcVectQuation(single_plane.x_normal_, single_plane.y_normal_, single_plane.normal_, q);
  plane.pose.orientation = q;
  plane.scale.x = 3 * sqrt(single_plane.max_eigen_value_);
  plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value_);
  plane.scale.z = 2 * sqrt(single_plane.min_eigen_value_);
  plane.color.a = alpha;
  plane.color.r = rgb(0);
  plane.color.g = rgb(1);
  plane.color.b = rgb(2);
  plane.lifetime = ros::Duration();
  plane_pub.markers.push_back(plane);
}

void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec,
                                      geometry_msgs::Quaternion &q)
{
  Eigen::Matrix3d rot;
  rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
  Eigen::Matrix3d rotation = rot.transpose();
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) { v = vmin; }

  if (v > vmax) { v = vmax; }

  double dr, dg, db;

  if (v < 0.1242)
  {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  }
  else if (v < 0.3747)
  {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  }
  else if (v < 0.6253)
  {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  }
  else if (v < 0.8758)
  {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  }
  else
  {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

void VoxelMapManager::mapSliding()
{
  if((position_last_ - last_slide_position_).norm() < config_setting_.sliding_thresh_)
  {
    std::cout<<YELLOW<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position_).norm()<<RESET<<"\n";
    return;
  }

  //get global id now
  last_slide_position_ = position_last_;
  double t_sliding_start = omp_get_wtime();
  float loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = position_last_[j] / config_setting_.max_voxel_size_;
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  // VoxelLocation position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);//discrete global
  clearMemOutOfMap((int64_t)loc_xyz[0] + config_setting_.half_map_size_, (int64_t)loc_xyz[0] - config_setting_.half_map_size_,
                    (int64_t)loc_xyz[1] + config_setting_.half_map_size_, (int64_t)loc_xyz[1] - config_setting_.half_map_size_,
                    (int64_t)loc_xyz[2] + config_setting_.half_map_size_, (int64_t)loc_xyz[2] - config_setting_.half_map_size_);
  double t_sliding_end = omp_get_wtime();
  std::cout<<YELLOW<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs"<<RESET<<"\n";
  return;
}

void VoxelMapManager::clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min )
{
  int delete_voxel_count = 0;
  for (auto it = voxel_map_.begin(); it != voxel_map_.end(); )
  {
    const VoxelLocation& loc = it->first;
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
    if (should_remove){
      delete it->second->second;

      // Remove from LRU cache (direct deletion via iterator, O(1) time complexity)
      voxel_map_cache_.erase(it->second);

      it = voxel_map_.erase(it);
      delete_voxel_count++;
    } else {
      ++it;
    }
  }
  std::cout<<YELLOW<<"[DEBUG]: Delete "<<delete_voxel_count<<" root voxels"<<RESET<<"\n";
}

// Compute pillar location (fixed elevation direction as z-axis, pillar composed of x,y)
PillarLocation PillarMap::GetPillarLocation(const VoxelLocation &position) const
{
  return PillarLocation(position.x, position.y);
}

void PillarMap::init(const PillarMapConfig &config, double voxel_size)
{
  config_ = config;
  voxel_size_ = voxel_size;
  initNeighborOffsets();
}

void PillarMap::initNeighborOffsets()
{
  ring1_offsets_.clear();
  ring2_offsets_.clear();

  auto addOffset = [](std::vector<VoxelLocation> &offsets, int dx, int dy, int dz) {
    VoxelLocation voxel_offset;
    voxel_offset.x = dx;
    voxel_offset.y = dy;
    voxel_offset.z = dz;
    offsets.push_back(voxel_offset);
  };

  // Ring 1: 6 face neighbors at distance exactly 1 voxel
  addOffset(ring1_offsets_, -1, 0, 0);
  addOffset(ring1_offsets_, 1, 0, 0);
  addOffset(ring1_offsets_, 0, -1, 0);
  addOffset(ring1_offsets_, 0, 1, 0);
  addOffset(ring1_offsets_, 0, 0, -1);
  addOffset(ring1_offsets_, 0, 0, 1);

  // Ring 2: 12 edge neighbors at distance sqrt(2) voxels
  addOffset(ring2_offsets_, -1, -1, 0);
  addOffset(ring2_offsets_, -1, 1, 0);
  addOffset(ring2_offsets_, 1, -1, 0);
  addOffset(ring2_offsets_, 1, 1, 0);
  addOffset(ring2_offsets_, -1, 0, -1);
  addOffset(ring2_offsets_, -1, 0, 1);
  addOffset(ring2_offsets_, 1, 0, -1);
  addOffset(ring2_offsets_, 1, 0, 1);
  addOffset(ring2_offsets_, 0, -1, -1);
  addOffset(ring2_offsets_, 0, -1, 1);
  addOffset(ring2_offsets_, 0, 1, -1);
  addOffset(ring2_offsets_, 0, 1, 1);
}

void PillarMap::setVoxelPointLabels(PillarMapVoxel* voxel, int8_t label)
{
  voxel->is_redundant_voxel_ = (label == LABEL_REDUNDANT);
  voxel->is_isolated_voxel_  = (label == LABEL_ISOLATED);
  for (size_t idx : voxel->point_indices_) {
    if (idx < point_labels_.size()) {
      point_labels_[idx] = label;
    }
  }
}

// pillar_maps must be sorted by z key (done at the end of BuildPillarMap);
// the begin/next/prev/rbegin arithmetic below relies on that order
void PillarMap::updatePillarFlag(const PillarLocation &pillar_key, PillarMapArray &pillar_maps)
{
  // Step 1: Bottom voxel redundant check. Point-count gate: only dense voxels
  // (> config_.min_num_ points) are confirmed redundant. History: the layer
  // directly above (bottom_z + 1) seen in any of the last pillar_buffer
  // frames means the missing close-above is a transient sampling hole
  PillarMapVoxel* bottom_voxel = &pillar_maps.begin()->second;
  bool has_close_above = std::next(pillar_maps.begin()) != pillar_maps.end()
      && (std::next(pillar_maps.begin())->second).center_z_ - bottom_voxel->center_z_ < (voxel_size_ * 2);
  if (!has_close_above) has_close_above = seenInHistory(pillar_key, pillar_maps.begin()->first + 1);
  if (!has_close_above && bottom_voxel->point_count_ > config_.min_num_) {
    bottom_voxel->is_redundant_voxel_ = true;
    voxel_label_count_++;
  }

  // Step 2: Isolated voxel detection, expressed as adjacent-layer occupancy:
  // the top voxel is a candidate when the layer directly below it (z - 1) is
  // empty (key diff to the voxel below >= 2); a middle voxel when BOTH z - 1
  // and z + 1 are empty. Point-count gate: only sparse voxels
  // (< config_.min_num_ points) are confirmed isolated. History: a gap whose
  // intermediate layers were occupied in the window is a transient sampling
  // hole, not real free space
  const int64_t bottom_key = pillar_maps.begin()->first;
  if (pillar_maps.size() == 2)
  {
    const int64_t top_key = std::next(pillar_maps.begin())->first;
    PillarMapVoxel* top_voxel = &std::next(pillar_maps.begin())->second;

    if (top_key - bottom_key >= 2 && top_voxel->point_count_ < config_.min_num_ &&
        !gapSeenInHistory(pillar_key, bottom_key, top_key)) {
      top_voxel->is_isolated_voxel_ = true;
      voxel_label_count_++;
    }
  }
  if (pillar_maps.size() > 2)
  {
    auto pillar_iter = std::next(pillar_maps.begin());
    for (; pillar_iter != std::prev(pillar_maps.end()); ++pillar_iter)
    {
      const int64_t down_key = std::prev(pillar_iter)->first;
      const int64_t this_key = pillar_iter->first;
      const int64_t up_key = std::next(pillar_iter)->first;

      if (this_key - down_key >= 2 && up_key - this_key >= 2 &&
          pillar_iter->second.point_count_ < config_.min_num_ &&
          !gapSeenInHistory(pillar_key, down_key, this_key) &&
          !gapSeenInHistory(pillar_key, this_key, up_key)) {
        pillar_iter->second.is_isolated_voxel_ = true;
        voxel_label_count_++;
      }
    }

    const int64_t top_key = pillar_maps.rbegin()->first;
    const int64_t prev_key = std::next(pillar_maps.rbegin())->first;
    if (top_key - prev_key >= 2 && pillar_maps.rbegin()->second.point_count_ < config_.min_num_ &&
        !gapSeenInHistory(pillar_key, prev_key, top_key)) {
      pillar_maps.rbegin()->second.is_isolated_voxel_ = true;
      voxel_label_count_++;
    }
  }
}

// Scan one neighbor ring around current_pos: count occupied voxels whose
// virtual point is height-consistent with the query (the gate applies to
// dz = 0 neighbors only — dz != 0 neighbors are ungated, their layer offset
// already bounds the height difference). With use_history, slots empty in the
// current frame still count when the history window shows recent occupancy
// there (sampling flicker must not read as isolation). Returns true as soon
// as adjacent_count reaches threshold (early exit)
bool PillarMap::scanNeighborRing(const VoxelLocation &current_pos, const std::vector<VoxelLocation> &offsets,
                                 int threshold, double current_vp_z, double height_threshold, int &adjacent_count,
                                 bool use_history)
{
  for (const auto &voxel_offset : offsets)
  {
    VoxelLocation adjacent_pos = {
      current_pos.x + voxel_offset.x,
      current_pos.y + voxel_offset.y,
      current_pos.z + voxel_offset.z
    };

    auto pillar_iter = pillars_.find(GetPillarLocation(adjacent_pos));
    bool counted = false;
    if (pillar_iter != pillars_.end() && !pillar_iter->second.empty())
    {
      // Pillar voxel array is sorted by z key: binary search for the target layer
      const PillarMapArray &voxels = pillar_iter->second;
      auto voxel_iter = std::lower_bound(voxels.begin(), voxels.end(), adjacent_pos.z,
          [](const std::pair<int64_t, PillarMapVoxel> &entry, int64_t z) { return entry.first < z; });
      if (voxel_iter != voxels.end() && voxel_iter->first == adjacent_pos.z)
      {
        counted = !(voxel_offset.z == 0 &&
                    std::abs(voxel_iter->second.virtual_point_.z() - current_vp_z) > height_threshold);
      }
    }
    if (!counted && use_history &&
        seenInHistory(GetPillarLocation(adjacent_pos), adjacent_pos.z))
    {
      counted = true;  // empty now, but the window shows recent structure there
    }

    if (counted && ++adjacent_count >= threshold) return true;
  }
  return false;
}

// Ring-ordered 3D neighborhood test: ring 1 = 6 face neighbors at distance
// exactly 1 voxel, ring 2 = 12 edge neighbors at distance sqrt(2). Ring 1 is
// probed first; the threshold early-exits inside either ring
bool PillarMap::hasAdjacentVoxel(const VoxelLocation &current_pos, int threshold, double current_vp_z, bool use_history)
{
  if (threshold <= 0) {
    return false;
  }

  const double height_threshold = voxel_size_ * config_.height_consistency_ratio_;
  int adjacent_count = 0;
  if (scanNeighborRing(current_pos, ring1_offsets_, threshold, current_vp_z, height_threshold, adjacent_count, use_history)) {
    return true;
  }
  if (config_.neighbor_ring_num_ >= 2) {
    return scanNeighborRing(current_pos, ring2_offsets_, threshold, current_vp_z, height_threshold, adjacent_count, use_history);
  }
  return false;
}

void PillarMap::BuildPillarMap(const PointCloudXYZI::Ptr &input_cloud)
{
  const size_t num_points = input_cloud->points.size();
  const double inv_voxel_size = 1.0 / voxel_size_;

  point_labels_.assign(num_points, LABEL_NORMAL);
  point_cloud_ptr_ = input_cloud;

  pillars_.reserve(num_points / 2);

  for (size_t i = 0; i < num_points; ++i)
  {
    const PointType& point = input_cloud->points[i];

    VoxelLocation voxel_location;
    voxel_location.x = static_cast<int64_t>(std::floor(point.x * inv_voxel_size));
    voxel_location.y = static_cast<int64_t>(std::floor(point.y * inv_voxel_size));
    voxel_location.z = static_cast<int64_t>(std::floor(point.z * inv_voxel_size));

    PillarLocation pillar_loc = GetPillarLocation(voxel_location);
    int64_t voxel_key = voxel_location.z;

    PillarMapArray& pillar_maps = pillars_[pillar_loc];
    // Pillars typically hold only a few z voxels: linear scan beats a keyed lookup
    PillarMapVoxel* voxel = nullptr;
    for (auto& entry : pillar_maps) {
      if (entry.first == voxel_key) { voxel = &entry.second; break; }
    }
    if (voxel == nullptr) {
      double center_z = (voxel_location.z + 0.5) * voxel_size_;
      pillar_maps.emplace_back(voxel_key, PillarMapVoxel(center_z));
      voxel = &pillar_maps.back().second;
    }

    voxel->point_indices_.push_back(i);

    // Update virtual point via running average
    voxel->point_count_++;
    double inv_count = 1.0 / voxel->point_count_;
    voxel->virtual_point_.x() += (point.x - voxel->virtual_point_.x()) * inv_count;
    voxel->virtual_point_.y() += (point.y - voxel->virtual_point_.y()) * inv_count;
    voxel->virtual_point_.z() += (point.z - voxel->virtual_point_.z()) * inv_count;
  }

  // Sort each pillar's voxels by z key (ascending order was implicit with
  // std::map; updatePillarFlag and hasAdjacentVoxel rely on it)
  for (auto& pillar_entry : pillars_) {
    std::sort(pillar_entry.second.begin(), pillar_entry.second.end(),
              [](const std::pair<int64_t, PillarMapVoxel> &a, const std::pair<int64_t, PillarMapVoxel> &b) {
                return a.first < b.first;
              });
  }
}

// New point detection: a point whose pillar voxel was not occupied in any of
// the last pillar_buffer_ frames is flagged new. Runs right after
// BuildPillarMap — point_is_new_ is independent of point_labels_ (a point can
// be new AND redundant/isolated), so ordering vs. pillarDetection() does not
// matter, only vs. ClearPillarMapVoxels() (this must run first). The first
// pillar_buffer_ frames only accumulate the reference window; detection
// output starts at frame pillar_buffer_ + 1. Runs BEFORE UpdateHistory(),
// so the window consulted here is exactly the last n previous frames.
// No-op when dyn_detect_en_ is false (the history itself is advanced by
// UpdateHistory() every frame — the redundant/isolated checks consume it too).
void PillarMap::DetectNewPoints()
{
  if (!config_.dyn_detect_en_) return;

  const size_t num_points = point_cloud_ptr_ ? point_cloud_ptr_->points.size() : 0;
  point_is_new_.assign(num_points, 0);

  const int n = std::max(config_.pillar_buffer_, 0);
  const bool detection_on = history_frame_count_ >= static_cast<size_t>(n);
  const int adjacent_threshold = config_.adjacent_dyn_threshold_;

  // Flag points of voxels absent from the current window (once the window is
  // full). Sparse-neighborhood confirmation: a voxel surrounded by same-layer
  // occupied neighbors is existing surface, not newly seen — hasAdjacentVoxel
  // answers ">= threshold consistent neighbors", so the candidate is confirmed
  // only when that is false; threshold <= 0 keeps every candidate
  for (auto &pillar_entry : pillars_)
  {
    const int64_t bottom_key = pillar_entry.second.begin()->first;
    for (auto &voxel_entry : pillar_entry.second)
    {
      const PillarMapKey key(pillar_entry.first, voxel_entry.first);
      auto &ev = pillar_evidence_[key];

      // Bottom voxels are supported from below (likely ground): their
      // occupancy always counts as structure
      if (voxel_entry.first == bottom_key)
      {
        ev.static_count++;
        continue;
      }

      // Evidence update runs unconditionally (warm-up, gated voxels — every
      // occupied voxel contributes): window-absent frames accumulate dynamic
      // evidence, in-window frames static evidence
      if (!detection_on || history_counts_.find(key) != history_counts_.end())
      {
        ev.static_count++;  // seen within the window: static evidence
        continue;
      }
      ev.dyn_count++;  // window-absent: dynamic evidence (candidate)

      // Confirmation gate: accumulated dynamic evidence must outweigh static
      if (ev.dyn_count > ev.static_count)
      {
        if (adjacent_threshold > 0)
        {
          const VoxelLocation voxel_pos = {pillar_entry.first.axis1, pillar_entry.first.axis2, voxel_entry.first};
          if (hasAdjacentVoxel(voxel_pos, adjacent_threshold, voxel_entry.second.virtual_point_.z(), /*use_history=*/true))
            continue;
        }

        voxel_entry.second.is_new_voxel_ = true;
        for (const size_t idx : voxel_entry.second.point_indices_)
        {
          point_is_new_[idx] = 1;  // indices < num_points by construction in BuildPillarMap
        }
      }
    }
  }

  // Clustering confirmation (MDetector-style post-processing): scattered
  // candidates that fail to form a cluster are downgraded to normal before
  // anything downstream (publishing, delete_dyn) sees them
  if (config_.dyn_cluster_en_) confirmClusteredNewPoints();

  // Cross-frame dynamic-point buffer: the frame stamp advances FIRST (the
  // aging/prune inside dyn_bridge_insert and the last_frame stamp assigned to
  // re-confirmed voxels both use it), then confirmed voxels enter dyn_buffer_.
  // Runs every frame whenever the buffer has a consumer (expansion rescue or
  // display bridge) — even with zero detections — so stale voxels expire
  // correctly. NOTE: placed after the dyn_detect_en_ gate — the buffer is fed
  // by detection, so dyn_detect_en must be on for it to do anything
  if (config_.dyn_buffer_max_age_ > 0 &&
      (config_.dyn_bridge_display_ || (config_.dyn_cluster_en_ && config_.dyn_cluster_expansion_)))
  {
    dyn_bridge_frame_ = static_cast<int>(history_frame_count_);
    dyn_bridge_insert();
  }
}

// Advance the n-frame occupancy window by one frame: collect this frame's
// occupied voxel keys, insert them, then evict frames beyond the window.
// Runs every frame whenever the pillar map is enabled — regardless of the
// dyn_detect_en_ gate — because the redundant/isolated checks consume
// the same history. Must run AFTER DetectNewPoints() (whose check must see
// the window WITHOUT the current frame) and BEFORE pillarDetection() (whose
// vertical-continuity checks may include it)
// Cross-frame dynamic-point buffer (M-Detector umap-style): confirmed voxels
// enter dyn_buffer_, feeding the clustering rescue sweep (key membership) and,
// when dyn_bridge_display_ is on, the stale-point re-publish on frames where
// detection is missed. The frame stamp advances and aged voxels are pruned
// every frame (even with zero detections) so stale voxels expire correctly
void PillarMap::dyn_bridge_insert()
{
  // The frame stamp is assigned by the caller (DetectNewPoints) before this
  // runs, so the rescue sweep sees a consistent stamp for the whole frame

  for (auto &pillar_entry : pillars_)
  {
    for (auto &voxel_entry : pillar_entry.second)
    {
      if (!voxel_entry.second.is_new_voxel_) continue;
      auto &entry = dyn_buffer_[PillarMapKey(pillar_entry.first, voxel_entry.first)];
      if (config_.dyn_bridge_display_)
      {
        // Points are only needed for the stale-point re-publish; the rescue
        // sweep consumes key membership, so skip the copy when display is off
        entry.points.clear();  // re-confirmed voxel: replace with this frame's points
        for (const size_t idx : voxel_entry.second.point_indices_)
        {
          const auto &pt = point_cloud_ptr_->points[idx];
          entry.points.emplace_back(pt.x, pt.y, pt.z);
        }
      }
      entry.last_frame = dyn_bridge_frame_;
    }
  }

  for (auto it = dyn_buffer_.begin(); it != dyn_buffer_.end();)
  {
    if (dyn_bridge_frame_ - it->second.last_frame > config_.dyn_buffer_max_age_)
      it = dyn_buffer_.erase(it);
    else
      ++it;
  }

  // Cache this frame's buffer components for the bridge publish (the state
  // here is final: inserted, confirmed-marked, and aged). Display-only —
  // the rescue sweep reads the buffer directly and skips this
  if (config_.dyn_bridge_display_) dyn_comps_ = dyn_bridge_components();
}

// Extract connected components (26-adjacency) of the dynamic-point buffer:
// each component carries all member points, its stale subset (voxels not
// confirmed this frame), the current-cloud indices of its fresh points, the
// point centroid, and its live flag
std::vector<PillarMap::DynComponent> PillarMap::dyn_bridge_components()
{
  std::vector<DynComponent> comps;
  if (dyn_buffer_.empty()) return comps;

  // index the buffer, then label connected components over 26-adjacency
  std::unordered_map<PillarMapKey, size_t> key_to_idx;
  std::vector<const PillarMapKey *> keys;
  keys.reserve(dyn_buffer_.size());
  for (const auto &entry : dyn_buffer_)
  {
    key_to_idx.emplace(entry.first, keys.size());
    keys.push_back(&entry.first);
  }

  std::vector<int> comp_id(keys.size(), -1);
  int num_comp = 0;
  std::vector<size_t> stack;
  for (size_t s = 0; s < keys.size(); ++s)
  {
    if (comp_id[s] >= 0) continue;
    comp_id[s] = num_comp;
    stack.push_back(s);
    while (!stack.empty())
    {
      const size_t cur = stack.back();
      stack.pop_back();
      const PillarMapKey &ck = *keys[cur];
      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dz = -1; dz <= 1; ++dz)
          {
            if (!dx && !dy && !dz) continue;
            auto it = key_to_idx.find(PillarMapKey(PillarLocation(ck.pillar.axis1 + dx, ck.pillar.axis2 + dy), ck.z + dz));
            if (it == key_to_idx.end() || comp_id[it->second] >= 0) continue;
            comp_id[it->second] = num_comp;
            stack.push_back(it->second);
          }
        }
      }
    }
    ++num_comp;
  }

  std::vector<std::vector<size_t>> comp_keys(num_comp);
  for (size_t i = 0; i < keys.size(); ++i) comp_keys[comp_id[i]].push_back(i);
  // NOTE: component centroids average fresh AND stale (up to max_age frames
  // old) member points, biasing them toward older positions for intermittently
  // detected fast targets — the association gate absorbs some of this

  comps.resize(num_comp);
  for (int c = 0; c < num_comp; ++c)
  {
    DynComponent &comp = comps[c];
    for (const size_t ki : comp_keys[c])
    {
      const auto &entry = dyn_buffer_.at(*keys[ki]);
      for (const auto &p : entry.points)
      {
        comp.points.push_back(p);
        comp.centroid += p;
      }
      if (entry.last_frame == dyn_bridge_frame_)
      {
        comp.live = true;
      }
      else
      {
        for (const auto &p : entry.points) comp.stale_points.push_back(p);
      }
    }
    if (!comp.points.empty()) comp.centroid /= static_cast<double>(comp.points.size());
  }
  return comps;
}

void PillarMap::UpdateHistory()
{
  history_frame_count_++;

  std::vector<PillarMapKey> current_keys;
  current_keys.reserve(pillars_.size());
  for (const auto &pillar_entry : pillars_)
  {
    for (const auto &voxel_entry : pillar_entry.second)
    {
      current_keys.emplace_back(pillar_entry.first, voxel_entry.first);
    }
  }

  history_frames_.push_back(std::move(current_keys));
  for (const PillarMapKey &key : history_frames_.back())
  {
    history_counts_[key]++;
  }

  const int n = std::max(config_.pillar_buffer_, 0);
  while (history_frames_.size() > static_cast<size_t>(n))
  {
    for (const PillarMapKey &key : history_frames_.front())
    {
      auto it = history_counts_.find(key);
      if (it != history_counts_.end() && --(it->second) <= 0) history_counts_.erase(it);
    }
    history_frames_.pop_front();
  }
}

// History-window occupancy oracle shared by the redundant/isolated checks:
// was the (pillar, z) voxel occupied in any of the last pillar_buffer
// frames?
bool PillarMap::seenInHistory(const PillarLocation &pillar, int64_t z_key) const
{
  return history_counts_.count(PillarMapKey(pillar, z_key)) > 0;
}

// True when any z layer strictly between low_key and high_key was occupied
// within the history window: the vertical gap seen in the current frame is a
// transient sampling hole, not real free space. The probe is capped at 4096
// layers (>= 1 km at any sane voxel size) as insurance against degenerate keys
bool PillarMap::gapSeenInHistory(const PillarLocation &pillar, int64_t low_key, int64_t high_key) const
{
  for (int64_t z = low_key + 1; z < high_key && z - low_key <= 4096; ++z)
  {
    if (seenInHistory(pillar, z)) return true;
  }
  return false;
}

// Clustering confirmation for candidate new points: candidates must form a
// Euclidean cluster of >= dyn_cluster_min_num points (tolerance = one
// voxel) to stay flagged. Scattered candidates — quantization hops of static
// surfaces near voxel boundaries — are downgraded to normal points: they are
// neither published nor deleted downstream. A connected chain of hop voxels
// still clusters (inherent to the approach); dense-neighborhood candidates are
// already suppressed by adjacent_dyn_threshold. Surviving voxels re-mark
// ALL their points: a voxel absent from the history window holds no static
// surface, so its cluster-split points are genuine new observations too, and
// point_is_new_ stays voxel-wise all-or-nothing for DefineSkipPoints retention
void PillarMap::confirmClusteredNewPoints()
{
  if (!point_cloud_ptr_ || point_is_new_.empty()) return;

  // Clamp to >= 2: min 1 would confirm every singleton, making the filter a no-op
  const int min_cluster = std::max(config_.dyn_cluster_min_num_, 2);

  // A. Gather candidate point indices (empty => nothing flagged, e.g. warm-up)
  std::vector<size_t> cand_idx;
  cand_idx.reserve(point_is_new_.size() / 5);
  const auto &cloud = *point_cloud_ptr_;
  for (size_t i = 0; i < point_is_new_.size() && i < cloud.points.size(); ++i)
  {
    if (point_is_new_[i]) cand_idx.push_back(i);
  }
  if (cand_idx.empty()) return;

  // B. Rescue sweep (dyn_cluster_expansion): current points whose voxel sits
  //    in the dynamic-point buffer within dyn_buffer_max_age_ are target
  //    measurements the occupancy window blocked (the target re-occupied its
  //    own recent voxels) — pool them with the candidates for clustering.
  //    Iterating the current voxels (not the whole cloud) keeps this
  //    O(occupied voxels)
  std::vector<size_t> rescue_idx;
  if (config_.dyn_cluster_expansion_ && config_.dyn_buffer_max_age_ > 0 && !dyn_buffer_.empty())
  {
    for (const auto &pillar_entry : pillars_)
    {
      for (const auto &voxel_entry : pillar_entry.second)
      {
        if (voxel_entry.second.is_new_voxel_) continue;  // already a candidate
        const PillarMapKey key(pillar_entry.first, voxel_entry.first);
        if (dyn_buffer_.find(key) == dyn_buffer_.end()) continue;  // not a recent-dynamic voxel
        for (const size_t idx : voxel_entry.second.point_indices_)
          rescue_idx.push_back(idx);
      }
    }
  }

  // Pool smaller than min_cluster: no cluster can pass — downgrade everything
  const int pool_size = static_cast<int>(cand_idx.size() + rescue_idx.size());
  if (pool_size < min_cluster)
  {
    std::fill(point_is_new_.begin(), point_is_new_.end(), 0);
    for (auto &pillar_entry : pillars_)
    {
      for (auto &voxel_entry : pillar_entry.second)
      {
        voxel_entry.second.is_new_voxel_ = false;
      }
    }
    return;
  }

  // C. Euclidean clustering over the pooled cloud (candidates first, then
  // rescued points): tolerance = one voxel, so points of adjacent voxels
  // connect while isolated hops stay singletons
  pcl::PointCloud<pcl::PointXYZ>::Ptr pooled(new pcl::PointCloud<pcl::PointXYZ>());
  pooled->reserve(pool_size);
  for (const size_t idx : cand_idx)
  {
    pcl::PointXYZ p;
    p.x = cloud.points[idx].x;
    p.y = cloud.points[idx].y;
    p.z = cloud.points[idx].z;
    pooled->points.push_back(p);
  }
  for (const size_t idx : rescue_idx)
  {
    pcl::PointXYZ p;
    p.x = cloud.points[idx].x;
    p.y = cloud.points[idx].y;
    p.z = cloud.points[idx].z;
    pooled->points.push_back(p);
  }

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(voxel_size_);
  ec.setMinClusterSize(min_cluster);
  ec.setMaxClusterSize(pool_size);
  ec.setInputCloud(pooled);
  ec.extract(cluster_indices);

  // D. Confirm: every pooled point inside a valid (non-flat) cluster — both
  // this frame's candidates and rescued points — is confirmed; pooled points
  // outside any valid cluster are downgraded
  std::vector<char> confirmed(cand_idx.size() + rescue_idx.size(), 0);
  for (const auto &cluster : cluster_indices)
  {
    if (config_.dyn_flat_filter_en_ &&
        isFlatCluster(*pooled, cluster.indices, config_.dyn_flat_band_))
      continue;
    for (const int local_idx : cluster.indices)
    {
      confirmed[local_idx] = 1;
    }
  }
  for (size_t k = 0; k < cand_idx.size(); ++k)
  {
    if (!confirmed[k]) point_is_new_[cand_idx[k]] = 0;
  }
  for (size_t k = 0; k < rescue_idx.size(); ++k)
  {
    if (confirmed[cand_idx.size() + k]) point_is_new_[rescue_idx[k]] = 1;
  }

  // E. Voxel flags follow their points: voxels without any surviving new
  // point are downgraded; surviving voxels re-mark all their points
  // (all-or-nothing)
  for (auto &pillar_entry : pillars_)
  {
    for (auto &voxel_entry : pillar_entry.second)
    {
      if (!voxel_entry.second.is_new_voxel_) continue;
      bool any_new = false;
      for (const size_t idx : voxel_entry.second.point_indices_)
      {
        if (point_is_new_[idx]) { any_new = true; break; }
      }
      if (any_new)
      {
        for (const size_t idx : voxel_entry.second.point_indices_)
        {
          point_is_new_[idx] = 1;
        }
      }
      else
      {
        voxel_entry.second.is_new_voxel_ = false;
      }
    }
  }
}

// True when the cluster fits in a thin slab along any coordinate axis (per-axis
// extent < band): constant-height layers, ground stripes, axis-aligned wall
// slivers — not moving-object blobs. band <= 0 never triggers (check off)
bool PillarMap::isFlatCluster(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                   const std::vector<int> &indices, double band)
{
  if (indices.empty()) return false;

  const auto &first = cloud.points[indices.front()];
  float x_min = first.x, x_max = first.x;
  float y_min = first.y, y_max = first.y;
  float z_min = first.z, z_max = first.z;
  for (const int idx : indices)
  {
    const auto &p = cloud.points[idx];
    x_min = std::min(x_min, p.x); x_max = std::max(x_max, p.x);
    y_min = std::min(y_min, p.y); y_max = std::max(y_max, p.y);
    z_min = std::min(z_min, p.z); z_max = std::max(z_max, p.z);
  }
  return (x_max - x_min < band) || (y_max - y_min < band) || (z_max - z_min < band);
}

void PillarMap::pillarDetection()
{
  // Step 1: Initial redundant/isolated voxel flag per pillar
  voxel_label_count_ = 0;
  for (auto& pillar_entry : pillars_)
  {
    updatePillarFlag(pillar_entry.first, pillar_entry.second);
  }

  // Early-exit: no candidate voxels flagged in Step 1 — skip the adjacency
  // check and label assignment. point_labels_ stays all-LABEL_NORMAL
  // (set in BuildPillarMap), so DefineSkipPoints and PublishPillarMapCloud will
  // correctly produce empty results.
  if (voxel_label_count_ == 0) {
    ROS_DEBUG("[pillarDetection] Early-exit: no redundant/isolated candidates after Step 1");
    return;
  }

  // Step 2: Adjacency check for all redundant and isolated voxels
  for (auto& pillar_entry : pillars_)
  {
    const PillarLocation& pillar_key = pillar_entry.first;
    auto& pillar_maps = pillar_entry.second;

    for (auto voxel_iter = pillar_maps.begin(); voxel_iter != pillar_maps.end(); ++voxel_iter)
    {
      if (!voxel_iter->second.is_redundant_voxel_ && !voxel_iter->second.is_isolated_voxel_) continue;

      VoxelLocation voxel_loc;
      voxel_loc.x = pillar_key.axis1;
      voxel_loc.y = pillar_key.axis2;
      voxel_loc.z = voxel_iter->first;

      if (voxel_iter->second.is_redundant_voxel_) {
        bool has_adjacent_redundant = hasAdjacentVoxel(voxel_loc, config_.adjacent_redundant_threshold_, voxel_iter->second.virtual_point_.z(), /*use_history=*/true);
        if (!has_adjacent_redundant) {
          voxel_iter->second.is_redundant_voxel_ = false;
        }
      }
      if (voxel_iter->second.is_isolated_voxel_) {
        bool has_adjacent_isolated = hasAdjacentVoxel(voxel_loc, config_.adjacent_isolated_threshold_, voxel_iter->second.virtual_point_.z(), /*use_history=*/true);
        if (has_adjacent_isolated) {
          voxel_iter->second.is_isolated_voxel_ = false;
        }
      }
    }
  }

  // Step 3: Assign point labels for all redundant and isolated voxels
  for (auto& pillar_entry : pillars_)
  {
    for (auto& voxel_pair : pillar_entry.second)
    {
      if (voxel_pair.second.is_redundant_voxel_)
        setVoxelPointLabels(&voxel_pair.second, LABEL_REDUNDANT);
      else if (voxel_pair.second.is_isolated_voxel_)
        setVoxelPointLabels(&voxel_pair.second, LABEL_ISOLATED);
    }
  }
}

// Mark flagged points for deletion in skip_list_ (consumed by
// removeFlaggedPoints): binary per-category switches — delete_redundant /
// delete_isolated / delete_dyn; a point carrying several flags is deleted
// when ANY of its categories says delete. Kept flagged points enter the voxel
// map as usual; /cloud_pillarmap coloring is unaffected (it reads the labels
// before this filter)
void VoxelMapManager::DefineSkipPoints(const PointCloudXYZI::Ptr &feats_down_world)
{
  const size_t point_num = feats_down_world->points.size();
  skip_list_.assign(point_num, false);

  if (point_num == 0) {
    ROS_DEBUG("[DefineSkipPoints] Empty cloud, skip nothing.");
    return;
  }

  const PillarMapConfig &cfg = pillar_map_.config_;
  int isolated_count = 0;
  int redundant_count = 0;
  int new_count = 0;
  int final_skip_count = 0;
  const bool new_detect_on = cfg.dyn_detect_en_;

  // Binary per-category deletion: each flagged category has its own delete
  // switch; a point carrying several flags is deleted when ANY of them says
  // delete (e.g. delete_dyn off + delete_redundant on still removes a point
  // that is both new and redundant)
  for (size_t i = 0; i < point_num; ++i)
  {
    const int8_t label = pillar_map_.GetPointLabel(i);
    bool del = false;
    if (label == LABEL_ISOLATED)
    {
      isolated_count++;
      del = cfg.delete_isolated_;
    }
    else if (label == LABEL_REDUNDANT)
    {
      redundant_count++;
      del = cfg.delete_redundant_;
    }

    if (new_detect_on && pillar_map_.GetPointIsNew(i))
    {
      new_count++;
      del = del || cfg.delete_dyn_;
    }

    if (del)
    {
      skip_list_[i] = true;
      final_skip_count++;
    }
  }

  ROS_DEBUG("[DefineSkipPoints]: Isolated: %d, Redundant: %d, New: %d, Skip_total: %d/%zu (%.1f%%)",
            isolated_count, redundant_count, new_count, final_skip_count, point_num,
            100.0 * final_skip_count / point_num);

  // Update statistics
  current_skip_count_ = final_skip_count;
  total_skip_count_ += final_skip_count;
  total_point_count_ += point_num;
}

// Unified pillar map output: redundant, isolated and new points in ONE RGB
// cloud, so RViz can show all three categories on a single topic. Colors:
// new = red (priority when a point qualifies for several categories),
// isolated = blue, redundant = purple. Same self-gating pattern as before
// (no cloud assembly at all when nobody subscribes). Must run before
// ClearPillarMapVoxels()/removeFlaggedPoints() — the flags index the
// pre-compaction cloud
void PillarMap::PublishPillarMapCloud(const ros::Publisher &pub)
{
  if (pub.getNumSubscribers() == 0 || !point_cloud_ptr_ || point_cloud_ptr_->points.empty())
    return;

  pcl::PointCloud<pcl::PointXYZRGB> pillar_cloud;
  pillar_cloud.points.reserve(point_cloud_ptr_->points.size() / 4);

  const auto &cloud = *point_cloud_ptr_;
  for (size_t i = 0; i < cloud.points.size() && i < point_labels_.size(); ++i)
  {
    const bool is_new = i < point_is_new_.size() && point_is_new_[i] != 0;
    if (!is_new && point_labels_[i] == LABEL_NORMAL) continue;

    pcl::PointXYZRGB p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;
    if (is_new)                                  { p.r = 255; p.g = 0; p.b = 0;   }  // new: red
    else if (point_labels_[i] == LABEL_ISOLATED) { p.r = 0;   p.g = 0; p.b = 255; }  // isolated: blue
    else                                         { p.r = 255; p.g = 0; p.b = 255; }  // redundant: purple
    pillar_cloud.points.push_back(p);
  }

  // Dynamic-point bridge (M-Detector umap-style): stale points of live buffer
  // components keep intermittently detected targets visible. Display-only,
  // red — never re-enter the skip pipeline
  if (config_.dyn_bridge_display_)
  {
    for (const auto &comp : dyn_comps_)
    {
      if (!comp.live) continue;
      for (const auto &p : comp.stale_points)
      {
        pcl::PointXYZRGB q;
        q.x = p(0);
        q.y = p(1);
        q.z = p(2);
        q.r = 255;
        q.g = 0;
        q.b = 0;
        pillar_cloud.points.push_back(q);
      }
    }
  }

  if (pillar_cloud.points.empty()) return;

  pillar_cloud.width = pillar_cloud.points.size();
  pillar_cloud.height = 1;
  pillar_cloud.is_dense = true;

  sensor_msgs::PointCloud2 pillar_msg;
  pcl::toROSMsg(pillar_cloud, pillar_msg);
  pillar_msg.header.stamp = ros::Time::now();
  pillar_msg.header.frame_id = "world";
  pub.publish(pillar_msg);
}

void VoxelMapManager::ClearPillarMapVoxels()
{
  // PillarMapVoxel is a lightweight value type, no manual delete needed.
  // Just clear the containers (whole structure is rebuilt next frame)
  pillar_map_.pillars_.clear();
  pillar_map_.point_labels_.clear();
  // Per-frame new-point flags; the n-frame history window
  // (history_frames_/history_counts_) deliberately survives
  pillar_map_.point_is_new_.clear();
}

// Delete flagged points from the frame outright: they neither contribute ICP
// residuals nor enter the voxel map. body_cloud (body frame) and world_cloud
// (world frame) are index-aligned views of the same frame, and skip_flags
// indexes that frame too (retention already applied by DefineSkipPoints), so
// the two clouds are compacted together to keep every downstream stage
// consistent. Survivors swap in place (caller-held Ptrs stay valid);
// skip_flags is consumed (cleared) — every kept point participates fully.
// Returns the number of removed points.
size_t PillarMap::removeFlaggedPoints(const PointCloudXYZI::Ptr &body_cloud,
                                           const PointCloudXYZI::Ptr &world_cloud,
                                           std::vector<uint8_t> &skip_flags)
{
  const size_t n = std::min(world_cloud->points.size(), skip_flags.size());

  PointCloudXYZI::Ptr body_kept(new PointCloudXYZI());
  PointCloudXYZI::Ptr world_kept(new PointCloudXYZI());
  body_kept->reserve(n);
  world_kept->reserve(n);

  size_t removed = 0;
  for (size_t i = 0; i < n; i++)
  {
    if (skip_flags[i]) { removed++; continue; }
    body_kept->push_back(body_cloud->points[i]);
    world_kept->push_back(world_cloud->points[i]);
  }

  if (removed > 0)
  {
    body_cloud->swap(*body_kept);
    world_cloud->swap(*world_kept);
  }
  skip_flags.clear();
  return removed;
}