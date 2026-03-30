/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "voxel_map.h"
#include <limits>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

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
  nh.param<int>("lio/max_iterations", voxel_config.max_iterations_, 5);
  nh.param<int>("lio/capacity", voxel_config.capacity, 100000);
  nh.param<bool>("lio/rf_enhance_en", voxel_config.rf_enhance_en_, false);
  nh.param<bool>("lio/intensity_fusion_en", voxel_config.intensity_fusion_en_, false);

  nh.param<bool>("local_map/map_sliding_en", voxel_config.map_sliding_en, false);
  nh.param<int>("local_map/half_map_size", voxel_config.half_map_size, 100);
  nh.param<double>("local_map/sliding_thresh", voxel_config.sliding_thresh, 8);
  }

void loadPillarVoxelConfig(ros::NodeHandle &nh, PillarVoxelConfig &config)
{
  nh.param<bool>("pillar_voxel/pillar_voxel_en", config.pillar_voxel_en_, false);
  nh.param<double>("pillar_voxel/voxel_size", config.voxel_size_, 1.0);
  nh.param<int>("pillar_voxel/min_adjacent_ground_num", config.min_adjacent_ground_num_, 3);
  nh.param<int>("pillar_voxel/min_adjacent_isolated_num", config.min_adjacent_isolated_num_, 3);
  nh.param<int>("pillar_voxel/ground_detection_method", config.ground_detection_method_, 0);
  nh.param<double>("pillar_voxel/plane_fitting_distance_threshold", config.plane_fitting_distance_threshold_, 0.1);
  nh.param<int>("pillar_voxel/skip_type", config.skip_type_, 0);
}

void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane)
{
  // 1. Initialize plane parameters
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
  plane->covariance_ = Eigen::Matrix3d::Zero();
  plane->center_ = Eigen::Vector3d::Zero();
  plane->normal_ = Eigen::Vector3d::Zero();
  plane->points_size_ = points.size();
  plane->radius_ = 0;
  plane->mean_intensity_ = 0.0f;

  // 2. Compute covariance matrix and centroid
  double intensity_sum = 0.0;

  for (auto pv : points)
  {
    plane->covariance_ += pv.point_w * pv.point_w.transpose();
    plane->center_ += pv.point_w;
    intensity_sum += static_cast<double>(pv.intensity);
  }
  plane->center_ = plane->center_ / plane->points_size_;
  plane->covariance_ = plane->covariance_ / plane->points_size_ - plane->center_ * plane->center_.transpose();
  plane->mean_intensity_ = static_cast<float>(intensity_sum / static_cast<double>(plane->points_size_));

  // 3. Compute intensity variance
  double intensity_variance = 0.0;
  for (auto pv : points)
  {
    double diff = static_cast<double>(pv.intensity) - plane->mean_intensity_;
    intensity_variance += diff * diff;
  }
  plane->intensity_std_ = sqrt(intensity_variance / static_cast<double>(plane->points_size_));

  // 4. Eigenvalue decomposition to extract plane normal and other parameters
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

  // 5. Check if points form a plane based on min eigenvalue
  if (evalsReal(evalsMin) < planer_threshold_)
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
  }
  else
  {
    plane->is_update_ = true;
    plane->is_plane_ = false;
  }
}

void VoxelOctoTree::init_octo_tree()
{
  if (temp_points_.size() > points_size_threshold_)
  {
    init_plane(temp_points_, plane_ptr_);
    if (plane_ptr_->is_plane_ == true)
    {
      octo_state_ = 0;  // Current voxel is plane, set as leaf node
      // Release memory when too many points
      if (temp_points_.size() > max_points_num_)
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
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
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
          if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_)
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
      if (update_enable_)
      {
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
          leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
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
        if (update_enable_)
        {
          new_points_++;
          temp_points_.push_back(pv);
          if (new_points_ > update_size_threshold_)
          {
            init_plane(temp_points_, plane_ptr_);
            new_points_ = 0;
          }
          if (temp_points_.size() > max_points_num_)
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
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
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

  // build_residual_time = 0.0;
  // ekf_time = 0.0;
  // double t0 = omp_get_wtime();

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
    point_crossmat << SKEW_SYM_MATRX(point_this);
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
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
    // build_residual_time += t2 - t1;

    // Compute total residual
    for (int i = 0; i < ptpl_list_.size(); i++)
    {
      total_residual += fabs(ptpl_list_[i].dis_to_plane_);
    }
    effct_feat_num_ = ptpl_list_.size();
    double current_average_residual = total_residual / effct_feat_num_;

    cout << "[ LIO ] Raw feature num: " << feats_undistort_->size() << ", downsampled feature num:" << feats_down_size_
         << ", effective feature num: " << effct_feat_num_ << ", average residual: " << current_average_residual
         << ", time: " << t2 - t1 << "s"<< endl;

    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
     * ***/
    // Build observation model, initialize observation matrix
    MatrixXd Hsub(effct_feat_num_, 6); // Observation Jacobian matrix
    MatrixXd Hsub_T_R_inv(6, effct_feat_num_); // Weighted observation Jacobian matrix
    VectorXd R_inv(effct_feat_num_); // Inverse covariance matrix of observation noise
    VectorXd meas_vec(effct_feat_num_); // Observation residual vector
    meas_vec.setZero();

    // Compute observation Jacobian matrix and observation noise covariance for each effective feature point
    for (int i = 0; i < effct_feat_num_; i++)
    {
      auto &ptpl = ptpl_list_[i];
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_;
      V3D point_body(ptpl.point_b_);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

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
    MatrixXd K(DIM_STATE, effct_feat_num_);
    // auto &&Hsub_T = Hsub.transpose();
    auto &&HTz = Hsub_T_R_inv * meas_vec;
    // fout_dbg<<"HTz: "<<HTz<<endl;
    H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
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
    V3D euler_cur = state_.rot_end.eulerAngles(2, 1, 0);

    /*** Rematch Judgement ***/
    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (config_setting_.max_iterations_ - 2)))) { rematch_num++; }

    /*** Convergence Judgements and Covariance Update ***/
    if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == config_setting_.max_iterations_ - 1)))
    {
      /*** Covariance Update ***/
      // _state.cov = (I_STATE - G) * _state.cov;
      state_.cov.block<DIM_STATE, DIM_STATE>(0, 0) =
          (I_STATE.block<DIM_STATE, DIM_STATE>(0, 0) - G.block<DIM_STATE, DIM_STATE>(0, 0)) * state_.cov.block<DIM_STATE, DIM_STATE>(0, 0);
      // total_distance += (_state.pos_end - position_last).norm();
      position_last_ = state_.pos_end;
      geoQuat_ = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

      // VD(DIM_STATE) K_sum  = K.rowwise().sum();
      // VD(DIM_STATE) P_diag = _state.cov.diagonal();
      EKF_stop_flg = true;
    }
    if (EKF_stop_flg) break;
  }

  // double t3 = omp_get_wtime();
  // scan_count++;
  // ekf_time = t3 - t0 - build_residual_time;

  // ave_build_residual_time = ave_build_residual_time * (scan_count - 1) / scan_count + build_residual_time / scan_count;
  // ave_ekf_time = ave_ekf_time * (scan_count - 1) / scan_count + ekf_time / scan_count;

  // cout << "[ Mapping ] ekf_time: " << ekf_time << "s, build_residual_time: " << build_residual_time << "s" << endl;
  // cout << "[ Mapping ] ave_ekf_time: " << ave_ekf_time << "s, ave_build_residual_time: " << ave_build_residual_time << "s" << endl;
}

void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR_ * p + extT_) + t);
    pcl::PointXYZI pi;
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
    M3D var;
    // Compute point covariance matrix in sensor coordinate system
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
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

    // Compute voxel coordinates
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      // If voxel already exists, add point directly to that voxel
      // voxel_map_[position]->temp_points_.push_back(p_v);
      // voxel_map_[position]->new_points_++;
      iter->second->second->temp_points_.push_back(p_v);
      iter->second->second->new_points_++;
    }
    else
    {
      // If voxel does not exist, create new voxel and add point
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      // voxel_map_[position] = octo_tree;
      // voxel_map_[position]->quater_length_ = voxel_size / 4;
      // voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      // voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      // voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      // voxel_map_[position]->temp_points_.push_back(p_v);
      // voxel_map_[position]->new_points_++;
      // voxel_map_[position]->layer_init_num_ = layer_init_num;
      // if (config_setting_.pillar_voxel_en_) {
      //   RegisterVoxelToPillar(position, voxel_map_[position]);
      // }

      voxel_map_cache_.emplace_front(position, octo_tree);
      voxel_map_.insert({position, voxel_map_cache_.begin()});
      voxel_map_cache_.begin()->second->quater_length_ = voxel_size / 4;
      voxel_map_cache_.begin()->second->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_cache_.begin()->second->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_cache_.begin()->second->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_cache_.begin()->second->temp_points_.push_back(p_v);
      voxel_map_cache_.begin()->second->new_points_++;
      voxel_map_cache_.begin()->second->layer_init_num_ = layer_init_num;
    }
  }

  // 4. Octree initialization phase, traverse all voxels and initialize octree for each voxel
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter)
  {
    // iter->second->init_octo_tree();
    iter->second->second->init_octo_tree();
  }
}

V3F VoxelMapManager::RGBFromVoxel(const V3D &input_point)
{
  int64_t loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = floor(input_point[j] / config_setting_.max_voxel_size_);
  }

  VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  int64_t ind = loc_xyz[0] + loc_xyz[1] + loc_xyz[2];
  uint k((ind + 100000) % 3);
  V3F RGB((k == 0) * 255.0, (k == 1) * 255.0, (k == 2) * 255.0);
  // cout<<"RGB: "<<RGB.transpose()<<endl;
  return RGB;
}

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)
{
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = config_setting_.layer_init_num_;

  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar p_v = input_points[i];
    // Compute voxel coordinates
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      // voxel_map_[position]->UpdateOctoTree(p_v);
      iter->second->second->UpdateOctoTree(p_v);
      voxel_map_cache_.splice(voxel_map_cache_.begin(), voxel_map_cache_, iter->second); // Update value and move to head
    }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      // voxel_map_[position] = octo_tree;
      // voxel_map_[position]->layer_init_num_ = layer_init_num;
      // voxel_map_[position]->quater_length_ = voxel_size / 4;
      // voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      // voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      // voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      // voxel_map_[position]->UpdateOctoTree(p_v);
      // if (config_setting_.pillar_voxel_en_) {
      //   RegisterVoxelToPillar(position, voxel_map_[position]);
      // }

      octo_tree->quater_length_ = voxel_size / 4;
      octo_tree->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      octo_tree->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      octo_tree->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      octo_tree->temp_points_.push_back(p_v);
      octo_tree->new_points_++;
      octo_tree->layer_init_num_ = layer_init_num;

      // Insert new node at head
      voxel_map_cache_.emplace_front(position, octo_tree);
      voxel_map_.insert({position, voxel_map_cache_.begin()});
    }
  }

  // Capacity check, remove tail nodes (only perform LRU cache management when capacity > 0)
  if (config_setting_.capacity > 0)
  {
    while (voxel_map_cache_.size() >= config_setting_.capacity)
    {
      delete voxel_map_cache_.back().second;
      auto last_key = voxel_map_cache_.back().first;
      voxel_map_.erase(last_key);
      voxel_map_cache_.pop_back();
    }
  }
}

// Find corresponding plane for each point and compute point-to-plane distance for ICP registration
void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
{
  int max_layer = config_setting_.max_layer_;
  double voxel_size = config_setting_.max_voxel_size_;
  double sigma_num = config_setting_.sigma_num_;
  // std::mutex mylock;
  ptpl_list.clear();
  std::vector<PointToPlane> all_ptpl_list(pv_list.size());
  std::vector<bool> useful_ptpl(pv_list.size());
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

    if (!skip_list.empty() && skip_list[i]) {
      // Skip points that don't need processing
      continue;
    }

    // Compute voxel location of point
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = pv.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]); // Create voxel for current point

    // Find corresponding voxel in voxel map
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) // If voxel found
    {
      // VoxelOctoTree *current_octo = iter->second; // Get octree node corresponding to voxel
      VoxelOctoTree *current_octo = iter->second->second; // Get octree node corresponding to voxel

      PointToPlane single_ptpl{}; // Store plane information for current point
      bool is_sucess = false; // Mark whether plane was successfully found
      bool is_surface = false; // Mark whether point is surface point
      double prob = 0; // Store probability value of point to plane

      // Find corresponding plane in octree of current voxel, build point-to-plane residual if found
      build_single_residual(pv, current_octo, 0, is_sucess, is_surface, prob, single_ptpl);
      if (!is_sucess)
      {
        // If no valid plane found in current voxel, check adjacent voxels
        VOXEL_LOCATION near_position = position;

        // Receptive field enhancement: determine search range based on configuration
        // Helper function: compute single-axis offset
        auto calc_offset = [&](double coord, double center, double quater_len) -> int {
          if (coord > center + quater_len) {
            if (config_setting_.rf_enhance_en_ && coord > center + 2 * quater_len)
              return 2;
            else
              return 1;
          } else if (coord < center - quater_len) {
            if (config_setting_.rf_enhance_en_ && coord < center - 2 * quater_len)
              return -2;
            else
              return -1;
          }
          return 0;
        };

        near_position.x += calc_offset(loc_xyz[0], current_octo->voxel_center_[0], current_octo->quater_length_);
        near_position.y += calc_offset(loc_xyz[1], current_octo->voxel_center_[1], current_octo->quater_length_);
        near_position.z += calc_offset(loc_xyz[2], current_octo->voxel_center_[2], current_octo->quater_length_);

        // Find plane in adjacent voxels, build residual if found
        auto iter_near = voxel_map_.find(near_position);
        if (iter_near != voxel_map_.end()) { build_single_residual(pv, (*(iter_near->second)).second, 0, is_sucess, is_surface, prob, single_ptpl); }
      }

      if (is_surface)
      {
        current_octo->is_surface_voxel_ = true;
      }

      if (is_sucess)
      {
        // mylock.lock();
        useful_ptpl[i] = true;
        all_ptpl_list[i] = single_ptpl;
        // mylock.unlock();
      }
      else
      {
        // mylock.lock();
        useful_ptpl[i] = false;
        // mylock.unlock();
      }
    }
  }
  for (size_t i = 0; i < useful_ptpl.size(); i++)
  {
    if (useful_ptpl[i]) { ptpl_list.push_back(all_ptpl_list[i]); }
  }
}

void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, bool &is_surface,
                                            double &prob, PointToPlane &single_ptpl)
{
  int max_layer = config_setting_.max_layer_;
  double sigma_num = config_setting_.sigma_num_;

  double radius_k = 3.0;
  Eigen::Vector3d p_w = pv.point_w;
  if (current_octo->plane_ptr_->is_plane_) // Check if current voxel contains valid plane
  {
    VoxelPlane &plane = *current_octo->plane_ptr_;

    Eigen::Vector3d p_world_to_center = p_w - plane.center_;
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
        is_surface = true; // If point-to-plane distance is small, consider point as plane point
        is_sucess = true;

        double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);

        // Enable 3D intensity probability fusion based on configuration
        if (config_setting_.intensity_fusion_en_)
        {
          // Prevent division by zero: intensity standard deviation at least 1e-6
          double intensity_std_safe = std::max(plane.intensity_std_, 1e-6);
          double intensity_std_sq = intensity_std_safe * intensity_std_safe;

          // Intensity probability: based on historical average intensity
          double intensity_diff = static_cast<double>(pv.intensity) - plane.mean_intensity_;
          double intensity_prob = 1.0 / (sqrt(2.0 * M_PI) * intensity_std_safe) *
                                  exp(-0.5 * intensity_diff * intensity_diff / intensity_std_sq);
          this_prob = 0.5 * this_prob + 0.5 * intensity_prob;
        }

        if (this_prob > prob) // When point may match multiple planes, select the one with highest probability
        {
          prob = this_prob;
          pv.normal = plane.normal_;
          single_ptpl.body_cov_ = pv.body_var;
          single_ptpl.point_b_ = pv.point_b;
          single_ptpl.point_w_ = pv.point_w;
          single_ptpl.intensity_ = pv.intensity;  // Pass intensity information
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
        // is_sucess = false;
        return;
      }
    }
    else
    {
      // is_sucess = false;
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
          build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, is_surface, prob, single_ptpl);
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
  plane.header.frame_id = "camera_init";
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
  if((position_last_ - last_slide_position).norm() < config_setting_.sliding_thresh)
  {
    std::cout<<YELLOW<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position).norm()<<RESET<<"\n";
    return;
  }

  //get global id now
  last_slide_position = position_last_;
  double t_sliding_start = omp_get_wtime();
  float loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = position_last_[j] / config_setting_.max_voxel_size_;
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  // VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);//discrete global
  clearMemOutOfMap((int64_t)loc_xyz[0] + config_setting_.half_map_size, (int64_t)loc_xyz[0] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[1] + config_setting_.half_map_size, (int64_t)loc_xyz[1] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[2] + config_setting_.half_map_size, (int64_t)loc_xyz[2] - config_setting_.half_map_size);
  double t_sliding_end = omp_get_wtime();
  std::cout<<YELLOW<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs"<<RESET<<"\n";
  return;
}

void VoxelMapManager::clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min )
{
  int delete_voxel_cout = 0;
  // double delete_time = 0;
  // double last_delete_time = 0;
  for (auto it = voxel_map_.begin(); it != voxel_map_.end(); )
  {
    const VOXEL_LOCATION& loc = it->first;
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
    if (should_remove){
      // last_delete_time = omp_get_wtime();
      // delete it->second;
      VOXEL_LOCATION remove_loc = loc;
      VoxelOctoTree *voxel_ptr = it->second->second;
      delete voxel_ptr;

      // Remove from LRU cache (direct deletion via iterator, O(1) time complexity)
      voxel_map_cache_.erase(it->second);

      it = voxel_map_.erase(it);
      // delete_time += omp_get_wtime() - last_delete_time;
      delete_voxel_cout++;
    } else {
      ++it;
    }
  }
  std::cout<<YELLOW<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels"<<RESET<<"\n";
  // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}

// Compute pillar location (fixed elevation direction as z-axis, pillar composed of x,y)
PILLAR_LOCATION PillarVoxelMap::GetPillarLocation(const VOXEL_LOCATION &position) const
{
  return PILLAR_LOCATION(position.x, position.y);
}

void PillarVoxelMap::init(const PillarVoxelConfig &config, double voxel_size)
{
  config_ = config;
  voxel_size_ = voxel_size;
  initHorizontalNeighborOffsets();
}

void PillarVoxelMap::initHorizontalNeighborOffsets()
{
  neighbor_offsets_.clear();

  const std::vector<std::pair<int, int>> all_offsets = {
    {-1, -1}, {-1, 0}, {-1, 1},
    {0, -1},           {0, 1},
    {1, -1},  {1, 0},  {1, 1}
  };

  neighbor_offsets_.reserve(8);
  for (const auto& offset : all_offsets) {
    VOXEL_LOCATION voxel_offset;
    voxel_offset.x = offset.first;
    voxel_offset.y = offset.second;
    voxel_offset.z = 0;
    neighbor_offsets_.push_back(voxel_offset);
  }
}

void PillarVoxelMap::UpdateGroundFlagForPillar(const PILLAR_LOCATION &pillar_key,
                                                std::map<int64_t, PillarVoxel> &pillar_voxels,
                                                const Eigen::Vector3d& current_pos)
{
  (void)pillar_key;
  (void)current_pos;

  auto setVoxelPointLabels = [this](PillarVoxel* voxel, int8_t label)
  {
    voxel->is_isolated_voxel_ = (label == LABEL_ISOLATED);
    for (size_t idx : voxel->point_indices_) {
      if (idx < point_labels_.size()) {
        point_labels_[idx] = label;
      }
    }
  };

  for (auto &voxel_pair : pillar_voxels)
  {
    voxel_pair.second.is_ground_voxel_ = false;
    setVoxelPointLabels(&voxel_pair.second, LABEL_NORMAL);
  }

  PillarVoxel* bottom_voxel = &pillar_voxels.begin()->second;
  bottom_voxel->is_ground_voxel_ = true;

  for (size_t idx : bottom_voxel->point_indices_) {
    if (idx < point_labels_.size()) {
      point_labels_[idx] = LABEL_GROUND;
    }
  }

  if (pillar_voxels.size() == 1) {
    setVoxelPointLabels(bottom_voxel, LABEL_ISOLATED);
  }
  else if (pillar_voxels.size() == 2)
  {
    PillarVoxel* top_voxel = &std::next(pillar_voxels.begin())->second;
    auto z_diff = top_voxel->center_z_ - bottom_voxel->center_z_;

    if (z_diff > voxel_size_) {
      setVoxelPointLabels(bottom_voxel, LABEL_ISOLATED);
      setVoxelPointLabels(top_voxel, LABEL_ISOLATED);
    }
  }
  else{
    auto pillar_iter = std::next(pillar_voxels.begin());
    double down_z_diff = 0;
    double up_z_diff = 0;
    for (; pillar_iter != std::prev(pillar_voxels.end()); ++pillar_iter)
    {
      down_z_diff = pillar_iter->second.center_z_ - std::prev(pillar_iter)->second.center_z_;
      up_z_diff = std::next(pillar_iter)->second.center_z_ - pillar_iter->second.center_z_;

      if (down_z_diff > voxel_size_ && up_z_diff > voxel_size_) {
        setVoxelPointLabels(&pillar_iter->second, LABEL_ISOLATED);
      }
    }
    if (up_z_diff > voxel_size_) {
      setVoxelPointLabels(&pillar_voxels.rbegin()->second, LABEL_ISOLATED);
    }
  }

  if (!pillar_voxels.empty()) {
    PillarVoxel* top_voxel = &pillar_voxels.rbegin()->second;
    if (top_voxel->is_isolated_voxel_) {
      VOXEL_LOCATION voxel_loc;
      // Derive X,Y from PILLAR_LOCATION (pillar_key.axis1, axis2 are voxel indices)
      voxel_loc.x = pillar_key.axis1;
      voxel_loc.y = pillar_key.axis2;
      voxel_loc.z = pillar_voxels.rbegin()->first;

      if (hasAdjacentTopVoxel(voxel_loc)) {
        setVoxelPointLabels(top_voxel, LABEL_NORMAL);
      }
    }
  }
}

bool PillarVoxelMap::hasAdjacentGroundVoxel(const PillarVoxel *voxel, const VOXEL_LOCATION &current_pos)
{
  (void)voxel;

  if (config_.min_adjacent_ground_num_ <= 0) {
    return false;
  }

  int adjacent_ground_count = 0;
  int64_t current_elevation = current_pos.z;

  for (const auto& voxel_offset : neighbor_offsets_) {
    VOXEL_LOCATION adjacent_pos = {
      current_pos.x + voxel_offset.x,
      current_pos.y + voxel_offset.y,
      current_pos.z + voxel_offset.z
    };

    PILLAR_LOCATION adjacent_pillar = GetPillarLocation(adjacent_pos);

    auto pillar_iter = pillars_.find(adjacent_pillar);

    if (pillar_iter != pillars_.end() && !pillar_iter->second.empty()) {
      const PillarVoxel *first_voxel = &pillar_iter->second.begin()->second;

      if (first_voxel && first_voxel->is_ground_voxel_) {
        int64_t height_diff = std::abs(current_elevation - pillar_iter->second.begin()->first);

        if (height_diff <= 0) {
          adjacent_ground_count++;
        }

        if (adjacent_ground_count >= config_.min_adjacent_ground_num_){
          return true;
        }
      }
    }
  }

  return false;
}

bool PillarVoxelMap::hasAdjacentTopVoxel(const VOXEL_LOCATION &current_pos)
{
  if (config_.min_adjacent_isolated_num_ <= 0) {
    return false;
  }

  int adjacent_count = 0;
  int64_t current_elevation = current_pos.z;

  for (const auto& voxel_offset : neighbor_offsets_) {
    VOXEL_LOCATION adjacent_pos = {
      current_pos.x + voxel_offset.x,
      current_pos.y + voxel_offset.y,
      current_pos.z + voxel_offset.z
    };

    PILLAR_LOCATION adjacent_pillar = GetPillarLocation(adjacent_pos);

    auto pillar_iter = pillars_.find(adjacent_pillar);
    if (pillar_iter != pillars_.end() && !pillar_iter->second.empty()) {
      const PillarVoxel *last_voxel = &pillar_iter->second.rbegin()->second;

      if (last_voxel) {
        int64_t height_diff = std::abs(current_elevation - pillar_iter->second.rbegin()->first);

        if (height_diff  <= 1) {
          adjacent_count++;
        }

        if (adjacent_count >= config_.min_adjacent_isolated_num_){
          return true;
        }
      }
    }
  }

  return false;
}

void PillarVoxelMap::BuildPillarMap(const PointCloudXYZI::Ptr &input_cloud)
{
  const size_t num_points = input_cloud->points.size();
  const double inv_voxel_size = 1.0 / voxel_size_;

  point_labels_.assign(num_points, LABEL_NORMAL);
  point_cloud_ptr_ = input_cloud;

  current_pillars_.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i)
  {
    const PointType& point = input_cloud->points[i];

    VOXEL_LOCATION voxel_location;
    voxel_location.x = static_cast<int64_t>(std::floor(point.x * inv_voxel_size));
    voxel_location.y = static_cast<int64_t>(std::floor(point.y * inv_voxel_size));
    voxel_location.z = static_cast<int64_t>(std::floor(point.z * inv_voxel_size));

    PILLAR_LOCATION pillar_loc = GetPillarLocation(voxel_location);
    int64_t voxel_key = voxel_location.z;

    auto& pillar_map = pillars_[pillar_loc];
    auto voxel_it = pillar_map.find(voxel_key);

    if (voxel_it == pillar_map.end())
    {
      double center_z = (voxel_location.z + 0.5) * voxel_size_;
      voxel_it = pillar_map.emplace(voxel_key, PillarVoxel(center_z)).first;
    }

    PillarVoxel& voxel = voxel_it->second;
    voxel.point_indices_.push_back(i);

    current_pillars_.insert(pillar_loc);
  }
}

void PillarVoxelMap::GroundDetection(const Eigen::Vector3d& current_pos)
{
  plane_fitted_ = false;

  for (const auto& pillar_key : current_pillars_)
  {
    auto pillar_iter = pillars_.find(pillar_key);
    if (pillar_iter != pillars_.end() && !pillar_iter->second.empty())
    {
      for (auto& voxel_pair : pillar_iter->second)
      {
        voxel_pair.second.is_ground_voxel_ = false;
      }
    }
  }

  for (const auto& pillar_key : current_pillars_)
  {
    auto pillar_iter = pillars_.find(pillar_key);
    if (pillar_iter != pillars_.end() && !pillar_iter->second.empty())
    {
      UpdateGroundFlagForPillar(pillar_key, pillar_iter->second, current_pos);
    }
  }

  if (config_.min_adjacent_ground_num_ > 0)
  {
    for (const auto& pillar_key : current_pillars_)
    {
      auto pillar_iter = pillars_.find(pillar_key);
      if (pillar_iter == pillars_.end() || pillar_iter->second.empty())
        continue;

      auto& pillar_voxels = pillar_iter->second;

      PillarVoxel* bottom_voxel = &pillar_voxels.begin()->second;
      if (!bottom_voxel->is_ground_voxel_) continue;

      VOXEL_LOCATION bottom_loc;
      bottom_loc.x = pillar_key.axis1;
      bottom_loc.y = pillar_key.axis2;
      bottom_loc.z = pillar_voxels.begin()->first;
      bool has_enough_adjacent_ground = hasAdjacentGroundVoxel(bottom_voxel, bottom_loc);
      if (!has_enough_adjacent_ground)
      {
        bottom_voxel->is_ground_voxel_ = false;
        for (size_t idx : bottom_voxel->point_indices_) {
          if (idx < point_labels_.size()) {
            point_labels_[idx] = LABEL_NORMAL;
          }
        }
      }
    }
  }

  if (config_.ground_detection_method_ == 1)
  {
    std::vector<Eigen::Vector3d> seed_points;
    for (const auto& pillar_key : current_pillars_)
    {
      auto pillar_iter = pillars_.find(pillar_key);
      if (pillar_iter == pillars_.end() || pillar_iter->second.empty())
        continue;

      PillarVoxel* bottom_voxel = &pillar_iter->second.begin()->second;

      if (bottom_voxel->is_ground_voxel_)
      {
        for (size_t idx : bottom_voxel->point_indices_)
        {
          if (idx < point_cloud_ptr_->points.size()) {
            const PointType& pt = point_cloud_ptr_->points[idx];
            seed_points.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
          }
        }
      }
    }

    if (!seed_points.empty())
    {
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      for (const auto& pt : seed_points)
      {
        center += pt;
      }
      center /= static_cast<double>(seed_points.size());

      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (const auto& pt : seed_points)
      {
        const Eigen::Vector3d diff = pt - center;
        covariance += diff * diff.transpose();
      }
      covariance /= static_cast<double>(seed_points.size());

      Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Vector3d plane_normal = svd.matrixU().col(2);

      if (plane_normal(2) < 0) plane_normal = -plane_normal;

      double plane_d = -plane_normal.dot(center);

      fitted_plane_normal_ = plane_normal;
      fitted_plane_d_ = plane_d;
      plane_fitted_ = true;

      const double dist_threshold = config_.plane_fitting_distance_threshold_;
      for (const auto& pillar_key : current_pillars_)
      {
        auto pillar_iter = pillars_.find(pillar_key);
        if (pillar_iter == pillars_.end() || pillar_iter->second.empty())
          continue;

        PillarVoxel* bottom_voxel = &pillar_iter->second.begin()->second;

        bool is_near_plane = false;

        for (size_t idx : bottom_voxel->point_indices_)
        {
          if (idx < point_cloud_ptr_->points.size()) {
            const PointType& pt = point_cloud_ptr_->points[idx];
            const double distance = plane_normal.dot(Eigen::Vector3d(pt.x, pt.y, pt.z)) + plane_d;

            if (std::abs(distance) < dist_threshold)
            {
              is_near_plane = true;
              point_labels_[idx] = LABEL_GROUND;
            }
            else {
              point_labels_[idx] = LABEL_NORMAL;
            }
          }
        }

        bottom_voxel->is_ground_voxel_ = is_near_plane;
      }

      std::cout << "[GroundDetection] Method 1 (Plane Fitting): normal=[" << plane_normal.transpose()
                << "], d=" << plane_d << ", using " << seed_points.size() << " seed points" << std::endl;
    }
  }
  else if (config_.ground_detection_method_ == 2)
  {
    // Method 2: neighborhood (use adjacency check result directly)
    int ground_voxel_count = 0;
    for (const auto& pillar_key : current_pillars_)
    {
      auto pillar_iter = pillars_.find(pillar_key);
      if (pillar_iter == pillars_.end() || pillar_iter->second.empty())
        continue;

      PillarVoxel* bottom_voxel = &pillar_iter->second.begin()->second;
      if (bottom_voxel->is_ground_voxel_)
      {
        ground_voxel_count++;
      }
    }

    std::cout << "[GroundDetection] Method 2 (neighborhood): " << ground_voxel_count
              << " ground voxels detected via adjacency check" << std::endl;
  }
}

void VoxelMapManager::DefineSkipPoints(const PointCloudXYZI::Ptr &feats_down_world)
{
  const size_t point_num = feats_down_world->points.size();
  skip_list.assign(point_num, false);

  if (point_num == 0) {
    std::cout << "[DefineSkipPoints] Empty cloud, skip nothing." << std::endl;
    return;
  }

  int isolated_count = 0;
  int ground_count = 0;
  int final_skip_count = 0;

  for (size_t i = 0; i < point_num; ++i)
  {
    int8_t label = pillar_map_.GetPointLabel(i);

    if (label == LABEL_ISOLATED)
    {
      skip_list[i] = true;
      isolated_count++;
      final_skip_count++;
    }
    else if (label == LABEL_GROUND)
    {
      ground_count++;
    }
  }

  if (pillar_map_.config_.ground_detection_method_ == 0)
  {
    std::cout << "[DefineSkipPoints] Method 0 (None): Isolated: " << isolated_count
              << "/" << point_num << " ("
              << std::fixed << std::setprecision(1)
              << (100.0 * isolated_count / point_num) << "%)" << std::endl;

    // Update statistics
    current_skip_count_ = final_skip_count;
    total_skip_count_ += final_skip_count;
    total_point_count_ += point_num;
    return;
  }
  else if (pillar_map_.config_.ground_detection_method_ == 2)
  {
    for (size_t i = 0; i < point_num; ++i)
    {
      if (pillar_map_.GetPointLabel(i) == LABEL_GROUND)
      {
        if (!skip_list[i])
        {
          skip_list[i] = true;
          final_skip_count++;
        }
      }
    }

    std::cout << "[DefineSkipPoints] Method 2 (Neighborhood): Isolated: " << isolated_count
              << ", Ground: " << ground_count
              << ", Skip_total: " << final_skip_count
              << "/" << point_num << " ("
              << std::fixed << std::setprecision(1)
              << (100.0 * final_skip_count / point_num) << "%)" << std::endl;

    // Update statistics
    current_skip_count_ = final_skip_count;
    total_skip_count_ += final_skip_count;
    total_point_count_ += point_num;
    return;
  }

  if (!pillar_map_.plane_fitted_)
  {
    std::cout << "[DefineSkipPoints] Method 1 (Plane Fitting) but no plane fitted! Isolated: " << isolated_count
              << "/" << point_num << std::endl;

    // Update statistics
    current_skip_count_ = final_skip_count;
    total_skip_count_ += final_skip_count;
    total_point_count_ += point_num;
    return;
  }

  const Eigen::Vector3d& plane_normal = pillar_map_.fitted_plane_normal_;
  const double plane_d = pillar_map_.fitted_plane_d_;
  const double distance_threshold = pillar_map_.config_.plane_fitting_distance_threshold_;

  int ground_near_count = 0;
  int ground_below_count = 0;

  for (size_t i = 0; i < point_num; ++i)
  {
    if (skip_list[i]) continue;

    const PointType& point_world = feats_down_world->points[i];
    Eigen::Vector3d point_vec(point_world.x, point_world.y, point_world.z);

    const double distance = plane_normal.dot(point_vec) + plane_d;

    // Count points by distance to plane
    if (distance < -distance_threshold)
    {
      ground_below_count++;
    }
    else if (distance < distance_threshold)
    {
      ground_near_count++;
    }

    bool skip = false;
    if (pillar_map_.config_.skip_type_ == 2)
    {
      skip = (distance < distance_threshold);
    }
    else if (pillar_map_.config_.skip_type_ == 1)
    {
      skip = (distance < -distance_threshold);
    }

    if (skip)
    {
      skip_list[i] = true;
      final_skip_count++;
    }
  }

  std::cout << "[DefineSkipPoints] Method 1 (Plane Fitting): Isolated: " << isolated_count
            << ", Ground_near: " << ground_near_count
            << ", Below_ground: " << ground_below_count
            << ", Skip_total: " << final_skip_count
            << "/" << point_num << " ("
            << std::fixed << std::setprecision(1)
            << (100.0 * final_skip_count / point_num) << "%)" << std::endl;

  // Update statistics
  current_skip_count_ = final_skip_count;
  total_skip_count_ += final_skip_count;
  total_point_count_ += point_num;
}

void PillarVoxelMap::PublishPillarPoints(const ros::Publisher &pubGround, const ros::Publisher &pubIsolated)
{
  size_t total_ground_points = 0;
  size_t total_isolated_points = 0;

  // First pass: count all ground and isolated points across ALL voxel layers
  for (const auto& pillar_key : current_pillars_)
  {
    auto pillar_iter = pillars_.find(pillar_key);
    if (pillar_iter != pillars_.end() && !pillar_iter->second.empty())
    {
      for (const auto& voxel_pair : pillar_iter->second)
      {
        const PillarVoxel& voxel = voxel_pair.second;
        if (voxel.is_ground_voxel_)
        {
          total_ground_points += voxel.point_indices_.size();
        }
        else if (voxel.is_isolated_voxel_)
        {
          total_isolated_points += voxel.point_indices_.size();
        }
      }
    }
  }

  if (total_ground_points == 0 && total_isolated_points == 0) return;

  PointCloudXYZI::Ptr ground_cloud(new PointCloudXYZI());
  PointCloudXYZI::Ptr isolated_cloud(new PointCloudXYZI());

  if (total_ground_points > 0)
  {
    ground_cloud->points.reserve(total_ground_points);
  }

  if (total_isolated_points > 0)
  {
    isolated_cloud->points.reserve(total_isolated_points);
  }

  for (const auto& pillar_key : current_pillars_)
  {
    auto pillar_iter = pillars_.find(pillar_key);
    if (pillar_iter == pillars_.end() || pillar_iter->second.empty())
      continue;

    for (const auto& voxel_pair : pillar_iter->second)
    {
      const PillarVoxel& voxel = voxel_pair.second;

      if (voxel.is_ground_voxel_)
      {
        for (size_t idx : voxel.point_indices_)
        {
          if (idx < point_cloud_ptr_->points.size()) {
            ground_cloud->points.push_back(point_cloud_ptr_->points[idx]);
          }
        }
      }
      else if (voxel.is_isolated_voxel_)
      {
        for (size_t idx : voxel.point_indices_)
        {
          if (idx < point_cloud_ptr_->points.size()) {
            isolated_cloud->points.push_back(point_cloud_ptr_->points[idx]);
          }
        }
      }
    }
  }

  if (total_ground_points > 0)
  {
    ground_cloud->width = ground_cloud->points.size();
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    ground_msg.header.stamp = ros::Time::now();
    ground_msg.header.frame_id = "camera_init";
    pubGround.publish(ground_msg);
  }

  if (total_isolated_points > 0)
  {
    isolated_cloud->width = isolated_cloud->points.size();
    isolated_cloud->height = 1;
    isolated_cloud->is_dense = true;

    sensor_msgs::PointCloud2 isolated_msg;
    pcl::toROSMsg(*isolated_cloud, isolated_msg);
    isolated_msg.header.stamp = ros::Time::now();
    isolated_msg.header.frame_id = "camera_init";
    pubIsolated.publish(isolated_msg);
  }
}

void VoxelMapManager::ClearPillarVoxels()
{
  // PillarVoxel is now a lightweight value type, no manual delete needed
  // Just clear the containers
  pillar_map_.pillars_.clear();
  pillar_map_.current_pillars_.clear();

  // Clear point labels array (3-pass scan merge optimization)
  pillar_map_.point_labels_.clear();
}
