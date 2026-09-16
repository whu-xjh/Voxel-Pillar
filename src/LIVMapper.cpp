
#include "LIVMapper.h"
#include <chrono>
#include <iomanip>
#include <sys/stat.h>
#include <fstream>
#include <type_traits>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

LIVMapper::LIVMapper(ros::NodeHandle &nh)
    : extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(nh);

  VoxelMapConfig voxel_config;
  loadVoxelConfig(nh, voxel_config);

  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());

  loadPillarVoxelConfig(nh, pillar_config);
  voxelmap_manager.reset(new VoxelMapManager(voxel_config));
  voxelmap_manager->pillar_map_.init(pillar_config, pillar_config.voxel_size_);
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
}

LIVMapper::~LIVMapper()
{
}

void LIVMapper::readParameters(ros::NodeHandle &nh)
{
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
  nh.param<int>("common/lidar_en", lidar_en, 1);

  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
  nh.param<bool>("uav/gravity_align_en", gravity_align_en, false);

  nh.param<string>("evo/seq_name", seq_name, "01");
  nh.param<bool>("evo/pose_output_en", pose_output_en, false);
  nh.param<string>("evo/pose_output_dir", pose_output_dir, "");
  nh.param<double>("imu/gyr_cov", gyr_cov, 1.0);
  nh.param<double>("imu/acc_cov", acc_cov, 1.0);
  nh.param<double>("imu/b_gyr_cov", b_gyr_cov, 0.0001);
  nh.param<double>("imu/b_acc_cov", b_acc_cov, 0.0001);
  nh.param<bool>("lio/intensity_noise_est_en", intensity_noise_est_en_, true);
  nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
  nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, true);

  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  nh.param<int>("lidar_save/interval", save_interval, -1);
  nh.param<bool>("lidar_save/save_en", save_en, false);
  nh.param<double>("lidar_save/filter_size_pcd", filter_size_pcd, 0.5);
  nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());

  nh.param<bool>("publish/pub_effect_en", pub_effect_en, false);
  nh.param<bool>("publish/dense_map_en", dense_map_en, false);
  nh.param<bool>("publish/pub_cloud_body", pub_body_en, false);

  // Read external IMU parameters
  nh.param<string>("common/external_imu_topic", external_imu_topic, "/novatel/oem7/odom");
  nh.param<bool>("external_imu/enable", external_imu_enable, false);
  nh.param<int>("external_imu/external_imu_init_frame", external_imu_int_frame, 3);
  nh.param<double>("external_imu/time_offset", external_imu_time_offset, 0.0);
  nh.param<int>("external_imu/buffer_size", external_imu_buffer_size, 1000);
  nh.param<bool>("external_imu/external_imu_only", external_imu_only, false);
  nh.param<vector<double>>("external_imu/external_T", external_imu_T_vec, vector<double>());
  nh.param<vector<double>>("external_imu/external_R", external_imu_R_vec, vector<double>());

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents() 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  external_imu_T << VEC_FROM_ARRAY(external_imu_T_vec);
  external_imu_R << MAT_FROM_ARRAY(external_imu_R_vec);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
  p_imu->set_imu_init_frame_num(imu_int_frame);
  p_imu->set_external_imu_init_frame_num(external_imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();

  slam_mode_ = imu_en ? ONLY_LIO : ONLY_LO;

  }

void LIVMapper::initializeFiles()
{
  // Create timestamp subdirectory
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
  session_timestamp_ = ss.str();

  // Create PCD timestamp subdirectory (under Log/PCD/)
  // pcd_session_dir_ = "/media/xjh/Extreme SSD/data/shanxi/car/output/" + session_timestamp_;
  pcd_session_dir_ = root_dir + "Log/PCD/" + session_timestamp_;
  if (save_en) {
    int result = mkdir(pcd_session_dir_.c_str(), 0777);
    if (result == 0) {
      ROS_INFO("PCD session directory created: %s", pcd_session_dir_.c_str());
    } else {
      ROS_ERROR("Failed to create PCD session directory: %s (error: %d)", pcd_session_dir_.c_str(), errno);
    }
    printf("ROOT_DIR: %s, pcd_session_dir_: %s\n", root_dir.c_str(), pcd_session_dir_.c_str());
  }

  if(save_interval > 0) fout_pcd_pos.open(pcd_session_dir_ + "/scans_pos.json", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}


void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
{
  sub_pcl = p_pre->lidar_type == AVIA ?
            nh.subscribe(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this):
            nh.subscribe(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
  sub_imu = nh.subscribe(imu_topic, 200000, &LIVMapper::imu_cbk, this);

  if (external_imu_enable) {
      sub_external_imu = nh.subscribe(external_imu_topic, 200000, &LIVMapper::odom_cbk, this);
      ROS_INFO("External IMU enabled, subscribing to topic: %s", external_imu_topic.c_str());
  } else {
      ROS_INFO("External IMU disabled");
  }

  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  pubLaserCloudBody = nh.advertise<sensor_msgs::PointCloud2>("/cloud_body", 100);
  pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
  pubPillarMapCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_pillarmap", 100);
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
  mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImuPropOdom = nh.advertise<nav_msgs::Odometry>("/imu_propagate", 10000);

  imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);
  voxelmap_manager->voxel_map_pub_= nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    first_lidar_time_ = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = first_lidar_time_; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment()
{
  // Unify vertical reference, ensure z-axis aligns with gravity direction
  if (!p_imu->imu_need_init && !gravity_align_finished)
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(state_.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    state_.pos_end = G_R_I0 * state_.pos_end;
    state_.rot_end = G_R_I0 * state_.rot_end;
    state_.vel_end = G_R_I0 * state_.vel_end;
    state_.gravity = G_R_I0 * state_.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu()
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, state_, feats_undistort, external_imu_buffer, external_imu_enable, external_imu_only);  // Call IMU processing module with external IMU buffer and enable state

  if (gravity_align_en) gravityAlignment();

  state_propagat = state_;  // Update state propagation variable
  voxelmap_manager->state_ = state_;  // Update voxel map manager state: pose, IMU bias, covariance matrix, etc.
  /*
      1. Coordinate transformation reference:
        - Voxel map uses this state for point cloud world coordinate transformation
        - Affects accuracy of point cloud registration and map building
      2. ICP registration:
        - Used as initial guess in StateEstimation()
        - Affects convergence and accuracy of ICP algorithm
      3. Map update:
        - Determines how new point clouds merge with existing voxel map
        - Affects map consistency and accuracy
      4. Uncertainty propagation:
        - State covariance affects point cloud matching weights
        - Affects confidence of state estimation
  */
  voxelmap_manager->feats_undistort_ = feats_undistort; // Update undistorted point cloud

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << state_.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping()
{
  switch (LidarMeasures.ekf_state_flg)
  {
    case LIO:
    case LO:
      handleLIO();
      break;
    default:
      break;
  }
}

// Median of a float vector (reorders the input in place)
static double medianOf(std::vector<float> &v)
{
  if (v.empty()) { return 0.0; }
  size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  double m = v[mid];
  if (v.size() % 2 == 0)
  {
    std::nth_element(v.begin(), v.begin() + mid - 1, v.end());
    m = 0.5 * (m + v[mid - 1]);
  }
  return m;
}

// Estimate the per-point intensity measurement noise during the init window,
// where the platform is expected to be stationary: consecutive scans observe
// the same surfaces, so frame-to-frame nearest-neighbor intensity differences
// reflect pure repeat-measurement noise. For two independent measurements of
// the same true value var(diff) = 2*sigma^2, hence the sqrt(2) divisor, and
// the robust MAD-to-sigma factor 1.4826. The result replaces the conservative
// default of VoxelPlane::intensity_meas_var_ and takes effect immediately in
// the intensity fusion scoring and association gate.
void LIVMapper::estimateIntensityNoise()
{
  if (!intensity_noise_est_en_ || intensity_noise_done_ || feats_down_world->empty()) { return; }

  const int window = std::max(imu_int_frame, 15); // estimation window: reuse IMU init frames
  const int hard_stop = 3 * window;               // give up after this many frames
  const size_t min_pairs = 2000;                  // minimum samples for a robust estimate
  const size_t max_pairs = 200000;                // memory bound
  const double max_trans = 0.01;                  // stationarity gates: strict, so pairs are
  const double max_rot_deg = 0.01;                // guaranteed near-identical viewpoints
  const float pair_sq_dist_th = 0.01f * 0.01f;    // 1cm NN pairing radius

  // Track observed intensity dynamic range for the sanity clamp
  if (!intensity_noise_has_prev_)
  {
    intensity_noise_min_ = intensity_noise_max_ = feats_down_world->points[0].intensity;
  }
  for (auto &p : feats_down_world->points)
  {
    if (p.intensity < intensity_noise_min_) { intensity_noise_min_ = p.intensity; }
    if (p.intensity > intensity_noise_max_) { intensity_noise_max_ = p.intensity; }
  }

  if (intensity_noise_has_prev_)
  {
    Eigen::Matrix3d R_rel = intensity_noise_prev_rot_.transpose() * state_.rot_end;
    double rot_deg = std::acos(std::max(-1.0, std::min(1.0, 0.5 * (R_rel.trace() - 1.0)))) * 180.0 / M_PI;
    bool stationary = (state_.pos_end - intensity_noise_prev_pos_).norm() < max_trans && rot_deg < max_rot_deg;

    if (stationary)
    {
      pcl::KdTreeFLANN<PointType> tree;
      tree.setInputCloud(intensity_noise_prev_cloud_);
      std::vector<int> idx(1);
      std::vector<float> sq_dist(1);
      for (auto &p : feats_down_world->points)
      {
        if (tree.nearestKSearch(p, 1, idx, sq_dist) > 0 && sq_dist[0] < pair_sq_dist_th)
        {
          intensity_noise_diffs_.push_back(p.intensity - intensity_noise_prev_cloud_->points[idx[0]].intensity);
        }
        if (intensity_noise_diffs_.size() >= max_pairs) { break; }
      }
    }
  }

  // Deep copy: feats_down_world is overwritten every frame
  intensity_noise_prev_cloud_.reset(new PointCloudXYZI(*feats_down_world));
  intensity_noise_prev_pos_ = state_.pos_end;
  intensity_noise_prev_rot_ = state_.rot_end;
  intensity_noise_has_prev_ = true;
  intensity_noise_frames_++;

  bool window_over = intensity_noise_frames_ >= window;
  bool enough = intensity_noise_diffs_.size() >= min_pairs;
  if (!(window_over && enough) && intensity_noise_frames_ < hard_stop && intensity_noise_diffs_.size() < max_pairs)
  {
    return;
  }

  intensity_noise_done_ = true;
  if (!enough)
  {
    // Estimation failed: do NOT model the intensity measurement noise for this
    // run (sigma_meas = 0). Intensity fusion/gate stay enabled and rely only on
    // the per-plane intensity statistics.
    std::cout << "[ IntensityNoise ]: FAILED (insufficient pairs: " << intensity_noise_diffs_.size()
              << " < " << min_pairs << ") - intensity measurement noise NOT considered (set to 0)" << std::endl;
    VoxelPlane::intensity_meas_var_ = 0.0;
    return;
  }

  double med = medianOf(intensity_noise_diffs_);
  std::vector<float> abs_dev(intensity_noise_diffs_.size());
  for (size_t i = 0; i < intensity_noise_diffs_.size(); i++)
  {
    abs_dev[i] = std::fabs(intensity_noise_diffs_[i] - med);
  }
  double mad = medianOf(abs_dev);

  double sigma = 1.4826 * mad / std::sqrt(2.0);
  // Sanity clamps: numeric floor, and at most 10% of the observed dynamic range
  double range = static_cast<double>(intensity_noise_max_ - intensity_noise_min_);
  if (range > 1e-6) { sigma = std::min(sigma, 0.1 * range); }
  sigma = std::max(sigma, 1e-3);

  VoxelPlane::intensity_meas_var_ = sigma * sigma;
  std::cout << "[ IntensityNoise ]: estimated sigma_meas = " << sigma << " (variance " << sigma * sigma
            << ") from " << intensity_noise_diffs_.size() << " pairs over " << intensity_noise_frames_
            << " frames" << std::endl;
  if (sigma < 0.5) // close to uint8 quantization noise (1/sqrt(12) ~ 0.29)
  {
    std::cout << "[ IntensityNoise ]: WARNING: near-zero noise, the intensity channel may be "
              << "constant/invalid - consider disabling intensity fusion/gate" << std::endl;
  }
}

// Execute LiDAR-Inertial Odometry (LIO) processing
void LIVMapper::handleLIO()
{
  // Record the propagated state before the update: relative timestamp, Euler
  // angles (deg), position, velocity, gyro bias, accel bias
  euler_cur = RotMtoEuler(state_.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - first_lidar_time_ << " " << euler_cur.transpose() * 57.3 << " "
           << state_.pos_end.transpose() << " " << state_.vel_end.transpose() << " " << state_.bias_g.transpose() << " "
           << state_.bias_a.transpose() << endl;

  // Check if undistorted point cloud is empty
  if (feats_undistort->empty())
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  // Point cloud downsampling
  double t0 = omp_get_wtime();
  downSizeFilterSurf.setInputCloud(feats_undistort);
  if (filter_size_surf_min == 0.0) {
    *feats_down_body = *feats_undistort; // filter disabled: copy instead of alias, so pillar deletion never truncates feats_undistort
  } else {
    downSizeFilterSurf.filter(*feats_down_body); // voxel filter (filter_size_surf)
  }
  double t_down = omp_get_wtime();

  // Transform downsampled cloud to world frame and feed the voxel map manager
  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  voxelmap_manager->TransformLidar(state_.rot_end, state_.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;

  // Auto-estimate intensity measurement noise during the init window (static platform)
  if (!intensity_noise_done_) { estimateIntensityNoise(); }

  double t_pillar1 = 0.0, t_pillar2 = 0.0;
  if (pillar_config.pillar_voxel_en_)
  {
    t_pillar1 = omp_get_wtime();
    voxelmap_manager->pillar_map_.BuildPillarMap(feats_down_world);
    voxelmap_manager->pillar_map_.DetectNewPoints();  // mark new points (no-op unless enabled)
    voxelmap_manager->pillar_map_.UpdateHistory();    // advance the n-frame window (every frame; also feeds redundant/isolated checks)
    voxelmap_manager->pillar_map_.pillarDetection();
    voxelmap_manager->DefineSkipPoints(feats_down_world);
    voxelmap_manager->pillar_map_.PublishPillarMapCloud(pubPillarMapCloud);
    voxelmap_manager->ClearPillarVoxels();

    // Delete flagged points from the frame outright: they neither contribute
    // ICP residuals nor enter the voxel map (pv_list_ is rebuilt from
    // feats_down_body_). Body/world clouds are compacted together inside.
    const size_t removed = voxelmap_manager->pillar_map_.removeFlaggedPoints(
        feats_down_body, feats_down_world, voxelmap_manager->skip_list_);
    if (removed > 0)
    {
      feats_down_size = feats_down_body->points.size();
      voxelmap_manager->feats_down_size_ = feats_down_size;
      std::cout << "[ Pillar ] Deleted " << removed << " points, kept " << feats_down_size << std::endl;
    }
    t_pillar2 = omp_get_wtime();
  }

  // Build voxel map on first run, based on octree structure. Runs after the
  // pillar pass so that first-frame flagged points are excluded as well.
  if (!lidar_map_inited)
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  // State estimation: ICP registration based on voxel map to estimate current frame pose
  double t1 = omp_get_wtime();
  voxelmap_manager->StateEstimation(state_propagat);
  state_ = voxelmap_manager->state_;
  double t2 = omp_get_wtime();

  // If IMU propagation is enabled, update related flags and state for high-frequency IMU propagation
  if (imu_prop_enable)
  {
    ekf_finish_once = true;
    latest_ekf_state = state_;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  savePoseTrajectory();

  // Publish odometry (also broadcasts the world -> aft_mapped tf)
  euler_cur = RotMtoEuler(state_.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  // Insert current frame point cloud into voxel map, update voxel map.
  // Per-point covariance combines measurement noise (rotated to world), pose
  // rotation uncertainty (via the point cross matrix) and pose translation
  // uncertainty; it weights each point in the next ICP update.
  double t3 = omp_get_wtime();
  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  voxelmap_manager->TransformLidar(state_.rot_end, state_.pos_end, feats_down_body, world_lidar);

  for (size_t i = 0; i < world_lidar->points.size(); i++)
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (state_.rot_end * extR) * var * (state_.rot_end * extR).transpose() +
          (-point_crossmat) * state_.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + state_.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  double t4 = omp_get_wtime();

  // Map sliding window: remove voxels far from current pose to save memory
  if (voxelmap_manager->config_setting_.map_sliding_en_)
  {
    voxelmap_manager->mapSliding();
  }

  // Select whether to publish dense or downsampled point cloud, then
  // transform it to the world frame for publishing
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  double t5 = omp_get_wtime();
  for (int i = 0; i < size; i++)
  {
    pointBodyToWorld(laserCloudFullRes->points[i], laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;
  double t6 = omp_get_wtime();

  publishAndSave();

  frame_num++;
  double t7 = omp_get_wtime();
  reportTiming(t0, t_down, t_pillar1, t_pillar2, t1, t2, t3, t4, t5, t6, t7);

  // Record the updated state after this frame's ICP correction
  euler_cur = RotMtoEuler(state_.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - first_lidar_time_ << " " << euler_cur.transpose() * 57.3 << " "
            << state_.pos_end.transpose() << " " << state_.vel_end.transpose() << " " << state_.bias_g.transpose() << " "
            << state_.bias_a.transpose() << " " << feats_undistort->points.size() << std::endl;
}

// Append the current pose to the evo trajectory file (evo/pose_output_en)
void LIVMapper::savePoseTrajectory()
{
  if (!pose_output_en) return;

  static bool pos_opend = false;
  std::ofstream evoFile;

  // Determine output directory
  std::string output_dir;
  if (pose_output_dir.empty())
  {
    output_dir = std::string(ROOT_DIR) + "Log/result/";
  }
  else
  {
    output_dir = pose_output_dir;
    // Ensure directory ends with '/'
    if (output_dir.back() != '/') output_dir += '/';
  }

  std::string output_path = output_dir + seq_name + ".txt";

  if (!pos_opend)
  {
    evoFile.open(output_path, std::ios::out);
    pos_opend = true;
  }
  else
  {
    evoFile.open(output_path, std::ios::app);
  }
  if (!evoFile.is_open())
  {
    ROS_ERROR("open fail\n");
    return;
  }

  Eigen::Quaterniond q(state_.rot_end);
  evoFile << std::fixed;
  evoFile << LidarMeasures.last_lio_update_time << " " << state_.pos_end[0] << " " << state_.pos_end[1] << " " << state_.pos_end[2] << " "
          << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

// Publish all clouds/path/mavros and save PCD (tail of handleLIO)
void LIVMapper::publishAndSave()
{
  publish_frame_world(pubLaserCloudFullRes); // Publish registered point cloud in world frame
  if (pub_body_en) publish_frame_body(pubLaserCloudBody); // Publish point cloud in body coordinate system
  if (pub_effect_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_); // Publish effective point cloud
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap(); // Publish voxel map
  if (save_en) save_frame_world(voxelmap_manager->ptpl_list_); // Save point cloud to file
  publish_path(pubPath); // Publish path
  publish_mavros(mavros_pose_publisher); // Publish MAVROS pose
}

// Accumulate per-stage timings and print the per-frame statistics table
void LIVMapper::reportTiming(double t0, double t_down, double t_pillar1, double t_pillar2, double t1, double t2,
                             double t3, double t4, double t5, double t6, double t7)
{
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t7 - t0) / frame_num;

  total_downsample_time += (t_down - t0);
  total_pillar_process_time += (t_pillar2 - t_pillar1);
  total_icp_time += (t2 - t1);
  total_update_voxel_map_time += (t4 - t3);
  total_point_transform_time += (t6 - t5);
  total_publish_save_time += (t7 - t6);

  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                        LIO Mapping Time                     |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-13s %-13s |\033[0m\n", "Algorithm Stage", "Current", "Average");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-13f %-13f |\033[0m\n", "DownSample", t_down - t0, total_downsample_time / frame_num);
  printf("\033[1;36m| %-29s | %-13f %-13f |\033[0m\n", "Pillar Process", pillar_config.pillar_voxel_en_ ? (t_pillar2 - t_pillar1) : 0.0, pillar_config.pillar_voxel_en_ ? (total_pillar_process_time / frame_num) : 0.0);
  printf("\033[1;36m| %-29s | %-13f %-13f |\033[0m\n", "ICP", t2 - t1, total_icp_time / frame_num);
  printf("\033[1;36m| %-29s | %-13f %-13f |\033[0m\n", "updateVoxelMap", t4 - t3, total_update_voxel_map_time / frame_num);
  printf("\033[1;36m| %-29s | %-13f %-13f |\033[0m\n", "Point Transform", t6 - t5, total_point_transform_time / frame_num);
  printf("\033[1;36m| %-29s | %-13f %-13f |\033[0m\n", "Publish and Save", t7 - t6, total_publish_save_time / frame_num);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t7 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  // Skip point statistics
  if (feats_down_world->points.size() > 0)
  {
    double current_skip_pct = 100.0 * voxelmap_manager->current_skip_count_ / feats_down_world->points.size();
    double avg_skip_pct = (voxelmap_manager->total_point_count_ > 0) ?
                          100.0 * voxelmap_manager->total_skip_count_ / voxelmap_manager->total_point_count_ : 0.0;
    printf("\033[1;36m| %-29s | %-13f %-13f |\033[0m\n", "Skip Points", current_skip_pct, avg_skip_pct);
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  }
}

void LIVMapper::savePCD()
{
  if (save_en && pcl_wait_save_intensity->points.size() > 0 && save_interval < 0)
  {
    std::string raw_points_dir = pcd_session_dir_ + "/all_raw_points.pcd";

    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
    std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
              << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
  }
}

void LIVMapper::run() 
{
  ros::Rate rate(5000);
  while (ros::ok()) 
  {
    ros::spinOnce();
    if (!sync_packages(LidarMeasures))
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    processImu();

    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback(const ros::TimerEvent &e)
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // Control propagate frequency to match IMU frequency
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = newest_imu.header.stamp.toSec() - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom.publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(state_.rot_end * (extR * p_body + extT) + state_.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));

  if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = msg->header.stamp.toSec();
  ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    ROS_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  if (!imu_en) return;
  
  if (last_timestamp_lidar < 0.0) return;
  // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);
  double timestamp = msg->header.stamp.toSec();

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = ros::Time().fromSec(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

void LIVMapper::odom_cbk(const nav_msgs::Odometry::ConstPtr &msg_in)
{
    // If external IMU not enabled, return directly
    if (!external_imu_enable) {
        return;
    }

    mtx_buffer.lock();

    double timestamp = msg_in->header.stamp.toSec() + external_imu_time_offset;

    // Check if timestamp is valid
    if (timestamp < last_timestamp_lidar) {
        mtx_buffer.unlock();
        return;
    }

    // Create external IMU data structure
    ExternalIMUData external_data;
    external_data.timestamp = timestamp;
    external_data.is_valid = true;

    // Extract position information
    external_data.position << msg_in->pose.pose.position.x,
                             msg_in->pose.pose.position.y,
                             msg_in->pose.pose.position.z;

    // Extract linear velocity information
    external_data.linear_velocity << msg_in->twist.twist.linear.x,
                                   msg_in->twist.twist.linear.y,
                                   msg_in->twist.twist.linear.z;

    // Extract linear velocity covariance information
    // nav_msgs Odometry twist.covariance is a 36-element array in row-major order
    // Linear velocity covariance at positions [0], [7], [14]
    external_data.velocity_covariance << msg_in->twist.covariance[0],    // x velocity variance
                                          msg_in->twist.covariance[7],    // y velocity variance
                                          msg_in->twist.covariance[14];   // z velocity variance

    // Extract orientation information (quaternion to rotation matrix)
    Eigen::Quaterniond q(msg_in->pose.pose.orientation.w,
                         msg_in->pose.pose.orientation.x,
                         msg_in->pose.pose.orientation.y,
                         msg_in->pose.pose.orientation.z);

    // Use extrinsic parameters from config file for coordinate transformation
    // First apply rotation transform from external IMU to internal IMU
    external_data.linear_velocity = external_imu_R * external_data.linear_velocity;

    // Transform velocity covariance
    // Covariance transformation formula: C_internal = R * C_external * R^T
    // For diagonal covariance matrix, full 3x3 transformation needed
    Eigen::Matrix3d external_cov_matrix = Eigen::Matrix3d::Zero();
    external_cov_matrix.diagonal() = external_data.velocity_covariance;
    Eigen::Matrix3d internal_cov_matrix = external_imu_R * external_cov_matrix * external_imu_R.transpose();
    external_data.velocity_covariance = internal_cov_matrix.diagonal();

    // Add to buffer and manage buffer size
    external_imu_buffer.push_back(external_data);
    if (external_imu_buffer.size() > external_imu_buffer_size) {
      external_imu_buffer.pop_front();
    }

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.ekf_state_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    lidar_pushed = false; // sync one whole lidar scan.
    return true;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed)
    {
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.ekf_state_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
}

void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes)
{
  if (pcl_w_wait_pub->empty()) return;

  /*** Publish Frame ***/
  sensor_msgs::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
  laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "world";
  pubLaserCloudFullRes.publish(laserCloudmsg);

  if(!save_en)
  {
    PointCloudXYZI().swap(*pcl_w_wait_pub);
  }
}

void LIVMapper::publish_frame_body(const ros::Publisher &pubLaserCloudBody)
{
  if (feats_undistort->empty()) return;

  sensor_msgs::PointCloud2 laserCloudBodyMsg;
  pcl::toROSMsg(*feats_undistort, laserCloudBodyMsg);
  laserCloudBodyMsg.header.stamp = ros::Time::now();
  laserCloudBodyMsg.header.frame_id = "body";
  pubLaserCloudBody.publish(laserCloudBodyMsg);
}

void LIVMapper::save_frame_world(const std::vector<PointToPlane> &ptpl_list)
{
  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (ptpl_list.empty()) return;
  if (save_en)
  {
    *pcl_wait_save_intensity += *pcl_w_wait_pub;
    scan_wait_num++;

    if (pcl_wait_save_intensity->size() > 0 && save_interval > 0 && scan_wait_num >= save_interval)
    {
      pcd_index++;
      string all_points_dir(pcd_session_dir_ + "/" + to_string(pcd_index) + ".pcd");

      pcl::PCDWriter pcd_writer;
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
      PointCloudXYZI().swap(*pcl_wait_save_intensity);
      Eigen::Quaterniond q(state_.rot_end);
      fout_pcd_pos << state_.pos_end[0] << " " << state_.pos_end[1] << " " << state_.pos_end[2] << " " << q.w() << " " << q.x() << " " << q.y()
                    << " " << q.z() << " " << endl;
      scan_wait_num = 0;
    }
  }
  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
    laserCloudWorld->points[i].intensity = ptpl_list[i].intensity_;
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time::now();
  laserCloudFullRes3.header.frame_id = "world";
  pubLaserCloudEffect.publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = state_.pos_end(0);
  out.position.y = state_.pos_end(1);
  out.position.z = state_.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "world";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = ros::Time::now(); 
  // odomAftMapped.header.stamp = ros::Time(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(state_.pos_end(0), state_.pos_end(1), state_.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br.sendTransform( tf::StampedTransform(transform, odomAftMapped.header.stamp, "world", "aft_mapped") );
  pubOdomAftMapped.publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "world";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher.publish(msg_body_pose);
}

void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "world";
  path.poses.push_back(msg_body_pose);
  pubPath.publish(path);
}