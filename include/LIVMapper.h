
#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "preprocess.h"
#include "voxel_map.h"
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <condition_variable>

class LIVMapper
{
public:
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();

  // External IMU variables
  deque<ExternalIMUData> external_imu_buffer;
  bool external_imu_enable = false;
  double external_imu_time_offset = 0.0;
  int external_imu_buffer_size = 1000;
  bool external_imu_only = false;
  ros::Subscriber sub_external_imu;
  string external_imu_topic;
  void odom_cbk(const nav_msgs::Odometry::ConstPtr &msg_in);

  // External IMU extrinsics
  V3D external_imu_T;
  M3D external_imu_R;
  std::vector<double> external_imu_T_vec, external_imu_R_vec;

  void initializeSubscribersAndPublishers(ros::NodeHandle &nh);
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleLIO();
  void savePCD();
  void processImu();
  void estimateIntensityNoise();

  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback(const ros::TimerEvent &e);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes);
  void save_frame_world(const std::vector<PointToPlane> &ptpl_list);
  void publish_frame_body(const ros::Publisher &pubLaserCloudBody);

  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  void publish_path(const ros::Publisher pubPath);
  void readParameters(ros::NodeHandle &nh);
  template <typename T> void set_posestamp(T &out);

private:
  void publishAndSave();      // publish clouds/path/mavros and save PCD (tail of handleLIO)
  void savePoseTrajectory();  // append current pose to the evo trajectory file (if enabled)
  void reportTiming(double t0, double t_down, double t_pillar1, double t_pillar2, double t1, double t2,
                    double t3, double t4, double t5, double t6, double t7);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;

  string root_dir;
  string lid_topic, imu_topic, seq_name, pose_output_dir;
  V3D extT;
  M3D extR;

  int feats_down_size = 0;

  double gyr_cov = 0, acc_cov = 0;
  double b_gyr_cov = 0, b_acc_cov = 0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double first_lidar_time_ = 0.0;

  bool lidar_map_inited = false, save_en = false, pub_effect_en = false, pub_body_en = false, pose_output_en = false, ros_driver_fix_en = false;

  int save_interval = -1, pcd_index = 0, scan_wait_num = 0;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  ros::Publisher pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, ba_bg_est_en = true;
  bool dense_map_en = false;
  int imu_int_frame = 3, external_imu_int_frame = 3;

  // ---- Online intensity noise estimation (init window, static platform) ----
  // Frame-to-frame NN intensity differencing estimates sigma_meas, which is
  // stored in VoxelPlane::intensity_meas_var_ and used by the intensity
  // fusion scoring and association gate in the voxel map.
  bool intensity_noise_est_en_ = true;  // lio/intensity_noise_est_en: enable online estimation
  bool intensity_noise_done_ = false;
  bool intensity_noise_has_prev_ = false;
  int intensity_noise_frames_ = 0;
  float intensity_noise_min_ = 0.0f, intensity_noise_max_ = 0.0f;
  std::vector<float> intensity_noise_diffs_;
  PointCloudXYZI::Ptr intensity_noise_prev_cloud_ = nullptr;
  V3D intensity_noise_prev_pos_ = V3D::Zero();
  M3D intensity_noise_prev_rot_ = M3D::Identity();
  int lidar_en = 1;
  bool is_first_frame = false;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  vector<double> extrinT;
  vector<double> extrinR;

  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_pcd_pos;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup state_;
  StatesGroup state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  PillarVoxelConfig pillar_config;

  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubRedundantCloud;
  ros::Publisher pubIsolatedCloud;
  ros::Publisher pubNewPointsCloud;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  ros::Publisher pubLaserCloudBody;
  ros::Publisher mavros_pose_publisher;
  ros::Timer imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double total_downsample_time = 0;
  double total_icp_time = 0;
  double total_update_voxel_map_time = 0;
  double total_point_transform_time = 0;
  double total_publish_save_time = 0;
  double total_pillar_process_time = 0;

  std::string session_timestamp_;
  std::string pcd_session_dir_;

};
#endif
