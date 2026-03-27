/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <map>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <memory>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

static int voxel_plane_id = 0;

typedef struct VoxelMapConfig
{
  double max_voxel_size_;
  int max_layer_;
  int max_iterations_;
  std::vector<int> layer_init_num_;
  int max_points_num_;
  double planner_threshold_;
  double beam_err_;
  double dept_err_;
  double sigma_num_;
  bool is_pub_plane_map_;

  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;

  int capacity;
  bool rf_enhance_en_;
  bool intensity_fusion_en_;

} VoxelMapConfig;

typedef struct PointToPlane
{
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  M3D body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
  float intensity_;
} PointToPlane;

typedef struct VoxelPlane
{
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  double mean_intensity_ = 0.0f;
  double intensity_std_ = 1.0f;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

class PILLAR_LOCATION
{
public:
  int64_t axis1, axis2;

  PILLAR_LOCATION(int64_t v1 = 0, int64_t v2 = 0) : axis1(v1), axis2(v2) {}

  bool operator==(const PILLAR_LOCATION &other) const { return (axis1 == other.axis1 && axis2 == other.axis2); }
};

namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};

template <> struct hash<PILLAR_LOCATION>
{
  int64_t operator()(const PILLAR_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return (((s.axis2) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.axis1));
  }
};
} // namespace std

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree
{

public:
  VoxelOctoTree() = default;
  std::vector<pointWithVar> temp_points_;
  std::vector<size_t> point_indices_;
  VoxelPlane *plane_ptr_;
  int layer_;
  int octo_state_;
  VoxelOctoTree *leaves_[8];
  double voxel_center_[3];
  std::vector<int> layer_init_num_;
  float quater_length_;
  float planer_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;
  bool is_ground_voxel_ = false;
  bool is_isolated_voxel_ = false;
  bool is_surface_voxel_ = false;

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold), is_ground_voxel_(false), is_isolated_voxel_(false), is_surface_voxel_(false)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
      leaves_[i] = nullptr;
    }
    delete plane_ptr_;
    plane_ptr_ = nullptr;
  }
  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);
  void init_octo_tree();
  void cut_octo_tree();
  void UpdateOctoTree(const pointWithVar &pv);

  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
  VoxelOctoTree *Insert(const pointWithVar &pv);
};

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

// Point label enumeration for direct label access (3-pass scan merge optimization)
enum PointLabel
{
  LABEL_NORMAL = 0,
  LABEL_GROUND = 1,
  LABEL_ISOLATED = 2
};

// Lightweight voxel structure for Pillar Voxel Map (decoupled from VoxelOctoTree)
// Designed specifically for ground/isolated point detection without octree overhead
struct PillarVoxel
{
  std::vector<size_t> point_indices_;  // Point indices in original point cloud
  double center_z_;                    // Voxel center Z coordinate (X,Y derived from PILLAR_LOCATION)
  bool is_ground_voxel_ = false;
  bool is_isolated_voxel_ = false;

  PillarVoxel(double z = 0.0) : center_z_(z)
  {
    point_indices_.reserve(10);  // Pre-allocate to reduce reallocations
  }

  // Disable copy to avoid deep copy of vector
  PillarVoxel(const PillarVoxel&) = delete;
  PillarVoxel& operator=(const PillarVoxel&) = delete;

  // Enable move semantics
  PillarVoxel(PillarVoxel&&) = default;
  PillarVoxel& operator=(PillarVoxel&&) = default;

  // Clear state for reuse (optional, for object pool pattern)
  void clear()
  {
    point_indices_.clear();
    is_ground_voxel_ = false;
    is_isolated_voxel_ = false;
  }
};

typedef struct PillarVoxelConfig
{
  bool pillar_voxel_en_;
  double voxel_size_;
  int min_adjacent_ground_num_;
  int min_adjacent_isolated_num_;
  int ground_detection_method_;
  double plane_fitting_distance_threshold_;
  int skip_type_;

  PillarVoxelConfig() : pillar_voxel_en_(false), voxel_size_(1.0), min_adjacent_ground_num_(3),
                       min_adjacent_isolated_num_(3),
                       ground_detection_method_(0), plane_fitting_distance_threshold_(0.1),
                       skip_type_(0) {}
} PillarVoxelConfig;

void loadPillarVoxelConfig(ros::NodeHandle &nh, PillarVoxelConfig &config);

class PillarVoxelMap
{
public:
  PillarVoxelMap() = default;
  PillarVoxelConfig config_;
  double voxel_size_;
  std::unordered_map<PILLAR_LOCATION, std::map<int64_t, PillarVoxel>> pillars_;
  std::unordered_set<PILLAR_LOCATION> current_pillars_;
  std::vector<VOXEL_LOCATION> neighbor_offsets_;

  std::vector<int8_t> point_labels_;
  PointCloudXYZI::Ptr point_cloud_ptr_;

  Eigen::Vector3d fitted_plane_normal_ = Eigen::Vector3d::Zero();
  double fitted_plane_d_ = 0.0;
  bool plane_fitted_ = false;
  void init(const PillarVoxelConfig &config, double voxel_size);
  void BuildPillarMap(const PointCloudXYZI::Ptr &input_cloud);
  void GroundDetection(const Eigen::Vector3d& current_pos);
  void PublishPillarPoints(const ros::Publisher &pubGround, const ros::Publisher &pubIsolated);

  inline int8_t GetPointLabel(size_t index) const {
    return (index < point_labels_.size()) ? point_labels_[index] : LABEL_NORMAL;
  }

private:
  void initHorizontalNeighborOffsets();
  PILLAR_LOCATION GetPillarLocation(const VOXEL_LOCATION &position) const;
  void UpdateGroundFlagForPillar(const PILLAR_LOCATION &pillar_key, std::map<int64_t, PillarVoxel> &pillar_voxels, const Eigen::Vector3d& current_pos);
  bool hasAdjacentGroundVoxel(const PillarVoxel *voxel, const VOXEL_LOCATION &current_pos);
  bool hasAdjacentTopVoxel(const VOXEL_LOCATION &current_pos);
};

class VoxelMapManager
{
public:
  VoxelMapManager() = default;
  VoxelMapConfig config_setting_;
  int current_frame_id_ = 0;
  ros::Publisher voxel_map_pub_;

  std::mutex voxel_cache_mutex_;
  std::list<std::pair<VOXEL_LOCATION, VoxelOctoTree*>> voxel_map_cache_;
  std::unordered_map<VOXEL_LOCATION, std::list<std::pair<VOXEL_LOCATION, VoxelOctoTree*>>::iterator> voxel_map_;

  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;

  M3D extR_;
  V3D extT_;
  float build_residual_time, ekf_time;
  float ave_build_residual_time = 0.0;
  float ave_ekf_time = 0.0;
  int scan_count = 0;
  StatesGroup state_;
  V3D position_last_;

  V3D last_slide_position = {0,0,0};

  geometry_msgs::Quaternion geoQuat_;

  int feats_down_size_;
  int effct_feat_num_;
  std::vector<M3D> cross_mat_list_;
  std::vector<M3D> body_cov_list_;
  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;
  std::vector<bool> skip_list;

  PillarVoxelMap pillar_map_;

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, std::list<std::pair<VOXEL_LOCATION, VoxelOctoTree*>>::iterator> &voxel_map)
    : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
  };

  void StateEstimation(StatesGroup &state_propagat);
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  void BuildVoxelMap();
  V3F RGBFromVoxel(const V3D &input_point);

  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  void DefineSkipPoints(const PointCloudXYZI::Ptr &feats_down_world);

  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, bool &is_surface, double &prob,
                             PointToPlane &single_ptpl);

  void pubVoxelMap();

  void mapSliding();
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );
  void ClearPillarVoxels();

private:
  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_