
#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <math.h>
#include <map>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <memory>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

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

  double sliding_thresh_;
  bool map_sliding_en_;
  int half_map_size_;

  int capacity_;
  bool intensity_fusion_en_;
  bool intensity_gate_en_;
  double intensity_gate_k_;

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
  // Whether batch intensity statistics have been initialized at least once
  // (after that, stats evolve only via EMA so re-inits never wipe history)
  bool intensity_init_ = false;
  // Total number of observations folded into the intensity statistics
  // (maturity condition for the intensity gate)
  int intensity_obs_count_ = 0;
  // Squared per-point intensity measurement noise (sigma_meas^2), estimated online
  // during the init window (frame-to-frame differencing on static data).
  // Added at use sites so the effective variance never collapses to zero
  // (replaces the former hard std floor of 1e-3). Conservative default until ready.
  static double intensity_meas_var_;
  // EMA alpha for intensity statistics update (higher = faster adaptation).
  // Configurable via lio/intensity_ema_alpha, clamped to (0, 1]
  static double intensity_ema_alpha_;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

class VoxelLocation
{
public:
  int64_t x, y, z;

  VoxelLocation(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VoxelLocation &other) const { return (x == other.x && y == other.y && z == other.z); }
};

class PillarLocation
{
public:
  int64_t axis1, axis2;

  PillarLocation(int64_t v1 = 0, int64_t v2 = 0) : axis1(v1), axis2(v2) {}

  bool operator==(const PillarLocation &other) const { return (axis1 == other.axis1 && axis2 == other.axis2); }
};

// Composite key of one pillar voxel: (x,y) pillar index + z voxel index.
// Struct instead of std::pair: named members match the VoxelLocation/PillarLocation
// style, and a custom hash is required either way (std::hash has no pair
// specialization), so the struct costs nothing extra.
struct PillarVoxelKey
{
  PillarLocation pillar;
  int64_t z;

  PillarVoxelKey(const PillarLocation &p = PillarLocation(), int64_t vz = 0) : pillar(p), z(vz) {}

  bool operator==(const PillarVoxelKey &other) const { return (pillar == other.pillar && z == other.z); }
};

namespace std
{
// Note: y/z are folded with a fixed modulus while x is added unmoded, so the
// intermediate products stay within int64 only for voxel indices up to ~1e13.
// That is far beyond any physical map extent (voxel_size 0.5 m -> 5e12 m).
template <> struct hash<VoxelLocation>
{
  int64_t operator()(const VoxelLocation &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};

template <> struct hash<PillarLocation>
{
  int64_t operator()(const PillarLocation &s) const
  {
    using std::hash;
    using std::size_t;
    return (((s.axis2) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.axis1));
  }
};

template <> struct hash<PillarVoxelKey>
{
  int64_t operator()(const PillarVoxelKey &s) const
  {
    using std::hash;
    using std::size_t;
    // Fold z with the fixed modulus, then add the existing pillar hash unmodded
    // (same pattern as hash<VoxelLocation>)
    return ((s.z * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + hash<PillarLocation>()(s.pillar));
  }
};
} // namespace std

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree
{

public:
  std::vector<pointWithVar> temp_points_;
  std::vector<size_t> point_indices_;
  VoxelPlane *plane_ptr_;
  int layer_;
  int octo_state_;
  VoxelOctoTree *leaves_[8];
  double voxel_center_[3];
  std::vector<int> layer_init_num_;
  float quater_length_;
  float planner_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;
  bool is_redundant_voxel_ = false;
  bool is_isolated_voxel_ = false;

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planner_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planner_threshold_(planner_threshold), is_redundant_voxel_(false), is_isolated_voxel_(false)
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
  LABEL_REDUNDANT = 1,
  LABEL_ISOLATED = 2
};

// Lightweight voxel structure for Pillar Voxel Map (decoupled from VoxelOctoTree)
// Designed specifically for redundant/isolated point detection without octree overhead
struct PillarVoxel
{
  std::vector<size_t> point_indices_;  // Point indices in original point cloud
  double center_z_;                    // Voxel center Z coordinate (X,Y derived from PillarLocation)
  Eigen::Vector3d virtual_point_ = Eigen::Vector3d::Zero();  // Average XYZ of inserted points
  size_t point_count_ = 0;            // Number of accumulated points for running average
  bool is_redundant_voxel_ = false;
  bool is_isolated_voxel_ = false;
  bool is_new_voxel_ = false;          // set by DetectNewPoints: voxel unseen in the last history_frame_num frames

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
    is_redundant_voxel_ = false;
    is_isolated_voxel_ = false;
    is_new_voxel_ = false;
    virtual_point_.setZero();
    point_count_ = 0;
  }
};

typedef struct PillarVoxelConfig
{
  bool pillar_voxel_en_;
  double voxel_size_;
  int adjacent_redundant_threshold_;
  int keep_num_per_voxel_;   // 0=skip all, n=keep n newest points per redundant voxel
  bool keep_redundant_;      // true=apply keep_num_per_voxel to redundant voxels, false=skip all
  bool keep_isolated_;       // true=apply keep_num_per_voxel to isolated voxels, false=skip all
  int adjacent_isolated_threshold_;
  int min_num_;                  // redundant voxel needs point_count_ > this, isolated voxel needs < this (default: 5)
  int redundant_neighbor_type_;     // 0=4-neighbor, 1=8-neighbor (for redundant detection)
  int isolated_neighbor_type_;   // 0=4-neighbor, 1=8-neighbor (for isolated detection)
  double height_consistency_ratio_;  // ratio of voxel_size for height consistency check (default: 0.25)
  bool new_point_detect_en_;     // mark points whose pillar voxel was unseen in the last n frames (default: false)
  int history_frame_num_;        // history reference frame count n; detection starts at frame n+1 (n<=0: no reference kept, every point new)
  bool keep_new_point_;          // true=apply keep_num_per_voxel retention to new voxels, false=skip all new points
  int adjacent_new_point_threshold_;  // candidate new voxel confirmed only if occupied same-layer neighbors < this (0=check off)

  PillarVoxelConfig() : pillar_voxel_en_(false), voxel_size_(1.0), adjacent_redundant_threshold_(3),
                       keep_num_per_voxel_(0), keep_redundant_(true), keep_isolated_(false),
                       adjacent_isolated_threshold_(3), min_num_(5),
                       redundant_neighbor_type_(1), isolated_neighbor_type_(1),
                       height_consistency_ratio_(0.25), new_point_detect_en_(false), history_frame_num_(10),
                       keep_new_point_(true), adjacent_new_point_threshold_(0) {}
} PillarVoxelConfig;

void loadPillarVoxelConfig(ros::NodeHandle &nh, PillarVoxelConfig &config);

// Per-pillar voxel array sorted by z key (built once per frame in BuildPillarMap).
// Flat vector instead of std::map: most pillars hold only a few z voxels and the
// whole structure is rebuilt every frame, so cache locality and allocation count
// beat tree lookups.
typedef std::vector<std::pair<int64_t, PillarVoxel>> PillarVoxelArray;

class PillarVoxelMap
{
public:
  PillarVoxelMap() = default;
  PillarVoxelConfig config_;
  double voxel_size_;
  std::unordered_map<PillarLocation, PillarVoxelArray> pillars_;
  std::vector<VoxelLocation> redundant_neighbor_offsets_;
  std::vector<VoxelLocation> isolated_neighbor_offsets_;

  std::vector<int8_t> point_labels_;
  PointCloudXYZI::Ptr point_cloud_ptr_;

  // Count of voxels flagged as redundant/isolated candidate during Step 1 of pillarDetection.
  // Used for early-exit: if 0 after Step 1, skip the remaining steps entirely.
  size_t voxel_label_count_ = 0;

  // --- New point detection (n-frame history reference window) ---
  // Occupied voxel keys per frame, newest at back; at most history_frame_num_
  // frames retained. Deliberately survives ClearPillarVoxels().
  std::deque<std::vector<PillarVoxelKey>> history_frames_;
  // Voxel key -> number of frames of the current window containing it.
  // "New" <=> key absent here (checked before the current frame is inserted)
  std::unordered_map<PillarVoxelKey, int> history_counts_;
  // Per-point new flag, index-aligned with point_cloud_ptr_, reset each frame in
  // DetectNewPoints(). Kept separate from point_labels_: a point can be new AND
  // redundant/isolated; deletion/retention is decided per keep_new_point in
  // DefineSkipPoints()
  std::vector<int8_t> point_is_new_;
  // Frames processed since startup while the feature is enabled; detection
  // output starts at frame history_frame_num_ + 1 (earlier frames only
  // accumulate the history window)
  size_t history_frame_count_ = 0;

  void init(const PillarVoxelConfig &config, double voxel_size);
  void BuildPillarMap(const PointCloudXYZI::Ptr &input_cloud);
  void DetectNewPoints();
  void pillarDetection();
  size_t removeFlaggedPoints(const PointCloudXYZI::Ptr &body_cloud, const PointCloudXYZI::Ptr &world_cloud,
                             std::vector<bool> &skip_flags);
  void PublishPillarPoints(const ros::Publisher &pubRedundant, const ros::Publisher &pubIsolated);
  void PublishNewPoints(const ros::Publisher &pubNew);

  inline int8_t GetPointLabel(size_t index) const {
    return (index < point_labels_.size()) ? point_labels_[index] : LABEL_NORMAL;
  }

  inline int8_t GetPointIsNew(size_t index) const {
    return (index < point_is_new_.size()) ? point_is_new_[index] : 0;
  }

private:
  void setVoxelPointLabels(PillarVoxel* voxel, int8_t label);
  void initHorizontalNeighborOffsets();
  PillarLocation GetPillarLocation(const VoxelLocation &position) const;
  void updatePillarFlag(PillarVoxelArray &pillar_voxels);
  bool hasAdjacentVoxel(const VoxelLocation &current_pos, int threshold, const std::vector<VoxelLocation> &neighbor_offsets, double current_vp_z);
};

class VoxelMapManager
{
public:
  VoxelMapConfig config_setting_;
  int current_frame_id_ = 0;
  ros::Publisher voxel_map_pub_;

  // LRU voxel cache: voxel_map_cache_ front = most recently updated voxel.
  // voxel_map_ maps location -> list iterator for O(1) hit/splice/evict.
  // No lock is needed: all writes happen on the sequential per-frame pipeline
  // and the OpenMP section (BuildResidualListOMP) only reads the containers.
  std::list<std::pair<VoxelLocation, VoxelOctoTree*>> voxel_map_cache_;
  std::unordered_map<VoxelLocation, std::list<std::pair<VoxelLocation, VoxelOctoTree*>>::iterator> voxel_map_;

  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;

  M3D extR_;
  V3D extT_;
  StatesGroup state_;
  V3D position_last_;

  V3D last_slide_position_ = {0,0,0};

  int feats_down_size_;
  int effect_feat_num_;
  std::vector<M3D> cross_mat_list_;
  std::vector<M3D> body_cov_list_;
  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;
  std::vector<bool> skip_list_;

  // Skip point statistics
  int current_skip_count_ = 0;
  int total_skip_count_ = 0;
  int total_point_count_ = 0;

  PillarVoxelMap pillar_map_;

  // Total voxels evicted by the LRU since start (statistics for capacity tuning)
  size_t evicted_voxel_count_ = 0;

  explicit VoxelMapManager(const VoxelMapConfig &config_setting) : config_setting_(config_setting)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    // Pre-bucket for the expected steady-state size to avoid rehash churn
    if (config_setting_.capacity_ > 1) { voxel_map_.reserve(config_setting_.capacity_); }
  };

  void StateEstimation(StatesGroup &state_propagat);
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      PointCloudXYZI::Ptr &trans_cloud);

  void BuildVoxelMap();

  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  void DefineSkipPoints(const PointCloudXYZI::Ptr &feats_down_world);

  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_success, double &prob,
                             PointToPlane &single_ptpl);

  void pubVoxelMap();

  void mapSliding();
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );
  void ClearPillarVoxels();

private:
  // Evict least-recently-updated voxels from the LRU tail until the cache is
  // within capacity. No-op when capacity is disabled (<= 1: 0 means off, and 1
  // would degenerate into evicting the entry right after inserting it).
  // Returns the number of voxels evicted by this call.
  size_t enforceCapacity();

  // Shared retention pass for DefineSkipPoints: keep the newest keep_num points
  // per flagged voxel (point_indices_ tail), mark the rest in skip_list_.
  // voxel_class: 0 = redundant/isolated voxels, 1 = new-point voxels only
  void applyVoxelRetention(int keep_num, int &flagged_total, int &flagged_kept, int &final_skip_count,
                           int voxel_class = 0);

  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_