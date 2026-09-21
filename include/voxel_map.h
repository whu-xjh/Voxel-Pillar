
#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <fstream>
#include <math.h>
#include <map>
#include <omp.h>
#include <pcl/common/io.h>
#include <pcl/segmentation/extract_clusters.h>
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

  // R-VoxelMap-style plane-init refinements
  bool plane_refine_en_;              // master switch: false = fully revert to original behavior (no distance filter, no check_and_update)
  double init_distance_threshold_;    // init_plane: drop points farther than this from the first PCA fit (m)
  bool plane_valid_check_en_;         // coplanar-disjoint-surface guard (2D grid DFS clustering)
  int valid_check_max_layer_;         // run the guard only on voxels at/above this layer
  int valid_check_min_points_size_;   // min retained points to run the guard
  int valid_check_resolution_;        // projection grid resolution factor
  double valid_check_p_threshold_;    // largest-cluster point-ratio threshold

  double sliding_thresh_;
  bool map_sliding_en_;
  int half_map_size_;

  int capacity_;
  bool intensity_fusion_en_;
  bool intensity_gate_en_;
  double intensity_gate_k_;
  double intensity_fusion_weight_;  // intensity weight in the joint association score (0.5 = equal trust; default: 0.8)

  // Degeneracy-adaptive intensity fusion (requires intensity_fusion_en)
  bool degeneracy_adaptive_en_;      // engage intensity fusion only in degenerate scenes (default: false)
  double degeneracy_on_threshold_;   // enter degenerate mode when λ_min/λ_max < this (default: 0.1)
  double degeneracy_off_threshold_;  // exit hysteresis threshold; must be > on (default: 0.2)
  int degeneracy_min_frames_;        // consecutive frames to switch state (debounce; default: 2)

  // Incidence-angle compensation for the intensity channel (Intensity-SLAM-style):
  // gain = max(|cosθ|, floor)^k with θ the angle between the laser beam and the
  // matched plane normal; corrected intensity = raw / gain, variance / gain².
  // Plane intensity statistics then express material (normal-incidence
  // reflectivity) instead of viewing geometry. Bit-identical behavior when off
  bool intensity_angle_comp_en_;      // master switch (default: false)
  double intensity_angle_exp_;        // Lambertian exponent k (default: 1.0; <= 0 also disables)
  double intensity_angle_cos_floor_;  // cosθ clip floor against grazing-angle blow-up (default: 0.2)
  bool intensity_angle_debug_en_;     // dump matched (intensity, gain) pairs to Log/intensity_angle.txt (default: false)

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
  // Geometric incidence-angle gain (max(|cosθ|, floor)^k) of the matched
  // association, recorded for the intensity_angle_debug_en_ validation dump
  // regardless of whether compensation is enabled
  float intensity_gain_ = 1.0f;
} PointToPlane;

typedef struct VoxelPlane
{
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  // Incremental sufficient statistics (Σp·pᵀ over stored points), maintained
  // by init_plane and check_and_update for O(1) insertion trials
  Eigen::Matrix3d sum_ppt_;
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
  // Set when accepted insertions shift the incremental statistics. plane_var_
  // is currently recomputed (eagerly) at the next refit; the flag is kept as
  // the hook for a future lazy refresh
  bool cov_need_update_ = false;
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
struct PillarMapKey
{
  PillarLocation pillar;
  int64_t z;

  PillarMapKey(const PillarLocation &p = PillarLocation(), int64_t vz = 0) : pillar(p), z(vz) {}

  bool operator==(const PillarMapKey &other) const { return (pillar == other.pillar && z == other.z); }
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

template <> struct hash<PillarMapKey>
{
  int64_t operator()(const PillarMapKey &s) const
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
  const VoxelMapConfig *config_ptr_ = nullptr;  // plane-init refinement parameters (outlier distance, valid check)

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planner_threshold,
                const VoxelMapConfig *config_ptr)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planner_threshold_(planner_threshold), is_redundant_voxel_(false), is_isolated_voxel_(false), config_ptr_(config_ptr)
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
  void init_plane(std::vector<pointWithVar> &points, VoxelPlane *plane);
  void init_octo_tree();
  void cut_octo_tree();
  // R-VoxelMap check_and_update: O(1) rank-1 trial — accept the point only if
  // the min eigenvalue of the augmented covariance stays below the planarity
  // threshold; on acceptance the incremental statistics are updated in place
  bool check_and_update(const pointWithVar &pv);
  // Coplanar-disjoint-surface guard (ported from R-VoxelMap): prune points to
  // the largest 4-connected cluster of the plane-projected 2D grid. Returns
  // false when even the largest cluster is too small to trust the plane
  bool plane_valid_check(std::vector<pointWithVar> &points, const Eigen::Vector3d &center,
                         const Eigen::Vector3d &x_normal, const Eigen::Vector3d &y_normal);
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
struct PillarMapVoxel
{
  std::vector<size_t> point_indices_;  // Point indices in original point cloud
  double center_z_;                    // Voxel center Z coordinate (X,Y derived from PillarLocation)
  Eigen::Vector3d virtual_point_ = Eigen::Vector3d::Zero();  // Average XYZ of inserted points
  size_t point_count_ = 0;            // Number of accumulated points for running average
  bool is_redundant_voxel_ = false;
  bool is_isolated_voxel_ = false;
  bool is_new_voxel_ = false;          // set by DetectNewPoints: voxel unseen in the last pillar_buffer frames

  PillarMapVoxel(double z = 0.0) : center_z_(z)
  {
    point_indices_.reserve(10);  // Pre-allocate to reduce reallocations
  }

  // Disable copy to avoid deep copy of vector
  PillarMapVoxel(const PillarMapVoxel&) = delete;
  PillarMapVoxel& operator=(const PillarMapVoxel&) = delete;

  // Enable move semantics
  PillarMapVoxel(PillarMapVoxel&&) = default;
  PillarMapVoxel& operator=(PillarMapVoxel&&) = default;

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

typedef struct PillarMapConfig
{
  bool pillar_map_en_;
  double voxel_size_;
  int adjacent_redundant_threshold_;
  bool delete_redundant_;    // true=delete redundant points before they enter the voxel map, false=keep them (default: false)
  bool delete_isolated_;     // true=delete isolated points before they enter the voxel map, false=keep them (default: false)
  int adjacent_isolated_threshold_;
  int neighbor_ring_num_;        // max ring probed by hasAdjacentVoxel (horizontal-only): 1 = ring 1 only (4 face neighbors at distance 1), 2 = + ring 2 (4 diagonals at sqrt(2); together the horizontal 8-neighborhood) (default: 1)
  bool dyn_bridge_display_;      // display-only bridge: re-publish stale points of intermittently detected targets, red (default: false; needs dyn_buffer_max_age_ > 0; the buffer stores points only when this is on)
  int dyn_buffer_max_age_;       // dyn_buffer_ freshness window: buffer aging/prune AND the rescue-sweep pooling horizon (default: 3; <=0: buffer off)
  int min_num_;                  // redundant voxel needs point_count_ > this, isolated voxel needs < this (default: 5)
  double height_consistency_ratio_;  // ratio of voxel_size for the height-consistency gate on neighbors (all horizontal, same-layer) (default: 0.25)
  bool dyn_detect_en_;     // mark points whose pillar voxel was unseen in the last n frames (default: false)
  int pillar_buffer_;            // n-frame pillar occupancy window; detection starts at frame n+1 (n<=0: no reference kept, every point new)
  bool delete_dyn_;        // true=delete new points before they enter the voxel map, false=keep them (default: false)
  bool dyn_cluster_en_;    // cluster candidate new points, keep only valid clusters (default: false)
  int dyn_cluster_min_num_;  // min points per cluster to confirm as new (default: 5)
  bool dyn_cluster_expansion_;  // rescue sweep: pool current-frame points sitting in recent dyn_buffer_ voxels into the cluster (default: false; needs dyn_cluster_en_ + dyn_buffer_max_age_ > 0)

  PillarMapConfig() : pillar_map_en_(false), voxel_size_(1.0), adjacent_redundant_threshold_(3),
                       delete_redundant_(false), delete_isolated_(false),
                       adjacent_isolated_threshold_(3), neighbor_ring_num_(1),
                       dyn_bridge_display_(false), dyn_buffer_max_age_(3),
                       min_num_(5),
                       height_consistency_ratio_(0.25), dyn_detect_en_(false), pillar_buffer_(10),
                       delete_dyn_(false),
                       dyn_cluster_en_(false), dyn_cluster_min_num_(5), dyn_cluster_expansion_(false) {}
} PillarMapConfig;

void loadPillarMapConfig(ros::NodeHandle &nh, PillarMapConfig &config);

// Per-pillar voxel array sorted by z key (built once per frame in BuildPillarMap).
// Flat vector instead of std::map: most pillars hold only a few z voxels and the
// whole structure is rebuilt every frame, so cache locality and allocation count
// beat tree lookups.
typedef std::vector<std::pair<int64_t, PillarMapVoxel>> PillarMapArray;

class PillarMap
{
public:
  PillarMap() = default;
  PillarMapConfig config_;
  double voxel_size_;
  std::unordered_map<PillarLocation, PillarMapArray> pillars_;
  // Two-ring horizontal neighborhood (fixed geometry): ring 1 = 4 face
  // neighbors at distance 1; ring 2 = 4 horizontal diagonals at distance
  // sqrt(2) — together the horizontal 8-neighborhood. Vertical continuity is
  // judged in updatePillarFlag, so all offsets are same-layer (dz = 0).
  // Ring 1 is always probed first, ring 2 only when the threshold is not yet met
  std::vector<VoxelLocation> ring1_offsets_;
  std::vector<VoxelLocation> ring2_offsets_;

  std::vector<int8_t> point_labels_;
  PointCloudXYZI::Ptr point_cloud_ptr_;

  // Count of voxels flagged as redundant/isolated candidate during Step 1 of pillarDetection.
  // Used for early-exit: if 0 after Step 1, skip the remaining steps entirely.
  size_t voxel_label_count_ = 0;

  // --- Pillar map history (n-frame occupancy reference window) ---
  // Occupied voxel keys per frame, newest at back; at most pillar_buffer_
  // frames retained. Maintained by UpdateHistory() every frame; deliberately
  // survives ClearPillarMapVoxels(). Consumed by new-point detection AND the
  // redundant/isolated vertical-continuity checks
  std::deque<std::vector<PillarMapKey>> history_frames_;
  // Voxel key -> number of frames of the current window containing it.
  // "New" <=> key absent here (checked before the current frame is inserted)
  std::unordered_map<PillarMapKey, int> history_counts_;
  // Per-point new flag, index-aligned with point_cloud_ptr_, reset each frame in
  // DetectNewPoints(). Kept separate from point_labels_: a point can be new AND
  // redundant/isolated; deletion is decided per delete_dyn in
  // DefineSkipPoints()
  std::vector<int8_t> point_is_new_;
  // Frames processed since startup; the new-point gate uses it so detection
  // output starts at frame pillar_buffer_ + 1 (earlier frames only
  // accumulate the history window)
  size_t history_frame_count_ = 0;

  // --- Dynamic-point buffer (M-Detector umap-style) ---
  // Confirmed new-point voxels persist across frames; maintained only when a
  // consumer exists (dyn_cluster_expansion_ rescue sweep or dyn_bridge_display_
  // bridge). Voxels stale beyond dyn_buffer_max_age_ frames are dropped.
  // Survives ClearPillarMapVoxels()
  struct DynVoxelEntry
  {
    // world-frame points stored at last detection — populated ONLY when
    // dyn_bridge_display_ is on (the stale-point re-publish needs the
    // coordinates; the rescue sweep consumes key membership only)
    std::vector<Eigen::Vector3d> points;
    int last_frame = 0;
  };
  std::unordered_map<PillarMapKey, DynVoxelEntry> dyn_buffer_;
  int dyn_bridge_frame_ = 0;  // frame stamp of the current detection pass

  // Long-term per-voxel evidence (ERASOR-style): dyn_count = frames the voxel
  // appeared window-absent (dynamic candidate), static_count = frames it was
  // seen within the window or as supported structure (bottom voxels).
  // Persists across ClearPillarMapVoxels; the dyn_count > static_count gate
  // separates real dynamic entries from sampling flicker on static surfaces
  struct PillarEvidence
  {
    int dyn_count = 0;
    int static_count = 0;
  };
  std::unordered_map<PillarMapKey, PillarEvidence> pillar_evidence_;

  // Connected component of the dynamic-point buffer (26-adjacency): all
  // member voxel points, its stale subset (voxels not confirmed this frame),
  // the centroid, and its live flag
  struct DynComponent
  {
    std::vector<Eigen::Vector3d> points;         // all member voxel points
    std::vector<Eigen::Vector3d> stale_points;   // points from voxels not confirmed this frame
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    bool live = false;                           // contains a voxel confirmed this frame
  };
  std::vector<DynComponent> dyn_comps_;     // this frame's buffer components (detect-time cache)

  void init(const PillarMapConfig &config, double voxel_size);
  void BuildPillarMap(const PointCloudXYZI::Ptr &input_cloud);
  void DetectNewPoints();
  void UpdateHistory();
  void pillarDetection();
  size_t removeFlaggedPoints(const PointCloudXYZI::Ptr &body_cloud, const PointCloudXYZI::Ptr &world_cloud,
                             std::vector<uint8_t> &skip_flags);
  // Unified pillar map output: redundant (purple), isolated (blue) and new
  // (red, priority on overlap) points in one RGB cloud
  void PublishPillarMapCloud(const ros::Publisher &pub);

  inline int8_t GetPointLabel(size_t index) const {
    return (index < point_labels_.size()) ? point_labels_[index] : LABEL_NORMAL;
  }

  inline int8_t GetPointIsNew(size_t index) const {
    return (index < point_is_new_.size()) ? point_is_new_[index] : 0;
  }

private:
  void setVoxelPointLabels(PillarMapVoxel* voxel, int8_t label);
  void initNeighborOffsets();
  PillarLocation GetPillarLocation(const VoxelLocation &position) const;
  void updatePillarFlag(const PillarLocation &pillar_key, PillarMapArray &pillar_maps);
  // Scan one neighbor ring (same-layer offsets around current_pos): count
  // occupied voxels, gating every neighbor with the height-consistency test.
  // With use_history, slots empty in the current frame still count when the
  // history window shows recent occupancy there (sampling flicker must not
  // read as isolation). Returns true as soon as adjacent_count reaches
  // threshold (early exit)
  bool scanNeighborRing(const VoxelLocation &current_pos, const std::vector<VoxelLocation> &offsets,
                        int threshold, double current_vp_z, double height_threshold, int &adjacent_count,
                        bool use_history);
  bool hasAdjacentVoxel(const VoxelLocation &current_pos, int threshold, double current_vp_z, bool use_history);
  // History-window occupancy oracle: was (pillar, z) occupied in any of the
  // last pillar_buffer frames?
  bool seenInHistory(const PillarLocation &pillar, int64_t z_key) const;
  // True when any z layer strictly between low_key and high_key was occupied
  // within the history window (vertical gap is a transient sampling hole)
  bool gapSeenInHistory(const PillarLocation &pillar, int64_t low_key, int64_t high_key) const;
  // Cross-frame bridge: insert this frame's confirmed voxels into dyn_buffer_,
  // advance the frame stamp, prune voxels stale beyond dyn_buffer_max_age_
  void dyn_bridge_insert();
  // Extract connected components (26-adjacency) of the dynamic-point buffer:
  // each carries all member points, its stale subset, the current-cloud
  // indices of fresh points, the centroid, and its live flag
  std::vector<DynComponent> dyn_bridge_components();
  // Clustering confirmation for candidate new points: candidates must form a
  // cluster of >= dyn_cluster_min_num points to stay flagged. Scattered
  // candidates (quantization hops of static surfaces near voxel boundaries)
  // are downgraded to normal points — neither published nor deleted downstream;
  // surviving voxels re-mark all their points, fully-downgraded ones lose
  // is_new_voxel_
  void confirmClusteredNewPoints();
};

class VoxelMapManager
{
public:
  VoxelMapConfig config_setting_;
  int current_frame_id_ = 0;
  ros::Publisher voxel_map_pub_;

  // Degeneracy-adaptive intensity fusion state (updated per frame in
  // StateEstimation; intensity_fusion_active_ governs the NEXT frame's
  // residual association)
  double degeneracy_factor_ = 1.0;       // λ_min/λ_max of the last geometric observation Hessian
  bool intensity_fusion_active_ = false; // evaluated per frame via the hysteresis state machine
  int degenerate_streak_ = 0;            // consecutive frames below the ON threshold
  int normal_streak_ = 0;                // consecutive frames above the OFF threshold

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
  // NOT vector<bool>: its bit-packed proxy writes are non-atomic read-modify-
  // write on the whole word, which races with the parallel skip passes
  std::vector<uint8_t> skip_list_;

  // Skip point statistics
  int current_skip_count_ = 0;
  int total_skip_count_ = 0;
  int total_point_count_ = 0;

  // Incidence-angle validation dump (intensity_angle_debug_en_): per-frame
  // matched (raw intensity, geometric gain) pairs, filled in the sequential
  // consolidation pass of BuildResidualListOMP, written out by StateEstimation
  std::vector<float> angle_dbg_intensity_;
  std::vector<float> angle_dbg_gain_;

  PillarMap pillar_map_;

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
  void ClearPillarMapVoxels();

private:
  // Evict least-recently-updated voxels from the LRU tail until the cache is
  // within capacity. No-op when capacity is disabled (<= 1: 0 means off, and 1
  // would degenerate into evicting the entry right after inserting it).
  // Returns the number of voxels evicted by this call.
  size_t enforceCapacity();

  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_