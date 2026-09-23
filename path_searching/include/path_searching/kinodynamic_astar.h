// GPL-3.0: HKU-MARS kinodynamic A*, ROS-independent port; see THIRD_PARTY_NOTICES.md.
#pragma once
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <array>
#include <chrono>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

/// Search and obstacle-map configuration in SI units.
/// Limits are per axis, not Euclidean norms. Bounds do not imply observed free space.
/// The constructor copies this configuration; changing the caller's copy has no effect.
struct SearchConfig {
  // Primitive duration (s), initial fixed-acceleration duration (s), velocity (m/s), acceleration (m/s²).
  double max_tau = 0.6, init_max_tau = 0.8, max_vel = 2.0, max_acc = 2.0;
  // Cost integrates squared acceleration + w_time; horizon is displacement from start (m).
  // lambda_heu weights the upstream heuristic, so global optimality is not guaranteed.
  double w_time = 10.0, horizon = 100.0, lambda_heu = 5.0;
  // Spatial hash cell size (m); time hash bin size (s), used only with dynamic=true.
  double resolution = 0.1, time_resolution = 0.8;
  // Clearance (m), voxel edge (m), maximum collision-sampling travel interval (m).
  // Queries add sqrt(3)*voxel_size + collision_step/2 to safe_distance.
  double safe_distance = 0.45, voxel_size = 0.1, collision_step = 0.05;
  double search_budget = 0.08;  // steady-clock seconds per search, excludes map construction
  // Node-pool capacity; valid cloud updates per bank before alternating to the other tree.
  int allocate_num = 100000, tree_period = 50;
  // Limit for a single input cloud and the sum of both filtered banks; overflow invalidates the map.
  std::size_t max_cloud_points = 500000;
  // Inclusive world-frame position bounds (m), shared by primitives and terminal curves.
  Eigen::Vector3d lower{-50.0, -50.0, 0.2}, upper{50.0, 50.0, 10.0};
  /// Throws std::invalid_argument for invalid numeric values, bounds or resource limits.
  void validate() const;
};

// Position c0+c1*t+c2*t^2+c3*t^3. Derivatives come directly from the search, without refitting.
struct TrajectorySegment {
  // Local segment time spans [0, duration] seconds; concatenate segments in returned order.
  double duration = 0.0;
  // Rows: world x/y/z; columns: c0/c1/c2/c3. Position and velocity are continuous at joins.
  Eigen::Matrix<double, 3, 4> coefficients = Eigen::Matrix<double, 3, 4>::Zero();
};
/// A sample of the polynomial trajectory, not a newly fitted or smoothed reference.
struct TrajectorySample {
  // Seconds from the beginning of the entire trajectory, not a ROS/Unix timestamp.
  double time = 0.0;
  // World-frame SI values. Acceleration may jump at a segment boundary.
  Eigen::Vector3d position, velocity, acceleration;
};
enum NodeState { NOT_EXPAND, IN_OPEN_SET, IN_CLOSE_SET };
// Search-owned node: state=[position, velocity], input=acceleration from parent to this node.
struct PathNode {
  Eigen::Vector3i index = Eigen::Vector3i::Zero();
  Eigen::Matrix<double, 6, 1> state = Eigen::Matrix<double, 6, 1>::Zero();
  double g_score = 0, f_score = 0, duration = 0, time = 0;
  Eigen::Vector3d input = Eigen::Vector3d::Zero();
  int time_idx = 0;
  PathNode* parent = nullptr;
  NodeState node_state = NOT_EXPAND;
};
using PathNodePtr = PathNode*;
// Snapshot scores prevent decrease-key operations from corrupting priority_queue ordering.
struct QueueEntry { double score; PathNodePtr node; };
struct NodeComparator {
  bool operator()(const QueueEntry& a, const QueueEntry& b) const { return a.score > b.score; }
};
template <typename T> struct matrix_hash {
  std::size_t operator()(const T& m) const {
    std::size_t seed = 0;
    for (Eigen::Index i = 0; i < m.size(); ++i)
      seed ^= std::hash<typename T::Scalar>()(m.data()[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};
// Upstream pruning keys contain position (and optionally time), not velocity.
// Different velocity states can share a cell; this is not a complete six-dimensional state lattice.
class NodeHashTable {
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>> data_4d_;
 public:
  void insert(Eigen::Vector3i i, PathNodePtr n) { data_3d_.emplace(i, n); }
  void insert(Eigen::Vector3i i, int t, PathNodePtr n) { data_4d_.emplace(Eigen::Vector4i(i.x(), i.y(), i.z(), t), n); }
  PathNodePtr find(Eigen::Vector3i i) { auto p = data_3d_.find(i); return p == data_3d_.end() ? nullptr : p->second; }
  PathNodePtr find(Eigen::Vector3i i, int t) {
    auto p = data_4d_.find(Eigen::Vector4i(i.x(), i.y(), i.z(), t));
    return p == data_4d_.end() ? nullptr : p->second;
  }
  void clear() { data_3d_.clear(); data_4d_.clear(); }
};

/**
 * Single-threaded, ROS-independent planner with a persistent obstacle map.
 *
 * Typical use: construct -> setKdtree -> search -> inspect status -> getSegments/sampleTrajectory.
 * All cloud points and state vectors must share a fixed world frame and use SI units.
 * The caller handles transforms, input age and coverage; the core has no sensor timestamps.
 * Serialize all calls on an instance, including isSafe(), which reuses query buffers.
 */
class KinodynamicAstar {
 public:
  // REACH_END reaches the requested position/velocity; REACH_HORIZON is only a partial path
  // and does not promise a stopped endpoint. NEAR_END is retained but currently never returned.
  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4,
         INVALID_INPUT = 5, NO_MAP = 6, TIMEOUT = 7, NODE_LIMIT = 8 };
  /// Validates config and allocates the node pool; may throw invalid_argument or allocation errors.
  explicit KinodynamicAstar(const SearchConfig& config = SearchConfig());
  ~KinodynamicAstar();
  KinodynamicAstar(const KinodynamicAstar&) = delete;
  KinodynamicAstar& operator=(const KinodynamicAstar&) = delete;
  /// Clear search state/results, retaining the obstacle map. search() also does this automatically.
  void reset();
  /// Release both map banks and reset search state; mapReady() becomes false.
  void clearMap();
  /**
   * Accumulate a registered cloud and rebuild the current bank; the caller retains ownership.
   * Banks alternate every tree_period accepted updates, not elapsed seconds.
   * Empty/nonfinite/over-capacity input returns false and clears the map and search results.
   * A successful update does not revalidate an existing trajectory: run search() again.
   */
  bool setKdtree(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  /// True means an accepted nonempty map exists, not that it is fresh or covers the requested route.
  bool mapReady() const { return map_ready_; }
  /// Sum of filtered points in both banks, including points duplicated across banks.
  std::size_t mapPointCount() const;
  /// Point-clearance query with conservative inflation; false for no map or out-of-bounds input.
  /// True only concerns stored obstacle points, not unobserved space or future obstacle motion.
  bool isSafe(double x, double y, double z);
  /// Finite position inside inclusive configured bounds; does not query obstacles.
  bool inBounds(const Eigen::Vector3d& p) const;
  /**
   * Search from position/velocity to goal/goal_velocity; replaces the previous search result.
   * init=true constrains the first expansion to acceleration; false uses the acceleration lattice.
   * acceleration must be finite and within limits even when init=false.
   * dynamic=true adds time bins to the hash using time_start (s) as origin; collision queries
   * remain spatial and do NOT predict moving obstacles. The ROS wrapper uses dynamic=false.
   * Check the returned status before consuming a trajectory. Failures expose no trajectory;
   * TIMEOUT is cooperative, not a hard deadline for the whole caller operation.
   */
  int search(Eigen::Vector3d start, Eigen::Vector3d velocity, Eigen::Vector3d acceleration,
             Eigen::Vector3d goal, Eigen::Vector3d goal_velocity, bool init,
             bool dynamic = false, double time_start = 0.0);
  /// Return owning copies of the last result's primitives and optional terminal cubic.
  std::vector<TrajectorySegment> getSegments() const;
  /// Sample at intervals <= step seconds, including exact endpoints and every segment boundary.
  /// Boundary acceleration is the preceding segment's end value. Empty result yields no samples.
  /// Throws invalid_argument for nonpositive/nonfinite step, length_error above the sample cap.
  std::vector<TrajectorySample> sampleTrajectory(double step) const;
  /// Position-only convenience view of sampleTrajectory(); identical sampling and exceptions.
  std::vector<Eigen::Vector3d> getKinoTraj(double step) const;
  /// Diagnostic view of allocated nodes (open and closed), not just expanded nodes.
  /// Pointers are borrowed: never delete/mutate them or rely on contents after reset/search/clearMap.
  std::vector<PathNodePtr> getVisitedNodes();
  /// Read-only configuration owned by this instance; reference expires with the planner.
  const SearchConfig& config() const { return config_; }

 private:
  SearchConfig config_;
  std::vector<PathNodePtr> path_node_pool_, path_nodes_;
  int use_node_num_ = 0, iter_num_ = 0;
  NodeHashTable expanded_nodes_;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, NodeComparator> open_set_;
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_ = Eigen::Matrix<double, 6, 6>::Identity();
  std::array<pcl::search::KdTree<pcl::PointXYZ>, 2> trees_;
  std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, 2> clouds_;
  // Reuse nearest-neighbor buffers: collision checks are the inner search loop.
  std::vector<int> nearest_indices_{0};
  std::vector<float> nearest_distances_{0.0f};
  std::size_t cloud_input_num_ = 0;
  bool map_ready_ = false, is_shot_succ_ = false;
  Eigen::Matrix<double, 3, 4> coef_shot_ = Eigen::Matrix<double, 3, 4>::Zero();
  double t_shot_ = 0.0;
  double max_tau_, init_max_tau_, max_vel_, max_acc_, w_time_, horizon_, lambda_heu_;
  int allocate_num_;
  double tie_breaker_ = 1.0 + 1.0 / 10000.0;
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_;
  double time_origin_ = 0.0;
  std::chrono::steady_clock::time_point deadline_;
  Eigen::Vector3i posToIndex(Eigen::Vector3d point);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr node);
  std::vector<double> cubic(double a, double b, double c, double d);
  std::vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd start, Eigen::VectorXd end, double duration);
  double estimateHeuristic(Eigen::VectorXd start, Eigen::VectorXd end, double& duration);
  void stateTransit(Eigen::Matrix<double, 6, 1>& start, Eigen::Matrix<double, 6, 1>& end,
                    Eigen::Vector3d acceleration, double duration);
};
