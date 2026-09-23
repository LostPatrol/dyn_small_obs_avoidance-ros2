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

// Per-axis limits match upstream. Planning bounds do not represent observed free space.
struct SearchConfig {
  double max_tau = 0.6, init_max_tau = 0.8, max_vel = 2.0, max_acc = 2.0;
  double w_time = 10.0, horizon = 100.0, lambda_heu = 5.0;
  double resolution = 0.1, time_resolution = 0.8;
  double safe_distance = 0.45, voxel_size = 0.1, collision_step = 0.05;
  double search_budget = 0.08;  // steady-clock seconds per search, excludes map construction
  int allocate_num = 100000, tree_period = 50;
  std::size_t max_cloud_points = 500000;
  Eigen::Vector3d lower{-50.0, -50.0, 0.2}, upper{50.0, 50.0, 10.0};
  void validate() const;
};

// Position c0+c1*t+c2*t^2+c3*t^3. Derivatives come directly from the search, without refitting.
struct TrajectorySegment {
  double duration = 0.0;
  Eigen::Matrix<double, 3, 4> coefficients = Eigen::Matrix<double, 3, 4>::Zero();
};
struct TrajectorySample {
  double time = 0.0;
  Eigen::Vector3d position, velocity, acceleration;
};
enum NodeState { NOT_EXPAND, IN_OPEN_SET, IN_CLOSE_SET };
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

// Single-threaded core: caller supplies registered clouds and consistent world-frame states.
class KinodynamicAstar {
 public:
  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4,
         INVALID_INPUT = 5, NO_MAP = 6, TIMEOUT = 7, NODE_LIMIT = 8 };
  explicit KinodynamicAstar(const SearchConfig& config = SearchConfig());
  ~KinodynamicAstar();
  KinodynamicAstar(const KinodynamicAstar&) = delete;
  KinodynamicAstar& operator=(const KinodynamicAstar&) = delete;
  void reset();
  void clearMap();
  // Empty/invalid/over-capacity clouds fail; never silently drop excess obstacles.
  bool setKdtree(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  bool mapReady() const { return map_ready_; }
  std::size_t mapPointCount() const;
  bool isSafe(double x, double y, double z);
  bool inBounds(const Eigen::Vector3d& p) const;
  int search(Eigen::Vector3d start, Eigen::Vector3d velocity, Eigen::Vector3d acceleration,
             Eigen::Vector3d goal, Eigen::Vector3d goal_velocity, bool init,
             bool dynamic = false, double time_start = 0.0);
  std::vector<TrajectorySegment> getSegments() const;
  std::vector<TrajectorySample> sampleTrajectory(double step) const;
  std::vector<Eigen::Vector3d> getKinoTraj(double step) const;
  std::vector<PathNodePtr> getVisitedNodes();
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
