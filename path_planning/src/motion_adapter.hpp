// Service-only geometry adapter; preserves the deployed search and nearest-centroid collision rule.
#pragma once
#include <path_searching/kinodynamic_astar.h>
#include <algorithm>
#include <cmath>

namespace path_planning {
// Discrete checks retain the old core's sampling limitation; they are not continuous clearance proofs.
class MotionAdapter {
 public:
  static constexpr std::size_t kMaxSegments = 256;
  static constexpr std::size_t kMaxChecks = 100000;
  using Clock = std::chrono::steady_clock;
  explicit MotionAdapter(KinodynamicAstar& core)
      : core_(core), deadline_(Clock::now()+std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(core.config().search_budget))) {}

  int line(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    if (!core_.inBounds(start) || !core_.inBounds(goal)) return KinodynamicAstar::INVALID_INPUT;
    if (!core_.mapReady()) return KinodynamicAstar::NO_MAP;
    const double intervals=std::ceil((goal-start).norm()/core_.config().collision_step);
    std::size_t n;
    if (!reserveChecks(intervals,n)) return KinodynamicAstar::INVALID_INPUT;
    for (std::size_t i=0;i<=n;++i) {
      const int status=point(start+(goal-start)*(double(i)/n));
      if (status!=KinodynamicAstar::REACH_END) return status;
    }
    return KinodynamicAstar::REACH_END;
  }

  // Check the remaining ordered curve, including both endpoints. Ignore already executed geometry.
  int trajectory(const std::vector<TrajectorySegment>& segments, double elapsed,
                 const Eigen::Vector3d& goal, double max_duration) {
    if (segments.empty() || segments.size()>kMaxSegments || !std::isfinite(elapsed) || elapsed<0 ||
        !core_.inBounds(goal)) return KinodynamicAstar::INVALID_INPUT;
    double total=0;
    Eigen::Vector3d previous_position=Eigen::Vector3d::Zero(),previous_velocity=Eigen::Vector3d::Zero();
    for (std::size_t i=0;i<segments.size();++i) {
      const auto& s=segments[i];
      if (!std::isfinite(s.duration) || s.duration<=0 || !s.coefficients.allFinite() ||
          !std::isfinite(total+=s.duration) || total>max_duration) return KinodynamicAstar::INVALID_INPUT;
      if (i && ((s.coefficients.col(0)-previous_position).norm()>1e-5 ||
                (s.coefficients.col(1)-previous_velocity).norm()>1e-5)) return KinodynamicAstar::INVALID_INPUT;
      previous_position=position(s,s.duration);
      previous_velocity=velocity(s,s.duration);
      if (!previous_position.allFinite() || !previous_velocity.allFinite()) return KinodynamicAstar::INVALID_INPUT;
    }
    if (elapsed>total || (previous_position-goal).norm()>1e-6) return KinodynamicAstar::INVALID_INPUT;
    if (!core_.mapReady()) return KinodynamicAstar::NO_MAP;
    double offset=0;
    for (const auto& s:segments) {
      const double begin=std::clamp(elapsed-offset,0.0,s.duration);
      offset+=s.duration;
      if (begin==s.duration && offset<total) continue;
      // Velocity is quadratic. Its Bezier control hull bounds speed, hence traveled distance
      // between checks even for returning curves; no modification of search collision sampling.
      const auto first=velocity(s,begin),last=velocity(s,s.duration);
      const Eigen::Vector3d middle=first+
        (s.coefficients.col(2)+3*s.coefficients.col(3)*begin)*(s.duration-begin);
      const double speed=std::max({first.norm(),middle.norm(),last.norm()});
      std::size_t n;
      if (!reserveChecks(std::ceil((s.duration-begin)*speed/core_.config().collision_step),n))
        return KinodynamicAstar::INVALID_INPUT;
      for (std::size_t i=0;i<=n;++i) {
        const int status=point(position(s,begin+(s.duration-begin)*double(i)/n));
        if (status!=KinodynamicAstar::REACH_END) return status;
      }
    }
    return KinodynamicAstar::REACH_END;
  }

 private:
  static Eigen::Vector3d position(const TrajectorySegment& s,double t) {
    return ((s.coefficients.col(3)*t+s.coefficients.col(2))*t+s.coefficients.col(1))*t+s.coefficients.col(0);
  }
  static Eigen::Vector3d velocity(const TrajectorySegment& s,double t) {
    return (3*s.coefficients.col(3)*t+2*s.coefficients.col(2))*t+s.coefficients.col(1);
  }
  bool reserveChecks(double intervals,std::size_t& n) {
    if (!std::isfinite(intervals) || intervals<0 || intervals>=kMaxChecks) return false;
    n=std::max<std::size_t>(1,static_cast<std::size_t>(intervals));
    if (n+1>kMaxChecks-checks_) return false;
    checks_+=n+1;
    return true;
  }
  int point(const Eigen::Vector3d& p) {
    if (Clock::now()>=deadline_) return KinodynamicAstar::TIMEOUT;
    if (!core_.inBounds(p)) return KinodynamicAstar::INVALID_INPUT;
    return core_.isSafe(p.x(),p.y(),p.z()) ? KinodynamicAstar::REACH_END : KinodynamicAstar::NO_PATH;
  }
  KinodynamicAstar& core_;
  Clock::time_point deadline_;
  std::size_t checks_=0;
};
}  // namespace path_planning
