// GPL-3.0: standalone Jazzy planning orchestration; preserves upstream search, removes demo flight coupling.
#include <path_searching/kinodynamic_astar.h>
#include <path_planning/msg/plan_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <limits>

using Result = path_planning::msg::PlanResult;
using Clock = std::chrono::steady_clock;
using Vector = Eigen::Vector3d;

namespace {
constexpr double kMaxPlanningRateHz = 1000.0;  // Configuration guard, not an achievable-rate promise.
constexpr double kFutureStampToleranceSeconds = 0.05;  // Small source/host clock lead accepted in ros mode.
constexpr double kQuaternionNormTolerance = 0.01;  // Dimensionless; accepted quaternions are normalized.
constexpr uint32_t kNanosecondsPerSecond = 1000000000u;  // ROS time requires nanosec in [0, 1e9).
constexpr uint32_t kFloat32Bytes = 4;  // PointField::FLOAT32 wire width, independent of struct padding.
constexpr uint32_t kXyzFieldCount = 3;  // Required scalar x/y/z; other fields may coexist.
}  // namespace

// Single executor serializes map updates and searches; depth-one subscriptions bound backlog.
class Planner : public rclcpp::Node {
 public:
  Planner() : Node("path_planning") {
    // Parameters are copied at startup; changing ROS parameter values later does not reconfigure
    // these members. Restart with the desired configuration. Construction validates before subscribing.
    SearchConfig c;
#define PARAM(field) c.field = declare_parameter("search." #field, c.field)
    PARAM(max_tau); PARAM(init_max_tau); PARAM(max_vel); PARAM(max_acc); PARAM(w_time);
    PARAM(horizon); PARAM(lambda_heu); PARAM(resolution); PARAM(time_resolution);
    PARAM(safe_distance); PARAM(voxel_size); PARAM(collision_step); PARAM(search_budget);
    PARAM(allocate_num); PARAM(tree_period);
#undef PARAM
    const int max_points = declare_parameter("search.max_cloud_points", static_cast<int>(c.max_cloud_points));
    if (max_points < 1) throw std::invalid_argument("max_cloud_points must be positive");
    c.max_cloud_points = max_points;
    auto lower = declare_parameter("search.lower", std::vector<double>{-50,-50,0.2});
    auto upper = declare_parameter("search.upper", std::vector<double>{50,50,10});
    if (lower.size()!=3 || upper.size()!=3) throw std::invalid_argument("bounds require three coordinates");
    c.lower=Eigen::Map<Vector>(lower.data()); c.upper=Eigen::Map<Vector>(upper.data());
    core_ = std::make_unique<KinodynamicAstar>(c);
    frame_=declare_parameter("planning_frame", "camera_init");
    rate_=declare_parameter("planning_rate",10.0); timeout_=declare_parameter("input_timeout",0.5);
    step_=declare_parameter("sample_step",0.02); stamp_clock_=declare_parameter("stamp_clock","ros");
    body_twist_=declare_parameter("odometry_twist_in_body",true);
    if (frame_.empty() || !positive(rate_) || rate_>kMaxPlanningRateHz || !positive(timeout_) || !positive(step_) ||
        (stamp_clock_!="ros" && stamp_clock_!="receive")) throw std::invalid_argument("invalid wrapper parameters");
    // Best-effort subscribers match either sensor reliability policy. Results/goals are reliable
    // and volatile: a newly connected consumer must wait for a fresh result rather than a latched path.
    auto sensor_qos=rclcpp::SensorDataQoS().keep_last(1);
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("cloud",sensor_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){ cloud(*m); });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("odom",sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m){ odom(*m); });
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("goal",rclcpp::QoS(1),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){ goal(*m); });
    result_pub_=create_publisher<Result>("plan_result",rclcpp::QoS(1));
    path_pub_=create_publisher<nav_msgs::msg::Path>("kino_path",rclcpp::QoS(1));
    timer_=create_wall_timer(std::chrono::duration<double>(1/rate_),[this]{ plan(); });
    RCLCPP_INFO(get_logger(),"Standalone planner ready: frame=%s, %.1f Hz, clock=%s",frame_.c_str(),rate_,stamp_clock_.c_str());
  }

 private:
  static bool positive(double d) { return std::isfinite(d) && d>0; }
  static Vector vec(const geometry_msgs::msg::Point& p) { return {p.x,p.y,p.z}; }
  static Vector vec(const geometry_msgs::msg::Vector3& p) { return {p.x,p.y,p.z}; }
  bool correctFrame(const std_msgs::msg::Header& h) const { return h.frame_id==frame_; }
  static bool validStamp(const builtin_interfaces::msg::Time& stamp) {
    return stamp.sec>=0 && stamp.nanosec<kNanosecondsPerSecond;
  }
  // Reception timeout always uses steady time, including paused simulation playback.
  // "ros" additionally checks source age; "receive" cannot detect sensor-side delay/clock offset.
  bool fresh(Clock::time_point received, const builtin_interfaces::msg::Time& stamp) const {
    if (std::chrono::duration<double>(Clock::now()-received).count()>timeout_) return false;
    if (stamp_clock_=="receive") return true;
    const double age=(now()-rclcpp::Time(stamp)).seconds();
    return age>=-kFutureStampToleranceSeconds && age<=timeout_;
  }
  // Drop geometry as well as readiness: recovery starts a new accumulation window.
  void invalidateCloud(uint8_t status, const std::string& detail) {
    core_->clearMap(); cloud_status_=status; cloud_detail_=detail;
  }
  // Check PointCloud2 layout before PCL conversion, including padding and scalar XYZ fields.
  void cloud(const sensor_msgs::msg::PointCloud2& m) {
    if (!correctFrame(m.header)) { invalidateCloud(Result::FRAME_MISMATCH,"cloud frame mismatch"); return; }
    if (!validStamp(m.header.stamp)) { invalidateCloud(Result::INVALID_INPUT,"invalid cloud timestamp"); return; }
    const int64_t stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    if (have_cloud_stamp_ && stamp<=last_cloud_stamp_) {
      if (stamp<last_cloud_stamp_) { invalidateCloud(Result::STALE_INPUT,"cloud time moved backwards; map reset"); last_cloud_stamp_=stamp; }
      return;  // duplicates never refresh reception age or accumulate the same observation
    }
    last_cloud_stamp_=stamp; have_cloud_stamp_=true;
    const uint64_t count=uint64_t(m.width)*m.height;
    bool valid=count>0 && count<=core_->config().max_cloud_points && !m.is_bigendian && m.point_step>=kXyzFieldCount*kFloat32Bytes &&
        uint64_t(m.row_step)>=uint64_t(m.width)*m.point_step && m.data.size()==uint64_t(m.row_step)*m.height;
    for (const auto* name:{"x","y","z"}) {
      bool found=false;
      for (const auto& f:m.fields) if (f.name==name && f.datatype==sensor_msgs::msg::PointField::FLOAT32 &&
          f.count==1 && uint64_t(f.offset)+kFloat32Bytes<=m.point_step) found=true;
      valid=valid && found;
    }
    if (!valid) { invalidateCloud(Result::INVALID_INPUT,"empty, oversized or malformed XYZ cloud"); return; }
    const auto received=Clock::now();
    if (!fresh(received,m.header.stamp)) { invalidateCloud(Result::STALE_INPUT,"cloud stamp outside age limit"); return; }
    if (cloud_status_==0 && !fresh(cloud_received_,cloud_stamp_)) core_->clearMap();
    pcl::PointCloud<pcl::PointXYZ> points;
    pcl::fromROSMsg(m,points);
    if (!core_->setKdtree(points)) { invalidateCloud(Result::INVALID_INPUT,"nonfinite cloud or map capacity exceeded"); return; }
    cloud_received_=received; cloud_stamp_=m.header.stamp; cloud_status_=0;
  }
  // Pose is already in the planning frame. By Odometry convention, twist is in child_frame_id;
  // rotate its linear component using the pose orientation unless the producer explicitly uses world twist.
  // Covariance/angular velocity are not used, and no interpolation to the cloud timestamp is performed.
  void odom(const nav_msgs::msg::Odometry& m) {
    if (!correctFrame(m.header)) { odom_status_=Result::FRAME_MISMATCH; return; }
    if (!validStamp(m.header.stamp)) { odom_status_=Result::INVALID_INPUT; return; }
    const int64_t stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    if (have_odom_stamp_ && stamp<=last_odom_stamp_) {
      if (stamp<last_odom_stamp_) { odom_status_=Result::STALE_INPUT; last_odom_stamp_=stamp; }
      return;
    }
    have_odom_stamp_=true; last_odom_stamp_=stamp;
    position_=vec(m.pose.pose.position); velocity_=vec(m.twist.twist.linear);
    const auto& q=m.pose.pose.orientation;
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    if (!position_.allFinite() || !velocity_.allFinite() || !rotation.coeffs().allFinite() ||
        std::abs(rotation.norm()-1)>kQuaternionNormTolerance || (body_twist_ && m.child_frame_id.empty())) {
      odom_status_=Result::INVALID_INPUT; return;
    }
    rotation.normalize(); if (body_twist_) velocity_=rotation*velocity_;
    odom_received_=Clock::now(); odom_stamp_=m.header.stamp;
    odom_status_=fresh(odom_received_,odom_stamp_)?0:Result::STALE_INPUT;
  }
  // Goals persist until replaced; only position/frame are consumed, not orientation or timestamp.
  // Every received goal advances the revision, including repeated coordinates and invalid requests.
  void goal(const geometry_msgs::msg::PoseStamped& m) {
    ++goal_revision_;
    if (!correctFrame(m.header)) { goal_status_=Result::FRAME_MISMATCH; return; }
    goal_=vec(m.pose.position);
    goal_status_=goal_.allFinite()?0:Result::INVALID_INPUT;
  }
  // Every attempt emits an atomic status; failure also emits an empty visualization path.
  void plan() {
    // Readiness flags are internal: zero means valid input, not a published PlanResult status.
    // The first failing gate determines the reported reason; this is not an aggregate diagnostic.
    Result result; result.header.stamp=now(); result.header.frame_id=frame_;
    result.sequence=++sequence_; result.goal_revision=goal_revision_;
    result.cloud_stamp=cloud_stamp_; result.odometry_stamp=odom_stamp_;
    if (goal_status_) { result.status=goal_status_; result.detail="goal unavailable or invalid"; }
    else if (odom_status_) { result.status=odom_status_; result.detail="odometry unavailable or invalid"; }
    else if (cloud_status_) { result.status=cloud_status_; result.detail=cloud_detail_; }
    else if (!fresh(cloud_received_,cloud_stamp_) || !fresh(odom_received_,odom_stamp_)) {
      result.status=Result::STALE_INPUT; result.detail="cloud or odometry expired";
      if (!fresh(cloud_received_,cloud_stamp_)) invalidateCloud(Result::STALE_INPUT,"cloud expired; map reset");
    } else {
      const auto began=Clock::now();
      // No acceleration measurement in Odometry: use the unrestricted primitive search, with actual p/v.
      result.status=core_->search(position_,velocity_,Vector::Zero(),goal_,Vector::Zero(),false);
      result.planning_ms=std::chrono::duration<double,std::milli>(Clock::now()-began).count();
      result.detail=result.status==Result::REACH_END?"goal reached by trajectory":
          result.status==Result::REACH_HORIZON?"partial horizon; endpoint is not goal":"search failed";
      // Processing must not extend the input freshness contract.
      if (!fresh(cloud_received_,cloud_stamp_) || !fresh(odom_received_,odom_stamp_)) {
        result.status=Result::STALE_INPUT; result.detail="input expired during search";
      }
    }
    nav_msgs::msg::Path path; path.header=result.header;
    result.trajectory.header=result.header; result.trajectory.joint_names={"position"};
    if (result.status==Result::REACH_END || result.status==Result::REACH_HORIZON) {
      // Serialize exact search coefficients; the sampled trajectory and Path are views of these
      // same curves, not separately interpolated paths. Identity orientation does not plan yaw.
      for (const auto& s:core_->getSegments()) {
        path_planning::msg::PolynomialSegment msg; msg.duration=s.duration;
        for (int i=0;i<4;++i) { msg.x[i]=s.coefficients(0,i); msg.y[i]=s.coefficients(1,i); msg.z[i]=s.coefficients(2,i); }
        result.segments.push_back(msg);
      }
      try {
        for (const auto& s:core_->sampleTrajectory(step_)) {
          trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
          geometry_msgs::msg::Transform p; p.rotation.w=1;
          p.translation.x=s.position.x(); p.translation.y=s.position.y(); p.translation.z=s.position.z();
          geometry_msgs::msg::Twist v,a;
          v.linear.x=s.velocity.x(); v.linear.y=s.velocity.y(); v.linear.z=s.velocity.z();
          a.linear.x=s.acceleration.x(); a.linear.y=s.acceleration.y(); a.linear.z=s.acceleration.z();
          point.transforms.push_back(p); point.velocities.push_back(v); point.accelerations.push_back(a);
          point.time_from_start=rclcpp::Duration::from_seconds(s.time); result.trajectory.points.push_back(point);
          geometry_msgs::msg::PoseStamped pose; pose.header=result.header;
          pose.header.stamp=rclcpp::Time(result.header.stamp)+rclcpp::Duration::from_seconds(s.time);
          pose.pose.position.x=s.position.x(); pose.pose.position.y=s.position.y(); pose.pose.position.z=s.position.z();
          pose.pose.orientation.w=1; path.poses.push_back(pose);
        }
      } catch (const std::exception& e) {
        result.status=Result::INVALID_INPUT; result.detail=e.what(); result.segments.clear();
        result.trajectory.points.clear(); path.poses.clear();
      }
    }
    result.map_points=core_->mapPointCount();
    result_pub_->publish(result); path_pub_->publish(path);
  }
  std::unique_ptr<KinodynamicAstar> core_;
  std::string frame_,stamp_clock_,cloud_detail_="no cloud received";
  double rate_,timeout_,step_; bool body_twist_;
  Vector position_=Vector::Zero(),velocity_=Vector::Zero(),goal_=Vector::Zero();
  Clock::time_point cloud_received_,odom_received_;
  builtin_interfaces::msg::Time cloud_stamp_,odom_stamp_;
  uint8_t cloud_status_=Result::NO_MAP, odom_status_=Result::WAITING_FOR_INPUT, goal_status_=Result::WAITING_FOR_INPUT;
  uint64_t sequence_=0,goal_revision_=0;
  int64_t last_cloud_stamp_=0,last_odom_stamp_=0; bool have_cloud_stamp_=false,have_odom_stamp_=false;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<Result>::SharedPtr result_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<Planner>()); }
  catch (const std::exception& e) { std::cerr<<"planner: "<<e.what()<<std::endl; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
