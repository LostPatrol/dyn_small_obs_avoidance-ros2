// Geometry-service regressions against the unchanged deployed collision core.
#include "motion_adapter.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <thread>

namespace {
using Vec=Eigen::Vector3d;
using Adapter=path_planning::MotionAdapter;
SearchConfig configuration() {
  SearchConfig c;
  c.safe_distance=.25; c.voxel_size=.1; c.search_budget=.3;
  c.lower=Vec(-50,-50,-2); c.upper=Vec(50,50,10);
  return c;
}
void map(KinodynamicAstar& core,Vec obstacle) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(obstacle.x(),obstacle.y(),obstacle.z()));
  core.clearMap(); ASSERT_TRUE(core.setKdtree(cloud));
}
TrajectorySegment returningCurve() {
  TrajectorySegment s; s.duration=1;
  s.coefficients.col(0)=Vec(0,0,1);
  s.coefficients.col(1)=Vec(4,0,0);
  s.coefficients.col(2)=Vec(-4,0,0);
  return s;
}
}  // namespace

TEST(MotionAdapter, EntireLineIncludesStartAndGoalAndStrictRadius) {
  KinodynamicAstar core(configuration());
  ASSERT_EQ(Adapter(core).line(Vec(0,0,1),Vec(2,0,1)),KinodynamicAstar::NO_MAP);
  map(core,Vec(1,0,1));
  EXPECT_EQ(Adapter(core).line(Vec(0,0,1),Vec(2,0,1)),KinodynamicAstar::NO_PATH);
  map(core,Vec(2,0,1));
  EXPECT_EQ(Adapter(core).line(Vec(0,0,1),Vec(2,0,1)),KinodynamicAstar::NO_PATH);
  map(core,Vec(0,0,1));
  EXPECT_EQ(Adapter(core).line(Vec(0,0,1),Vec(0,0,1)),KinodynamicAstar::NO_PATH);
  map(core,Vec(1,.25,1));  // Exactly representable float equality: old core uses strict '<'.
  EXPECT_EQ(Adapter(core).line(Vec(0,0,1),Vec(2,0,1)),KinodynamicAstar::REACH_END);
  map(core,Vec(10,10,1));  // Full-snapshot replacement must allow a disappeared obstacle.
  EXPECT_EQ(Adapter(core).line(Vec(0,0,1),Vec(2,0,1)),KinodynamicAstar::REACH_END);
}

TEST(MotionAdapter, ReturningCurveAndExecutedPrefix) {
  KinodynamicAstar core(configuration()); map(core,Vec(1,0,1));
  const auto s=returningCurve();
  EXPECT_EQ(Adapter(core).trajectory({s},0,Vec(0,0,1),60),KinodynamicAstar::NO_PATH);
  EXPECT_EQ(Adapter(core).trajectory({s},.9,Vec(0,0,1),60),KinodynamicAstar::REACH_END);
  map(core,Vec(0,0,1));
  EXPECT_EQ(Adapter(core).trajectory({s},1,Vec(0,0,1),60),KinodynamicAstar::NO_PATH);
}

TEST(MotionAdapter, RejectsMalformedCurvesAndResourceOverflow) {
  KinodynamicAstar core(configuration()); map(core,Vec(10,10,1));
  auto s=returningCurve();
  EXPECT_EQ(Adapter(core).trajectory({},0,Vec(0,0,1),60),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(Adapter(core).trajectory({s},1.01,Vec(0,0,1),60),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(Adapter(core).trajectory({s},0,Vec(1,0,1),60),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(Adapter(core).trajectory({s},0,Vec(0,0,1),.5),KinodynamicAstar::INVALID_INPUT);
  s.duration=0;
  EXPECT_EQ(Adapter(core).trajectory({s},0,Vec(0,0,1),60),KinodynamicAstar::INVALID_INPUT);
  s=returningCurve();s.coefficients(0,1)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(Adapter(core).trajectory({s},0,Vec(0,0,1),60),KinodynamicAstar::INVALID_INPUT);
  s=returningCurve();s.coefficients(0,1)=1e9;s.coefficients(0,2)=-1e9;
  EXPECT_EQ(Adapter(core).trajectory({s},0,Vec(0,0,1),60),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(Adapter(core).line(Vec(0,0,1),Vec(51,0,1)),KinodynamicAstar::INVALID_INPUT);
  s=returningCurve();auto discontinuous=s;discontinuous.coefficients(1,0)=1;
  EXPECT_EQ(Adapter(core).trajectory({s,discontinuous},0,Vec(0,1,1),60),KinodynamicAstar::INVALID_INPUT);
  std::vector<TrajectorySegment> oversized(Adapter::kMaxSegments+1,s);
  EXPECT_EQ(Adapter(core).trajectory(oversized,0,Vec(0,0,1),60),KinodynamicAstar::INVALID_INPUT);
}

TEST(MotionAdapter, CooperativeDeadline) {
  auto c=configuration();c.search_budget=.001;
  KinodynamicAstar core(c);map(core,Vec(10,10,1));
  Adapter adapter(core);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_EQ(adapter.line(Vec(0,0,1),Vec(2,0,1)),KinodynamicAstar::TIMEOUT);
}
