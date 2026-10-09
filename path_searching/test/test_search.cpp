// Core regression tests use raw point geometry, not the planner's collision predicate, as the oracle.
#include <gtest/gtest.h>
#include <path_searching/kinodynamic_astar.h>
#include <limits>
#include <map>
#include <set>
using Eigen::Vector3d;
namespace {
SearchConfig config() { SearchConfig c; c.search_budget=2.0; return c; }
pcl::PointCloud<pcl::PointXYZ> background() {
  pcl::PointCloud<pcl::PointXYZ> c; c.push_back({-20,-20,1}); return c;
}
int plan(KinodynamicAstar& k, Vector3d a={0,0,1}, Vector3d b={4,0,1}, Vector3d v={0,0,0}) {
  return k.search(a,v,Vector3d::Zero(),b,Vector3d::Zero(),false);
}
// Dense independent evaluation checks original points, analytic derivatives, bounds and C1 continuity.
void validate(KinodynamicAstar& k, const pcl::PointCloud<pcl::PointXYZ>& cloud, Vector3d end) {
  auto segments=k.getSegments(); ASSERT_FALSE(segments.empty());
  Vector3d last_p, last_v; bool first=true;
  for (const auto& s:segments) {
    ASSERT_GT(s.duration,0); ASSERT_TRUE(s.coefficients.allFinite());
    const auto& c=s.coefficients;
    if (!first) { EXPECT_LT((c.col(0)-last_p).norm(),1e-8); EXPECT_LT((c.col(1)-last_v).norm(),1e-8); }
    for (int i=0;i<=1000;++i) {
      double t=s.duration*i/1000;
      Vector3d p=c.col(0)+c.col(1)*t+c.col(2)*t*t+c.col(3)*t*t*t;
      Vector3d v=c.col(1)+2*c.col(2)*t+3*c.col(3)*t*t;
      Vector3d a=2*c.col(2)+6*c.col(3)*t;
      ASSERT_LE(v.cwiseAbs().maxCoeff(),k.config().max_vel+1e-8);
      ASSERT_LE(a.cwiseAbs().maxCoeff(),k.config().max_acc+1e-8);
      ASSERT_TRUE((p.array()>=k.config().lower.array()-1e-8).all());
      ASSERT_TRUE((p.array()<=k.config().upper.array()+1e-8).all());
      for (const auto& q:cloud) ASSERT_GE((p-Vector3d(q.x,q.y,q.z)).norm(),k.config().safe_distance);
      last_p=p; last_v=v;
    }
    first=false;
  }
  EXPECT_LT((last_p-end).norm(),1e-8);
  auto samples=k.sampleTrajectory(0.017);
  ASSERT_FALSE(samples.empty()); EXPECT_DOUBLE_EQ(samples.front().time,0);
  EXPECT_LT((samples.back().position-end).norm(),1e-8);
  for (std::size_t i=1;i<samples.size();++i) EXPECT_GT(samples[i].time,samples[i-1].time);
}
}
TEST(Search, StraightAndExactEndpoint) {
  KinodynamicAstar k(config()); auto c=background(); ASSERT_TRUE(k.setKdtree(c));
  ASSERT_EQ(plan(k),KinodynamicAstar::REACH_END); validate(k,c,{4,0,1});
}
// ROS1 uses only SAFE_DIST, even after voxel filtering; an exact boundary is accepted.
TEST(Search, Ros1ClearanceHasNoExtraInflationAndUsesStrictBoundary) {
  pcl::PointCloud<pcl::PointXYZ> cloud; cloud.push_back({0,0,1});
  KinodynamicAstar k(config()); ASSERT_TRUE(k.setKdtree(cloud));
  EXPECT_FALSE(k.isSafe(0.44,0,1));
  EXPECT_TRUE(k.isSafe(0.46,0,1));  // Previously rejected by the 0.648205 m inflated radius.
  auto c=config(); c.safe_distance=0.5; KinodynamicAstar boundary(c);
  ASSERT_TRUE(boundary.setKdtree(cloud));
  EXPECT_FALSE(boundary.isSafe(0.49,0,1));
  EXPECT_TRUE(boundary.isSafe(0.5,0,1));  // 0.5² is exactly representable in PCL's float distances.
  EXPECT_TRUE(boundary.isSafe(0.51,0,1));
}
// PCL's float query/distances previously rejected both the exact double boundary and outside 1nm.
TEST(Search, DoubleEndpointBoundaryAndNanometreOffset) {
  KinodynamicAstar k(config()); pcl::PointCloud<pcl::PointXYZ> cloud; cloud.push_back({.5,0,1});
  ASSERT_TRUE(k.setKdtree(cloud));
  EXPECT_TRUE(k.isSafe(.05,0,1));
  EXPECT_TRUE(k.isSafe(.049999999,0,1));
  EXPECT_FALSE(k.isSafe(.050000001,0,1));
  EXPECT_EQ(plan(k,{.05,0,1},{-.5,0,1}),KinodynamicAstar::REACH_END);
  EXPECT_EQ(plan(k,{.049999999,0,1},{-.5,0,1}),KinodynamicAstar::REACH_END);
  EXPECT_EQ(plan(k,{.050000001,0,1},{-.5,0,1}),KinodynamicAstar::NO_PATH);
  // A held boundary position exercises both start and goal gates without interpolation roundoff.
  EXPECT_EQ(plan(k,{.05,0,1},{.05,0,1}),KinodynamicAstar::REACH_END);
  EXPECT_EQ(plan(k,{.049999999,0,1},{.049999999,0,1}),KinodynamicAstar::REACH_END);
  // Two PCL float distances tie at 0.5m; the double query is 1nm closer to the right centroid.
  auto cfg=config(); cfg.safe_distance=.5; KinodynamicAstar tied(cfg);
  cloud.clear(); cloud.push_back({-.5,0,1}); cloud.push_back({.5,0,1});
  ASSERT_TRUE(tied.setKdtree(cloud));
  EXPECT_TRUE(tied.isSafe(0,0,1));
  EXPECT_FALSE(tied.isSafe(.000000001,0,1));
  EXPECT_FALSE(tied.isSafe(-.000000001,0,1));
}
TEST(Search, NonzeroThreeAxisVelocityAndHigherAltitude) {
  KinodynamicAstar k(config()); auto c=background(); k.setKdtree(c);
  ASSERT_EQ(plan(k,{0,0,3},{3,2,4},{0.3,-0.2,0.1}),KinodynamicAstar::REACH_END);
  EXPECT_LT((k.getSegments().front().coefficients.col(1)-Vector3d(0.3,-0.2,0.1)).norm(),1e-10);
  validate(k,c,{3,2,4});
}
TEST(Search, GoesAroundVerticalThinPole) {
  KinodynamicAstar k(config()); auto c=background();
  for (float z=0.2;z<10;z+=0.025f) c.push_back({2,0,z});
  k.setKdtree(c); ASSERT_EQ(plan(k),KinodynamicAstar::REACH_END); validate(k,c,{4,0,1});
  double lateral=0; for (const auto& s:k.sampleTrajectory(0.01)) lateral=std::max(lateral,std::abs(s.position.y()));
  EXPECT_GT(lateral,0.45);
}
TEST(Search, MissingEmptyInvalidAndOversizedClouds) {
  auto cfg=config(); cfg.max_cloud_points=2; KinodynamicAstar k(cfg);
  EXPECT_EQ(plan(k),KinodynamicAstar::NO_MAP); EXPECT_TRUE(k.getSegments().empty());
  EXPECT_FALSE(k.setKdtree({})); auto c=background(); c[0].x=std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(k.setKdtree(c)); c=background(); c.push_back({3,3,3}); c.push_back({4,4,4});
  EXPECT_FALSE(k.setKdtree(c)); EXPECT_FALSE(k.mapReady());
}
TEST(Search, InvalidParametersAndStates) {
  auto c=config(); c.max_acc=0; EXPECT_THROW(KinodynamicAstar k(c),std::invalid_argument);
  c=config(); c.lower.z()=c.upper.z(); EXPECT_THROW(KinodynamicAstar k(c),std::invalid_argument);
  KinodynamicAstar k(config()); k.setKdtree(background());
  EXPECT_EQ(plan(k,{0,0,-1}),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(plan(k,{0,0,1},{4,0,1},{0,0,3}),KinodynamicAstar::INVALID_INPUT);
  EXPECT_THROW(k.sampleTrajectory(0),std::invalid_argument);
}
TEST(Search, BlockedStartAndGoalInvalidatePreviousPath) {
  KinodynamicAstar k(config()); auto c=background(); k.setKdtree(c); ASSERT_EQ(plan(k),KinodynamicAstar::REACH_END);
  c.push_back({4,0,1}); k.setKdtree(c); EXPECT_EQ(plan(k),KinodynamicAstar::NO_PATH);
  EXPECT_TRUE(k.getSegments().empty()); k.clearMap(); c=background(); c.push_back({0,0,1}); k.setKdtree(c);
  EXPECT_EQ(plan(k),KinodynamicAstar::NO_PATH);
}
TEST(Search, CoincidentStartGoal) {
  KinodynamicAstar k(config()); auto c=background(); k.setKdtree(c);
  ASSERT_EQ(plan(k,{0,0,1},{0,0,1}),KinodynamicAstar::REACH_END); validate(k,c,{0,0,1});
}
TEST(Search, NodeAndTimeBudgets) {
  auto c=config(); c.allocate_num=2; KinodynamicAstar limited(c); limited.setKdtree(background());
  EXPECT_EQ(plan(limited),KinodynamicAstar::NODE_LIMIT); EXPECT_TRUE(limited.getSegments().empty());
  c=config(); c.search_budget=1e-9; KinodynamicAstar timed(c); timed.setKdtree(background());
  EXPECT_EQ(plan(timed),KinodynamicAstar::TIMEOUT); EXPECT_TRUE(timed.getSegments().empty());
}
TEST(Search, HorizonIsPartialAndDynamicIndexIsDefined) {
  auto c=config(); c.horizon=1; KinodynamicAstar k(c); k.setKdtree(background());
  EXPECT_EQ(plan(k,{0,0,1},{10,0,1}),KinodynamicAstar::REACH_HORIZON);
  EXPECT_FALSE(k.getSegments().empty());
  EXPECT_EQ(k.search({0,0,1},{0,0,0},{0,0,0},{0.5,0,1},{0,0,0},false,true,12.3),KinodynamicAstar::REACH_END);
}
// A near-goal direct shot never exercises dynamic hashing. Force real expansions and revisit
// a position cell at multiple times; dynamic still uses the unchanged spatial collision map.
TEST(Search, DynamicSearchRetainsMultipleTimesInOnePositionCell) {
  auto c=config(); c.horizon=0.2; c.time_resolution=0.04;
  KinodynamicAstar k(c); ASSERT_TRUE(k.setKdtree(background()));
  constexpr double origin=12.3;
  ASSERT_EQ(k.search({0,0,1},{0,0,0},{0,0,0},{4,0,1},{0,0,0},true,true,origin),
            KinodynamicAstar::REACH_HORIZON);
  std::map<std::array<int,3>,std::set<int>> time_bins;
  for (const auto node:k.getVisitedNodes()) {
    EXPECT_EQ(node->time_idx,static_cast<int>(std::floor((node->time-origin)/c.time_resolution)));
    time_bins[{node->index.x(),node->index.y(),node->index.z()}].insert(node->time_idx);
  }
  std::size_t largest=0;
  for (const auto& cell:time_bins) largest=std::max(largest,cell.second.size());
  EXPECT_GE(largest,3u);
}
// In this scene all 124 nonzero default controls land in distinct cells on the first expansion.
// The 125th pool slot must be usable; exhaustion occurs only when another node is requested.
TEST(Search, DefaultLatticeAndLastPoolSlotAreUsable) {
  auto c=config(); c.horizon=0.1; c.allocate_num=125;
  KinodynamicAstar k(c); ASSERT_TRUE(k.setKdtree(background()));
  ASSERT_EQ(plan(k),KinodynamicAstar::REACH_HORIZON);
  const auto visited=k.getVisitedNodes(); ASSERT_EQ(visited.size(),125u);
  std::set<std::array<double,3>> controls;
  controls.insert({0,0,0});
  for (const auto node:visited) {
    if (!node->parent) continue;
    ASSERT_EQ(node->parent,visited.front());
    EXPECT_DOUBLE_EQ(node->duration,c.max_tau);
    EXPECT_LE(node->input.cwiseAbs().maxCoeff(),c.max_acc);
    controls.insert({node->input.x(),node->input.y(),node->input.z()});
  }
  ASSERT_EQ(controls.size(),125u);
  for (double x:{-2.,-1.,0.,1.,2.}) for (double y:{-2.,-1.,0.,1.,2.})
    for (double z:{-2.,-1.,0.,1.,2.}) EXPECT_EQ(controls.count({x,y,z}),1u);
  c.allocate_num=124; KinodynamicAstar short_pool(c); ASSERT_TRUE(short_pool.setKdtree(background()));
  EXPECT_EQ(plan(short_pool),KinodynamicAstar::NODE_LIMIT);
  EXPECT_TRUE(short_pool.getSegments().empty());
}
// Read actual allocated primitive inputs, including init mode, rather than duplicating enumeration.
TEST(Search, SmallPositiveParametersRespectAccelerationAndTwentyInitialDurations) {
  for (double initial_duration:{0.8,1e-6}) {
    auto c=config(); c.max_acc=1e-4; c.init_max_tau=initial_duration;
    c.time_resolution=initial_duration/40; c.horizon=0.1;
    KinodynamicAstar k(c); ASSERT_TRUE(k.setKdtree(background()));
    ASSERT_EQ(k.search({0,0,1},{1,0,0},{1e-4,-1e-4,0},{4,0,1},{0,0,0},true,true),
              KinodynamicAstar::REACH_HORIZON);
    const auto nodes=k.getVisitedNodes(); ASSERT_FALSE(nodes.empty());
    std::set<double> initial_durations;
    for (const auto node:nodes) {
      EXPECT_LE(node->input.cwiseAbs().maxCoeff(),c.max_acc);
      if (node->parent!=nodes.front()) continue;
      EXPECT_EQ(node->input,Vector3d(1e-4,-1e-4,0));
      EXPECT_GT(node->duration,0); EXPECT_LE(node->duration,initial_duration);
      initial_durations.insert(node->duration);
    }
    ASSERT_EQ(initial_durations.size(),20u);
    EXPECT_DOUBLE_EQ(*initial_durations.rbegin(),initial_duration);
  }
  for (double acceleration:{1e-4,1e-8,1e-12}) {
    auto c=config(); c.max_acc=acceleration; c.horizon=0.1;
    KinodynamicAstar k(c); ASSERT_TRUE(k.setKdtree(background()));
    const auto began=std::chrono::steady_clock::now();
    ASSERT_EQ(plan(k,{0,0,1},{4,0,1},{1,0,0}),KinodynamicAstar::REACH_HORIZON);
    EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),0.5);
    for (const auto node:k.getVisitedNodes()) EXPECT_LE(node->input.cwiseAbs().maxCoeff(),acceleration);
  }
}
TEST(Search, NumericRangeAndAccelerationInputFailuresAreExplicit) {
  const double denormal=std::numeric_limits<double>::denorm_min();
  for (double tiny:{denormal,std::numeric_limits<double>::min()}) {
    auto c=config(); c.init_max_tau=tiny; EXPECT_THROW(c.validate(),std::invalid_argument);
    c=config(); c.max_tau=tiny; EXPECT_THROW(c.validate(),std::invalid_argument);
  }
  auto c=config(); c.max_acc=denormal; EXPECT_THROW(c.validate(),std::invalid_argument);
  c=config(); c.time_resolution=denormal; EXPECT_THROW(c.validate(),std::invalid_argument);
  c.time_resolution=std::numeric_limits<double>::min(); EXPECT_NO_THROW(c.validate());
  c=config(); c.max_acc=std::numeric_limits<double>::max(); EXPECT_THROW(c.validate(),std::invalid_argument);
  c=config(); c.init_max_tau=std::numeric_limits<double>::max(); EXPECT_THROW(c.validate(),std::invalid_argument);
  // Exercise both sides of the smallest representable initial squared-duration boundary.
  c=config(); c.init_max_tau=20*std::sqrt(denormal); EXPECT_NO_THROW(c.validate());
  c.init_max_tau/=2; EXPECT_THROW(c.validate(),std::invalid_argument);
  c=config(); c.max_acc=1e-8; KinodynamicAstar k(c); ASSERT_TRUE(k.setKdtree(background()));
  for (bool init:{false,true}) {
    EXPECT_EQ(k.search({0,0,1},{0,0,0},{std::nextafter(c.max_acc,INFINITY),0,0},
                       {4,0,1},{0,0,0},init),KinodynamicAstar::INVALID_INPUT);
    EXPECT_TRUE(k.getSegments().empty());
  }
  EXPECT_EQ(k.search({0,0,1},{0,0,0},{0,0,0},{4,0,1},{0,0,0},false,true,1e300),
            KinodynamicAstar::INVALID_INPUT);
  c=config(); c.max_acc=1e150; c.search_budget=0.02;
  KinodynamicAstar large(c); ASSERT_TRUE(large.setKdtree(background()));
  const auto began=std::chrono::steady_clock::now();
  const int status=plan(large);
  EXPECT_TRUE(status==KinodynamicAstar::NO_PATH || status==KinodynamicAstar::TIMEOUT);
  EXPECT_TRUE(large.getSegments().empty());
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),0.5);
}
// The normal period is 50 accepted clouds. Mark each bank independently at its first update.
TEST(Search, DefaultTreePeriodSwitchesAtExactAcceptedUpdateBoundaries) {
  KinodynamicAstar k(config());
  for (int update=1;update<=101;++update) {
    auto cloud=background();
    if (update==1) cloud.push_back({2,0,1});
    if (update==51) cloud.push_back({4,0,1});
    ASSERT_TRUE(k.setKdtree(cloud));
    if (update==49 || update==50 || update==51 || update==99 || update==100 || update==101) {
      EXPECT_EQ(k.isSafe(2,0,1),update==101) << "update " << update;
      EXPECT_EQ(k.isSafe(4,0,1),update<51) << "update " << update;
      EXPECT_LE(k.mapPointCount(),4u);
    }
  }
}
TEST(Search, SampleCapIsExactAcrossSegmentJoinsAndRejectsHugeRatios) {
  auto c=config(); c.horizon=0.1; KinodynamicAstar k(c); ASSERT_TRUE(k.setKdtree(background()));
  ASSERT_EQ(plan(k),KinodynamicAstar::REACH_HORIZON);
  const auto samples=k.sampleTrajectory(0.1); ASSERT_GT(samples.size(),1u);
  EXPECT_EQ(k.sampleTrajectory(0.1,samples.size()).size(),samples.size());
  EXPECT_THROW(k.sampleTrajectory(0.1,samples.size()-1),std::length_error);
  EXPECT_THROW(k.sampleTrajectory(std::numeric_limits<double>::denorm_min()),std::length_error);
  EXPECT_THROW(k.sampleTrajectory(0.1,0),std::length_error);
  KinodynamicAstar full_planner(config()); ASSERT_TRUE(full_planner.setKdtree(background()));
  ASSERT_EQ(plan(full_planner),KinodynamicAstar::REACH_END);
  const auto full=full_planner.sampleTrajectory(0.07);
  EXPECT_GT(full_planner.getSegments().size(),1u);
  EXPECT_EQ(full_planner.sampleTrajectory(0.07,full.size()).size(),full.size());
  EXPECT_THROW(full_planner.sampleTrajectory(0.07,full.size()-1),std::length_error);
}
TEST(Search, TwoTreesRetainThenExpireAndStayBounded) {
  auto c=config(); c.tree_period=2; KinodynamicAstar k(c); auto cloud=background(); cloud.push_back({2,0,1});
  ASSERT_TRUE(k.setKdtree(cloud)); EXPECT_FALSE(k.isSafe(2,0,1));
  for (int i=0;i<3;++i) { k.setKdtree(background()); EXPECT_FALSE(k.isSafe(2,0,1)); }
  k.setKdtree(background()); EXPECT_TRUE(k.isSafe(2,0,1));
  for (int i=0;i<300;++i) ASSERT_TRUE(k.setKdtree(background()));
  EXPECT_LE(k.mapPointCount(),2u);
}
TEST(Search, ClosedWallTerminatesWithoutTrajectory) {
  auto cfg=config(); cfg.search_budget=0.08; cfg.lower={-1,-1,0.2}; cfg.upper={5,1,2};
  KinodynamicAstar k(cfg); auto c=background();
  for (float y=-1;y<=1;y+=0.15f) for (float z=0.2;z<=2;z+=0.15f) c.push_back({2,y,z});
  k.setKdtree(c);
  const auto began=std::chrono::steady_clock::now();
  const int status=plan(k);
  EXPECT_TRUE(status==KinodynamicAstar::NO_PATH || status==KinodynamicAstar::TIMEOUT);
  EXPECT_TRUE(k.getSegments().empty());
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),0.5);
}
TEST(Search, CollisionBetweenPrimitiveEndpoints) {
  auto cfg=config(); cfg.safe_distance=0.1; cfg.voxel_size=0.01; cfg.collision_step=0.025;
  KinodynamicAstar k(cfg); auto c=background(); c.push_back({0.36,0,1});
  k.setKdtree(c);
  ASSERT_EQ(plan(k,{0,0,1},{3,0,1},{1.2,0,0}),KinodynamicAstar::REACH_END);
  validate(k,c,{3,0,1});
}
// Closed-segment checks use independent point geometry, including strict equality and endpoints.
TEST(Search, ExactLineAndLatestObservationMap) {
  auto cfg=config(); cfg.safe_distance=.5;
  KinodynamicAstar k(cfg); auto cloud=background(); cloud.push_back({2,.5,1});
  ASSERT_TRUE(k.setKdtree(cloud));
  EXPECT_EQ(k.checkLine({0,0,1},{4,0,1}),KinodynamicAstar::REACH_END);
  cloud.push_back({2,.499,1}); ASSERT_TRUE(k.setKdtree(cloud));
  EXPECT_EQ(k.checkLine({0,0,1},{4,0,1}),KinodynamicAstar::NO_PATH);
  ASSERT_TRUE(k.setLatestObservation(background()));
  EXPECT_EQ(k.checkLine({0,0,1},{4,0,1}),KinodynamicAstar::REACH_END);
  cloud=background(); cloud.push_back({4,0,1}); ASSERT_TRUE(k.setLatestObservation(cloud));
  EXPECT_EQ(k.checkLine({0,0,1},{4,0,1}),KinodynamicAstar::NO_PATH);
  EXPECT_EQ(k.checkLine({4,0,1},{4,0,1}),KinodynamicAstar::NO_PATH);
  EXPECT_EQ(k.checkLine({0,0,-1},{4,0,1}),KinodynamicAstar::INVALID_INPUT);
  EXPECT_FALSE(k.setLatestObservation({}));
  EXPECT_EQ(k.checkLine({0,0,1},{4,0,1}),KinodynamicAstar::NO_MAP);
}
// Curved suffix checks catch unsampled interior collisions and ignore already executed geometry.
TEST(Search, ValidateRemainingCurveExactly) {
  KinodynamicAstar k(config()); auto cloud=background(); cloud.push_back({.5,0,1});
  ASSERT_TRUE(k.setKdtree(cloud));
  TrajectorySegment s; s.duration=4; s.coefficients.col(0)=Vector3d(0,0,1);
  s.coefficients.col(1)=Vector3d(1,0,0);
  EXPECT_EQ(k.validateTrajectory({s},0,{4,0,1}),KinodynamicAstar::NO_PATH);
  EXPECT_EQ(k.validateTrajectory({s},2,{4,0,1}),KinodynamicAstar::REACH_END);
  EXPECT_EQ(k.validateTrajectory({s},4,{4,0,1}),KinodynamicAstar::REACH_END);
  EXPECT_EQ(k.validateTrajectory({s},4.1,{4,0,1}),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(k.validateTrajectory({s},2,{5,0,1}),KinodynamicAstar::INVALID_INPUT);
  s.coefficients(0,0)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(k.validateTrajectory({s},0,{4,0,1}),KinodynamicAstar::INVALID_INPUT);
  // The endpoints' straight line is clear, but the cubic crosses the obstacle at t=.5.
  cloud=background(); cloud.push_back({.5,1.5,1}); ASSERT_TRUE(k.setLatestObservation(cloud));
  EXPECT_EQ(k.checkLine({0,0,1},{1,0,1}),KinodynamicAstar::REACH_END);
  s.duration=1; s.coefficients.setZero(); s.coefficients.col(0)=Vector3d(0,0,1);
  s.coefficients(0,1)=1; s.coefficients.row(1)<<0,8,-12,4;
  EXPECT_EQ(k.validateTrajectory({s},0,{1,0,1}),KinodynamicAstar::NO_PATH);
  EXPECT_EQ(k.validateTrajectory({s},.9,{1,0,1}),KinodynamicAstar::REACH_END);
}
// Dynamics are checked on exact derivatives; the exported trajectory is never clipped.
TEST(Search, OptionalHorizontalAndVerticalDynamics) {
  auto cfg=config(); cfg.max_horizontal_vel=1; cfg.max_vertical_vel=.2;
  cfg.max_horizontal_acc=.35; cfg.max_vertical_acc=.15; cfg.max_tau=1;
  KinodynamicAstar k(cfg); auto cloud=background(); ASSERT_TRUE(k.setKdtree(cloud));
  EXPECT_EQ(plan(k,{0,0,1},{1,0,1},{.8,.8,0}),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(plan(k,{0,0,1},{1,0,1},{0,0,.21}),KinodynamicAstar::INVALID_INPUT);
  ASSERT_EQ(plan(k,{0,0,1},{2,1,1.2},{.1,0,.05}),KinodynamicAstar::REACH_END);
  validate(k,cloud,{2,1,1.2});
  for (const auto& sample:k.sampleTrajectory(.001)) {
    EXPECT_LE(sample.velocity.head<2>().norm(),1+1e-8);
    EXPECT_LE(std::abs(sample.velocity.z()),.2+1e-8);
    EXPECT_LE(sample.acceleration.head<2>().norm(),.35+1e-8);
    EXPECT_LE(std::abs(sample.acceleration.z()),.15+1e-8);
  }
}
TEST(Search, ConstrainedDynamicsGoesAroundPole) {
  auto cfg=config(); cfg.max_horizontal_vel=1; cfg.max_vertical_vel=.2;
  cfg.max_horizontal_acc=.35; cfg.max_vertical_acc=.15; cfg.max_tau=1;
  cfg.max_vel=1; cfg.max_acc=.35; cfg.search_budget=.3;
  KinodynamicAstar k(cfg); auto cloud=background();
  // One point per 0.1m voxel makes the independent raw-point oracle equal the centroid contract.
  for (float z=.25;z<10;z+=.125f) cloud.push_back({2,0,z});
  ASSERT_TRUE(k.setKdtree(cloud)); ASSERT_EQ(plan(k),KinodynamicAstar::REACH_END);
  validate(k,cloud,{4,0,1});
  for (const auto& sample:k.sampleTrajectory(.001)) {
    ASSERT_LE(sample.velocity.head<2>().norm(),1+1e-8);
    ASSERT_LE(std::abs(sample.velocity.z()),.2+1e-8);
    ASSERT_LE(sample.acceleration.head<2>().norm(),.35+1e-8);
    ASSERT_LE(std::abs(sample.acceleration.z()),.15+1e-8);
  }
}
