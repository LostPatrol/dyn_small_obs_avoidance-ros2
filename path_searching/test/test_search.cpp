// Core regression tests use raw point geometry, not the planner's collision predicate, as the oracle.
#include <gtest/gtest.h>
#include <path_searching/kinodynamic_astar.h>
#include <algorithm>
#include <limits>
#include <map>
#include <set>
using Eigen::Vector3d;
namespace {
// Existing geometry/lattice regressions retain their explicit legacy envelope and primitive duration.
SearchConfig config() {
  SearchConfig c; c.search_budget=2.0; c.max_tau=0.6; c.max_vel=c.max_acc=2.0;
  c.max_vel_xy=c.max_acc_xy=std::sqrt(2.0)*2.0; c.max_vel_z=c.max_acc_z=2.0;
  c.safe_distance=0.45; c.lower.z()=0.2; return c;
}
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
      ASSERT_LE(v.head<2>().norm(),k.config().max_vel_xy+1e-8);
      ASSERT_LE(std::abs(v.z()),k.config().max_vel_z+1e-8);
      ASSERT_LE(a.head<2>().norm(),k.config().max_acc_xy+1e-8);
      ASSERT_LE(std::abs(a.z()),k.config().max_acc_z+1e-8);
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
  auto invalid_weight=config(); invalid_weight.vertical_distance_weight=-1;
  EXPECT_THROW(KinodynamicAstar k(invalid_weight),std::invalid_argument);
  invalid_weight.vertical_distance_weight=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(KinodynamicAstar k(invalid_weight),std::invalid_argument);
  auto c=config(); c.max_acc=0; EXPECT_THROW(KinodynamicAstar k(c),std::invalid_argument);
  c=config(); c.lower.z()=c.upper.z(); EXPECT_THROW(KinodynamicAstar k(c),std::invalid_argument);
  KinodynamicAstar k(config()); k.setKdtree(background());
  EXPECT_EQ(plan(k,{0,0,-1}),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(plan(k,{0,0,1},{4,0,1},{0,0,std::numeric_limits<double>::infinity()}),KinodynamicAstar::INVALID_INPUT);
  EXPECT_THROW(k.sampleTrajectory(0),std::invalid_argument);
}

// A measured overspeed state stays exact while the first checked primitive brakes into the envelope.
TEST(Search, OverspeedStartBrakesWithoutChangingInitialVelocity) {
  SearchConfig c;
  KinodynamicAstar planner(c); ASSERT_TRUE(planner.setKdtree(background()));
  const Vector3d velocity(1.2,0.3,0.26);
  ASSERT_EQ(planner.search({0,0,1},velocity,Vector3d::Zero(),{4,0,1},Vector3d::Zero(),false),
            KinodynamicAstar::REACH_END);
  const auto segments=planner.getSegments(); ASSERT_GT(segments.size(),1u);
  const auto& first=segments.front();
  EXPECT_LT((first.coefficients.col(1)-velocity).norm(),1e-12);
  const Vector3d acceleration=2*first.coefficients.col(2);
  EXPECT_LE(acceleration.head<2>().norm(),c.max_acc_xy+1e-9);
  EXPECT_LE(std::abs(acceleration.z()),c.max_acc_z+1e-9);
  const Vector3d recovered=velocity+acceleration*first.duration;
  EXPECT_NEAR(recovered.head<2>().norm(),c.max_vel_xy,1e-9);
  EXPECT_NEAR(recovered.z(),c.max_vel_z,1e-9);
  for (const auto& sample:planner.sampleTrajectory(0.002)) {
    const double factor=std::clamp(sample.time/first.duration,0.0,1.0);
    const Vector3d envelope=velocity+(recovered-velocity)*factor;
    EXPECT_LE(sample.velocity.head<2>().norm(),envelope.head<2>().norm()+1e-9);
    EXPECT_LE(std::abs(sample.velocity.z()),std::max(std::abs(envelope.z()),c.max_vel_z)+1e-9);
  }
  // Tiny excess speed can brake without leaving the root spatial cell.
  ASSERT_EQ(planner.search({0,0,1},{0,0,0.201},Vector3d::Zero(),{0.5,0,1},Vector3d::Zero(),true),
            KinodynamicAstar::REACH_END);
  EXPECT_DOUBLE_EQ(planner.getSegments().front().coefficients(2,1),0.201);
}

// Recovery cannot excuse crossing the height bound or an obstacle during the braking prefix.
TEST(Search, OverspeedRecoveryRetainsHeightAndCollisionChecks) {
  SearchConfig c; c.lower.z()=0.8; c.upper.z()=1.1;
  KinodynamicAstar planner(c); ASSERT_TRUE(planner.setKdtree(background()));
  EXPECT_EQ(planner.search({0,0,1.099},{0,0,0.3},Vector3d::Zero(),{0.5,0,1},Vector3d::Zero(),false),
            KinodynamicAstar::NO_PATH);
  EXPECT_TRUE(planner.getSegments().empty());
  planner.clearMap();
  pcl::PointCloud<pcl::PointXYZ> cloud; cloud.push_back({0.3,0,1});
  ASSERT_TRUE(planner.setKdtree(cloud));
  EXPECT_EQ(planner.search({0,0,1},{1.2,0,0},Vector3d::Zero(),{-0.5,0,1},Vector3d::Zero(),false),
            KinodynamicAstar::NO_PATH);
  EXPECT_TRUE(planner.getSegments().empty());
}

// A small vertical drift needs an exact stopping control rather than a large opposing lattice input.
TEST(Search, NarrowVerticalLatticeStopsSmallDriftBeforeCruise) {
  SearchConfig c; c.vertical_distance_weight=10;
  KinodynamicAstar planner(c); ASSERT_TRUE(planner.setKdtree(background()));
  for (double vz:{-0.02,0.02}) {
    ASSERT_EQ(planner.search({0,0,1},{0,0,vz},Vector3d::Zero(),{4,0,1},Vector3d::Zero(),false),
              KinodynamicAstar::REACH_END);
    // One regular-duration stop travels |vz|*tau/2 vertically; cruising must not keep accumulating it.
    const double stopping_travel=std::abs(vz)*c.max_tau/2;
    for (const auto& sample:planner.sampleTrajectory(0.002))
      EXPECT_LE(std::abs(sample.position.z()-1),stopping_travel+1e-9);
    validate(planner,background(),{4,0,1});
  }
}

// Compare two feasible detours under the deployed directional limits, without changing clearance.
TEST(Search, VerticalTravelCostPrefersHorizontalDetourAndRetainsVerticalEscape) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int y=-12;y<=12;++y)
    for (int z=-4;z<=4;++z)
      cloud.push_back({2.0f,float(y*0.05),float(1+z*0.05)});
  SearchConfig legacy;
  KinodynamicAstar original(legacy); ASSERT_TRUE(original.setKdtree(cloud));
  ASSERT_EQ(original.search({0,0,1},{0,0,0},{0,0,0},{4,0,1},{0,0,0},true),KinodynamicAstar::REACH_END);
  // An unweighted search may also find a horizontal route; do not require a worse baseline.
  validate(original,cloud,{4,0,1});
  auto preferred=legacy; preferred.vertical_distance_weight=10;
  KinodynamicAstar horizontal(preferred); ASSERT_TRUE(horizontal.setKdtree(cloud));
  ASSERT_EQ(horizontal.search({0,0,1},{0,0,0},{0,0,0},{4,0,1},{0,0,0},true),KinodynamicAstar::REACH_END);
  double lateral=0;
  for (const auto& s:horizontal.sampleTrajectory(0.002)) {
    EXPECT_NEAR(s.position.z(),1,1e-9);
    lateral=std::max(lateral,std::abs(s.position.y()));
  }
  EXPECT_GT(lateral,0.85);
  validate(horizontal,cloud,{4,0,1});
  // A wall across the allowed width leaves only a vertical detour, which must remain available.
  preferred.lower.y()=-0.5; preferred.upper.y()=0.5;
  KinodynamicAstar vertical(preferred); ASSERT_TRUE(vertical.setKdtree(cloud));
  ASSERT_EQ(vertical.search({0,0,1},{0,0,0},{0,0,0},{4,0,1},{0,0,0},true),KinodynamicAstar::REACH_END);
  double excursion=0;
  for (const auto& s:vertical.sampleTrajectory(0.002))
    excursion=std::max(excursion,std::abs(s.position.z()-1));
  EXPECT_GT(excursion,0.4);
  validate(vertical,cloud,{4,0,1});
}

// Check accumulated primitive cost against numerical |vz| integration, including a reversal.
TEST(Search, VerticalTravelCostCountsBothSidesOfPrimitiveTurnaround) {
  auto c=config(); c.vertical_distance_weight=10; c.horizon=0.05;
  KinodynamicAstar planner(c); ASSERT_TRUE(planner.setKdtree(background()));
  ASSERT_EQ(planner.search({0,0,1},{0.5,0,0.2},{0,0,-1},{4,0,1},{0,0,0},true),
            KinodynamicAstar::REACH_HORIZON);
  bool saw_reversal=false;
  for (const auto node:planner.getVisitedNodes()) {
    if (!node->parent) continue;
    const double vz=node->parent->state(5),az=node->input.z(),dt=node->duration/2000;
    double travel=0;
    for (int i=0;i<2000;++i) travel+=std::abs(vz+az*(i+0.5)*dt)*dt;
    saw_reversal|=vz*(vz+az*node->duration)<0;
    const double expected=node->parent->g_score+(node->input.squaredNorm()+c.w_time)*node->duration+
        c.vertical_distance_weight*travel;
    EXPECT_NEAR(node->g_score,expected,1e-5);
  }
  EXPECT_TRUE(saw_reversal);
}

// A horizontal direct connection stays level; moving starts retain their physical vertical speed.
TEST(Search, VerticalTravelCostPreservesShotBoundaryStates) {
  SearchConfig c; c.vertical_distance_weight=10;
  KinodynamicAstar planner(c); ASSERT_TRUE(planner.setKdtree(background()));
  for (double vz:{0.0,0.1}) {
    ASSERT_EQ(planner.search({0,0,1},{0,0,vz},{0,0,0},{1,0,1},{0,0,0},true),KinodynamicAstar::REACH_END);
    const auto samples=planner.sampleTrajectory(0.002);
    ASSERT_FALSE(samples.empty()); EXPECT_NEAR(samples.front().velocity.z(),vz,1e-12);
    if (vz==0) for (const auto& s:samples) EXPECT_NEAR(s.position.z(),1,1e-12);
    validate(planner,background(),{1,0,1});
  }
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
  // An exact binary bin makes division and reciprocal multiplication agree at boundaries.
  auto c=config(); c.horizon=0.2; c.time_resolution=1.0/32.0;
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
    // Stop after the root expansion, before later search can reparent these initial candidates.
    c.time_resolution=initial_duration/40; c.horizon=initial_duration/100;
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
  c=config(); c.max_acc=c.max_acc_xy=c.max_acc_z=1e150; c.search_budget=0.02;
  KinodynamicAstar large(c); ASSERT_TRUE(large.setKdtree(background()));
  const auto began=std::chrono::steady_clock::now();
  const int status=plan(large);
  // Feasible axis sampling can now produce finite controls even under a huge configured cap.
  if (status==KinodynamicAstar::REACH_END) validate(large,background(),{4,0,1});
  else {
    EXPECT_TRUE(status==KinodynamicAstar::NO_PATH || status==KinodynamicAstar::TIMEOUT);
    EXPECT_TRUE(large.getSegments().empty());
  }
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),0.5);
}
// The normal period is 20 accepted clouds. Mark each bank independently at its first update.
TEST(Search, DefaultTreePeriodSwitchesAtExactAcceptedUpdateBoundaries) {
  KinodynamicAstar k(config());
  for (int update=1;update<=41;++update) {
    auto cloud=background();
    if (update==1) cloud.push_back({2,0,1});
    if (update==21) cloud.push_back({4,0,1});
    ASSERT_TRUE(k.setKdtree(cloud));
    if (update==19 || update==20 || update==21 || update==39 || update==40 || update==41) {
      EXPECT_EQ(k.isSafe(2,0,1),update==41) << "update " << update;
      EXPECT_EQ(k.isSafe(4,0,1),update<21) << "update " << update;
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

// Native envelope is evaluated directly on exact output curves, without any retiming.
TEST(Search, NativeDirectionalEnvelopeOnPrimitivesAndTerminalShot) {
  SearchConfig c; KinodynamicAstar k(c); auto cloud=background(); ASSERT_TRUE(k.setKdtree(cloud));
  for (const Vector3d goal : {Vector3d(0.5,0.5,1.4),Vector3d(4,0,1),Vector3d(4,2,2.4),Vector3d(0,0,2.4)}) {
    ASSERT_EQ(plan(k,{0,0,1},goal),KinodynamicAstar::REACH_END);
    validate(k,cloud,goal);
  }
  // A measured start may brake into the envelope; requested terminal velocity must already fit it.
  EXPECT_EQ(k.search({0,0,1},{0,0,0},{0,0,0},{0.5,0.5,1.4},{0.8,0.8,0},false),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(k.search({0,0,1},{0,0,0},{0,0,0},{0.5,0.5,1.4},{0,0,0.21},false),KinodynamicAstar::INVALID_INPUT);
  EXPECT_EQ(k.search({0,0,1},{0,0,0},{1.5,1.5,0},{0.5,0.5,1.4},{0,0,0},true),
            KinodynamicAstar::INVALID_INPUT);
}

// A finite pole permits vertical avoidance; densely check the original obstacle points and time derivatives.
TEST(Search, NativeDirectionalEnvelopeAroundFinitePole) {
  SearchConfig c; KinodynamicAstar k(c); auto cloud=background();
  for (float z=0;z<=1.8f;z+=0.025f) cloud.push_back({1.5f,0,z});
  ASSERT_TRUE(k.setKdtree(cloud));
  ASSERT_EQ(plan(k,{0,0,1},{3,0,1}),KinodynamicAstar::REACH_END);
  validate(k,cloud,{3,0,1});
}

// Initial short coasts are valid prefixes even before they leave the root's spatial hash cell.
TEST(Search, InitialSameCellRetainsAllTwentyDurationsWithoutReplacingRoot) {
  SearchConfig c; c.horizon=0.1; KinodynamicAstar k(c); ASSERT_TRUE(k.setKdtree(background()));
  ASSERT_EQ(k.search({0,0,1},{0,0,0},{0,0,0},{4,0,1},{0,0,0},true),
            KinodynamicAstar::REACH_HORIZON);
  const auto visited=k.getVisitedNodes(); ASSERT_FALSE(visited.empty());
  std::set<double> durations;
  for (const auto node:visited) {
    if (node->parent!=visited.front()) continue;
    EXPECT_EQ(node->index,visited.front()->index);
    EXPECT_EQ(node->input,Vector3d::Zero());
    durations.insert(node->duration);
  }
  ASSERT_EQ(durations.size(),20u);
  EXPECT_DOUBLE_EQ(*durations.begin(),c.init_max_tau/20);
  EXPECT_DOUBLE_EQ(*durations.rbegin(),c.init_max_tau);
  const auto segments=k.getSegments(); ASSERT_GT(segments.size(),1u);
  EXPECT_DOUBLE_EQ(segments.front().duration,c.init_max_tau/20);
  EXPECT_EQ(segments.front().coefficients.col(2),Vector3d::Zero());
  EXPECT_EQ(visited.front()->parent,nullptr);
}
