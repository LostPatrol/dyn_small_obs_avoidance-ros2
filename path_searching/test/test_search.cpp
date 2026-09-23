// Core regression tests use raw point geometry, not the planner's collision predicate, as the oracle.
#include <gtest/gtest.h>
#include <path_searching/kinodynamic_astar.h>
#include <limits>
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
