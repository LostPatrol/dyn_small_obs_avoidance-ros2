// GPL-3.0: deterministic output-stage clock regression for the production publication gate.
#include "output_contract.hpp"
#include <gtest/gtest.h>
#include <chrono>

namespace {
using Result = path_planning::msg::PlanResult;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

// Populate the same three output containers as construction; last-success Path stays external.
void constructOutput(Result& result, nav_msgs::msg::Path& path) {
  result.status = Result::REACH_END;
  result.segments.resize(1);
  result.trajectory.points.resize(2);
  path.poses.resize(2);
}

// Independently expire either input after a successful search and a controlled construction
// interval. No ROS parameters or test-only delay hooks are added to the production node.
void checkInputExpiration(bool expire_cloud) {
  auto now = Clock::time_point{} + 450ms;
  const auto cloud_received = Clock::time_point{} + (expire_cloud ? 0ms : 200ms);
  const auto odom_received = Clock::time_point{} + (expire_cloud ? 200ms : 0ms);
  auto fresh = [&] { return now-cloud_received <= 500ms && now-odom_received <= 500ms; };
  ASSERT_TRUE(fresh());  // Both inputs pass the post-search gate.
  Result result;
  nav_msgs::msg::Path path;
  constructOutput(result, path);
  const auto last_success = path;
  now += 60ms;  // Output construction crosses exactly one input deadline.
  path_planning::finalizeOutput(result, path, fresh, [] { return true; });
  EXPECT_EQ(result.status, Result::STALE_INPUT);
  EXPECT_EQ(result.detail, "input expired during output construction");
  EXPECT_TRUE(result.segments.empty());
  EXPECT_TRUE(result.trajectory.points.empty());
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(last_success.poses.size(), 2u);
}

TEST(OutputContract, CloudExpiresDuringConstruction) { checkInputExpiration(true); }
TEST(OutputContract, OdometryExpiresDuringConstruction) { checkInputExpiration(false); }

TEST(OutputContract, FastConstructionKeepsSuccess) {
  auto now = Clock::time_point{} + 450ms;
  const auto received = Clock::time_point{};
  Result result;
  nav_msgs::msg::Path path;
  constructOutput(result, path);
  now += 10ms;
  path_planning::finalizeOutput(result, path, [&] { return now-received <= 500ms; },
                              [] { return true; });
  EXPECT_EQ(result.status, Result::REACH_END);
  EXPECT_EQ(result.segments.size(), 1u);
  EXPECT_EQ(result.trajectory.points.size(), 2u);
  EXPECT_EQ(path.poses.size(), 2u);
}

TEST(OutputContract, ConstructionBudgetClearsEveryOutput) {
  Result result;
  nav_msgs::msg::Path path;
  constructOutput(result, path);
  path_planning::finalizeOutput(result, path, [] { return true; }, [] { return false; });
  EXPECT_EQ(result.status, Result::TIMEOUT);
  EXPECT_TRUE(result.segments.empty());
  EXPECT_TRUE(result.trajectory.points.empty());
  EXPECT_TRUE(path.poses.empty());
}

TEST(OutputContract, StaleInputTakesPrecedenceOverConstructionBudget) {
  Result result;
  nav_msgs::msg::Path path;
  constructOutput(result, path);
  path_planning::finalizeOutput(result, path, [] { return false; }, [] { return false; });
  EXPECT_EQ(result.status, Result::STALE_INPUT);
  EXPECT_TRUE(result.segments.empty());
  EXPECT_TRUE(result.trajectory.points.empty());
  EXPECT_TRUE(path.poses.empty());
}

TEST(OutputContract, PartialSuccessUsesTheSameGate) {
  Result result;
  nav_msgs::msg::Path path;
  constructOutput(result, path);
  result.status = Result::REACH_HORIZON;
  path_planning::finalizeOutput(result, path, [] { return false; }, [] { return true; });
  EXPECT_EQ(result.status, Result::STALE_INPUT);
  EXPECT_TRUE(result.segments.empty());
  EXPECT_TRUE(result.trajectory.points.empty());
  EXPECT_TRUE(path.poses.empty());
}
}  // namespace
