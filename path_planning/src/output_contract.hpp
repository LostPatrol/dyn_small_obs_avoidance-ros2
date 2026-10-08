// GPL-3.0: final publication gate shared by the wrapper and deterministic clock regressions.
#pragma once

#include <path_planning/msg/plan_result.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cstdint>
#include <string>
#include <utility>

namespace path_planning {
// Clear only this attempt; a failed result must not replace the last-success RViz preview.
inline void rejectOutput(msg::PlanResult& result, nav_msgs::msg::Path& path,
                         uint8_t status, const std::string& detail) {
  result.status = status;
  result.detail = detail;
  result.segments.clear();
  result.trajectory.points.clear();
  path.poses.clear();
}

// Invoke both clocks after all construction. The callbacks let tests advance a controlled
// clock across construction without adding delay switches to the running ROS node.
template<class Fresh, class WithinBudget>
void finalizeOutput(msg::PlanResult& result, nav_msgs::msg::Path& path,
                    Fresh&& fresh, WithinBudget&& within_budget) {
  if (result.status != msg::PlanResult::REACH_END &&
      result.status != msg::PlanResult::REACH_HORIZON) return;
  if (!std::forward<Fresh>(fresh)()) {
    rejectOutput(result, path, msg::PlanResult::STALE_INPUT, "input expired during output construction");
  } else if (!std::forward<WithinBudget>(within_budget)()) {
    rejectOutput(result, path, msg::PlanResult::TIMEOUT, "output construction budget exceeded");
  }
}
}  // namespace path_planning
