#include "navflex_frontier_planner/frontier_astar_planner.hpp"

#include <chrono>
#include <sstream>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navflex_frontier_planner
{

void FrontierAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  core_.configure(parent, name, tf, costmap_ros);
}

void FrontierAStarPlanner::cleanup()
{
  core_.cleanup();
}

void FrontierAStarPlanner::activate()
{
  core_.activate();
}

void FrontierAStarPlanner::deactivate()
{
  core_.deactivate();
}

nav_msgs::msg::Path FrontierAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  std::string message;
  makePlan(start, goal, plan, message);
  return plan;
}

uint32_t FrontierAStarPlanner::makePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & plan,
  std::string & message)
{
  using Result = nav2_msgs::action::ComputePathToPose::Result;
  const auto started = std::chrono::steady_clock::now();
  plan = nav_msgs::msg::Path();

  if (start.header.frame_id.empty()) {
    message = "Invalid start: start.header.frame_id is empty";
    return Result::INVALID_START;
  }
  if (goal.header.frame_id.empty()) {
    message = "Invalid goal: goal.header.frame_id is empty";
    return Result::INVALID_GOAL;
  }

  auto candidates = core_.selectCandidates(start, goal, true);
  const auto candidates_done = std::chrono::steady_clock::now();
  if (candidates.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("FrontierAStarPlanner"),
      "FrontierAStar planning failed during candidate selection after %.2fms",
      std::chrono::duration<double, std::milli>(candidates_done - started).count());
    message =
      "No frontier candidate found. Check point cloud input, TF availability, and whether enough "
      "free/unknown boundary voxels have been observed.";
    return Result::NO_PATH_FOUND;
  }
  const std::size_t selected_index = 0;
  plan = core_.makeAStarPath(start, candidates[selected_index]);
  const auto path_done = std::chrono::steady_clock::now();
  if (plan.poses.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("FrontierAStarPlanner"),
      "FrontierAStar planning failed after %.2fms: phases_ms{candidates=%.2f path=%.2f}",
      std::chrono::duration<double, std::milli>(path_done - started).count(),
      std::chrono::duration<double, std::milli>(candidates_done - started).count(),
      std::chrono::duration<double, std::milli>(path_done - candidates_done).count());
    message =
      "The best frontier candidate cannot be connected through the accumulated point-cloud "
      "topology map. Check start_edges and goal_edges in the topology A* logs.";
    return Result::NO_PATH_FOUND;
  }

  std::ostringstream oss;
  oss << "Selected best frontier candidate 1/" << candidates.size()
      << " and generated an A* path on the accumulated point-cloud topology map";
  message = oss.str();
  core_.publishSelection(start, candidates, candidates[selected_index]);
  const auto finished = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    rclcpp::get_logger("FrontierAStarPlanner"),
    "FrontierAStar planning success: candidates=%zu path_poses=%zu total=%.2fms "
    "phases_ms{candidates=%.2f path=%.2f publish=%.2f}",
    candidates.size(), plan.poses.size(),
    std::chrono::duration<double, std::milli>(finished - started).count(),
    std::chrono::duration<double, std::milli>(candidates_done - started).count(),
    std::chrono::duration<double, std::milli>(path_done - candidates_done).count(),
    std::chrono::duration<double, std::milli>(finished - path_done).count());
  return Result::SUCCESS;
}

}  // namespace navflex_frontier_planner

PLUGINLIB_EXPORT_CLASS(
  navflex_frontier_planner::FrontierAStarPlanner,
  nav2_core::GlobalPlanner)
