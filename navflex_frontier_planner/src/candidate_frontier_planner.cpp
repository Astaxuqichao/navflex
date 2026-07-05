#include "navflex_frontier_planner/candidate_frontier_planner.hpp"

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navflex_frontier_planner
{

void CandidateFrontierPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  core_.configure(parent, name, tf, costmap_ros);
}

void CandidateFrontierPlanner::cleanup()
{
  core_.cleanup();
}

void CandidateFrontierPlanner::activate()
{
  core_.activate();
}

void CandidateFrontierPlanner::deactivate()
{
  core_.deactivate();
}

nav_msgs::msg::Path CandidateFrontierPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  std::string message;
  makePlan(start, goal, plan, message);
  return plan;
}

uint32_t CandidateFrontierPlanner::makePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & plan,
  std::string & message)
{
  using Result = nav2_msgs::action::ComputePathToPose::Result;
  plan = nav_msgs::msg::Path();

  if (start.header.frame_id.empty()) {
    message = "Invalid start: start.header.frame_id is empty";
    return Result::INVALID_START;
  }
  if (goal.header.frame_id.empty()) {
    message = "Invalid goal: goal.header.frame_id is empty";
    return Result::INVALID_GOAL;
  }

  auto candidates = core_.selectCandidates(start, goal);
  if (candidates.empty()) {
    message =
      "No frontier candidate found. Check point cloud input, TF availability, and whether enough "
      "free/unknown boundary voxels have been observed.";
    return Result::NO_PATH_FOUND;
  }

  plan = core_.makeCandidatePath(start, candidates);
  if (plan.poses.empty()) {
    message = "Frontier candidates were selected, but generated candidate path is empty";
    return Result::EMPTY_PATH;
  }

  message = "Selected frontier candidates";
  return Result::SUCCESS;
}

}  // namespace navflex_frontier_planner

PLUGINLIB_EXPORT_CLASS(
  navflex_frontier_planner::CandidateFrontierPlanner,
  nav2_core::GlobalPlanner)
