#include "navflex_frontier_planner/frontier_astar_planner.hpp"

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
  const auto & candidate = candidates.front();

  plan = core_.makeAStarPath(start, candidate, candidates);
  if (plan.poses.empty()) {
    std::ostringstream oss;
    oss << "Topology graph planning failed to find a connected road-graph path to selected "
        << "frontier candidate at ("
        << candidate.point.x << ", " << candidate.point.y << ", " << candidate.point.z
        << "). Check /frontier_exploration/topology_map, road_graph_dist, local_range, and "
        << "whether start/goal can connect to nearby topology nodes.";
    message = oss.str();
    return Result::NO_PATH_FOUND;
  }

  message = "Selected frontier candidate and planned topology graph path";
  return Result::SUCCESS;
}

}  // namespace navflex_frontier_planner

PLUGINLIB_EXPORT_CLASS(
  navflex_frontier_planner::FrontierAStarPlanner,
  nav2_core::GlobalPlanner)
