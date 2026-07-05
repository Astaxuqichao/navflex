#ifndef NAVFLEX_FRONTIER_PLANNER__FRONTIER_ASTAR_PLANNER_HPP_
#define NAVFLEX_FRONTIER_PLANNER__FRONTIER_ASTAR_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "navflex_frontier_planner/fael_frontier_core.hpp"

namespace navflex_frontier_planner
{

class FrontierAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
  uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & plan,
    std::string & message) override;

private:
  FaelFrontierCore core_;
};

}  // namespace navflex_frontier_planner

#endif  // NAVFLEX_FRONTIER_PLANNER__FRONTIER_ASTAR_PLANNER_HPP_
