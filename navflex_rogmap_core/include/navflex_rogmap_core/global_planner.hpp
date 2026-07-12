// Copyright 2026 Navflex contributors

#ifndef NAVFLEX_ROGMAP_CORE__GLOBAL_PLANNER_HPP_
#define NAVFLEX_ROGMAP_CORE__GLOBAL_PLANNER_HPP_
#include <cstdint>
#include <memory>
#include <string>
#include "navflex_rog_map/rog_map.hpp"
#include "navflex_rogmap_core/types.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
namespace navflex_rogmap_core
{
class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>; virtual ~GlobalPlanner() = default;
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    std::shared_ptr<tf2_ros::Buffer>, navflex_rog_map::RogMap::ConstPtr) = 0;
  virtual void cleanup() = 0; virtual void activate() = 0; virtual void deactivate() = 0;
  virtual uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::PoseStamped &,
    Trajectory3D &, std::string & message) = 0;
  virtual bool cancel() {return true;}
};
}  // namespace navflex_rogmap_core
#endif  // NAVFLEX_ROGMAP_CORE__GLOBAL_PLANNER_HPP_
