// Copyright 2026 Navflex contributors

#ifndef NAVFLEX_ROGMAP_CORE__CONTROLLER_HPP_
#define NAVFLEX_ROGMAP_CORE__CONTROLLER_HPP_
#include <cstdint>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "navflex_rog_map/rog_map.hpp"
#include "navflex_rogmap_core/types.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
namespace navflex_rogmap_core
{
class Controller
{
public:
  using Ptr = std::shared_ptr<Controller>; virtual ~Controller() = default;
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    std::shared_ptr<tf2_ros::Buffer>, navflex_rog_map::RogMap::ConstPtr) = 0;
  virtual void cleanup() = 0; virtual void activate() = 0; virtual void deactivate() = 0;
  virtual void setTrajectory(const Trajectory3D &) = 0;
  virtual uint32_t computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &, geometry_msgs::msg::TwistStamped &,
    nav2_core::GoalChecker *, std::string & message) = 0;
  virtual void setSpeedLimit(double, bool) = 0; virtual bool cancel() {return true;}
};
}  // namespace navflex_rogmap_core
#endif  // NAVFLEX_ROGMAP_CORE__CONTROLLER_HPP_
