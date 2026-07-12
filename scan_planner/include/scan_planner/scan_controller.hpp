// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SCAN_PLANNER__SCAN_CONTROLLER_HPP_
#define SCAN_PLANNER__SCAN_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "navflex_rogmap_core/controller.hpp"

namespace scan_planner
{

class ScanController : public navflex_rogmap_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    navflex_rog_map::RogMap::ConstPtr map) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setTrajectory(const navflex_rogmap_core::Trajectory3D & trajectory) override;
  uint32_t computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    geometry_msgs::msg::TwistStamped & command,
    nav2_core::GoalChecker * goal_checker,
    std::string & message) override;
  void setSpeedLimit(double speed_limit, bool percentage) override;
  bool cancel() override;

private:
  struct Point
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  bool occupied(const geometry_msgs::msg::Pose & pose) const;
  bool segmentFree(const Point & start, const Point & end) const;
  std::vector<Point> buildLocalTrajectory(const Point & current) const;
  Point evaluate(double time) const;
  Point evaluateVelocity(double time) const;
  static geometry_msgs::msg::Point toMessage(const Point & point);
  static double distance(const Point & first, const Point & second);
  static double clamp(double value, double minimum, double maximum);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  navflex_rog_map::RogMap::ConstPtr map_;
  navflex_rogmap_core::Trajectory3D input_trajectory_;
  std::vector<Point> local_trajectory_;
  mutable std::mutex mutex_;
  std::string name_;
  std::string global_frame_;
  navflex_rog_map::Footprint3D footprint_;
  double planning_horizon_{5.0};
  double sample_distance_{0.1};
  double lookahead_time_{0.8};
  double control_lookahead_time_{0.15};
  double max_velocity_{0.8};
  double max_vertical_velocity_{0.4};
  double max_acceleration_{1.5};
  double max_yaw_rate_{1.0};
  double position_gain_{1.5};
  double yaw_gain_{2.0};
  double speed_scale_{1.0};
  double trajectory_time_{0.0};
  rclcpp::Time last_time_;
  bool active_{false};
  bool canceled_{false};
};

}  // namespace scan_planner

#endif  // SCAN_PLANNER__SCAN_CONTROLLER_HPP_
