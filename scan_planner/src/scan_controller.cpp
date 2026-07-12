// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: BSD-3-Clause

#include "scan_planner/scan_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace scan_planner
{

double ScanController::clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

double ScanController::distance(const Point & first, const Point & second)
{
  return std::hypot(
    std::hypot(second.x - first.x, second.y - first.y),
    second.z - first.z);
}

geometry_msgs::msg::Point ScanController::toMessage(const Point & point)
{
  geometry_msgs::msg::Point message;
  message.x = point.x;
  message.y = point.y;
  message.z = point.z;
  return message;
}

void ScanController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  navflex_rog_map::RogMap::ConstPtr map)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("ScanController cannot lock lifecycle node");
  }
  if (!map) {
    throw std::invalid_argument("ScanController requires a ROG map");
  }
  name_ = name;
  tf_ = std::move(tf);
  map_ = std::move(map);
  global_frame_ = map_->frameId();

  auto declare = [this](const std::string & key, const auto & value) {
      nav2_util::declare_parameter_if_not_declared(
        node_, name_ + "." + key, rclcpp::ParameterValue(value));
    };
  declare("footprint_type", std::string("sphere"));
  declare("footprint_offset", std::vector<double>{0.0, 0.0, 0.0});
  declare("footprint_radius", 0.35);
  declare("footprint_height", 0.7);
  declare("footprint_size", std::vector<double>{0.7, 0.5, 0.4});
  declare("front_sphere_offset", std::vector<double>{0.25, 0.0, 0.0});
  declare("front_sphere_radius", 0.3);
  declare("rear_sphere_offset", std::vector<double>{-0.25, 0.0, 0.0});
  declare("rear_sphere_radius", 0.3);
  declare("footprint_safety_margin", 0.0);
  declare("planning_horizon", 5.0);
  declare("sample_distance", 0.1);
  declare("lookahead_time", 0.8);
  declare("control_lookahead_time", 0.15);
  declare("max_velocity", 0.8);
  declare("max_vertical_velocity", 0.4);
  declare("max_acceleration", 1.5);
  declare("max_yaw_rate", 1.0);
  declare("kp_position", 1.5);
  declare("kp_yaw", 2.0);

  footprint_.type = navflex_rog_map::footprintTypeFromString(
    node_->get_parameter(name_ + ".footprint_type").as_string());
  const auto offset = node_->get_parameter(name_ + ".footprint_offset").as_double_array();
  const auto size = node_->get_parameter(name_ + ".footprint_size").as_double_array();
  const auto front_offset =
    node_->get_parameter(name_ + ".front_sphere_offset").as_double_array();
  const auto rear_offset =
    node_->get_parameter(name_ + ".rear_sphere_offset").as_double_array();
  if (offset.size() != 3 || size.size() != 3 || front_offset.size() != 3 ||
    rear_offset.size() != 3)
  {
    throw std::invalid_argument("3D footprint offset and size parameters require three values");
  }
  footprint_.offset.x = offset[0];
  footprint_.offset.y = offset[1];
  footprint_.offset.z = offset[2];
  footprint_.radius = node_->get_parameter(name_ + ".footprint_radius").as_double();
  footprint_.height = node_->get_parameter(name_ + ".footprint_height").as_double();
  footprint_.size.x = size[0];
  footprint_.size.y = size[1];
  footprint_.size.z = size[2];
  footprint_.front_sphere.offset.x = front_offset[0];
  footprint_.front_sphere.offset.y = front_offset[1];
  footprint_.front_sphere.offset.z = front_offset[2];
  footprint_.front_sphere.radius =
    node_->get_parameter(name_ + ".front_sphere_radius").as_double();
  footprint_.rear_sphere.offset.x = rear_offset[0];
  footprint_.rear_sphere.offset.y = rear_offset[1];
  footprint_.rear_sphere.offset.z = rear_offset[2];
  footprint_.rear_sphere.radius =
    node_->get_parameter(name_ + ".rear_sphere_radius").as_double();
  footprint_.safety_margin =
    node_->get_parameter(name_ + ".footprint_safety_margin").as_double();
  navflex_rog_map::validateFootprint(footprint_);
  node_->get_parameter(name_ + ".planning_horizon", planning_horizon_);
  node_->get_parameter(name_ + ".sample_distance", sample_distance_);
  node_->get_parameter(name_ + ".lookahead_time", lookahead_time_);
  node_->get_parameter(name_ + ".control_lookahead_time", control_lookahead_time_);
  node_->get_parameter(name_ + ".max_velocity", max_velocity_);
  node_->get_parameter(name_ + ".max_vertical_velocity", max_vertical_velocity_);
  node_->get_parameter(name_ + ".max_acceleration", max_acceleration_);
  node_->get_parameter(name_ + ".max_yaw_rate", max_yaw_rate_);
  node_->get_parameter(name_ + ".kp_position", position_gain_);
  node_->get_parameter(name_ + ".kp_yaw", yaw_gain_);
  last_time_ = node_->now();
}

void ScanController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  local_trajectory_.clear();
  input_trajectory_ = navflex_rogmap_core::Trajectory3D();
  map_.reset();
  tf_.reset();
  node_.reset();
}

void ScanController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = true;
  canceled_ = false;
  last_time_ = node_->now();
}

void ScanController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
}

void ScanController::setTrajectory(const navflex_rogmap_core::Trajectory3D & trajectory)
{
  std::lock_guard<std::mutex> lock(mutex_);
  input_trajectory_ = trajectory;
  local_trajectory_.clear();
  trajectory_time_ = 0.0;
  canceled_ = false;
}

bool ScanController::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  canceled_ = true;
  return true;
}

bool ScanController::occupied(const geometry_msgs::msg::Pose & pose) const
{
  return !map_->isCollisionFree(pose, footprint_);
}

bool ScanController::segmentFree(const Point & start, const Point & end) const
{
  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose end_pose;
  start_pose.position = toMessage(start);
  end_pose.position = toMessage(end);
  const double yaw = std::atan2(end.y - start.y, end.x - start.x);
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  start_pose.orientation.x = orientation.x();
  start_pose.orientation.y = orientation.y();
  start_pose.orientation.z = orientation.z();
  start_pose.orientation.w = orientation.w();
  end_pose.orientation = start_pose.orientation;
  return map_->raycastFree(start_pose, end_pose, footprint_);
}

std::vector<ScanController::Point> ScanController::buildLocalTrajectory(
  const Point & current) const
{
  std::vector<Point> result{current};
  double accumulated_distance = 0.0;
  Point previous = current;
  for (const auto & pose : input_trajectory_.path.poses) {
    const Point candidate{
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
    const double step = distance(previous, candidate);
    if (step < sample_distance_) {
      continue;
    }
    if (!segmentFree(previous, candidate)) {
      break;
    }
    result.push_back(candidate);
    accumulated_distance += step;
    previous = candidate;
    if (accumulated_distance >= planning_horizon_) {
      break;
    }
  }
  return result.size() > 1 ? result : std::vector<Point>{};
}

ScanController::Point ScanController::evaluate(double time) const
{
  if (local_trajectory_.empty()) {
    return {};
  }
  double remaining = time * max_velocity_ * speed_scale_;
  size_t index = 0;
  while (index + 1 < local_trajectory_.size()) {
    const double segment_length = distance(
      local_trajectory_[index], local_trajectory_[index + 1]);
    if (remaining <= segment_length) {
      const double ratio = segment_length > 1e-6 ? remaining / segment_length : 0.0;
      return {
        local_trajectory_[index].x + ratio *
        (local_trajectory_[index + 1].x - local_trajectory_[index].x),
        local_trajectory_[index].y + ratio *
        (local_trajectory_[index + 1].y - local_trajectory_[index].y),
        local_trajectory_[index].z + ratio *
        (local_trajectory_[index + 1].z - local_trajectory_[index].z)};
    }
    remaining -= segment_length;
    ++index;
  }
  return local_trajectory_.back();
}

ScanController::Point ScanController::evaluateVelocity(double time) const
{
  constexpr double kDerivativeStep = 0.05;
  const Point first = evaluate(time);
  const Point second = evaluate(time + kDerivativeStep);
  return {(second.x - first.x) / kDerivativeStep,
    (second.y - first.y) / kDerivativeStep,
    (second.z - first.z) / kDerivativeStep};
}

uint32_t ScanController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  geometry_msgs::msg::TwistStamped & command,
  nav2_core::GoalChecker *, std::string & message)
{
  using Result = nav2_msgs::action::FollowPath::Result;
  command = geometry_msgs::msg::TwistStamped();
  command.header = pose.header;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !map_) {
    message = "ScanController is not active or has no ROG map";
    return Result::NOT_INITIALIZED;
  }
  if (canceled_) {
    message = "ScanController execution canceled";
    return Result::CANCELED;
  }
  if (input_trajectory_.path.poses.empty()) {
    message = "ScanController has no input 3D trajectory";
    return Result::INVALID_PATH;
  }

  const Point current{
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
  if (occupied(pose.pose)) {
    message = "Current robot position collides with the ROG map";
    return Result::BLOCKED_PATH;
  }
  if (local_trajectory_.empty() ||
    !segmentFree(current, evaluate(trajectory_time_ + lookahead_time_)))
  {
    local_trajectory_ = buildLocalTrajectory(current);
    trajectory_time_ = 0.0;
    if (local_trajectory_.empty()) {
      message = "No collision-free local SCAN trajectory in the ROG map";
      return Result::BLOCKED_PATH;
    }
  }

  const rclcpp::Time now = node_->now();
  const double time_step = clamp((now - last_time_).seconds(), 0.001, 0.1);
  last_time_ = now;
  trajectory_time_ += time_step;
  const Point desired = evaluate(trajectory_time_ + control_lookahead_time_);
  const Point feed_forward = evaluateVelocity(trajectory_time_);
  double velocity_x = feed_forward.x + position_gain_ * (desired.x - current.x);
  double velocity_y = feed_forward.y + position_gain_ * (desired.y - current.y);
  double velocity_z = feed_forward.z + position_gain_ * (desired.z - current.z);

  const double horizontal_limit = max_velocity_ * speed_scale_;
  const double horizontal_norm = std::hypot(velocity_x, velocity_y);
  if (horizontal_norm > horizontal_limit) {
    velocity_x *= horizontal_limit / horizontal_norm;
    velocity_y *= horizontal_limit / horizontal_norm;
  }
  velocity_z = clamp(
    velocity_z, -max_vertical_velocity_ * speed_scale_,
    max_vertical_velocity_ * speed_scale_);
  velocity_x = clamp(
    velocity_x, velocity.linear.x - max_acceleration_ * time_step,
    velocity.linear.x + max_acceleration_ * time_step);
  velocity_y = clamp(
    velocity_y, velocity.linear.y - max_acceleration_ * time_step,
    velocity.linear.y + max_acceleration_ * time_step);
  velocity_z = clamp(
    velocity_z, velocity.linear.z - max_acceleration_ * time_step,
    velocity.linear.z + max_acceleration_ * time_step);

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double desired_yaw = std::atan2(velocity_y, velocity_x);
  const double yaw_error = std::atan2(
    std::sin(desired_yaw - yaw), std::cos(desired_yaw - yaw));
  command.twist.linear.x = std::cos(yaw) * velocity_x + std::sin(yaw) * velocity_y;
  command.twist.linear.y = -std::sin(yaw) * velocity_x + std::cos(yaw) * velocity_y;
  command.twist.linear.z = velocity_z;
  command.twist.angular.z = clamp(
    yaw_gain_ * yaw_error, -max_yaw_rate_, max_yaw_rate_);
  message = "Tracking dynamically validated 3D SCAN trajectory using ROG map and ESDF";
  return Result::SUCCESS;
}

void ScanController::setSpeedLimit(double speed_limit, bool percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  speed_scale_ = percentage ? clamp(speed_limit / 100.0, 0.0, 1.0) :
    clamp(speed_limit / max_velocity_, 0.0, 1.0);
}

}  // namespace scan_planner

PLUGINLIB_EXPORT_CLASS(
  scan_planner::ScanController,
  navflex_rogmap_core::Controller)
