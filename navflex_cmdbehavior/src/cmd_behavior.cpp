#include "navflex_cmdbehavior/cmd_behavior.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/action/dummy_behavior.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navflex_cmdbehavior
{

// ---------------------------------------------------------------------------
// Utility: quaternion → yaw
// ---------------------------------------------------------------------------
static double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

static bool isFinite(double value)
{
  return std::isfinite(value);
}

// ---------------------------------------------------------------------------
// Shared parameter init — called by the costmap configure (primary path)
// ---------------------------------------------------------------------------
void CmdBehavior::init(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name)
{
  node_ = parent;
  name_ = name;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CmdBehavior: node is no longer valid");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".linear_vel", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".angular_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".xy_tolerance", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".yaw_tolerance", rclcpp::ParameterValue(0.017));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".timeout", rclcpp::ParameterValue(30.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".control_frequency", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cmd_vel_topic", rclcpp::ParameterValue(std::string("cmd_vel_nav")));

  std::string validation_message;
  if (!loadParameters(validation_message)) {
    throw std::runtime_error(validation_message);
  }

  cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    node->get_logger(),
    "CmdBehavior '%s' configured: linear_vel=%.2f angular_vel=%.2f "
    "xy_tol=%.3f yaw_tol=%.3f timeout=%.1f freq=%.1f cmd_vel_topic=%s",
    name_.c_str(), linear_vel_, angular_vel_,
    xy_tolerance_, yaw_tolerance_, timeout_, control_frequency_,
    cmd_vel_topic_.c_str());
}

bool CmdBehavior::loadParameters(std::string & message)
{
  auto node = node_.lock();
  if (!node) {
    message = "CmdBehavior: node is no longer valid";
    return false;
  }

  node->get_parameter(name_ + ".linear_vel", linear_vel_);
  node->get_parameter(name_ + ".angular_vel", angular_vel_);
  node->get_parameter(name_ + ".xy_tolerance", xy_tolerance_);
  node->get_parameter(name_ + ".yaw_tolerance", yaw_tolerance_);
  node->get_parameter(name_ + ".timeout", timeout_);
  node->get_parameter(name_ + ".control_frequency", control_frequency_);
  node->get_parameter(name_ + ".cmd_vel_topic", cmd_vel_topic_);

  return validateParameters(message);
}

bool CmdBehavior::validateParameters(std::string & message) const
{
  if (!isFinite(linear_vel_) || linear_vel_ <= 0.0) {
    message = "CmdBehavior: linear_vel must be finite and > 0";
    return false;
  }
  if (!isFinite(angular_vel_) || angular_vel_ <= 0.0) {
    message = "CmdBehavior: angular_vel must be finite and > 0";
    return false;
  }
  if (!isFinite(xy_tolerance_) || xy_tolerance_ < 0.0) {
    message = "CmdBehavior: xy_tolerance must be finite and >= 0";
    return false;
  }
  if (!isFinite(yaw_tolerance_) || yaw_tolerance_ < 0.0) {
    message = "CmdBehavior: yaw_tolerance must be finite and >= 0";
    return false;
  }
  if (!isFinite(timeout_) || timeout_ <= 0.0) {
    message = "CmdBehavior: timeout must be finite and > 0";
    return false;
  }
  if (!isFinite(control_frequency_) || control_frequency_ <= 0.0) {
    message = "CmdBehavior: control_frequency must be finite and > 0";
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// configure overload 1: TF + collision checker
//   Satisfies pure-virtual. Not the intended path — no costmap available.
// ---------------------------------------------------------------------------
void CmdBehavior::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker>/*cc*/)
{
  global_costmap_ = nullptr;  // no costmap — pose feedback unavailable
  init(parent, name);
}

// ---------------------------------------------------------------------------
// configure overload 2: costmap-based  — PRIMARY entry point
//   global_costmap->getRobotPose() is used to track motion completion.
// ---------------------------------------------------------------------------
void CmdBehavior::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*local_costmap*/)
{
  global_costmap_ = global_costmap;  // stored for getRobotPose() in runBehavior
  init(parent, name);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void CmdBehavior::cleanup()
{
  cmd_vel_pub_.reset();
  global_costmap_.reset();
}

void CmdBehavior::activate() {}
void CmdBehavior::deactivate() {}

void CmdBehavior::stop() {stop_ = true;}

// ---------------------------------------------------------------------------
// getCurrentPose — delegates to global_costmap->getRobotPose()
// ---------------------------------------------------------------------------
bool CmdBehavior::getCurrentPose(geometry_msgs::msg::PoseStamped & pose)
{
  if (!global_costmap_) {return false;}
  return global_costmap_->getRobotPose(pose);
}

// ---------------------------------------------------------------------------
// runBehavior
// ---------------------------------------------------------------------------
uint32_t CmdBehavior::runBehavior(std::string & message)
{
  using Result = nav2_msgs::action::DummyBehavior::Result;
  stop_ = false;

  auto node = node_.lock();
  if (!node) {
    message = "CmdBehavior: node is no longer valid";
    return Result::INTERNAL_ERROR;
  }

  // Re-read parameters so they can be changed between calls via ros2 param set
  if (!loadParameters(message)) {
    RCLCPP_ERROR(node->get_logger(), "%s", message.c_str());
    return Result::FAILURE;
  }

  // ---- Parse command: "linear <distance_m>", "rotate <angle_rad>", or "wait <seconds>"
  const auto space = message.find(' ');
  if (space == std::string::npos) {
    message = "CmdBehavior: invalid format, expected 'linear <m>' or 'rotate <deg>'";
    RCLCPP_ERROR(node->get_logger(), "%s", message.c_str());
    return Result::FAILURE;
  }

  const std::string cmd_type = message.substr(0, space);
  double value = 0.0;
  try {
    value = std::stod(message.substr(space + 1));
  } catch (const std::exception &) {
    message = "CmdBehavior: failed to parse value from '" + message + "'";
    RCLCPP_ERROR(node->get_logger(), "%s", message.c_str());
    return Result::FAILURE;
  }
  if (!std::isfinite(value)) {
    message = "CmdBehavior: command value must be finite";
    RCLCPP_ERROR(node->get_logger(), "%s", message.c_str());
    return Result::FAILURE;
  }

  // ---- Build velocity command ----------------------------------------------
  const auto period = std::chrono::milliseconds(
    std::max(1, static_cast<int>(1000.0 / control_frequency_)));
  geometry_msgs::msg::Twist cmd;
  double target_dist = 0.0;
  double target_angle_rad = 0.0;

  if (cmd_type == "linear") {
    cmd.linear.x = (value >= 0.0) ? linear_vel_ : -linear_vel_;
    target_dist = std::abs(value);
  } else if (cmd_type == "rotate") {
    target_angle_rad = value;
    cmd.angular.z = (value >= 0.0) ? angular_vel_ : -angular_vel_;
  } else if (cmd_type == "wait") {
    if (value <= 0.0) {
      message = "CmdBehavior: wait duration must be > 0";
      RCLCPP_ERROR(node->get_logger(), "%s", message.c_str());
      return Result::FAILURE;
    }
    RCLCPP_INFO(
      node->get_logger(), "CmdBehavior '%s': waiting %.3f s",
      name_.c_str(), value);
    const auto wait_end = node->now() + rclcpp::Duration::from_seconds(value);
    const auto deadline_wait = node->now() + rclcpp::Duration::from_seconds(timeout_);
    while (rclcpp::ok() && !stop_) {
      if (node->now() > deadline_wait) {
        message = "CmdBehavior: safety timeout during wait";
        RCLCPP_WARN(node->get_logger(), "%s", message.c_str());
        return Result::PAT_EXCEEDED;
      }
      if (node->now() >= wait_end) {break;}
      std::this_thread::sleep_for(period);
    }
    if (stop_) {
      message = "CmdBehavior canceled";
      RCLCPP_INFO(node->get_logger(), "%s", message.c_str());
      return Result::CANCELED;
    }
    message = "CmdBehavior wait completed";
    RCLCPP_INFO(node->get_logger(), "%s", message.c_str());
    return Result::SUCCESS;
  } else {
    message = "CmdBehavior: unknown type '" + cmd_type +
      "', expected 'linear', 'rotate' or 'wait'";
    RCLCPP_ERROR(node->get_logger(), "%s", message.c_str());
    return Result::FAILURE;
  }

  // ---- Get initial pose via global_costmap->getRobotPose() -----------------
  geometry_msgs::msg::PoseStamped start_pose;
  const bool has_pose = getCurrentPose(start_pose);
  if (!has_pose) {
    RCLCPP_WARN(
      node->get_logger(),
      "CmdBehavior '%s': global_costmap->getRobotPose() unavailable, "
      "falling back to time-based termination", name_.c_str());
  }

  const double start_x = has_pose ? start_pose.pose.position.x : 0.0;
  const double start_y = has_pose ? start_pose.pose.position.y : 0.0;
  double prev_yaw = has_pose ? quatToYaw(start_pose.pose.orientation) : 0.0;
  double accumulated_ang = 0.0;

  const double fallback_duration =
    (cmd_type == "linear") ?
    target_dist / linear_vel_ :
    std::abs(target_angle_rad) / angular_vel_;

  RCLCPP_INFO(
    node->get_logger(),
    "CmdBehavior '%s': %s %.3f  speed=%.3f  pose_feedback=%s",
    name_.c_str(), cmd_type.c_str(), value,
    (cmd_type == "linear") ? linear_vel_ : angular_vel_,
    has_pose ? "yes" : "no (time fallback)");

  // ---- Control loop --------------------------------------------------------
  const auto deadline = node->now() + rclcpp::Duration::from_seconds(timeout_);
  const auto fallback_end =
    has_pose ? rclcpp::Time(0, 0, RCL_ROS_TIME) :
    node->now() + rclcpp::Duration::from_seconds(fallback_duration);

  while (rclcpp::ok() && !stop_) {
    // Safety timeout
    if (node->now() > deadline) {
      geometry_msgs::msg::Twist zero;
      cmd_vel_pub_->publish(zero);
      message = "CmdBehavior: safety timeout before target reached";
      RCLCPP_WARN(node->get_logger(), "%s", message.c_str());
      return Result::PAT_EXCEEDED;
    }

    // Fallback: time-based termination
    if (!has_pose && node->now() > fallback_end) {break;}

    // Pose-based termination via global_costmap->getRobotPose()
    if (has_pose) {
      geometry_msgs::msg::PoseStamped cur;
      if (getCurrentPose(cur)) {
        if (cmd_type == "linear") {
          const double dx = cur.pose.position.x - start_x;
          const double dy = cur.pose.position.y - start_y;
          const double dist = std::sqrt(dx * dx + dy * dy);
          RCLCPP_DEBUG(
            node->get_logger(),
            "CmdBehavior linear: traveled=%.3f / %.3f m", dist, target_dist);
          if (dist >= std::max(0.0, target_dist - xy_tolerance_)) {break;}
        } else {
          double curr_yaw = quatToYaw(cur.pose.orientation);
          double delta = curr_yaw - prev_yaw;
          while (delta > M_PI) {delta -= 2.0 * M_PI;}
          while (delta < -M_PI) {delta += 2.0 * M_PI;}
          accumulated_ang += delta;
          prev_yaw = curr_yaw;
          RCLCPP_DEBUG(
            node->get_logger(),
            "CmdBehavior rotate: accumulated=%.3f / %.3f rad",
            accumulated_ang, target_angle_rad);
          if (std::abs(accumulated_ang) >=
            std::max(0.0, std::abs(target_angle_rad) - yaw_tolerance_)) {break;}
        }
      }
    }

    cmd_vel_pub_->publish(cmd);
    std::this_thread::sleep_for(period);
  }

  geometry_msgs::msg::Twist zero;
  cmd_vel_pub_->publish(zero);

  if (stop_) {
    message = "CmdBehavior canceled";
    RCLCPP_INFO(node->get_logger(), "%s", message.c_str());
    return Result::CANCELED;
  }

  message = "CmdBehavior completed";
  RCLCPP_INFO(node->get_logger(), "%s", message.c_str());
  return Result::SUCCESS;
}

}  // namespace navflex_cmdbehavior

PLUGINLIB_EXPORT_CLASS(navflex_cmdbehavior::CmdBehavior, nav2_core::Behavior)
