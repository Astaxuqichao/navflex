#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/behavior.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace navflex_cmdbehavior
{

/**
 * @class CmdBehavior
 * @brief Recovery behavior driven by a runtime command string.
 *
 * Primary configure: costmap overload — stores global_costmap and uses
 *   global_costmap->getRobotPose() to track motion completion.
 *
 * Command format passed to runBehavior(message):
 *   "linear <distance_m>"  — drive straight; + = forward, - = backward
 *   "rotate <angle_rad>"   — rotate in place; + = CCW (left), - = CW (right)
 *   "wait <seconds>"       — wait in place
 *
 * Parameters (yaml / ros2 param):
 *   linear_vel     (double, 0.2)   — linear speed       [m/s]
 *   angular_vel    (double, 0.5)   — angular speed      [rad/s]
 *   xy_tolerance   (double, 0.05)  — distance tolerance [m]
 *   yaw_tolerance  (double, 0.017) — angle tolerance    [rad]
 *   timeout        (double, 30.0)  — safety timeout     [s]
 */
class CmdBehavior : public nav2_core::Behavior
{
public:
  CmdBehavior() = default;
  ~CmdBehavior() override = default;

  // TF + collision checker overload — satisfies pure-virtual, not used
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker>
    collision_checker) override;

  // Costmap-based configure — PRIMARY entry point
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  uint32_t runBehavior(std::string & message) override;
  void stop() override;

private:
  // Shared parameter init
  void init(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name);

  // Get robot pose from global costmap
  bool getCurrentPose(geometry_msgs::msg::PoseStamped & pose);
  bool loadParameters(std::string & message);
  bool validateParameters(std::string & message) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;

  // Global costmap — provides getRobotPose() for motion feedback
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_;

  double linear_vel_{0.2};
  double angular_vel_{0.5};
  double xy_tolerance_{0.05};      // m
  double yaw_tolerance_{0.017};    // rad (~1 deg)
  double timeout_{30.0};           // s
  double control_frequency_{10.0}; // Hz
  std::string cmd_vel_topic_{"cmd_vel_nav"};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::atomic<bool> stop_{false};
};

}  // namespace navflex_cmdbehavior
