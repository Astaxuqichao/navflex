#pragma once
// New ControllerCostmapServer using NavflexActionBase framework.
// Original (SimpleActionServer-based) version preserved in controller_server_bak.hpp

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/server.hpp"
#include "navflex_utility/robot_information.h"
#include "navflex_base/controller_action.h"
#include "navflex_base/controller_execution.h"

namespace navflex_nav
{

/**
 * @class ControllerCostmapServer
 * @brief Lifecycle node that manages controller plugins and exposes a
 *        FollowPath action server using the NavflexActionBase framework.
 *
 * Follows the same architectural pattern as PlannerServer:
 *  - Plugins are loaded during on_configure()
 *  - A ControllerAction instance handles concurrency / execution management
 *  - Each incoming goal creates a ControllerExecution and passes it to
 *    ControllerAction::start()
 */
class ControllerCostmapServer : public nav2_util::LifecycleNode
{
public:
  using ControllerMap =
    std::unordered_map<std::string, nav2_core::Controller::Ptr>;

  using ActionFollowPath = nav2_msgs::action::FollowPath;
  using ServerGoalHandleFollowPath =
    rclcpp_action::ServerGoalHandle<ActionFollowPath>;
  using ServerGoalHandleFollowPathPtr =
    std::shared_ptr<ServerGoalHandleFollowPath>;

  /**
   * @brief Constructor
   * @param costmap_ros  Local costmap (used by controller plugins)
   * @param robot_info   Robot state provider
   * @param options      ROS2 node options
   */
  explicit ControllerCostmapServer(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const navflex_utility::RobotInformation::ConstPtr & robot_info,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ControllerCostmapServer();

protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

private:
  // -------- action server callbacks --------
  rclcpp_action::GoalResponse handleGoalFollowPath(
    const rclcpp_action::GoalUUID & uuid,
    ActionFollowPath::Goal::ConstSharedPtr goal);

  rclcpp_action::CancelResponse cancelActionFollowPath(
    ServerGoalHandleFollowPathPtr goal_handle);

  void callActionFollowPath(ServerGoalHandleFollowPathPtr goal_handle);

  // -------- helpers --------
  bool findControllerId(
    const std::string & requested,
    std::string & resolved) const;

  ControllerExecution::Ptr newControllerExecution(
    const std::string & controller_id,
    const nav2_core::GoalChecker::Ptr & goal_checker,
    double xy_goal_tolerance,
    double yaw_goal_tolerance);

  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);
  void publishZeroVelocity();

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // -------- plugins --------
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_;
  ControllerMap controllers_;

  // -------- ROS2 resources --------
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  navflex_utility::RobotInformation::ConstPtr robot_info_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
    vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    current_goal_publisher_;

  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

  rclcpp_action::Server<ActionFollowPath>::SharedPtr action_server_follow_path_;

  // -------- callback group (prevents execute from blocking goal/cancel responses) --------
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr action_executor_;
  std::unique_ptr<nav2_util::NodeThread> action_executor_thread_;

  // -------- action handler --------
  std::shared_ptr<ControllerAction> controller_action_;
  std::string name_action_follow_path_;

  // -------- dynamic params --------
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    dyn_params_handler_;
  mutable std::mutex dynamic_params_lock_;
};

}  // namespace navflex_nav
