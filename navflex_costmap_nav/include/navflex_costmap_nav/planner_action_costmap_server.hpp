#pragma once

// Standard library includes
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// ROS2 and Nav2 includes
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

// Local includes
#include "navflex_utility/robot_information.h"
#include "navflex_base/planner_action.h"
#include "navflex_base/planner_execution.h"

namespace navflex_costmap_nav
{

/**
 * @class PlannerCostmapServer
 * @brief Action server for path planning using global planner plugins
 *
 * This server implements the ComputePathToPose action interface and manages
 * various global planning algorithm plugins. It provides coordinated planning
 * with access to global costmap and robot information.
 *
 * **Key Responsibilities:**
 * - Load and manage global planner plugins
 * - Execute path planning through the action interface
 * - Maintain interaction with global costmap
 * - Coordinate with robot state information
 *
 * **Lifecycle Management:**
 * The server follows standard ROS2 lifecycle (configure → activate → deactivate
 * → cleanup)
 */
class PlannerCostmapServer : public nav2_util::LifecycleNode
{
public:
  // ========== Type Definitions ==========
  /// Map of planner plugins indexed by name
  using PlannerMap =
    std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;
  /// Action type for path planning requests
  using ActionToPose = nav2_msgs::action::ComputePathToPose;
  /// Goal handle for path planning action
  using ServerGoalHandleGetPath = rclcpp_action::ServerGoalHandle<ActionToPose>;
  /// Shared pointer to goal handle
  using ServerGoalHandleGetPathPtr = std::shared_ptr<ServerGoalHandleGetPath>;

  /**
   * @brief Constructor for PlannerCostmapServer
   *
   * Initializes the planner server with costmap and robot information.
   * Actual planner plugin loading is deferred to on_configure().
   *
   * @param costmap_ros Shared pointer to the global costmap
   * @param robot_info Robot state information (position, velocity, etc.)
   * @param options Additional ROS2 node options
   */
  explicit PlannerCostmapServer(
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const navflex_utility::RobotInformation::ConstPtr & robot_info,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for PlannerCostmapServer
   */
  ~PlannerCostmapServer();

protected:
  /**
   * @brief Configure lifecycle callback - initializes planner plugins
   *
   * **Operations:**
   * - Load planner plugin configurations from parameters
   * - Instantiate planner plugins via pluginlib
   * - Setup action server
   * - Initialize tf buffer and subscribers
   *
   * @param state Reference to lifecycle state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate lifecycle callback - enables planning operations
   *
   * **Operations:**
   * - Activate planner plugins
   * - Enable resource polling subscribers
   * - Start accepting planning requests
   *
   * @param state Reference to lifecycle state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate lifecycle callback - stops planning operations
   *
   * **Operations:**
   * - Deactivate planner plugins
   * - Stop processing new requests
   * - Disable resource subscribers
   *
   * @param state Reference to lifecycle state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup lifecycle callback - releases resources
   *
   * **Operations:**
   * - Unload planner plugins
   * - Cleanup TF buffer
   * - Release costmap resources
   *
   * @param state Reference to lifecycle state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown lifecycle callback - emergency cleanup
   *
   * Called when node is shutting down.
   *
   * @param state Reference to lifecycle state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

  // ========== Action Server Handlers ==========

  /**
   * @brief Goal reception handler for path planning action
   *
   * Validates incoming goal and returns acceptance status.
   *
   * @param uuid Unique identifier for this goal
   * @param goal Goal containing start, goal, and planning parameters
   * @return Goal acceptance status
   */
  rclcpp_action::GoalResponse handleGoalGetPath(
    const rclcpp_action::GoalUUID & uuid,
    ActionToPose::Goal::ConstSharedPtr goal);

  /**
   * @brief Execute path planning for accepted goal
   *
   * Called when goal is accepted by action server.
   * Runs planning algorithm and communicates progress/results.
   *
   * @param goal_handle The accepted goal handle
   */
  void callActionGetPath(ServerGoalHandleGetPathPtr goal_handle);

  /**
   * @brief Cancel handler for planning action
   *
   * Handles client cancellation requests for planning.
   *
   * @param goal_handle The goal being canceled
   * @return Cancel acceptance status
   */
  rclcpp_action::CancelResponse cancelActionGetPath(
    ServerGoalHandleGetPathPtr goal_handle);

  navflex_costmap_nav::PlannerExecution::Ptr newPlannerExecution(
    const std::string & plugin_name,
    const nav2_core::GlobalPlanner::Ptr & plugin_ptr);

private:
  // ========== Plugin Management ==========
  /// Map of loaded planner plugins
  PlannerMap planners_;

  /// Plugin loader for instantiating global planners
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;

  /// Default planner IDs from configuration
  std::vector<std::string> default_ids_;
  /// Default planner types from configuration
  std::vector<std::string> default_types_;

  /// Currently active planner IDs
  std::vector<std::string> planner_ids_;
  /// Currently active planner types (must match planner_ids_)
  std::vector<std::string> planner_types_;

  /// Concatenated planner IDs for logging/debugging
  std::string planner_ids_concat_;

  // ========== TF and Resources ==========
  /// TF2 buffer for coordinate frame transformations
  std::shared_ptr<tf2_ros::Buffer> tf_;

  /// Global costmap for planning
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  /// Raw pointer to costmap data (from costmap_ros_)
  nav2_costmap_2d::Costmap2D * costmap_;

  // ========== Action Server ==========
  /// ROS2 action server for path planning requests
  rclcpp_action::Server<ActionToPose>::SharedPtr action_server_get_path_ptr_;

  /// Dedicated callback group to prevent execute from blocking goal/cancel responses
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr action_executor_;
  std::unique_ptr<std::thread> action_executor_thread_;

  /// Planner action wrapper managing concurrent executions
  PlannerAction::Ptr planner_action_;

  // ========== Robot State ==========
  /// Robot configuration and state information
  navflex_utility::RobotInformation::ConstPtr robot_info_;

  std::string name_action_get_path_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    dyn_params_handler_;
  std::mutex dynamic_params_lock_;
};

}  // namespace navflex_costmap_nav
