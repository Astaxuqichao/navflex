#pragma once

// Standard library includes
#include <memory>

// ROS2 and Nav2 includes
#include "navflex_rog_map/rog_map_ros.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

// Nav2 service interfaces
#include "nav2_msgs/srv/check_point.hpp"
#include "nav2_msgs/srv/check_pose.hpp"
#include "nav2_msgs/srv/check_path.hpp"

// Local includes
#include "navflex_nav/rogmap_nav/planner_action_rogmap_server.hpp"
#include "navflex_nav/rogmap_nav/controller_action_rogmap_server.hpp"
#include "navflex_nav/rogmap_nav/behavior_action_rogmap_server.hpp"

namespace navflex_nav
{

/**
 * @class RogMapNavNode
 * @brief Main navigation node integrating costmaps and navigation servers
 *
 * This node orchestrates the complete navigation stack by managing:
 * - Global and local costmaps
 * - Planner server (path planning)
 * - Controller server (path tracking)
 * - Behavior server (recovery behaviors)
 *
 * **Lifecycle:**
 * The node manages three independent thread groups, each running a server
 * with its own dedicated execution thread. The lifecycle transitions
 * coordinate startup and shutdown of all components.
 *
 * **Thread Architecture:**
 * ```
 * Main ROS Thread
 *   ├─ Global Costmap Thread (updates global costmap)
 *   ├─ Local Costmap Thread (updates local costmap)
 *   ├─ Planner Server Thread (handles planning requests)
 *   ├─ Controller Server Thread (handles control requests)
 *   └─ Behavior Server Thread (handles recovery requests)
 * ```
 *
 * **Component Interactions:**
 * - Costmaps: Updated in separate threads, accessed by servers
 * - PlannerServer: Uses global costmap for path planning
 * - ControllerServer: Uses local costmap for trajectory tracking
 * - BehaviorServer: May use costmaps for recovery behaviors
 *
 * **State Synchronization:**
 * - Robot state accessed via shared RobotInformation object
 * - All servers follow standard ROS2 lifecycle
 */
class RogMapNavNode : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for RogMapNavNode
   *
   * Initializes the navigation node with default ROS2 options.
   * Actual component creation is deferred to on_configure().
   *
   * @param options Additional ROS2 node options
   */
  explicit RogMapNavNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   *
   * Ensures all threads are properly joined and resources cleaned up.
   */
  ~RogMapNavNode();

  /**
   * @brief Configure lifecycle - setup all navigation components
   *
   * **Operations:**
   * 1. Load robot configuration (frames, costmap resolution)
   * 2. Create and configure global and local costmaps
   * 3. Instantiate navigation servers (planner, controller, behavior)
   * 4. Launch dedicated threads for each costmap and server
   * 5. Initialize shared robot state information
   *
   * @param state Lifecycle state
   * @return SUCCESS or FAILURE
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate lifecycle - enable all navigation operations
   *
   * **Operations:**
   * 1. Activate global and local costmaps (start updating)
   * 2. Activate all servers (accept requests)
   * 3. Begin processing navigation goals
   *
   * @param state Lifecycle state
   * @return SUCCESS or FAILURE
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate lifecycle - pause all navigation operations
   *
   * **Operations:**
   * 1. Deactivate all servers (stop accepting new requests)
   * 2. Cancel ongoing tasks if any
   * 3. Preserve resources for potential reactivation
   *
   * @param state Lifecycle state
   * @return SUCCESS or FAILURE
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup lifecycle - release all resources
   *
   * **Operations:**
   * 1. Join all execution threads (blocking)
   * 2. Unload planner and controller plugins
   * 3. Shutdown costmaps
   * 4. Clean up robot information
   *
   * @param state Lifecycle state
   * @return SUCCESS or FAILURE
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  // Original ROG-Map owns global probability, local inflation, and ESDF in
  // one map instance. ProbMap intentionally supports one initialization.
  std::shared_ptr<navflex_rog_map::RogMapROS> rog_map_;
  std::unique_ptr<nav2_util::NodeThread> rog_map_thread_;

  // ========== Navigation Servers ==========
  /// Planner server instances for path planning
  std::shared_ptr<navflex_nav::PlannerRogMapServer> planner_server_;
  /// Dedicated thread for planner server
  std::unique_ptr<nav2_util::NodeThread> planner_server_thread_;

  /// Controller server instance for trajectory tracking
  std::shared_ptr<navflex_nav::ControllerRogMapServer> controller_server_;
  /// Dedicated thread for controller server
  std::unique_ptr<nav2_util::NodeThread> controller_server_thread_;

  /// Behavior server instance for recovery behaviors
  std::shared_ptr<navflex_nav::RecoveryRogMapServer> behavior_server_;
  /// Dedicated thread for behavior server
  std::unique_ptr<nav2_util::NodeThread> behavior_server_thread_;

  // ========== Costmap Query Services ==========
  rclcpp::Service<nav2_msgs::srv::CheckPoint>::SharedPtr check_point_srv_;
  rclcpp::Service<nav2_msgs::srv::CheckPose>::SharedPtr check_pose_srv_;
  rclcpp::Service<nav2_msgs::srv::CheckPath>::SharedPtr check_path_srv_;

  void checkPointCallback(
    const std::shared_ptr<nav2_msgs::srv::CheckPoint::Request> request,
    std::shared_ptr<nav2_msgs::srv::CheckPoint::Response> response);

  void checkPoseCallback(
    const std::shared_ptr<nav2_msgs::srv::CheckPose::Request> request,
    std::shared_ptr<nav2_msgs::srv::CheckPose::Response> response);

  void checkPathCallback(
    const std::shared_ptr<nav2_msgs::srv::CheckPath::Request> request,
    std::shared_ptr<nav2_msgs::srv::CheckPath::Response> response);

  /// Returns global or local costmap based on the uint8 costmap field in requests.
  std::shared_ptr<navflex_rog_map::RogMapROS> selectCostmap(uint8_t costmap_id);

  static uint8_t occupancyToState(navflex_rog_map::OccupancyState state);

  // ========== Robot Configuration ==========
  /// Shared robot state information (position, velocity, etc.)
  navflex_utility::RobotInformation::Ptr robot_info_;

  TFPtr tf_listener_ptr_;

  /// TF frame of the robot (base_link, base_footprint, etc.)
  std::string robot_frame_;

  /// Global reference frame for navigation (map, odom, etc.)
  std::string global_frame_;
};

}  // namespace navflex_nav
