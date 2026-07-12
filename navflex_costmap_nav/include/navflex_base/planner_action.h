#pragma once

// Standard library includes
#include <memory>

// ROS2 includes
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>

// Local includes
#include "navflex_base/navflex_action_base.hpp"
#include "navflex_base/planner_execution.h"
#include "navflex_utility/robot_information.h"

namespace navflex_costmap_nav
{

/**
 * @class PlannerAction
 * @brief Action handler for path planning (ComputePathToPose)
 *
 * This class bridges between ROS2 action requests and the planning execution
 * framework. It receives goal poses from clients and executes path planning
 * using PlannerExecution objects.
 *
 * **Workflow:**
 * 1. Client sends ComputePathToPose goal (start pose, goal pose, tolerance)
 * 2. ROS2 action server receives goal and calls runImpl()
 * 3. runImpl() coordinates with PlannerExecution to compute path
 * 4. Path is transformed to global frame if needed
 * 5. Result sent back to client via goal handle
 *
 * **Responsibilities:**
 * - Implement runImpl() from NavflexActionBase
 * - Handle start/goal pose coordinate frame transformations
 * - Publish intermediate planning information for visualization
 * - Coordinate with RobotInformation for robot state
 *
 * **Threading:**
 * - Constructor and initialization: Main ROS thread
 * - runImpl() execution: Dedicated execution thread (from NavflexActionBase)
 */
class PlannerAction
  : public NavflexActionBase<nav2_msgs::action::ComputePathToPose,
    PlannerExecution>
{
public:
  // ========== Type Definitions ==========
  /// Shared pointer to PlannerAction instance
  typedef std::shared_ptr<PlannerAction> Ptr;

  /// Action type for path planning
  using ActionToPose = nav2_msgs::action::ComputePathToPose;

  /**
   * @brief Constructor for PlannerAction
   *
   * Initializes the planner action handler with robot information
   * and sets up publishers for goal visualization.
   *
   * @param node Shared pointer to ROS2 node for logging and publishing
   * @param name Name identifier for this action (used in logging)
   * @param robot_info Robot state information (position, velocity, configuration)
   */
  PlannerAction(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name,
    const navflex_utility::RobotInformation::ConstPtr & robot_info);

  /**
   * @brief Execute path planning for a goal (implements NavflexActionBase)
   *
   * **Execution Sequence:**
   * 1. Validates goal parameters
   * 2. Publishes goal pose to visualization topic
   * 3. Starts path planning via execution.start()
   * 4. Waits for execution to complete
   * 5. Transforms computed path to global frame
   * 6. Returns result with path and planning status
   *
   * **Thread Context:** Called in execution thread, not main ROS thread
   *
   * @param goal_handle ROS2 goal handle for sending feedback/result
   * @param execution PlannerExecution instance managing the actual planning
   *
   * @note Computation happens asynchronously in execution thread
   * @note Path transformation uses robot's current pose
   */
  void runImpl(
    const GoalHandlePtr & goal_handle,
    PlannerExecution & execution) override;

protected:
  /**
   * @brief Transform computed path to global frame
   *
   * **Purpose:**
   * The planner may compute paths in robot frame or local frame.
   * This method transforms the path to the global frame for client use.
   *
   * **Operations:**
   * 1. Query TF for frame transformations
   * 2. Transform each pose in the path
   * 3. Update frame_id to global_frame
   *
   * **Error Handling:**
   * Returns false if transformation fails (frame not available, etc.)
   *
   * @param plan Input plan in planner's frame
   * @param global_plan Output plan transformed to global frame
   * @return true: transformation successful; false: transformation failed
   *
   * @note Requires valid TF connections and buffer
   */
  bool transformPlanToGlobalFrame(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    std::vector<geometry_msgs::msg::PoseStamped> & global_plan);

private:
  // ========== Publishers and Communication ==========
  /// Publisher for goal pose visualization in rviz
  /// Useful for debugging and monitoring planning requests
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    current_goal_pub_;

  /// Publisher for the computed plan path
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
};

}  // namespace navflex_costmap_nav
