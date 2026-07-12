#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "navflex_utility/navigation_utility.h"
#include "navflex_utility/robot_information.h"
#include "navflex_base/navflex_action_base.hpp"
#include "navflex_base/controller_execution.h"

namespace navflex_nav
{

/**
 * @class ControllerAction
 * @brief Action handler for the FollowPath action using the NavflexActionBase
 *        concurrent-slot framework.
 *
 * Inherits from NavflexActionBase<FollowPath, ControllerExecution>.
 * Because nav2_msgs::action::FollowPath does not carry a concurrency_slot
 * field, the base class automatically uses slot 0 via the has_concurrency_slot
 * type trait + if constexpr in getSlotId().
 *
 * The start() override in ControllerAction adds plan-update optimization:
 * if the new goal requests the same controller on the already-active slot,
 * it updates the plan in-place without restarting the execution thread.
 */
class ControllerAction
  : public NavflexActionBase<nav2_msgs::action::FollowPath, ControllerExecution>
{
public:
  typedef std::shared_ptr<ControllerAction> Ptr;
  using ActionFollowPath = nav2_msgs::action::FollowPath;

  ControllerAction(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name,
    const navflex_utility::RobotInformation::ConstPtr & robot_info);

  /**
   * @brief Override start() to support in-place plan updates.
   *
   * If the new goal targets the same controller currently active in slot 0,
   * simply updates the plan without stopping the execution thread.
   * Otherwise falls back to the standard NavflexActionBase::start() logic.
   */
  void start(
    const GoalHandlePtr & goal_handle,
    ControllerExecution::Ptr execution_ptr) override;

  /**
   * @brief Implements the FollowPath state machine loop.
   *
   * Monitors the ControllerExecution state and reports feedback / result
   * back to the action client. Handles all ControllerState transitions.
   */
  void runImpl(
    const GoalHandlePtr & goal_handle,
    ControllerExecution & execution) override;

protected:
  /**
   * @brief Publish FollowPath feedback
   */
  void publishFeedback(
    GoalHandle & goal_handle,
    const geometry_msgs::msg::TwistStamped & cmd_vel,
    uint32_t outcome,
    const std::string & message);

  std::mutex goal_mtx_;  ///< Protects robot_pose_/goal_pose_/pending_goal_handle_
  geometry_msgs::msg::PoseStamped robot_pose_;  ///< Latest robot pose
  geometry_msgs::msg::PoseStamped goal_pose_;   ///< Goal pose (last path pose)
  /// Set by start() on in-place preempt; consumed by runImpl() each loop tick.
  GoalHandlePtr pending_goal_handle_;
  GoalHandlePtr pending_preempted_goal_handle_;
};

}  // namespace navflex_nav
