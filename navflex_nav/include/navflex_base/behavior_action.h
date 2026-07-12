#pragma once

#include <memory>
#include <string>

#include "nav2_msgs/action/dummy_behavior.hpp"
#include "navflex_utility/robot_information.h"
#include "navflex_base/navflex_action_base.hpp"
#include "navflex_base/behavior_execution.h"

namespace navflex_nav
{

/**
 * @class BehaviorAction
 * @brief Action handler for DummyBehavior using the NavflexActionBase framework.
 *
 * Wraps BehaviorExecution and manages the ROS2 action lifecycle for behavior
 * recovery requests (nav2_msgs::action::DummyBehavior).
 *
 * Since DummyBehavior goal has no concurrency_slot field, getSlotId() in
 * NavflexActionBase will always return 0 via the if constexpr path.
 */
class BehaviorAction
  : public NavflexActionBase<nav2_msgs::action::DummyBehavior,
    BehaviorExecution>
{
public:
  using Ptr = std::shared_ptr<BehaviorAction>;

  /**
   * @brief Constructor
   * @param node       Lifecycle node
   * @param name       Action name (e.g. "behavior_action")
   * @param robot_info Robot state provider
   */
  BehaviorAction(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name,
    const navflex_utility::RobotInformation::ConstPtr & robot_info);

  /**
   * @brief Execute behavior recovery (implements NavflexActionBase).
   *
   * State machine mirrors mbf RecoveryAction::runImpl().
   *
   * @param goal_handle ROS2 goal handle
   * @param execution   BehaviorExecution managing the plugin thread
   */
  void runImpl(
    const GoalHandlePtr & goal_handle,
    BehaviorExecution & execution) override;
};

}  // namespace navflex_nav
