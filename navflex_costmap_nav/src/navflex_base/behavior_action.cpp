#include "navflex_base/behavior_action.h"

namespace navflex_costmap_nav
{

using Action = nav2_msgs::action::DummyBehavior;

BehaviorAction::BehaviorAction(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const navflex_utility::RobotInformation::ConstPtr & robot_info)
: NavflexActionBase<Action, BehaviorExecution>(node, name, robot_info) {}

void BehaviorAction::runImpl(
  const GoalHandlePtr & goal_handle,
  BehaviorExecution & execution)
{
  RCLCPP_DEBUG(rclcpp::get_logger(name_), "Start action %s", name_.c_str());

  const auto & goal = *goal_handle->get_goal();
  auto result = std::make_shared<Action::Result>();

  const rclcpp::Time start_time = node_->now();

  // Helper: fill elapsed time and used_plugin, then return result
  auto fill_common = [&]() {
      result->used_plugin = goal.behavior;
      result->total_elapsed_time = node_->now() - start_time;
    };

  bool recovery_active = true;

  while (recovery_active && rclcpp::ok()) {
    // Check if action client requested cancellation
    if (goal_handle->is_canceling()) {
      execution.cancel();
      execution.stop();
      execution.join();
      fill_common();
      result->outcome = Action::Result::CANCELED;
      result->message = "Behavior canceled by client";
      goal_handle->canceled(result);
      RCLCPP_INFO(
        rclcpp::get_logger(name_),
        "Behavior action \"%s\" canceled by client.", goal.behavior.c_str());
      return;
    }

    const auto state = execution.getState();

    switch (state) {
      case BehaviorExecution::INITIALIZED:
        RCLCPP_DEBUG(
          rclcpp::get_logger(name_),
          "Behavior \"%s\" initialized, starting.",
          goal.command.data.c_str());
        execution.start();
        break;

      case BehaviorExecution::STARTED:
        RCLCPP_DEBUG(
          rclcpp::get_logger(name_),
          "Behavior \"%s\" was started.", goal.command.data.c_str());
        break;

      case BehaviorExecution::RECOVERING:
        if (execution.isPatienceExceeded()) {
          RCLCPP_INFO(
            rclcpp::get_logger(name_),
            "Behavior \"%s\" patience exceeded, canceling.",
            goal.command.data.c_str());
          execution.cancel();
        }
        RCLCPP_DEBUG_THROTTLE(
          rclcpp::get_logger(name_),
          *node_->get_clock(), 3000,
          "Recovering with behavior: %s",
          goal.command.data.c_str());
        break;

      case BehaviorExecution::CANCELED:
        recovery_active = false;
        fill_common();
        // Distinguish patience-exceeded cancel from other internal cancel
        if (execution.isPatienceExceeded()) {
          result->outcome = Action::Result::PAT_EXCEEDED;
          result->message = "Behavior patience exceeded";
        } else {
          const std::string execution_message = execution.getMessage();
          result->outcome = Action::Result::CANCELED;
          result->message = execution_message.empty() ?
            "Behavior was canceled" :
            execution_message;
        }
        goal_handle->abort(result);
        RCLCPP_WARN(
          rclcpp::get_logger(name_),
          "Behavior \"%s\" canceled (outcome %u): %s",
          goal.command.data.c_str(), result->outcome,
          result->message.c_str());
        break;

      case BehaviorExecution::RECOVERY_DONE:
        recovery_active = false;
        fill_common();
        result->outcome = execution.getOutcome();
        result->message = execution.getMessage();
        if (result->outcome == Action::Result::SUCCESS) {
          goal_handle->succeed(result);
          RCLCPP_INFO(
            rclcpp::get_logger(name_),
            "Behavior \"%s\" completed successfully (%.3fs).",
            goal.command.data.c_str(),
            rclcpp::Duration(result->total_elapsed_time).seconds());
        } else {
          goal_handle->abort(result);
          RCLCPP_ERROR(
            rclcpp::get_logger(name_),
            "Behavior \"%s\" failed with code %u: %s",
            goal.command.data.c_str(), result->outcome,
            result->message.c_str());
        }
        break;

      case BehaviorExecution::STOPPED:
        recovery_active = false;
        fill_common();
        result->outcome = Action::Result::STOPPED;
        result->message = "Behavior stopped";
        goal_handle->abort(result);
        RCLCPP_WARN(
          rclcpp::get_logger(name_),
          "Behavior \"%s\" stopped rigorously.", goal.command.data.c_str());
        break;

      case BehaviorExecution::INTERNAL_ERROR:
        recovery_active = false;
        fill_common();
        result->outcome = execution.getOutcome();
        result->message = execution.getMessage();
        goal_handle->abort(result);
        RCLCPP_FATAL(
          rclcpp::get_logger(name_),
          "Internal error in behavior \"%s\" (code %u): %s",
          goal.command.data.c_str(), result->outcome,
          result->message.c_str());
        break;

      default:
        recovery_active = false;
        fill_common();
        result->outcome = Action::Result::INTERNAL_ERROR;
        result->message = "Unknown execution state";
        goal_handle->abort(result);
        RCLCPP_FATAL(
          rclcpp::get_logger(name_),
          "Unknown state %d in behavior \"%s\" execution!",
          static_cast<int>(state), goal.command.data.c_str());
        break;
    }

    if (recovery_active) {
      execution.waitForStateUpdate(std::chrono::milliseconds(500));
    }
  }
}

}  // namespace navflex_costmap_nav
