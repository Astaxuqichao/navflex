#include "navflex_base/behavior_execution.h"
#include "nav2_msgs/action/dummy_behavior.hpp"

namespace navflex_costmap_nav
{

BehaviorExecution::BehaviorExecution(
  const std::string & name,
  nav2_core::Behavior::Ptr behavior,
  const navflex_utility::RobotInformation::ConstPtr & robot_info,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
: NavflexExecutionBase(name, robot_info, node),
  behavior_(behavior)
{
  // Read patience parameter if available
  double patience_sec = 15.0;
  if (!node->has_parameter("recovery_patience")) {
    node->declare_parameter("recovery_patience", patience_sec);
  }
  node->get_parameter("recovery_patience", patience_sec);
  patience_ = rclcpp::Duration::from_seconds(patience_sec);
}

bool BehaviorExecution::cancel()
{
  cancel_ = true;
  if (behavior_) {
    behavior_->stop();
  }
  return true;
}

bool BehaviorExecution::isPatienceExceeded() const
{
  std::lock_guard<std::mutex> tl(time_mtx_);
  return !(patience_ == rclcpp::Duration(0, 0)) &&
         (node_->now() - start_time_ > patience_);
}

BehaviorExecution::BehaviorState BehaviorExecution::getState() const
{
  std::lock_guard<std::mutex> lock(state_mtx_);
  return state_;
}

void BehaviorExecution::setState(BehaviorState s)
{
  std::lock_guard<std::mutex> lock(state_mtx_);
  state_ = s;
  condition_.notify_all();
}

void BehaviorExecution::run()
{
  cancel_ = false;

  {
    std::lock_guard<std::mutex> tl(time_mtx_);
    start_time_ = node_->now();
  }

  setState(STARTED);

  if (should_exit_) {
    setOutcomeAndMessage(
      nav2_msgs::action::DummyBehavior::Result::STOPPED,
      "Behavior execution stopped before start");
    setState(STOPPED);
    return;
  }

  setState(RECOVERING);

  try {
    if (!behavior_) {
      setOutcomeAndMessage(
        nav2_msgs::action::DummyBehavior::Result::INTERNAL_ERROR,
        "Behavior plugin is null");
      setState(INTERNAL_ERROR);
      return;
    }

    // The message field holds the command string (set before start() is called);
    // runBehavior() reads it as input and overwrites it with a status message.
    std::string cmd = getMessage();
    setOutcome(behavior_->runBehavior(cmd));
    setMessage(cmd);
  } catch (const std::exception & e) {
    setOutcomeAndMessage(
      nav2_msgs::action::DummyBehavior::Result::INTERNAL_ERROR,
      std::string("Exception in runBehavior: ") + e.what());
    setState(INTERNAL_ERROR);
    return;
  } catch (...) {
    setOutcomeAndMessage(
      nav2_msgs::action::DummyBehavior::Result::INTERNAL_ERROR,
      "Unknown exception in runBehavior");
    setState(INTERNAL_ERROR);
    return;
  }

  using Result = nav2_msgs::action::DummyBehavior::Result;
  if (cancel_ || should_exit_) {
    setOutcomeAndMessage(Result::CANCELED, "Behavior was canceled");
    setState(CANCELED);
  } else if (getOutcome() == Result::SUCCESS) {
    setState(RECOVERY_DONE);
  } else {
    setState(INTERNAL_ERROR);
  }
}

}  // namespace navflex_costmap_nav
