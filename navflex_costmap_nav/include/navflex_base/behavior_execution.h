#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/behavior.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "navflex_utility/robot_information.h"
#include "navflex_base/navflex_execution_base.h"

namespace navflex_costmap_nav
{

/**
 * @class BehaviorExecution
 * @brief Execution class wrapping a nav2_core::Behavior plugin.
 *
 * Runs the behavior in a dedicated thread (via NavflexExecutionBase) and
 * tracks its state through the BehaviorState enum.
 */
class BehaviorExecution : public NavflexExecutionBase
{
public:
  using Ptr = std::shared_ptr<BehaviorExecution>;

  enum BehaviorState
  {
    INITIALIZED,     ///< Created but not yet started
    STARTED,         ///< Thread started, before runBehavior() is called
    RECOVERING,      ///< runBehavior() is executing
    RECOVERY_DONE,   ///< runBehavior() returned successfully
    CANCELED,        ///< Cancelled while RECOVERING
    STOPPED,         ///< Thread was interrupted / stopped externally
    INTERNAL_ERROR,  ///< An unexpected exception was thrown
  };

  /**
   * @brief Constructor
   * @param name         Plugin name identifier
   * @param behavior     Shared pointer to the loaded behavior plugin
   * @param robot_info   Robot state provider
   * @param node         Lifecycle node (for logging / parameters)
   */
  BehaviorExecution(
    const std::string & name,
    nav2_core::Behavior::Ptr behavior,
    const navflex_utility::RobotInformation::ConstPtr & robot_info,
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  ~BehaviorExecution() override = default;

  /** @brief Request cancellation (delegates to behavior plugin). */
  bool cancel() override;

  /** @brief Returns true when patience duration has expired. */
  bool isPatienceExceeded() const;

  /** @brief Thread-safe getter for the current BehaviorState. */
  BehaviorState getState() const;

  /** @brief Set the command string to pass to runBehavior(). Must be called before start(). */
  void setCommand(const std::string & cmd) {setMessage(cmd);}

protected:
  /** @brief Main execution loop — calls runBehavior() and updates state. */
  void run() override;

private:
  void setState(BehaviorState state);

  nav2_core::Behavior::Ptr behavior_;

  mutable std::mutex state_mtx_;
  BehaviorState state_{INITIALIZED};

  rclcpp::Duration patience_{0, 0};
  rclcpp::Time start_time_;
  mutable std::mutex time_mtx_;
};

}  // namespace navflex_costmap_nav
