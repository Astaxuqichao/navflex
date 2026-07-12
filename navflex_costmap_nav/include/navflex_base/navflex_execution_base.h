#pragma once

// Standard library includes
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"

// Local includes
#include "navflex_utility/robot_information.h"

namespace navflex_costmap_nav
{

/**
 * @class NavflexExecutionBase
 * @brief Base class for running concurrent/asynchronous navigation tasks
 *
 * This class provides a framework for executing long-running operations (like
 * path planning, trajectory tracking) in a dedicated thread, with support for
 * progress monitoring and cancellation.
 *
 * **Key Features:**
 * - Dedicated execution thread with controlled lifecycle
 * - State-change notification via condition variable
 * - Cancellation support (graceful stop request)
 * - Outcome and message tracking
 * - Pre/post-execution hooks for setup/cleanup
 *
 * **Thread Model:**
 * ```
 * Main Thread (NavflexActionBase::start)
 *   ├─ Create NavflexExecutionBase instance
 *   └─ Call start() to spawn execution thread
 *        │
 *        └─ New Thread (NavflexActionBase::run)
 *            ├─ preRun()  [setup]
 *            ├─ run()     [concrete implementation]
 *            └─ postRun() [cleanup]
 * ```
 *
 * **Execution States:**
 * - Running: Execution thread is active
 * - Ready-to-cancel: cancel() called, execution should stop
 * - Completed: run() finishes, outcome_ and message_ are valid
 * - Stopped: Thread has been joined
 *
 * **Synchronization:**
 * The condition_variable allows waiting threads to be notified of:
 * - State changes (outcome_ or message_ updates)
 * - Execution completion
 * - Cancellation completion
 */
class NavflexExecutionBase
{
public:
  /**
   * @brief Constructor for NavflexExecutionBase
   *
   * Initializes execution state with robot information and node reference.
   * No thread is created until start() is called.
   *
   * @param name Name identifier for this execution (used in logging/debugging)
   * @param robot_info Shared reference to current robot state information
   * @param node Shared pointer to ROS2 node for logging and parameter access
   */
  NavflexExecutionBase(
    const std::string & name,
    const navflex_utility::RobotInformation::ConstPtr & robot_info,
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Virtual destructor
   *
   * Ensures proper cleanup of any resources held by derived classes.
   * Note: Does NOT automatically stop/join the thread (call stop() first).
   */
  virtual ~NavflexExecutionBase();

  /**
   * @brief Start the execution thread
   *
   * **Operations:**
   * 1. Records current state as "thread starting"
   * 2. Spawns new execution thread
   * 3. Thread immediately calls run()
   * 4. Returns to caller (asynchronous)
   *
   * **Return Value:**
   * - true: Thread successfully created and started
   * - false: Thread was already running (cannot start again)
   *
   * @return true if thread was started, false if already running
   *
   * @note Must call stop() before calling start() again
   */
  virtual bool start();

  /**
   * @brief Stop the execution (blocking)
   *
   * **Operations:**
   * 1. Sets should_exit_ flag (tells run() to stop)
   * 2. Signals condition_variable to wake run() if waiting
   * 3. **Blocks** until execution thread finishes
   * 4. Joins thread (waits for completion)
   *
   * **Thread Safety:** Mutex-protected
   * **Blocking:** Waits for execution to actually stop
   *
   * @note This is a blocking operation, may wait indefinitely
   * @see cancel() for non-blocking stop request
   */
  virtual void stop();

  /**
   * @brief Request cancellation of current execution (non-blocking)
   *
   * **Behavior:**
   * - Sets cancel_ flag to true
   * - Does NOT wait for execution to actually stop
   * - Actual cancellation is implementation-specific
   *
   * **Default Implementation:**
   * Base class only sets flag; derived classes should act on it in run()
   *
   * @return true: Cancellation was requested; false: not implemented/running
   *
   * @note This is NON-BLOCKING, execution may continue briefly
   * @see stop() for blocking termination
   */
  virtual bool cancel()
  {
    return false;
  }

  /**
   * @brief Wait for thread to finish (blocking join)
   *
   * Blocks calling thread until execution thread completes.
   * Caller is responsible for calling stop() or cancel() first.
   *
   * @warning May block indefinitely if thread never finishes
   * @note This just joins the thread; does not request cancellation
   */
  void join();

  /**
   * @brief Wait for state update with timeout
   *
   * **Behavior:**
   * - Blocks until a state change notification
   * - Returns after duration expires (timeout)
   * - Returns immediately if state already changed
   *
   * **State Changes that Trigger:**
   * - outcome_ updated
   * - message_ updated
   * - Any explicit condition_variable notify calls
   *
   * @param duration Maximum time to wait
   * @return std::cv_status::no_timeout: state changed; timeout: timed out
   *
   * @note Useful for polling state with bounded wait time
   */
  std::cv_status waitForStateUpdate(std::chrono::microseconds const & duration);

  /**
   * @brief Get the current execution outcome
   *
   * The outcome is typically an error code or status enumerator:
   * - 0: Success
   * - Non-zero: Error, specific meaning depends on derived class
   *
   * Valid only after execution completes.
   *
   * @return Current outcome code (0 = success)
   *
   * @see getMessage() for human-readable outcome description
   */
  uint32_t getOutcome() const;

  /**
   * @brief Get human-readable execution message
   *
   * Provides detailed information about execution result.
   * Examples: "Plan found", "Failed to compute path", etc.
   *
   * Valid only after execution completes.
   *
   * @return Copy of the current outcome message string
   */
  std::string getMessage() const;

  /**
   * @brief Get the name identifier for this execution
   *
   * The name is set during construction and never changes.
   * Used for logging and debugging.
   *
   * @return Const reference to name string
   */
  const std::string & getName() const;

  /**
   * @brief Optional pre-execution setup hook
   *
   * Called in execution thread BEFORE run() is invoked.
   * Derived classes can override for initialization.
   *
   * **Execution Context:** Execution thread
   * **Timing:** Before run() starts
   *
   * @note Override if you need pre-execution setup (resources, logging, etc.)
   */
  virtual void preRun() {}

  /**
   * @brief Optional post-execution cleanup hook
   *
   * Called in execution thread AFTER run() completes.
   * Derived classes can override for cleanup.
   *
   * **Execution Context:** Execution thread
   * **Timing:** After run() finishes, before thread exits
   *
   * @note Override if you need post-execution cleanup (resource release, etc.)
   */
  virtual void postRun() {}

protected:
  /**
   * @brief Main execution method - derived classes override this
   *
   * This is where the actual work happens in the execution thread.
   * Default implementation does nothing.
   *
   * **Execution Context:** Dedicated execution thread (NOT main ROS thread)
   * **Typical Responsibilities:**
   * - Poll robot state from robot_info_
   * - Execute planning/control/behavior algorithms
   * - Update outcome_ and message_ as progress/completion occurs
   * - Check cancel_ flag for cancellation requests
   * - Notify condition_variable when state changes
   *
   * **Thread Safety:**
   * - outcome_ access: use setOutcome() / getOutcome()
   * - message_ access: use setMessage() / getMessage()
   * - Special flags (cancel_, should_exit_): use atomics / provided mutexes
   *
   * @note Default implementation provided; override in derived classes
   * @see preRun(), postRun() for setup/cleanup
   */
  virtual void run() {}

  void setOutcome(uint32_t outcome);
  void setMessage(const std::string & message);
  void setOutcomeAndMessage(uint32_t outcome, const std::string & message);

  // ========== Protected Member Access ==========
  /// Condition variable for thread synchronization - notify when state changes
  std::condition_variable condition_;

  /// The execution thread object (created on start, joined on stop)
  std::thread thread_;

  /// Mutex protecting should_exit_ flag
  std::mutex should_exit_mutex_;

  /// Shared mutex used by waitForStateUpdate() and condition_ notifications
  std::mutex state_wait_mutex_;

  /// Flag indicating thread should exit (set by stop(), checked by run())
  std::atomic<bool> should_exit_;

  /// Flag indicating cancellation was requested (set by cancel())
  std::atomic<bool> cancel_;

  /// Latest execution outcome code (0 = success, others = error)
  std::atomic<uint32_t> outcome_;

  /// Human-readable outcome message ("Success", "Failed to plan", etc.)
  std::string message_;
  mutable std::mutex message_mtx_;

  /// Name identifier for this execution (for logging)
  std::string name_;

  /// Reference to current robot state (position, velocity, etc.)
  navflex_utility::RobotInformation::ConstPtr robot_info_;

  /// Pointer to ROS2 node (for logger access)
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  /* namespace navflex_costmap_nav */
