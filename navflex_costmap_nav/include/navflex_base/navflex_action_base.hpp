#pragma once

// Standard library includes
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

// ROS2 includes
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Local includes
#include "navflex_utility/robot_information.h"
#include "navflex_base/navflex_execution_base.h"

namespace navflex_costmap_nav
{

// ========== Type Trait: has_concurrency_slot ==========
// Detects if an action Goal type has a concurrency_slot field.
// Used to conditionally extract slot ID from the goal message.
template<typename T, typename = void>
struct has_concurrency_slot : std::false_type {};

template<typename T>
struct has_concurrency_slot<T,
  std::void_t<decltype(std::declval<T>().concurrency_slot)>>
  : std::true_type {};

/**
 * @class NavflexActionBase
 * @brief Base class for managing multiple concurrent action executions.
 *
 * This template class provides a framework for handling multiple parallel
 * action executions using concurrent slots. Each slot can host one execution at
 * a time, and supports replacing ongoing executions with new ones.
 *
 * **Key Features:**
 * - Manages concurrent executions through numbered slots (0-255)
 * - Thread-safe slot management with mutex protection
 * - Automatic cleanup of finished executions
 * - Support for canceling individual or all executions
 *
 * **Thread Model:**
 * - Main thread: Handles start/cancel requests, creates/joins execution threads
 * - Execution threads: Run the actual runImpl() in a dedicated thread per slot
 *
 * **Usage pattern:**
 * 1. Subclass this template with specific Action and Execution types
 * 2. Override runImpl() to implement execution logic
 * 3. Call start() to initiate execution in a given slot
 * 4. Call cancel() or cancelAll() to stop executions
 *
 * @tparam Action An ROS2 action type (e.g.,
 * nav2_msgs::action::ComputePathToPose)
 * @tparam Execution A class derived from NavflexExecutionBase representing the
 * task
 *
 * @note The Execution type must define a shared_ptr typedef as Execution::Ptr
 */
template<typename Action, typename Execution>
class NavflexActionBase
{
public:
  // ========== Type Definitions ==========
  /// Goal handle type for this action
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  /// Shared pointer to goal handle
  using GoalHandlePtr = std::shared_ptr<GoalHandle>;
  /// Slot ID type (0-255, supporting 256 concurrent executions max)
  using SlotId = uint8_t;
  /**
   * @struct ConcurrencySlot
   * @brief Container for a single execution slot
   *
   * Each slot represents an independent execution context that can run
   * concurrently with other slots. A slot can be repurposed for a new
   * execution once the previous one completes.
   */
  struct ConcurrencySlot
  {
    /// Default constructor initializes slot as unused
    ConcurrencySlot()
    : in_use(false) {}

    /// Pointer to the execution object (handles the actual work)
    typename Execution::Ptr execution;

    /// Thread managing this execution (created on start, joined on finish)
    std::unique_ptr<std::thread> thread_ptr;

    /// Goal handle for communicating results back to the action client
    GoalHandlePtr goal_handle;

    /// Flag indicating whether this slot is currently in use
    std::atomic<bool> in_use;
  };

  /// Internal map type: slot ID -> concurrency slot
  using ConcurrencyMap = std::map<SlotId, ConcurrencySlot>;

public:
  /**
   * @brief Constructor for NavflexActionBase
   *
   * Initializes the action base with node reference and robot information.
   * The actual action server creation happens in the derived class.
   *
   * @param node Shared pointer to the ROS2 node for logging and parameter
   * access
   * @param name Name identifier for this action (used in logging)
   * @param robot_info Robot state information (position, velocity, etc.)
   */
  NavflexActionBase(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name,
    const navflex_utility::RobotInformation::ConstPtr & robot_info)
  : node_(node), name_(name), robot_info_(robot_info) {}

  /**
   * @brief Destructor - ensures all executions are properly cleaned up
   *
   * **Cleanup sequence:**
   * 1. Acquires mutex lock to prevent new operations
   * 2. Cancels all running executions
   * 3. Joins all execution threads (blocking operation)
   * 4. Aborts all active goal handles to notify clients
   * 5. Releases all resources
   *
   * @note Non-recursive mutex means cancelAll() cannot be called from here
   */
  virtual ~NavflexActionBase() {cleanupAllSlots();}

  /**
   * @brief Start a new execution in the specified slot
   *
   * **Behavior:**
   * - If goal is already being canceled, immediately returns with canceled
   * status
   * - If slot already has an active execution:
   *   - Cancels the current execution
   *   - Waits for its thread to finish (blocking)
   *   - Clears the slot resources
   * - Creates and starts a new execution thread
   *
   * **Thread safety:** Mutex-protected
   * **Blocking:** May block calling thread waiting for old slot thread to
   * finish
   *
   * @param goal_handle The ROS2 action goal handle containing client request
   * @param execution_ptr The execution object to run in this slot
   *
   * @warning Blocking join() on old thread may delay starting new execution
   * @see cancel() for non-blocking cancellation
   */
  virtual void start(
    const GoalHandlePtr & goal_handle,
    typename Execution::Ptr execution_ptr)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Start called with null goal_handle");
      return;
    }

    SlotId slot_id = getSlotId(goal_handle);

    if (goal_handle->is_canceling()) {
      typename Action::Result::SharedPtr result =
        std::make_shared<typename Action::Result>();
      goal_handle->canceled(result);
      RCLCPP_INFO(
        node_->get_logger(),
        "Goal canceled before execution started (slot %u).", slot_id);
      return;
    }

    std::lock_guard<std::mutex> guard(slot_map_mtx_);
    cleanupSlot(slot_id);
    setupAndStartSlot(slot_id, goal_handle, execution_ptr);
  }

  /**
   * @brief Cancel execution in a specific slot
   *
   * **Behavior:**
   * - Finds the slot by ID
   * - Calls cancel() on its execution (request to stop, not forced)
   * - Does not wait for thread to actually finish
   *
   * **Thread safety:** Mutex-protected
   * **Non-blocking:** Returns immediately after cancel request
   *
   * @param goal_handle The goal handle whose slot should be canceled
   *
   * @note This is a cancel request, not forced termination. The execution
   *       thread may continue briefly while handling the cancellation.
   * @see start() for slot cleanup policy
   */
  virtual void cancel(GoalHandlePtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Cancel called with null goal_handle");
      return;
    }

    SlotId slot_id = getSlotId(goal_handle);

    std::lock_guard<std::mutex> guard(slot_map_mtx_);
    auto slot_it = concurrency_slots_.find(slot_id);
    if (slot_it != concurrency_slots_.end() && slot_it->second.execution) {
      slot_it->second.execution->cancel();
      RCLCPP_DEBUG(
        node_->get_logger(), "Cancel request sent to slot %u.",
        slot_id);
    }
  }

  /**
   * @brief Pure virtual method to be implemented by derived classes
   *
   * This method contains the actual action execution logic. It is called
   * from the run() method in a dedicated execution thread.
   *
   * **Execution context:** Runs in execution thread, not main ROS thread
   * **Responsibilities:**
   * - Interact with the execution object
   * - Update goal progress feedback if needed
   * - Handle execution completion and errors
   * - Set final result in goal_handle
   *
   * @param goal_handle Handle to communicate goal status and results
   * @param execution Reference to the execution object for this action
   */
  virtual void runImpl(
    const GoalHandlePtr & goal_handle,
    Execution & execution) = 0;

  /**
   * @brief Execution wrapper - called in dedicated execution thread
   *
   * **Execution sequence:**
   * 1. Calls execution->preRun() for setup
   * 2. Delegates to runImpl() for actual work
   * 3. Waits for execution thread to complete (sync point)
   * 4. Calls execution->postRun() for cleanup
   * 5. Marks slot as unused (available for reuse)
   *
   * **Thread context:** Execution thread (not main ROS callback thread)
   *
   * @param slot Reference to the concurrency slot being executed
   *
   * @note This is the main entry point for execution threads spawned by start()
   */
  virtual void run(ConcurrencySlot & slot)
  {
    slot.execution->preRun();
    try {
      runImpl(slot.goal_handle, *slot.execution);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(name_),
        "Exception in runImpl: " << e.what());
    }
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger(name_),
      "Finished action \""
        << name_ << "\" run method, waiting for execution to complete.");
    slot.execution->join();
    slot.execution->postRun();
    slot.in_use.store(false);
  }

  /**
   * @brief Cancel all active executions
   *
   * **Behavior:**
   * 1. Sends cancel request to all active execution pools
   * 2. Waits for all execution threads to finish (blocking)
   * 3. Updated log message about cancellation
   *
   * **Thread safety:** Mutex-protected
   * **Blocking:** Waits for all threads to complete
   *
   * @warning This is a blocking operation that may wait significant time
   * @see cancel() for non-blocking single slot cancellation
   */
  virtual void cancelAll()
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(name_),
      "Canceling all goals for \"" << name_ << "\" ("
                                   << concurrency_slots_.size()
                                   << " slots active).");
    std::lock_guard<std::mutex> guard(slot_map_mtx_);

    // First pass: cancel all execution
    for (auto & [slot_id, concurrency_slot] : concurrency_slots_) {
      if (concurrency_slot.execution) {
        concurrency_slot.execution->cancel();
      }
    }

    // Second pass: wait for all threads
    for (auto & [slot_id, concurrency_slot] : concurrency_slots_) {
      waitAndCleanupThread(concurrency_slot);
    }
  }

protected:
  /**
   * @brief Extract the concurrency slot ID from the goal handle.
   *
   * Default implementation uses `concurrency_slot` field from the action Goal
   * if available (detected via has_concurrency_slot type trait), otherwise
   * returns slot 0. Derived classes may override for custom slot selection.
   *
   * @param goal_handle The goal handle to extract the slot ID from
   * @return SlotId to use for this execution
   */
  virtual SlotId getSlotId(const GoalHandlePtr & goal_handle)
  {
    if constexpr (has_concurrency_slot<typename Action::Goal>::value) {
      return goal_handle->get_goal()->concurrency_slot;
    } else {
      (void)goal_handle;
      return 0;
    }
  }

  // ========== Member Variables ==========

  /// Shared pointer to the ROS2 lifecycle node (for logging, parameters, etc.)
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  /// Human-readable name for this action (used in logging)
  const std::string name_;

  /// Robot state information (position, velocity, configuration)
  navflex_utility::RobotInformation::ConstPtr robot_info_;

  /// Map of all concurrency slots indexed by slot ID
  ConcurrencyMap concurrency_slots_;

  /// Mutex protecting concurrent access to concurrency_slots_ map
  std::mutex slot_map_mtx_;

private:
  // ========== Private Helper Methods ==========

  /**
   * @brief Cleanup all slots (called during destruction)
   *
   * Iterates through all slots, cancels executions, joins threads,
   * and aborts goal handles.
   *
   * @note Must hold slot_map_mtx_ before calling
   */
  void cleanupAllSlots()
  {
    std::lock_guard<std::mutex> guard(slot_map_mtx_);

    // First pass: cancel all
    for (auto & [slot_id, concurrency_slot] : concurrency_slots_) {
      if (concurrency_slot.execution) {
        concurrency_slot.execution->cancel();
      }
    }

    // Second pass: cleanup threads and goal handles
    for (auto & [slot_id, concurrency_slot] : concurrency_slots_) {
      waitAndCleanupThread(concurrency_slot);

      // Notify client that goal is aborted
      if (concurrency_slot.goal_handle &&
        concurrency_slot.goal_handle->is_active())
      {
        typename Action::Result::SharedPtr result =
          std::make_shared<typename Action::Result>();
        concurrency_slot.goal_handle->abort(result);
      }
    }
  }

  /**
   * @brief Cleanup a specific slot (prepare for reuse)
   *
   * **Operations:**
   * 1. If slot exists and is in use: cancel its execution
   * 2. Wait for thread to complete (blocking)
   * 3. Reset thread pointer (release ownership)
   *
   * @param slot_id The slot to cleanup
   *
   * @note Must hold slot_map_mtx_ before calling
   * @warning May block if thread is still running
   */
  void cleanupSlot(SlotId slot_id)
  {
    auto slot_it = concurrency_slots_.find(slot_id);
    if (slot_it != concurrency_slots_.end()) {
      if (slot_it->second.in_use.load() && slot_it->second.execution) {
        slot_it->second.execution->cancel();
      }
      waitAndCleanupThread(slot_it->second);
    }
  }

  /**
   * @brief Setup and start execution in a slot
   *
   * **State after call:**
   * - Slot marked as in_use
   * - Goal handle and execution stored
   * - New execution thread created and started
   * - Slot is ready for concurrent execution
   *
   * @param slot_id Target slot ID
   * @param goal_handle Goal handle for this execution
   * @param execution_ptr Execution object to run
   *
   * @note Must hold slot_map_mtx_ before calling
   */
  void setupAndStartSlot(
    SlotId slot_id, const GoalHandlePtr & goal_handle,
    typename Execution::Ptr execution_ptr)
  {
    // Find or create slot
    auto [slot_it, inserted] = concurrency_slots_.try_emplace(slot_id);

    // Configure slot
    slot_it->second.in_use.store(true);
    slot_it->second.goal_handle = goal_handle;
    slot_it->second.execution = execution_ptr;

    // Get raw pointer before creating thread (thread will capture it)
    ConcurrencySlot * slot_ptr = &slot_it->second;

    // Create and start execution thread
    slot_it->second.thread_ptr =
      std::make_unique<std::thread>([this, slot_ptr]() {run(*slot_ptr);});

    RCLCPP_DEBUG(node_->get_logger(), "Started execution in slot %u.", slot_id);
  }

  /**
   * @brief Wait for thread to finish and cleanup its resources
   *
   * **Operations:**
   * 1. Check if thread exists and is joinable
   * 2. Wait for thread completion (blocking)
   * 3. Release thread ownership
   *
   * @param slot Reference to slot containing the thread
   *
   * @note May block indefinitely if thread never finishes
   */
  void waitAndCleanupThread(ConcurrencySlot & slot)
  {
    if (slot.thread_ptr && slot.thread_ptr->joinable()) {
      try {
        slot.thread_ptr->join();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(
          node_->get_logger(),
          "Error joining thread: " << e.what());
      }
    }
    slot.thread_ptr.reset();
  }
};

}  // namespace navflex_costmap_nav
