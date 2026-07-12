// Implementation of PlannerExecution class
// Manages the execution of global planning in a dedicated thread with dynamic reconfiguration support

#include "navflex_base/planner_execution.h"

// Standard library includes
#include <exception>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"

namespace navflex_nav
{

/**
 * @brief Helper function to compute total distance of a path
 *
 * Iterates through consecutive poses and sums up Euclidean distances.
 *
 * @tparam _Iter Iterator type (must support operator++ and dereference)
 * @param _begin Iterator to start of path
 * @param _end Iterator to end of path
 * @return Total cumulative distance, or 0 if path has fewer than 2 poses
 *
 * @note Requires at least 2 poses to compute meaningful distance
 */
template<typename _Iter>
double sumDistance(_Iter _begin, _Iter _end)
{
  // Helper function to compute the distance of a path.
  // In C++11, we could add static_assert on the iterator_type.
  double dist = 0.0;

  // Minimum length of the path is 2
  if (std::distance(_begin, _end) < 2) {
    return dist;
  }

  // Two-pointer iteration over consecutive poses
  for (_Iter next = _begin + 1; next != _end; ++_begin, ++next) {
    dist += navflex_utility::distance(*_begin, *next);
  }

  return dist;
}

// ========== Constructor ==========

/**
 * PlannerExecution Constructor
 *
 * Initializes the planner execution with:
 * - Robot information and node references
 * - Planner plugin instance
 * - Dynamic parameters (frequency, patience, max_retries)
 * - Dynamic reconfiguration callback
 */
PlannerExecution::PlannerExecution(
  const std::string & name,
  const PlannerPluginAdapter::Ptr & planner_ptr,
  const navflex_utility::RobotInformation::ConstPtr & robot_info,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node_handle)
: NavflexExecutionBase(name, robot_info, node_handle),
  planner_(planner_ptr),
  state_(INITIALIZED),
  max_retries_(0),
  planning_(false),
  has_new_start_(false),
  has_new_goal_(false),
  node_handle_(node_handle),
  patience_(0, 0)
{
  RCLCPP_DEBUG_STREAM(
    node_handle_->get_logger(),
    "Initializing PlannerExecution for planner: " << name);

  // ========== Initialize Parameters ==========
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

  // Declare planner_frequency parameter
  if (!node_handle_->has_parameter("planner_frequency")) {
    param_desc.description = "The rate in Hz at which to run the planning loop";
    node_handle_->declare_parameter(
      "planner_frequency",
      rclcpp::ParameterValue(0.0), param_desc);
  }

  // Declare planner_patience parameter
  if (!node_handle_->has_parameter("planner_patience")) {
    param_desc.description =
      "How long the planner will wait in seconds before giving up";
    node_handle_->declare_parameter(
      "planner_patience",
      rclcpp::ParameterValue(10.0), param_desc);
  }

  // Declare planner_max_retries parameter
  if (!node_handle_->has_parameter("planner_max_retries")) {
    param_desc.description =
      "How many times to retry planning before giving up";
    node_handle_->declare_parameter(
      "planner_max_retries",
      rclcpp::ParameterValue(10), param_desc);
  }

  // ========== Load Parameter Values ==========
  node_handle_->get_parameter("planner_frequency", frequency_);

  double patience_seconds = 0.0;
  node_handle_->get_parameter("planner_patience", patience_seconds);
  patience_ = rclcpp::Duration::from_seconds(patience_seconds);

  node_handle_->get_parameter("planner_max_retries", max_retries_);

  RCLCPP_INFO_STREAM(
    node_handle_->get_logger(),
    "Planner parameters - Frequency: " << frequency_ << " Hz, "
                                       << "Patience: " << patience_seconds << "s, "
                                       << "Max retries: " << max_retries_);

  // ========== Setup Dynamic Reconfiguration ==========
  // Register callback for dynamic parameter updates
  dyn_params_handler_ = node_handle_->add_on_set_parameters_callback(
    std::bind(&PlannerExecution::reconfigure, this, std::placeholders::_1));
}

/**
 * PlannerExecution Destructor
 *
 * Cleanup is handled by base class and parent node lifecycle
 */
PlannerExecution::~PlannerExecution()
{
  RCLCPP_DEBUG_STREAM(
    node_handle_->get_logger(),
    "Destroying PlannerExecution");
}

// ========== Public Methods ==========

/**
 * @brief Get the cost of the last computed plan
 * @return Cumulative cost of the path (sum of distances or planner-specific metric)
 */
double PlannerExecution::getCost() const
{
  return cost_;
}

/**
 * @brief Dynamic reconfiguration callback for planner parameters
 *
 * Allows runtime updates to:
 * - planner_frequency: Planning loop frequency (Hz)
 * - planner_patience: Maximum wait time for a plan (seconds)
 * - planner_max_retries: Maximum retry attempts
 *
 * **Thread safety:** Mutex-protected
 * **Parameters:** Vector of parameters being reconfigured
 * **Returns:** SetParametersResult with success status
 */
rcl_interfaces::msg::SetParametersResult
PlannerExecution::reconfigure(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> guard(configuration_mutex_);
  rcl_interfaces::msg::SetParametersResult result;

  for (const rclcpp::Parameter & param : parameters) {
    const auto & param_name = param.get_name();
    try {
      if (param_name == "planner_frequency") {
        frequency_ = param.as_double();
        RCLCPP_INFO_STREAM(
          node_handle_->get_logger(),
          "Updated planner_frequency to " << frequency_ << " Hz");
      } else if (param_name == "planner_patience") {
        double patience_sec = param.as_double();
        patience_ = rclcpp::Duration::from_seconds(patience_sec);
        RCLCPP_INFO_STREAM(
          node_handle_->get_logger(),
          "Updated planner_patience to " << patience_sec << "s");
      } else if (param_name == "planner_max_retries") {
        max_retries_ = param.as_int();
        RCLCPP_INFO_STREAM(
          node_handle_->get_logger(),
          "Updated planner_max_retries to " << max_retries_);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_STREAM(
        node_handle_->get_logger(),
        "Failed to reconfigure parameter " << param_name << ": " << ex.what());
      result.successful = false;
      return result;
    }
  }

  result.successful = true;
  return result;
}

/**
 * @brief Get current planning execution state (thread-safe)
 * @return Current PlanningState enum value
 */
PlannerExecution::PlanningState PlannerExecution::getState() const
{
  std::lock_guard<std::mutex> guard(state_mtx_);
  return state_;
}

/**
 * @brief Set internal planning state with optional notification
 *
 * Updates the internal state and optionally notifies waiting threads.
 * When signalling=true, also stops the planning loop.
 *
 * **Thread safety:** Mutex-protected
 *
 * @param state New state value
 * @param signalling If true, stops planning and notifies condition
 */
void PlannerExecution::setState(PlanningState state, bool signalling)
{
  std::lock_guard<std::mutex> guard(state_mtx_);
  state_ = state;

  // Exit planning loop if we are signaling a terminal state
  planning_ = !signalling;

  // Notify all waiting threads on state transitions
  if (signalling) {
    condition_.notify_all();
  }
}

/**
 * @brief Get the timestamp of the last valid plan
 * @return ROS timestamp when the last successful plan was computed
 */
rclcpp::Time PlannerExecution::getLastValidPlanTime() const
{
  std::lock_guard<std::mutex> guard(plan_mtx_);
  return last_valid_plan_time_;
}

/**
 * @brief Check if the patience duration has been exceeded
 *
 * **Behavior:**
 * - Returns false if patience is disabled (duration = 0s)
 * - Returns true if current_time - start_time > patience_duration
 *
 * **Use case:** Determines if planner should stop trying and give up
 *
 * @return true if patience exceeded, false otherwise
 */
bool PlannerExecution::isPatienceExceeded() const
{
  return !(patience_ == rclcpp::Duration(0, 0)) &&
         (node_handle_->now() - last_call_start_time_ > patience_);
}

/**
 * @brief Get the last computed plan (thread-safe copy)
 * @return Vector of poses representing the planned path
 */
std::vector<geometry_msgs::msg::PoseStamped> PlannerExecution::getPlan() const
{
  std::lock_guard<std::mutex> guard(plan_mtx_);
  // Return copy of plan for thread-safe access
  return plan_;
}

/**
 * @brief Update goal pose while planning is ongoing
 *
 * Signals the planning thread that a new goal is available.
 * The running thread will pick up this goal on the next iteration.
 *
 * **Thread safety:** Mutex-protected
 *
 * @param goal New goal pose in global frame
 * @param tolerance Goal position tolerance (meters)
 */
void PlannerExecution::setNewGoal(const geometry_msgs::msg::PoseStamped & goal, double tolerance)
{
  std::lock_guard<std::mutex> guard(goal_start_mtx_);
  goal_ = goal;
  tolerance_ = tolerance;
  has_new_goal_ = true;
}

/**
 * @brief Update start pose while planning is ongoing
 *
 * Signals the planning thread that a new start pose is available.
 * The running thread will pick up this start on the next iteration.
 *
 * **Thread safety:** Mutex-protected
 *
 * @param start New start pose in global frame
 */
void PlannerExecution::setNewStart(const geometry_msgs::msg::PoseStamped & start)
{
  std::lock_guard<std::mutex> guard(goal_start_mtx_);
  start_ = start;
  has_new_start_ = true;
}

/**
 * @brief Simultaneously update both start and goal poses
 *
 * Signals the planning thread that both start and goal are new.
 * More efficient than separate setNewStart() and setNewGoal() calls.
 *
 * **Thread safety:** Mutex-protected
 *
 * @param start New start pose in global frame
 * @param goal New goal pose in global frame
 * @param tolerance Goal position tolerance (meters)
 */
void PlannerExecution::setNewStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double tolerance)
{
  std::lock_guard<std::mutex> guard(goal_start_mtx_);
  start_ = start;
  goal_ = goal;
  tolerance_ = tolerance;
  has_new_start_ = true;
  has_new_goal_ = true;
}

/**
 * @brief Start the planning execution thread
 *
 * **Behavior:**
 * - Checks if already planning, returns false if so
 * - Sets up start/goal poses and tolerance
 * - Calls parent's start() to create execution thread
 *
 * **Thread safety:** Mutex-protected
 *
 * @param start Start pose in global frame
 * @param goal Goal pose in global frame
 * @param tolerance Goal position tolerance (meters)
 * @return true if planning thread started, false if already planning
 */
bool PlannerExecution::start(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double tolerance)
{
  if (planning_) {
    RCLCPP_WARN_STREAM(
      node_handle_->get_logger(),
      "Cannot start planning: already planning!");
    return false;
  }

  std::lock_guard<std::mutex> guard(planning_mtx_);
  planning_ = true;
  start_ = start;
  goal_ = goal;
  tolerance_ = tolerance;

  const geometry_msgs::msg::Point & s = start.pose.position;
  const geometry_msgs::msg::Point & g = goal.pose.position;

  RCLCPP_INFO_STREAM(
    node_handle_->get_logger(),
    "Starting path planning from (" << s.x << ", " << s.y << ", " << s.z << ")"
      " to (" << g.x << ", " << g.y << ", " << g.z << ") "
      "with tolerance " << tolerance << "m");

  return NavflexExecutionBase::start();
}

/**
 * @brief Request cancellation of planning (non-blocking)
 *
 * **Behavior:**
 * - Sets cancel_ flag to true (signals planning thread)
 * - Calls planner_->cancel() to request plugin-level cancellation
 * - Returns false if cancellation not supported by planner
 *
 * **Thread safety:** cancel_ flag is thread-safe
 *
 * @return true if cancellation was accepted/implemented, false otherwise
 *
 * @note Cancellation is a request; actual stop may take time
 */
bool PlannerExecution::cancel()
{
  // Force cancel flag immediately
  // The planner->cancel() call may take time
  cancel_ = true;

  // Keep cancellation local to execution thread for compatibility with
  // planners implementing only nav2_core::GlobalPlanner::createPlan().
  RCLCPP_DEBUG_STREAM(
    node_handle_->get_logger(),
    "Cancellation flag set for planner execution thread");
  return true;
}

/**
 * @brief Call the planner plugin's makePlan method
 *
 * **Behavior:**
 * - Delegates to planner_->makePlan()
 * - Captures outcome code, plan, cost, and status message
 *
 * **Thread context:** Execution thread
 *
 * @param start Start pose for planning
 * @param goal Goal pose for planning
 * @param tolerance Goal tolerance
 * @param [out] plan Computed path (filled by planner)
 * @param [out] cost Total path cost (filled by planner)
 * @param [out] message Status message (filled by planner)
 * @return Outcome code (0=success, non-zero=error)
 */
uint32_t PlannerExecution::makePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double tolerance,
  std::vector<geometry_msgs::msg::PoseStamped> & plan,
  double & cost,
  std::string & message)
{
  (void)tolerance;

  nav_msgs::msg::Path nav2_plan;
  RCLCPP_DEBUG(
    node_handle_->get_logger(),
    "[PlannerExecution] calling planner plugin id=%s",
    name_.c_str());
  const uint32_t outcome = planner_->makePlan(start, goal, nav2_plan, message);
  plan = nav2_plan.poses;
  cost = sumDistance(plan.begin(), plan.end());

  if (outcome == 0 && message.empty()) {
    message = "Plan successfully created";
  }

  return outcome;
}

/**
 * Main planning loop execution in dedicated thread
 *
 * **Purpose:**
 * Implements NavflexExecutionBase::run() pure virtual method. This is the core
 * planning loop that coordinators planning operations with retry logic, dynamic
 * goal/start updates, and timeout/patience management.
 *
 * **Thread Context:** Dedicated execution thread (called from NavflexExecutionBase::start())
 *
 * **State Machine:**
 * ```
 * INITIALIZED
 *    ↓
 * STARTED → STOPPED (if should_exit)
 * ↓
 * Check for new goal/start updates
 *    ↓
 * PLANNING → Check cancel flag
 *    ├→ CANCELED (if cancel_ set and patience not exceeded)
 *    ├→ FOUND_PLAN (if makePlan succeeded)
 *    ├→ MAX_RETRIES (if retries exceeded)
 *    ├→ PAT_EXCEEDED (if patience exceeded)
 *    ├→ NO_PLAN_FOUND (if max_retries == 0 and failed)
 *    └→ retry (loop back to PLANNING)
 * FOUND_PLAN/CANCELED/MAX_RETRIES/PAT_EXCEEDED/NO_PLAN_FOUND → exit loop
 * ```
 *
 * **Algorithm:**
 * 1. Initialize state to STARTED and copy initial goal/start
 * 2. Record planning start time and valid plan timestamp
 * 3. Loop while planning_ is true and ROS2 is ok():
 *    a. Check if should_exit_ (external interrupt request)
 *    b. Lock goal_start_mtx_ and check for new goal/start poses
 *    c. Check cancel_ flag (cancellation request)
 *    d. If not canceled, call makePlan() with current poses
 *    e. Evaluate result and update state:
 *       - Success: Store plan/cost, update valid plan time
 *       - Failure with retries left: Log and retry
 *       - Failure with no retries: Set NO_PLAN_FOUND state
 *       - Max retries exceeded: Set MAX_RETRIES state
 *       - Patience exceeded: Set PAT_EXCEEDED state
 *    f. Notify waiting threads via condition_
 * 4. Catch any exceptions and set INTERNAL_ERROR state
 * 5. Exit with planning_ = false
 *
 * **Dynamic Goal/Start Updates:**
 * While loop is running, external threads can call:
 * - setNewGoal() to update target pose
 * - setNewStart() to update start pose
 * - setNewStartAndGoal() to update both
 *
 * The run loop checks these flags every iteration and adapts without restart.
 *
 * **Patience Mechanism:**
 * - patience_ is a time duration (configured parameter)
 * - last_valid_plan_time_ tracks when a plan was successfully found
 * - isPatienceExceeded() returns true if (now - last_valid_plan_time_) > patience_
 * - Used to detect stalled planning and trigger timeout
 *
 * **Retry Mechanism:**
 * - max_retries_ = number of failures allowed before giving up (0 = infinite)
 * - retries_ counter increments on each failed planning attempt
 * - When retries > max_retries_, sets MAX_RETRIES state and exits
 *
 * **Cancellation:**
 * - cancel_ flag set by cancel() method triggers gentle cancel
 * - planner_->cancel() called first to request plugin-level cancellation
 * - If cancel_ still true after makePlan(), state is set to CANCELED
 * - Not all planners support cancellation (nav_core based)
 *
 * **Cost Estimation:**
 * - If planner returns cost_ == 0, estimate as sum of euclidean distances
 * - sumDistance() helper computes path length
 *
 * **Exception Handling:**
 * - Catch-all exception handler prevents thread crash
 * - Sets INTERNAL_ERROR state on any uncaught exception
 * - Notifies waiting threads so action server doesn't hang
 *
 * **Cost Values (Outcome):**
 * The outcome code semantics (from nav2_core::GlobalPlanner):
 * - < 10: Success codes (SUCCESS, etc.)
 * - >= 10: Failure codes (NO_VALID_PATH, START_OCCUPIED, etc.)
 *
 * @note This is a long-running method that blocks until planning_ becomes false
 * @note Thread must be started via NavflexExecutionBase::start()
 * @note State transitions notify condition_ for waiting threads
 * @note Calls setState(state, notify) to update execution state
 *
 * **Common Issues & Debugging:**
 * 1. Infinite retry loop: Set max_retries_ > 0 or patience_ timeout
 * 2. Stuck in PLANNING: Planner not responding, check planner plugin logs
 * 3. Plan cost is zero: Normal, sumDistance() estimate is used
 * 4. Patience exceeded: Increase patience parameter if planning is slow
 * 5. Thread doesn't exit: Check external threads not holding planning_mtx
 */
void PlannerExecution::run()
{
  setState(STARTED, false);
  std::lock_guard<std::mutex> guard(planning_mtx_);
  int retries = 0;
  geometry_msgs::msg::PoseStamped current_start = start_;
  geometry_msgs::msg::PoseStamped current_goal = goal_;
  double current_tolerance = tolerance_;

  last_call_start_time_ = node_handle_->now();
  last_valid_plan_time_ = node_handle_->now();

  try {
    while (planning_ && rclcpp::ok()) {
      if (should_exit_) {
        // Early exit if should_exit_ is set
        handle_thread_interrupted();
        return;
      }

      // call the planner
      std::vector<geometry_msgs::msg::PoseStamped> plan;
      double cost = 0.0;

      // lock goal start mutex
      goal_start_mtx_.lock();
      if (has_new_start_) {
        has_new_start_ = false;
        current_start = start_;
        RCLCPP_INFO_STREAM(
          node_handle_->get_logger(), "A new start pose is available. Planning "
          "with the new start pose!");
        const geometry_msgs::msg::Point & s = start_.pose.position;
        RCLCPP_INFO_STREAM(
          node_handle_->get_logger(),
          "New planning start pose: (" << s.x << ", " << s.y << ", " << s.z << ")");
      }
      if (has_new_goal_) {
        has_new_goal_ = false;
        current_goal = goal_;
        current_tolerance = tolerance_;
        RCLCPP_INFO_STREAM(
          node_handle_->get_logger(),
          "A new goal pose is available. Planning with the new goal pose and the tolerance: " <<
            current_tolerance);
        const geometry_msgs::msg::Point & g = goal_.pose.position;
        RCLCPP_INFO_STREAM(
          node_handle_->get_logger(),
          "New goal pose: (" << g.x << ", " << g.y << ", " << g.z << ")");
      }

      // unlock goal
      goal_start_mtx_.unlock();
      if (cancel_) {
        RCLCPP_INFO_STREAM(node_handle_->get_logger(), "The global planner has been canceled!");
        setState(CANCELED, true);
      } else {
        setState(PLANNING, false);

        // Reset per-attempt timer so patience tracks single makePlan() call duration
        last_call_start_time_ = node_handle_->now();
        std::string planning_message;
        const uint32_t outcome = makePlan(
          current_start, current_goal, current_tolerance, plan, cost,
          planning_message);
        setOutcomeAndMessage(outcome, planning_message);
        bool success = outcome < 10;

        std::lock_guard<std::mutex> guard(configuration_mutex_);

        if (cancel_ && !isPatienceExceeded()) {
          RCLCPP_INFO_STREAM(
            node_handle_->get_logger(),
            "The planner \"" << name_ << "\" has been canceled!");                    // but not due to patience exceeded
          setState(CANCELED, true);
        } else if (success) {
          RCLCPP_INFO(
            node_handle_->get_logger(),
            "[PlannerExecution] Successfully found a plan with planner plugin id=%s "
            "(outcome=%u, cost=%.3f, poses=%zu, elapsed=%.2fs)",
            name_.c_str(), outcome, cost, plan.size(),
            (node_handle_->now() - last_call_start_time_).seconds());

          std::lock_guard<std::mutex> plan_mtx_guard(plan_mtx_);
          plan_ = plan;
          cost_ = cost;
          // estimate the cost based on the distance if its zero.
          if (cost_ == 0) {
            cost_ = sumDistance(plan_.begin(), plan_.end());
          }

          last_valid_plan_time_ = node_handle_->now();
          setState(FOUND_PLAN, true);
        } else if (max_retries_ > 0 && ++retries > max_retries_) {
          RCLCPP_INFO_STREAM(
            node_handle_->get_logger(),
            "Planning reached max retries! (" << max_retries_ << ")");
          setState(MAX_RETRIES, true);
        } else if (isPatienceExceeded()) {
          // Patience exceeded is handled at two levels: here to stop retrying planning when max_retries is
          // disabled, and on the navigation server when the planner doesn't return for more that patience seconds.
          // In the second case, the navigation server has tried to cancel planning (possibly without success, as
          // old nav_core-based planners do not support canceling), and we add here the fact to the log for info
          RCLCPP_INFO_STREAM(
            node_handle_->get_logger(),
            "Planning patience (" << patience_.seconds() << "s) has been exceeded"
                                  << (cancel_ ? "; planner canceled!" : ""));
          setState(PAT_EXCEEDED, true);
        } else if (max_retries_ == 0) {
          RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Planning could not find a plan!");
          setState(NO_PLAN_FOUND, true);
        } else {
          const double elapsed = (node_handle_->now() - last_call_start_time_).seconds();
          RCLCPP_WARN(
            node_handle_->get_logger(),
            "[PlannerExecution] Planning attempt failed, retrying... "
            "(outcome=%u, retry=%d, elapsed=%.2fs/patience=%.1fs) | %s",
            outcome, retries + 1, elapsed, patience_.seconds(),
            planning_message.empty() ? "no message" : planning_message.c_str());

          const auto retry_delay = frequency_ > 0.0 ?
            std::chrono::duration<double>(1.0 / frequency_) :
            std::chrono::duration<double>(0.1);
          std::unique_lock<std::mutex> wait_lock(state_wait_mutex_);
          condition_.wait_for(wait_lock, retry_delay);
        }
      }
    }   // while (planning_ && ros::ok())
  } catch (...) {
    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Unknown error occurred.");
    setState(INTERNAL_ERROR, true);
    condition_.notify_all();
  }
}

/**
   * Handle external thread interruption (emergency stop)
   *
   * **Purpose:**
   * Called when the execution thread is interrupted externally, typically when
   * patience timeout is exceeded by the action server. This is an emergency stop
   * mechanism for when the planner takes too long.
   *
   * **Behavior:**
   * 1. Logs warning that thread is being interrupted
   * 2. Sets state to STOPPED
   * 3. Notifies all waiting threads via condition_
   *
   * **Thread Safety:**
   * - setState() handles all synchronization
   * - condition_.notify_all() wakes waiting threads
   *
   * **Thread Context:** Execution thread (called from run())
   *
   * **Triggers:**
   * - ROS2 lifecycle shutdown signal
   * - should_exit_ flag set by external thread
   * - Patience timeout exceeded (checked in run() loop)
   *
   * **Flow:**
   * ```
   * Action Server Patience Timer → cancels goal → sets should_exit_
   *                                                      ↓
   *                                            run() detects should_exit_
   *                                                      ↓
   *                                         calls handle_thread_interrupted()
   *                                                      ↓
   *                                              State → STOPPED
   *                                              Wakes action server thread
   * ```
   *
   * **Related Methods:**
   * - run() - checks for early exit at each iteration
   * - cancel() - soft cancellation request
   * - setState(STOPPED, true) - actually sets state
   *
   * @note Differs from cancel(): cancel is soft, this is forceful stop
   * @note Always sets state to STOPPED (not CANCELED or other states)
   * @note Notifies all threads waiting on condition_
   */
void PlannerExecution::handle_thread_interrupted()
{
  // Planner thread interrupted; probably we have exceeded planner patience
  RCLCPP_WARN_STREAM(node_handle_->get_logger(), "Planner thread interrupted!");
  setState(STOPPED, true);
  condition_.notify_all();
}

} /* namespace navflex_nav */
