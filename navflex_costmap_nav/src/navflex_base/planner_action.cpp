// Implementation of PlannerAction
// Coordinates ROS2 action interface with planner execution

// Standard library includes
#include <sstream>

// Local includes
#include "navflex_base/planner_action.h"

namespace navflex_costmap_nav
{

/**
 * PlannerAction Constructor
 *
 * Initializes the action handler and sets up publishers for goal visualization.
 */
PlannerAction::PlannerAction(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const navflex_utility::RobotInformation::ConstPtr & robot_info)
: NavflexActionBase(node, name, robot_info)
{
  // Setup publisher for goal visualization in rviz
  current_goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/current_goal", 1);
  // Setup publisher for the computed plan
  plan_publisher_ = node->create_publisher<nav_msgs::msg::Path>("plan", 1);
}

/**
 * Execute path planning action (implements NavflexActionBase::runImpl)
 *
 * **Execution Flow:**
 * 1. Extract goal parameters (target pose, start pose, tolerance)
 * 2. Publish current goal for visualization in RViz
 * 3. Get robot's current pose if not provided in goal
 * 4. Coordinate with execution thread via state machine:
 *    - INITIALIZED → start execution thread
 *    - PLANNING → monitor progress, check patience timeout
 *    - FOUND_PLAN → retrieve plan, transform to global frame, return result
 *    - CANCELED/STOPPED → abort action with appropriate message
 *    - ERROR States → report error details to client
 * 5. Poll execution state every 500ms
 * 6. Handle goal cancellation requests from action client
 *
 * **Thread Context:** Execution thread (dedicated to this action, not main ROS
 * thread)
 *
 * **Key Responsibilities:**
 * - Manage goal lifecycle: start → monitor → complete/abort
 * - Coordinate state transitions between action server and execution thread
 * - Handle dynamic goal/start updates
 * - Transform paths between coordinate frames
 * - Report progress and final results to action client
 *
 * **Error Handling:**
 * - TF_ERROR: Failed to get robot pose or transform plan to global frame
 * - INTERNAL_ERROR: Another planning operation already in progress
 * - STOPPED: Planning was stopped externally
 * - CANCELED: Canceled by action client
 * - EMPTY_PATH: Planner returned valid but empty path
 * - MAX_RETRIES: Planner failed multiple times within patience window
 * - PAT_EXCEEDED: Planning took longer than patience limit
 *
 * @param goal_handle ROS2 goal handle for action server
 * @param execution PlannerExecution instance managing the planning thread
 *
 * **Implementation Notes:**
 * - State polling interval is ~500ms (8 iterations max normally)
 * - Patience timeout mechanism prevents indefinite planning
 * - Empty paths are valid for goals already at start location
 */
void PlannerAction::runImpl(
  const GoalHandlePtr & goal_handle,
  PlannerExecution & execution)
{
  const ActionToPose::Goal & goal = *goal_handle->get_goal();

  ActionToPose::Result::SharedPtr result =
    std::make_shared<ActionToPose::Result>();
  geometry_msgs::msg::PoseStamped start_pose;

  result->path.header.frame_id = robot_info_->getGlobalFrame();

  double tolerance = goal.tolerance;
  bool use_start_pose = goal.use_start;
  current_goal_pub_->publish(goal.goal);

  bool planner_active = true;

  if (use_start_pose) {
    start_pose = goal.start;
    const geometry_msgs::msg::Point & p = start_pose.pose.position;
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger(name_), "Use the given start pose ("
        << p.x << ", " << p.y
        << "), " << p.z << ").");
  } else {
    // get the current robot pose
    if (!robot_info_->getRobotPose(start_pose)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(name_),
        "Could not get the current robot pose! Canceling the action call.");
      goal_handle->abort(result);
      return;
    } else {
      const geometry_msgs::msg::Point & p = start_pose.pose.position;
      RCLCPP_DEBUG_STREAM(
        rclcpp::get_logger(name_),
        "Got the current robot pose at ("
          << p.x << ", " << p.y << ", " << p.z << ").");
    }
  }

  PlannerExecution::PlanningState state_planning_input;

  std::vector<geometry_msgs::msg::PoseStamped> plan;

  while (planner_active && rclcpp::ok()) {
    // get the current state of the planning thread
    state_planning_input = execution.getState();

    if (goal_handle
      ->is_canceling())          // action client requested to cancel the action
                                 // and our server accepted that request
    {
      planner_active = false;
      execution.stop();
      execution.join();
      result->outcome = ActionToPose::Result::CANCELED;
      result->message = "Planning canceled by client";
      goal_handle->canceled(result);
      return;
    }

    switch (state_planning_input) {
      case PlannerExecution::INITIALIZED:
        RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger(name_),
          "planner state: initialized");
        if (!execution.start(start_pose, goal.goal, tolerance)) {
          RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(name_),
            "Another thread is still planning! Canceling the action call.");
          goal_handle->abort(result);
          planner_active = false;
        }
        break;

      case PlannerExecution::STARTED:
        RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger(name_),
          "planner state: started");
        break;

      case PlannerExecution::STOPPED:
        RCLCPP_WARN(
          rclcpp::get_logger(name_),
          "[PlannerAction] STOPPED: start=(%.3f, %.3f) goal=(%.3f, %.3f) tolerance=%.3fm",
          start_pose.pose.position.x, start_pose.pose.position.y,
          goal.goal.pose.position.x, goal.goal.pose.position.y,
          tolerance);
        result->outcome = ActionToPose::Result::STOPPED;
        result->message = "Planning has been stopped rigorously";
        goal_handle->abort(result);
        planner_active = false;
        break;

      case PlannerExecution::CANCELED:
        RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger(name_),
          "planner state: canceled");
        RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger(name_),
          "Global planner has been canceled successfully");
        result->path.header.stamp = node_->now();
        result->outcome = ActionToPose::Result::CANCELED;
        result->message = "Global planner has been canceled";
        goal_handle->canceled(result);
        planner_active = false;
        break;

      // in progress
      case PlannerExecution::PLANNING:
        if (execution.isPatienceExceeded()) {
          RCLCPP_ERROR(
            rclcpp::get_logger(name_),
            "[PlannerAction] Single makePlan() call exceeded patience (%.1fs)! "
            "Aborting. start=(%.3f, %.3f) goal=(%.3f, %.3f)",
            execution.getPatience(),
            start_pose.pose.position.x, start_pose.pose.position.y,
            goal.goal.pose.position.x, goal.goal.pose.position.y);
          // Request execution to stop (sets should_exit_ + cancel_ flags).
          // makePlan may still be blocking in the plugin, but the action
          // server must not be held hostage — report failure now.
          execution.stop();
          execution.cancel();
          result->outcome = ActionToPose::Result::PAT_EXCEEDED;
          result->message = "Planning patience exceeded while waiting for makePlan";
          goal_handle->abort(result);
          planner_active = false;
        } else {
          RCLCPP_DEBUG_THROTTLE(
            rclcpp::get_logger(name_), *node_->get_clock(),
            2000, "planner state: planning");
        }
        break;

      // found a new plan
      case PlannerExecution::FOUND_PLAN:
        // set time stamp to now
        result->path.header.stamp = node_->now();
        plan = execution.getPlan();

        RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger(name_),
          "planner state: found plan with cost: " << execution.getCost());

        if (plan.empty()) {
          RCLCPP_ERROR(
            rclcpp::get_logger(name_),
            "[PlannerAction] EMPTY_PATH: start=(%.3f, %.3f) goal=(%.3f, %.3f) tolerance=%.3fm",
            start_pose.pose.position.x, start_pose.pose.position.y,
            goal.goal.pose.position.x, goal.goal.pose.position.y,
            tolerance);
          result->outcome = ActionToPose::Result::EMPTY_PATH;
          result->message = "Global planner returned an empty path";
          goal_handle->abort(result);
          planner_active = false;
          break;
        }

        result->path.poses = plan;
        result->planning_time = rclcpp::Duration(0, 0);
        result->outcome = execution.getOutcome();
        result->message = execution.getMessage();
        result->cost = execution.getCost();
        RCLCPP_INFO(
          rclcpp::get_logger(name_),
          "[PlannerAction] FOUND_PLAN: start=(%.3f, %.3f) goal=(%.3f, %.3f) "
          "tolerance=%.3fm poses=%zu cost=%.3f",
          start_pose.pose.position.x, start_pose.pose.position.y,
          goal.goal.pose.position.x, goal.goal.pose.position.y,
          tolerance, plan.size(), result->cost);
        plan_publisher_->publish(result->path);
        goal_handle->succeed(result);

        planner_active = false;
        break;

      // no plan found
      case PlannerExecution::NO_PLAN_FOUND:
        {
          const std::string execution_message = execution.getMessage();
          RCLCPP_ERROR(
            rclcpp::get_logger(
              name_),
            "[PlannerAction] NO_PLAN_FOUND: start=(%.3f, %.3f) goal=(%.3f, %.3f) tolerance=%.3fm | %s",
            start_pose.pose.position.x, start_pose.pose.position.y,
            goal.goal.pose.position.x, goal.goal.pose.position.y,
            tolerance, execution_message.c_str());
          result->outcome = execution.getOutcome();
          result->message = execution_message;
          goal_handle->abort(result);
          planner_active = false;
          break;
        }

      case PlannerExecution::MAX_RETRIES:
        {
          const std::string execution_message = execution.getMessage();
          RCLCPP_ERROR(
            rclcpp::get_logger(
              name_),
            "[PlannerAction] MAX_RETRIES: start=(%.3f, %.3f) goal=(%.3f, %.3f) tolerance=%.3fm | %s",
            start_pose.pose.position.x, start_pose.pose.position.y,
            goal.goal.pose.position.x, goal.goal.pose.position.y,
            tolerance, execution_message.c_str());
          result->outcome = execution.getOutcome();
          result->message = execution_message;
          goal_handle->abort(result);
          planner_active = false;
          break;
        }

      case PlannerExecution::PAT_EXCEEDED:
        RCLCPP_ERROR(
          rclcpp::get_logger(name_),
          "[PlannerAction] PAT_EXCEEDED: start=(%.3f, %.3f) goal=(%.3f, %.3f) tolerance=%.3fm",
          start_pose.pose.position.x, start_pose.pose.position.y,
          goal.goal.pose.position.x, goal.goal.pose.position.y,
          tolerance);
        result->outcome = ActionToPose::Result::PAT_EXCEEDED;
        result->message = "Global planner exceeded the patience time";
        goal_handle->abort(result);
        planner_active = false;
        break;

      case PlannerExecution::INTERNAL_ERROR:
        {
          const std::string execution_message = execution.getMessage();
          RCLCPP_FATAL(
            rclcpp::get_logger(name_),
            "[PlannerAction] INTERNAL_ERROR: start=(%.3f, %.3f) goal=(%.3f, %.3f) | %s",
            start_pose.pose.position.x, start_pose.pose.position.y,
            goal.goal.pose.position.x, goal.goal.pose.position.y,
            execution_message.empty() ? "unknown error" : execution_message.c_str());
          result->outcome = ActionToPose::Result::INTERNAL_ERROR;
          result->message = "Internal planner error";
          planner_active = false;
          goal_handle->abort(result);
          break;
        }

      default:
        std::ostringstream ss;
        ss << "Internal error: Unknown state in a move base flex planner "
          "execution with the number: "
           << static_cast<int>(state_planning_input);
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(name_), ss.str());
        result->outcome = ActionToPose::Result::INTERNAL_ERROR;
        result->message = ss.str();
        goal_handle->abort(result);
        planner_active = false;
    }

    if (planner_active) {
      // try to sleep a bit
      // normally this thread should be woken up from the planner execution
      // thread in order to transfer the results to the controller.
      execution.waitForStateUpdate(std::chrono::milliseconds(500));
    }
  }  // while (planner_active && ros::ok())

  if (!planner_active) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger(name_),
      "\"" << name_ << "\" action ended properly.");
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(name_),
      "\"" << name_ << "\" action has been stopped!");
  }
}

/**
 * Transform path from local planner frame to global frame
 *
 * **Purpose:**
 * The global planner (e.g., NavFN) typically generates paths in the costmap
 * frame, which may differ from the global navigation frame. This method
 * converts each pose in the path to the global frame to ensure consistency with
 * the rest of the navigation stack.
 *
 * **Coordinate Transformation:**
 * Each path pose is transformed from its source frame (usually costmap_frame)
 * to the global frame using TF2 transforms. This is critical because:
 * - Costmap may be offset/rotated relative to global frame
 * - Navigation controllers expect paths in global frame
 * - RViz visualization requires consistent coordinate frames
 *
 * **Algorithm:**
 * 1. Clear and reserve space in output vector
 * 2. For each pose in input plan:
 *    a. Call transformPose utility to convert to global frame
 *    b. Use robot_info transform listener and timeout settings
 *    c. Collect transformed pose
 * 3. Return false immediately if any transformation fails
 * 4. Return true if all poses transformed successfully
 *
 * **Error Handling:**
 * Transformation can fail due to:
 * - Missing transform from source to global frame
 * - Transform timeout exceeded
 * - Invalid pose timestamps or frame IDs
 * - TF tree discontinuities
 *
 * **Thread Context:** Execution thread (inherited from runImpl)
 *
 * **Performance:**
 * - O(n) where n = number of poses in plan
 * - Uses reserve() to pre-allocate space and avoid reallocations
 * - Early exit on first transformation failure
 *
 * @param plan Input path with poses in local planner frame
 * @param global_plan Output path with poses in global frame
 * @return true if all poses transformed successfully, false if any transform
 * failed
 *
 * **Implementation Notes:**
 * - Uses utility function navflex_utility::transformPose for thread-safe
 * transformation
 * - Respects robot_info_->getTfTimeout() for transform wait times
 * - Logs the source and destination frames in error messages for debugging
 */
bool PlannerAction::transformPlanToGlobalFrame(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  std::vector<geometry_msgs::msg::PoseStamped> & global_plan)
{
  global_plan.clear();
  global_plan.reserve(plan.size());
  std::vector<geometry_msgs::msg::PoseStamped>::const_iterator iter;
  bool tf_success = false;
  for (iter = plan.begin(); iter != plan.end(); ++iter) {
    geometry_msgs::msg::PoseStamped global_pose;
    tf_success = navflex_utility::transformPose(
      node_, robot_info_->getTransformListener(),
      robot_info_->getGlobalFrame(), robot_info_->getTfTimeout(), *iter,
      global_pose);
    if (!tf_success) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(name_),
        "Can not transform pose from the \""
          << iter->header.frame_id << "\" frame into the \""
          << robot_info_->getGlobalFrame() << "\" frame !");
      return false;
    }
    global_plan.push_back(global_pose);
  }
  return true;
}

} /* namespace navflex_costmap_nav */
