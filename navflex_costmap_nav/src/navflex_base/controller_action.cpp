#include "navflex_base/controller_action.h"

#include <sstream>
#include <cmath>

#include "navflex_utility/navigation_utility.h"

namespace navflex_costmap_nav
{

ControllerAction::ControllerAction(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const navflex_utility::RobotInformation::ConstPtr & robot_info)
: NavflexActionBase(node, name, robot_info)
{
  if (!node_->has_parameter("controller_stuck_timeout")) {
    node_->declare_parameter(
      "controller_stuck_timeout",
      rclcpp::ParameterValue(60.0));
  }
  if (!node_->has_parameter("controller_stuck_distance")) {
    node_->declare_parameter(
      "controller_stuck_distance",
      rclcpp::ParameterValue(0.5));
  }
}

void ControllerAction::start(
  const GoalHandlePtr & goal_handle,
  ControllerExecution::Ptr execution_ptr)
{
  RCLCPP_DEBUG(
    node_->get_logger(), "[ControllerAction] start() called for new goal_handle %p",
    goal_handle.get());
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "start() called with null goal_handle");
    return;
  }

  if (goal_handle->is_canceling()) {
    RCLCPP_WARN(
      node_->get_logger(), "[ControllerAction] start() called but goal_handle is canceling");
    auto result = std::make_shared<ActionFollowPath::Result>();
    result->outcome = ActionFollowPath::Result::CANCELED;
    result->message = "Goal canceled before controller start";
    goal_handle->canceled(result);
    return;
  }

  // navflex_costmap_nav only supports slot 0 (no concurrency_slot in FollowPath)
  constexpr SlotId slot_id = 0;
  bool update_plan = false;
  {
    std::lock_guard<std::mutex> map_guard(slot_map_mtx_);
    auto slot_it = concurrency_slots_.find(slot_id);
    if (slot_it != concurrency_slots_.end() && slot_it->second.in_use) {
      // goal_mtx_ is locked ONCE here; do NOT lock it again inside this block.
      std::lock_guard<std::mutex> gp_guard(goal_mtx_);
      auto & active_handle = slot_it->second.goal_handle;
      if (active_handle && active_handle->is_active()) {
        const bool same_controller =
          slot_it->second.execution && execution_ptr &&
          (slot_it->second.execution->getName() == execution_ptr->getName());
        const bool same_goal_tolerance =
          slot_it->second.execution && execution_ptr &&
          slot_it->second.execution->getXYGoalTolerance() ==
          execution_ptr->getXYGoalTolerance() &&
          slot_it->second.execution->getYawGoalTolerance() ==
          execution_ptr->getYawGoalTolerance();

        if (same_controller && same_goal_tolerance) {
          RCLCPP_INFO(
            rclcpp::get_logger(name_),
            "[FollowPath] path update accepted: controller=%s poses=%zu",
            execution_ptr->getName().c_str(),
            goal_handle->get_goal()->path.poses.size());
          update_plan = true;
          // Update plan in the running execution.
          slot_it->second.execution->setNewPlan(goal_handle->get_goal()->path);
          // goal_mtx_ is already held – assign directly, no nested lock needed.
          const auto & poses = goal_handle->get_goal()->path.poses;
          goal_pose_ = poses.empty() ? geometry_msgs::msg::PoseStamped() : poses.back();
          // Store new goal handle; runImpl() will pick it up each loop tick.
          pending_goal_handle_ = goal_handle;
          pending_preempted_goal_handle_ = active_handle;
          // Update the slot's goal_handle to the new handle NOW (while both
          // slot_map_mtx_ and goal_mtx_ are held).  This is safe because
          // runImpl() copies slot.goal_handle into a local `current_handle` at
          // the very top of the function and never reads the parameter again.
          // Without this update, slot.goal_handle remains the aborted old handle,
          // so the next replanning cycle sees is_active()==false, falls through
          // to NavflexActionBase::start(), calls cancel() on the running execution,
          // and publishes a zero-velocity command before doing a full restart.
          active_handle = goal_handle;
          RCLCPP_DEBUG(
            rclcpp::get_logger(name_),
            "[ControllerAction] Preempt queued: slot=%d new_goal_handle=%p",
            slot_id, goal_handle.get());
        } else {
          RCLCPP_WARN(
            rclcpp::get_logger(
              name_),
            "[ControllerAction] Preempt requires full restart in slot %d. same_controller=%s (running=%s, requested=%s), same_goal_tolerance=%s (running=%.3f/%.3f, requested=%.3f/%.3f)",
            slot_id, same_controller ? "true" : "false",
            slot_it->second.execution ? slot_it->second.execution->getName().c_str() : "<null>",
            execution_ptr ? execution_ptr->getName().c_str() : "<null>",
            same_goal_tolerance ? "true" : "false",
            slot_it->second.execution ? slot_it->second.execution->getXYGoalTolerance() : 0.0,
            slot_it->second.execution ? slot_it->second.execution->getYawGoalTolerance() : 0.0,
            execution_ptr ? execution_ptr->getXYGoalTolerance() : 0.0,
            execution_ptr ? execution_ptr->getYawGoalTolerance() : 0.0);
        }
      }
    }
  }  // slot_map_mtx_ and goal_mtx_ released before abort()

  if (!update_plan) {
    // Otherwise run parent version of this method.
    NavflexActionBase::start(goal_handle, execution_ptr);
  }
}

void ControllerAction::runImpl(
  const GoalHandlePtr & goal_handle,
  ControllerExecution & execution)
{
  // Use a local copy of the goal handle so we never hold a reference into the
  // slot's shared_ptr while start() may reassign it from another thread.
  // In-place preempts are communicated through pending_goal_handle_ (under
  // goal_mtx_) and consumed at the top of each loop tick.
  GoalHandlePtr current_handle = goal_handle;
  RCLCPP_DEBUG(
    rclcpp::get_logger(
      name_), "[ControllerAction] runImpl() started for goal_handle %p", current_handle.get());

  auto build_result = [&](uint32_t fallback_outcome,
      const std::string & fallback_message) {
      auto result = std::make_shared<ActionFollowPath::Result>();
      geometry_msgs::msg::PoseStamped robot_pose;
      geometry_msgs::msg::PoseStamped goal_pose;
      {
        std::lock_guard<std::mutex> gp_guard(goal_mtx_);
        robot_pose = robot_pose_;
        goal_pose = goal_pose_;
      }

      const uint32_t execution_outcome = execution.getOutcome();
      const std::string execution_message = execution.getMessage();
      result->outcome = (execution_outcome == ActionFollowPath::Result::SUCCESS &&
        fallback_outcome != ActionFollowPath::Result::SUCCESS) ?
        fallback_outcome :
        execution_outcome;

      if (execution_message.empty()) {
        result->message = fallback_message;
      } else {
        result->message = execution_message;
      }

      result->final_pose = robot_pose;
      result->dist_to_goal = static_cast<float>(navflex_utility::distance(robot_pose, goal_pose));
      result->angle_to_goal = static_cast<float>(navflex_utility::angle(robot_pose, goal_pose));
      return result;
    };

  const auto & initial_path = current_handle->get_goal()->path;

  {
    std::lock_guard<std::mutex> gp_guard(goal_mtx_);
    robot_pose_ = geometry_msgs::msg::PoseStamped();
    goal_pose_ = initial_path.poses.empty() ? geometry_msgs::msg::PoseStamped() :
      initial_path.poses.back();
  }

  if (initial_path.poses.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        name_), "[ControllerAction] Controller started with an empty plan! goal_handle=%p",
      current_handle.get());
    auto result = build_result(
      ActionFollowPath::Result::INVALID_PATH,
      "Controller started with an empty path");
    current_handle->abort(result);
    RCLCPP_WARN(
      rclcpp::get_logger(
        name_), "[ControllerAction] runImpl() aborting due to empty plan for goal_handle %p",
      current_handle.get());
    return;
  }

  bool controller_active = true;
  double stuck_timeout_sec = 60.0;
  double stuck_distance_m = 0.5;
  node_->get_parameter("controller_stuck_timeout", stuck_timeout_sec);
  node_->get_parameter("controller_stuck_distance", stuck_distance_m);
  const bool stuck_detection_enabled =
    stuck_timeout_sec > 0.0 && stuck_distance_m >= 0.0;
  bool stuck_window_initialized = false;
  rclcpp::Time stuck_window_start;
  geometry_msgs::msg::PoseStamped stuck_window_pose;

  auto reset_stuck_window = [&](const geometry_msgs::msg::PoseStamped & pose,
      const std::string & reason) {
      if (!stuck_detection_enabled) {
        return;
      }
      stuck_window_initialized = true;
      stuck_window_start = node_->now();
      stuck_window_pose = pose;
      RCLCPP_DEBUG(
        rclcpp::get_logger(
          name_),
        "[ControllerAction] Reset stuck detection window (%s): pose=(%.3f, %.3f), timeout=%.2fs, distance=%.3fm",
        reason.c_str(), pose.pose.position.x, pose.pose.position.y,
        stuck_timeout_sec, stuck_distance_m);
    };

  auto check_stuck = [&](const geometry_msgs::msg::PoseStamped & pose,
      std::string & stuck_message) {
      if (!stuck_detection_enabled) {
        return false;
      }
      if (!stuck_window_initialized) {
        reset_stuck_window(pose, "initial pose");
        return false;
      }

      const rclcpp::Time now = node_->now();
      const rclcpp::Duration elapsed = now - stuck_window_start;
      if (elapsed.nanoseconds() < 0) {
        reset_stuck_window(pose, "time moved backwards");
        return false;
      }

      const double moved = navflex_utility::distance(pose, stuck_window_pose);
      if (moved >= stuck_distance_m) {
        reset_stuck_window(pose, "progress made");
        return false;
      }

      if (elapsed.seconds() >= stuck_timeout_sec) {
        std::ostringstream msg;
        msg << "Controller detected robot stuck: moved " << moved
            << " m in " << elapsed.seconds() << " s"
            << " (threshold: < " << stuck_distance_m << " m within "
            << stuck_timeout_sec << " s)";
        stuck_message = msg.str();
        return true;
      }
      return false;
    };

  if (stuck_detection_enabled) {
    RCLCPP_INFO(
      rclcpp::get_logger(name_),
      "[ControllerAction] Stuck detection enabled: timeout=%.2fs distance=%.3fm",
      stuck_timeout_sec, stuck_distance_m);
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(name_),
      "[ControllerAction] Stuck detection disabled: timeout=%.2fs distance=%.3fm",
      stuck_timeout_sec, stuck_distance_m);
  }

  while (controller_active && rclcpp::ok()) {
    // Consume any pending preempt goal handle (set by start() under goal_mtx_).
    // This is the only thread-safe way to switch handles mid-execution.
    GoalHandlePtr preempted_handle_to_abort;
    {
      std::lock_guard<std::mutex> gp_guard(goal_mtx_);
      if (pending_goal_handle_) {
        RCLCPP_DEBUG(
          rclcpp::get_logger(name_),
          "[ControllerAction] runImpl() switching to preempted goal_handle %p",
          pending_goal_handle_.get());
        current_handle = pending_goal_handle_;
        pending_goal_handle_.reset();
        preempted_handle_to_abort = pending_preempted_goal_handle_;
        pending_preempted_goal_handle_.reset();
      }
    }
    if (preempted_handle_to_abort && preempted_handle_to_abort->is_active()) {
      RCLCPP_DEBUG(
        rclcpp::get_logger(name_),
        "[ControllerAction] Aborting old goal_handle %p due to preempt",
        preempted_handle_to_abort.get());
      auto result = std::make_shared<ActionFollowPath::Result>();
      result->outcome = ActionFollowPath::Result::CANCELED;
      result->message = "Preempted by a newer FollowPath goal";
      preempted_handle_to_abort->abort(result);
    }

    // Update robot pose
    geometry_msgs::msg::PoseStamped current_pose;
    bool current_pose_valid = false;
    if (robot_info_ && robot_info_->getRobotPose(current_pose)) {
      std::lock_guard<std::mutex> gp_guard(goal_mtx_);
      robot_pose_ = current_pose;
      current_pose_valid = true;
    }

    // Check for cancel
    if (current_handle->is_canceling()) {
      RCLCPP_WARN(
        rclcpp::get_logger(
          name_), "[ControllerAction] runImpl() detected cancel for goal_handle %p",
        current_handle.get());
      execution.cancel();
      auto result = build_result(
        ActionFollowPath::Result::CANCELED,
        "FollowPath goal canceled");
      result->outcome = ActionFollowPath::Result::CANCELED;
      current_handle->canceled(result);
      RCLCPP_INFO(
        rclcpp::get_logger(
          name_), "[ControllerAction] runImpl() canceled and exited for goal_handle %p",
        current_handle.get());
      return;
    }

    if (current_pose_valid) {
      std::string stuck_message;
      if (check_stuck(current_pose, stuck_message)) {
        RCLCPP_ERROR(
          rclcpp::get_logger(name_),
          "[ControllerAction] %s goal_handle=%p",
          stuck_message.c_str(), current_handle.get());
        execution.cancel();
        auto result = build_result(
          ActionFollowPath::Result::ROBOT_STUCK,
          stuck_message);
        result->outcome = ActionFollowPath::Result::ROBOT_STUCK;
        result->message = stuck_message;
        current_handle->abort(result);
        return;
      }
    }

    const auto state = execution.getState();

    switch (state) {
      case ControllerExecution::INITIALIZED:
        RCLCPP_DEBUG(
          rclcpp::get_logger(
            name_), "[ControllerAction] State: INITIALIZED for goal_handle %p",
          current_handle.get());
        // Do not override a newer preempted plan that may have already been queued.
        if (!execution.hasNewPlan()) {
          execution.setNewPlan(current_handle->get_goal()->path);
        }
        execution.start();
        break;

      case ControllerExecution::STARTED:
        RCLCPP_DEBUG(
          rclcpp::get_logger(
            name_), "[ControllerAction] State: STARTED for goal_handle %p", current_handle.get());
        break;

      case ControllerExecution::STOPPED: {
          RCLCPP_WARN(
            rclcpp::get_logger(
              name_), "[ControllerAction] State: STOPPED for goal_handle %p. Controller stopped rigorously!",
            current_handle.get());
          auto result = build_result(
            ActionFollowPath::Result::STOPPED,
            "Controller stopped");
          current_handle->abort(result);
          controller_active = false;
          break;
        }

      case ControllerExecution::CANCELED: {
          RCLCPP_INFO(
            rclcpp::get_logger(
              name_), "[ControllerAction] State: CANCELED for goal_handle %p. Controller execution canceled.",
            current_handle.get());
          auto result = build_result(
            ActionFollowPath::Result::CANCELED,
            "Controller execution canceled");
          result->outcome = ActionFollowPath::Result::CANCELED;
          current_handle->abort(result);
          controller_active = false;
          break;
        }

      case ControllerExecution::PLANNING:
        RCLCPP_DEBUG(
          rclcpp::get_logger(
            name_), "[ControllerAction] State: PLANNING for goal_handle %p", current_handle.get());
        if (execution.isPatienceExceeded()) {
          RCLCPP_INFO(
            rclcpp::get_logger(
              name_), "[ControllerAction] Controller patience exceeded, canceling for goal_handle %p",
            current_handle.get());
          execution.cancel();
        }
        break;

      case ControllerExecution::MAX_RETRIES: {
          RCLCPP_WARN(
            rclcpp::get_logger(
              name_), "[ControllerAction] State: MAX_RETRIES for goal_handle %p. Controller exceeded max retries.",
            current_handle.get());
          auto result = build_result(
            ActionFollowPath::Result::NO_VALID_CMD,
            "Controller exceeded max retries");
          const std::string execution_message = execution.getMessage();
          RCLCPP_ERROR(
            rclcpp::get_logger(name_), "[ControllerAction] Abort reason: %s",
            execution_message.empty() ? "Controller exceeded max retries." : execution_message.c_str());
          current_handle->abort(result);
          controller_active = false;
          break;
        }

      case ControllerExecution::PAT_EXCEEDED: {
          RCLCPP_WARN(
            rclcpp::get_logger(
              name_), "[ControllerAction] State: PAT_EXCEEDED for goal_handle %p. Controller patience exceeded.",
            current_handle.get());
          auto result = build_result(
            ActionFollowPath::Result::PAT_EXCEEDED,
            "Controller patience exceeded");
          current_handle->abort(result);
          controller_active = false;
          break;
        }

      case ControllerExecution::NO_PLAN:
      case ControllerExecution::EMPTY_PLAN:
      case ControllerExecution::INVALID_PLAN: {
          RCLCPP_WARN(
            rclcpp::get_logger(
              name_), "[ControllerAction] State: INVALID_PLAN/NO_PLAN/EMPTY_PLAN for goal_handle %p. Controller state: %d",
            current_handle.get(), static_cast<int>(state));
          auto result = build_result(
            ActionFollowPath::Result::INVALID_PATH,
            "Controller received invalid path");
          current_handle->abort(result);
          controller_active = false;
          break;
        }

      case ControllerExecution::NO_LOCAL_CMD: {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger(name_),
            *node_->get_clock(), 3000,
            "[ControllerAction] State: NO_LOCAL_CMD for goal_handle %p. No velocity command from controller.",
            current_handle.get());
          if (!execution.isMoving()) {
            RCLCPP_ERROR(
              rclcpp::get_logger(
                name_), "[ControllerAction] State: NO_LOCAL_CMD and not moving, aborting goal_handle %p",
              current_handle.get());
            auto result = build_result(
              ActionFollowPath::Result::NO_VALID_CMD,
              "No valid velocity command from controller");
            const std::string execution_message = execution.getMessage();
            RCLCPP_ERROR(
              rclcpp::get_logger(name_), "[ControllerAction] Abort reason: %s",
              execution_message.empty() ? "No velocity command from controller." : execution_message.c_str());
            current_handle->abort(result);
            controller_active = false;
          } else {
            publishFeedback(
              *current_handle, execution.getVelocityCmd(),
              ActionFollowPath::Result::NO_VALID_CMD,
              "No valid velocity command this cycle (still moving)");
          }
          break;
        }

      case ControllerExecution::GOT_LOCAL_CMD:
        RCLCPP_DEBUG(
          rclcpp::get_logger(
            name_), "[ControllerAction] State: GOT_LOCAL_CMD for goal_handle %p",
          current_handle.get());
        publishFeedback(
          *current_handle, execution.getVelocityCmd(),
          ActionFollowPath::Result::SUCCESS,
          execution.getMessage());
        break;

      case ControllerExecution::ARRIVED_GOAL: {
          RCLCPP_INFO(
            rclcpp::get_logger(
              name_), "[ControllerAction] State: ARRIVED_GOAL for goal_handle %p. Controller arrived at goal.",
            current_handle.get());
          auto result = build_result(
            ActionFollowPath::Result::SUCCESS,
            "Goal reached");
          result->outcome = ActionFollowPath::Result::SUCCESS;
          current_handle->succeed(result);
          controller_active = false;
          break;
        }

      case ControllerExecution::INTERNAL_ERROR: {
          RCLCPP_ERROR(
            rclcpp::get_logger(
              name_), "[ControllerAction] State: INTERNAL_ERROR for goal_handle %p. Internal controller error!",
            current_handle.get());
          auto result = build_result(
            ActionFollowPath::Result::INTERNAL_ERROR,
            "Internal controller error");
          current_handle->abort(result);
          controller_active = false;
          break;
        }

      default: {
          RCLCPP_ERROR(
            rclcpp::get_logger(
              name_),
            "[ControllerAction] State: UNKNOWN (%d) for goal_handle %p", static_cast<int>(state),
            current_handle.get());
          auto result = build_result(
            ActionFollowPath::Result::INTERNAL_ERROR,
            "Unknown controller state");
          current_handle->abort(result);
          controller_active = false;
          break;
        }
    }

    if (controller_active) {
      RCLCPP_DEBUG(
        rclcpp::get_logger(
          name_), "[ControllerAction] Waiting for state update for goal_handle %p",
        current_handle.get());
      execution.waitForStateUpdate(std::chrono::milliseconds(50));
    } else {
      RCLCPP_DEBUG(
        rclcpp::get_logger(
          name_), "[ControllerAction] Controller loop ended for goal_handle %p",
        current_handle.get());
    }
  }

  {
    std::lock_guard<std::mutex> gp_guard(goal_mtx_);
    if (pending_goal_handle_ == current_handle) {
      pending_goal_handle_.reset();
    }
    if (pending_preempted_goal_handle_ == current_handle) {
      pending_preempted_goal_handle_.reset();
    }
  }

  if (current_handle && current_handle->is_canceling()) {
    auto result = build_result(
      ActionFollowPath::Result::CANCELED,
      "FollowPath goal canceled while controller loop exited");
    result->outcome = ActionFollowPath::Result::CANCELED;
    current_handle->canceled(result);
  } else if (current_handle && current_handle->is_active()) {
    auto result = build_result(
      ActionFollowPath::Result::INTERNAL_ERROR,
      "FollowPath controller loop exited without terminal state");
    result->outcome = ActionFollowPath::Result::INTERNAL_ERROR;
    current_handle->abort(result);
  }
}

void ControllerAction::publishFeedback(
  GoalHandle & goal_handle,
  const geometry_msgs::msg::TwistStamped & cmd_vel,
  uint32_t outcome,
  const std::string & message)
{
  auto feedback = std::make_shared<ActionFollowPath::Feedback>();
  {
    std::lock_guard<std::mutex> gp_guard(goal_mtx_);
    feedback->distance_to_goal =
      static_cast<float>(navflex_utility::distance(robot_pose_, goal_pose_));
    feedback->angle_to_goal =
      static_cast<float>(navflex_utility::angle(robot_pose_, goal_pose_));
    feedback->current_pose = robot_pose_;
  }
  feedback->speed = static_cast<float>(
    std::hypot(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y));
  feedback->last_cmd_vel = cmd_vel;
  feedback->outcome = outcome;
  feedback->message = message;
  goal_handle.publish_feedback(feedback);
}

}  // namespace navflex_costmap_nav
