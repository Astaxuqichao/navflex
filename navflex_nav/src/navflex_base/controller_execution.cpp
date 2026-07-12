#include "navflex_base/controller_execution.h"

#include "nav2_msgs/action/follow_path.hpp"

namespace navflex_nav
{

static const double DEFAULT_CONTROLLER_FREQUENCY = 20.0;
using ActionFollowPath = nav2_msgs::action::FollowPath;

ControllerExecution::ControllerExecution(
  const std::string & name,
  const ControllerPluginAdapter::Ptr & controller_ptr,
  const nav2_core::GoalChecker::Ptr & goal_checker,
  double xy_goal_tolerance,
  double yaw_goal_tolerance,
  const navflex_utility::RobotInformation::ConstPtr & robot_info,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr & vel_pub,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & current_goal_pub)
: NavflexExecutionBase(name, robot_info, node),
  controller_(controller_ptr),
  goal_checker_(goal_checker),
  vel_pub_(vel_pub),
  current_goal_pub_(current_goal_pub),
  new_plan_(false),
  state_(INITIALIZED),
  moving_(false),
  patience_(0, 0),
  max_retries_(-1),
  required_consecutive_valid_cmds_(10),
  consecutive_valid_cmds_(0),
  xy_goal_tolerance_(xy_goal_tolerance),
  yaw_goal_tolerance_(yaw_goal_tolerance),
  frequency_(DEFAULT_CONTROLLER_FREQUENCY),
  node_handle_(node)
{
  if (!node_handle_->has_parameter("controller_frequency")) {
    node_handle_->declare_parameter(
      "controller_frequency",
      rclcpp::ParameterValue(DEFAULT_CONTROLLER_FREQUENCY));
  }
  if (!node_handle_->has_parameter("controller_patience")) {
    node_handle_->declare_parameter(
      "controller_patience",
      rclcpp::ParameterValue(5.0));
  }
  if (!node_handle_->has_parameter("controller_max_retries")) {
    node_handle_->declare_parameter(
      "controller_max_retries",
      rclcpp::ParameterValue(-1));
  }
  if (!node_handle_->has_parameter("controller_required_consecutive_valid_cmds")) {
    node_handle_->declare_parameter(
      "controller_required_consecutive_valid_cmds",
      rclcpp::ParameterValue(10));
  }
  if (!node_handle_->has_parameter("robot_frame")) {
    node_handle_->declare_parameter(
      "robot_frame",
      rclcpp::ParameterValue(std::string("base_link")));
  }
  if (!node_handle_->has_parameter("map_frame")) {
    node_handle_->declare_parameter(
      "map_frame",
      rclcpp::ParameterValue(std::string("map")));
  }

  node_handle_->get_parameter("robot_frame", robot_frame_);
  node_handle_->get_parameter("map_frame", global_frame_);

  double frequency;
  node_handle_->get_parameter("controller_frequency", frequency);
  setControllerFrequency(frequency);

  double patience;
  node_handle_->get_parameter("controller_patience", patience);
  patience_ = rclcpp::Duration::from_seconds(patience);
  node_handle_->get_parameter("controller_max_retries", max_retries_);
  int required_consecutive_valid_cmds;
  node_handle_->get_parameter(
    "controller_required_consecutive_valid_cmds",
    required_consecutive_valid_cmds);
  setRequiredConsecutiveValidCmds(required_consecutive_valid_cmds);

  RCLCPP_DEBUG(
    node_handle_->get_logger(), "[ControllerExecution] Constructed for %s",
    name_.c_str());
  dyn_params_handler_ = node_handle_->add_on_set_parameters_callback(
    std::bind(&ControllerExecution::reconfigure, this, std::placeholders::_1));
}

ControllerExecution::~ControllerExecution() {}

bool ControllerExecution::setControllerFrequency(double frequency)
{
  if (frequency <= 0.0) {
    RCLCPP_ERROR(
      node_handle_->get_logger(),
      "Controller frequency must be > 0.0! Keeping current value.");
    return false;
  }
  frequency_ = frequency;
  loop_rate_ = std::make_shared<rclcpp::Rate>(frequency);
  return true;
}

bool ControllerExecution::setRequiredConsecutiveValidCmds(int count)
{
  if (count < 1) {
    RCLCPP_ERROR(
      node_handle_->get_logger(),
      "controller_required_consecutive_valid_cmds must be >= 1! Keeping current value.");
    return false;
  }
  required_consecutive_valid_cmds_ = count;
  consecutive_valid_cmds_ = 0;
  return true;
}

rcl_interfaces::msg::SetParametersResult ControllerExecution::reconfigure(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> guard(configuration_mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto & param : parameters) {
    if (param.get_name() == "controller_frequency") {
      setControllerFrequency(param.as_double());
    } else if (param.get_name() == "controller_patience") {
      patience_ = rclcpp::Duration::from_seconds(param.as_double());
    } else if (param.get_name() == "controller_max_retries") {
      max_retries_ = param.as_int();
    } else if (param.get_name() == "controller_required_consecutive_valid_cmds") {
      setRequiredConsecutiveValidCmds(param.as_int());
    }
  }
  result.successful = true;
  return result;
}

bool ControllerExecution::start()
{
  if (moving_.load()) {
    RCLCPP_WARN(
      node_handle_->get_logger(), "[ControllerExecution] start() called but already moving for %s",
      name_.c_str());
    return false;  // already running
  }
  moving_.store(true);
  setState(STARTED);
  RCLCPP_DEBUG(
    node_handle_->get_logger(), "[ControllerExecution] start() called, starting thread for %s",
    name_.c_str());
  bool started = NavflexExecutionBase::start();
  if (!started) {
    RCLCPP_ERROR(
      node_handle_->get_logger(), "[ControllerExecution] NavflexExecutionBase::start() failed for %s",
      name_.c_str());
    moving_.store(false);
  }
  return started;
}

void ControllerExecution::setState(ControllerState state)
{
  {
    std::lock_guard<std::mutex> guard(state_mtx_);
    if (state_ == state) {
      return;
    }
    state_ = state;
  }
  condition_.notify_all();
}

ControllerExecution::ControllerState ControllerExecution::getState() const
{
  std::lock_guard<std::mutex> guard(state_mtx_);
  return state_;
}

uint32_t ControllerExecution::computeVelocityCmd(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::TwistStamped & robot_velocity,
  geometry_msgs::msg::TwistStamped & vel_cmd,
  std::string & message)
{
  try {
    return controller_->computeVelocityCommands(
      robot_pose, robot_velocity.twist, vel_cmd, goal_checker_.get(), message);
  } catch (const std::exception & e) {
    message = e.what();
    RCLCPP_ERROR(
      node_handle_->get_logger(),
      "[ControllerExecution] computeVelocityCommands threw exception for %s: %s",
      name_.c_str(), e.what());
    return ActionFollowPath::Result::INVALID_PATH;
  }
}

void ControllerExecution::setNewPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> guard(plan_mtx_);
  new_plan_ = true;
  plan_ = path;

  if (!path.poses.empty()) {
    const auto & start = path.poses.front().pose.position;
    const auto & end = path.poses.back().pose.position;
    RCLCPP_DEBUG(
      node_handle_->get_logger(),
      "[ControllerExecution] Queued new plan for %s: frame='%s', poses=%zu, start=(%.3f, %.3f), end=(%.3f, %.3f)",
      name_.c_str(), path.header.frame_id.c_str(), path.poses.size(),
      start.x, start.y, end.x, end.y);
  } else {
    RCLCPP_WARN(
      node_handle_->get_logger(),
      "[ControllerExecution] Queued empty plan for %s",
      name_.c_str());
  }
}

bool ControllerExecution::hasNewPlan()
{
  std::lock_guard<std::mutex> guard(plan_mtx_);
  return new_plan_;
}

nav_msgs::msg::Path ControllerExecution::getNewPlan()
{
  std::lock_guard<std::mutex> guard(plan_mtx_);
  new_plan_ = false;
  return plan_;
}

geometry_msgs::msg::TwistStamped ControllerExecution::getVelocityCmd() const
{
  std::lock_guard<std::mutex> guard(vel_cmd_mtx_);
  return vel_cmd_stamped_;
}

bool ControllerExecution::isMoving() const
{
  return moving_.load();
}

bool ControllerExecution::isPatienceExceeded() const
{
  std::lock_guard<std::mutex> guard(lct_mtx_);
  if (!(patience_ == rclcpp::Duration(0, 0)) &&
    node_handle_->now() - start_time_ > patience_)
  {
    if (node_handle_->now() - last_call_time_ > patience_) {
      RCLCPP_WARN_THROTTLE(
        node_handle_->get_logger(),
        *node_handle_->get_clock(), 1000,
        "Controller \"%s\" needs more time than patience allows!",
        name_.c_str());
      return true;
    }
    if (node_handle_->now() - last_valid_cmd_time_ > patience_) {
      RCLCPP_DEBUG(
        node_handle_->get_logger(),
        "Controller \"%s\" has not returned a success in more "
        "than the patience time!", name_.c_str());
      return true;
    }
  }
  return false;
}

bool ControllerExecution::cancel()
{
  cancel_ = true;
  condition_.notify_all();
  if (waitForStateUpdate(std::chrono::milliseconds(50)) ==
    std::cv_status::timeout)
  {
    RCLCPP_DEBUG(
      node_handle_->get_logger(),
      "Timeout waiting for controller cancel to take effect.");
  }
  return true;
}

void ControllerExecution::publishZeroVelocity()
{
  geometry_msgs::msg::Twist zero;
  if (vel_pub_) {
    vel_pub_->publish(zero);
  }
}

void ControllerExecution::run()
{
  RCLCPP_DEBUG(
    node_handle_->get_logger(), "[ControllerExecution] run() thread started for %s",
    name_.c_str());
  start_time_ = node_handle_->now();

  nav_msgs::msg::Path path;
  if (!hasNewPlan()) {
    setOutcomeAndMessage(
      ActionFollowPath::Result::INVALID_PATH,
      "Controller started without a plan");
    setState(NO_PLAN);
    moving_.store(false);
    RCLCPP_ERROR(
      node_handle_->get_logger(), "[ControllerExecution] started without a plan! %s",
      name_.c_str());
    condition_.notify_all();
    RCLCPP_INFO(
      node_handle_->get_logger(), "[ControllerExecution] run() thread exiting (no plan) for %s",
      name_.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> guard(lct_mtx_);
    last_valid_cmd_time_ = node_handle_->now();
    last_call_time_ = node_handle_->now();
  }
  {
    std::lock_guard<std::mutex> guard(configuration_mutex_);
    consecutive_valid_cmds_ = 0;
  }

  int retries = 0;

  while (moving_.load() && rclcpp::ok()) {
    // Check for forced stop
    {
      std::unique_lock<std::mutex> lock(should_exit_mutex_);
      if (should_exit_) {
        RCLCPP_INFO(
          node_handle_->get_logger(), "[ControllerExecution] Forced stop detected for %s",
          name_.c_str());
        publishZeroVelocity();
        setState(STOPPED);
        condition_.notify_all();
        moving_.store(false);
        RCLCPP_INFO(
          node_handle_->get_logger(), "[ControllerExecution] run() thread exiting (forced stop) for %s",
          name_.c_str());
        return;
      }
    }

    // Check cancel flag
    if (cancel_) {
      RCLCPP_INFO(
        node_handle_->get_logger(), "[ControllerExecution] Cancel detected for %s",
        name_.c_str());
      publishZeroVelocity();
      setState(CANCELED);
      moving_.store(false);
      condition_.notify_all();
      RCLCPP_INFO(
        node_handle_->get_logger(), "[ControllerExecution] run() thread exiting (cancel) for %s",
        name_.c_str());
      return;
    }

    // Update plan if a new one has arrived
    if (hasNewPlan()) {
      path = getNewPlan();
      RCLCPP_INFO(
        node_handle_->get_logger(),
        "[FollowPath] tracking path update: controller=%s poses=%zu frame=%s",
        name_.c_str(), path.poses.size(),
        path.header.frame_id.c_str());
      if (path.poses.empty()) {
        setOutcomeAndMessage(
          ActionFollowPath::Result::INVALID_PATH,
          "Controller received an empty plan");
        setState(EMPTY_PLAN);
        moving_.store(false);
        condition_.notify_all();
        RCLCPP_WARN(
          node_handle_->get_logger(), "[ControllerExecution] Received empty plan, exiting for %s",
          name_.c_str());
        return;
      }
      if (path.header.frame_id != global_frame_) {
        RCLCPP_WARN(
          node_handle_->get_logger(),
          "[ControllerExecution] Plan frame '%s' differs from costmap global frame '%s' for %s. Controller will rely on TF transforms.",
          path.header.frame_id.c_str(), global_frame_.c_str(),
          name_.c_str());
      }
      controller_->setPlan(path);
      if (current_goal_pub_) {
        current_goal_pub_->publish(path.poses.back());
      }
    }

    // Skip if no plan yet
    if (path.poses.empty()) {
      setOutcomeAndMessage(
        ActionFollowPath::Result::INVALID_PATH,
        "Controller has no plan to follow");
      setState(NO_PLAN);
      moving_.store(false);
      condition_.notify_all();
      RCLCPP_ERROR(
        node_handle_->get_logger(), "[ControllerExecution] No plan available, exiting for %s",
        name_.c_str());
      return;
    }

    // Get robot pose
    if (!robot_info_->getRobotPose(robot_pose_)) {
      setOutcomeAndMessage(
        ActionFollowPath::Result::TF_ERROR,
        "Could not get the robot pose");
      publishZeroVelocity();
      setState(INTERNAL_ERROR);
      moving_.store(false);
      condition_.notify_all();
      RCLCPP_ERROR(
        node_handle_->get_logger(), "[ControllerExecution] Could not get robot pose, exiting for %s",
        name_.c_str());
      return;
    }

    geometry_msgs::msg::TwistStamped robot_velocity;
    robot_info_->getRobotVelocity(robot_velocity);

    // Check goal reached through the per-action GoalChecker.
    if (goal_checker_ &&
      goal_checker_->isGoalReached(
        robot_pose_.pose, path.poses.back().pose, robot_velocity.twist))
    {
      RCLCPP_INFO(
        node_handle_->get_logger(), "[ControllerExecution] Goal reached for %s", name_.c_str());
      publishZeroVelocity();
      setState(ARRIVED_GOAL);
      moving_.store(false);
      condition_.notify_all();
      break;
    }

    setState(PLANNING);
    RCLCPP_DEBUG(
      node_handle_->get_logger(), "[ControllerExecution] State set to PLANNING for %s",
      name_.c_str());

    // Record call time
    {
      std::lock_guard<std::mutex> guard(lct_mtx_);
      last_call_time_ = node_handle_->now();
    }

    // Compute velocity through outcome/message interface.
    geometry_msgs::msg::TwistStamped cmd_vel_stamped;
    std::string compute_message;
    const uint32_t outcome = computeVelocityCmd(
      robot_pose_, robot_velocity, cmd_vel_stamped, compute_message);
    setOutcomeAndMessage(outcome, compute_message);

    if (cmd_vel_stamped.header.frame_id.empty()) {
      cmd_vel_stamped.header.frame_id = robot_frame_;
    }

    if (outcome < 10) {
      // Success path
      {
        std::lock_guard<std::mutex> guard(vel_cmd_mtx_);
        vel_cmd_stamped_ = cmd_vel_stamped;
        if (vel_cmd_stamped_.header.stamp.sec == 0 &&
          vel_cmd_stamped_.header.stamp.nanosec == 0)
        {
          vel_cmd_stamped_.header.stamp = node_handle_->now();
        }
      }
      vel_pub_->publish(cmd_vel_stamped.twist);
      RCLCPP_DEBUG(
        node_handle_->get_logger(), "[ControllerExecution] Published velocity (%.3f, %.3f, %.3f) for %s", cmd_vel_stamped.twist.linear.x, cmd_vel_stamped.twist.linear.y, cmd_vel_stamped.twist.angular.z,
        name_.c_str());
      bool refresh_patience = false;
      bool log_first_refresh = false;
      int required_consecutive_valid_cmds = 0;
      {
        std::lock_guard<std::mutex> guard(configuration_mutex_);
        ++consecutive_valid_cmds_;
        refresh_patience =
          consecutive_valid_cmds_ >= required_consecutive_valid_cmds_;
        log_first_refresh =
          consecutive_valid_cmds_ == required_consecutive_valid_cmds_;
        required_consecutive_valid_cmds = required_consecutive_valid_cmds_;
      }
      if (refresh_patience) {
        std::lock_guard<std::mutex> guard(lct_mtx_);
        last_valid_cmd_time_ = node_handle_->now();
      }
      if (log_first_refresh) {
        RCLCPP_DEBUG(
          node_handle_->get_logger(),
          "[ControllerExecution] Refreshed patience after %d consecutive valid commands for %s",
          required_consecutive_valid_cmds, name_.c_str());
      }
      retries = 0;
      setState(GOT_LOCAL_CMD);
      RCLCPP_DEBUG(
        node_handle_->get_logger(), "[ControllerExecution] State set to GOT_LOCAL_CMD for %s",
        name_.c_str());
    } else if (outcome == ActionFollowPath::Result::CANCELED) {
      RCLCPP_INFO(
        node_handle_->get_logger(),
        "[ControllerExecution] Controller-handled cancel completed for %s",
        name_.c_str());
      cancel_ = true;
      continue;
    } else {
      std::lock_guard<std::mutex> guard(configuration_mutex_);
      consecutive_valid_cmds_ = 0;
      if (max_retries_ >= 0 && ++retries > max_retries_) {
        setState(MAX_RETRIES);
        publishZeroVelocity();
        moving_.store(false);
        RCLCPP_WARN(
          node_handle_->get_logger(), "[ControllerExecution] Exceeded max retries, exiting for %s",
          name_.c_str());
      } else if (isPatienceExceeded()) {
        setOutcome(ActionFollowPath::Result::PAT_EXCEEDED);
        if (getMessage().empty()) {
          setMessage("Controller patience exceeded");
        }
        setState(PAT_EXCEEDED);
        publishZeroVelocity();
        moving_.store(false);
        RCLCPP_WARN(
          node_handle_->get_logger(), "[ControllerExecution] Patience exceeded, exiting for %s",
          name_.c_str());
      } else {
        setState(NO_LOCAL_CMD);
        publishZeroVelocity();
        RCLCPP_WARN_THROTTLE(
          node_handle_->get_logger(), *node_handle_->get_clock(), 1000,
          "[ControllerExecution] No local cmd, will retry for %s", name_.c_str());
      }
    }

    condition_.notify_all();

    if (moving_.load()) {
      if (loop_rate_ && !loop_rate_->sleep()) {
        RCLCPP_WARN_THROTTLE(
          node_handle_->get_logger(),
          *node_handle_->get_clock(), 1000,
          "[ControllerExecution] Loop missed desired rate %.4f Hz for %s!",
          frequency_, name_.c_str());
      }
      {
        std::unique_lock<std::mutex> lock(should_exit_mutex_);
        if (should_exit_) {
          RCLCPP_DEBUG(
            node_handle_->get_logger(), "[ControllerExecution] Forced stop detected (loop) for %s",
            name_.c_str());
          publishZeroVelocity();
          setState(STOPPED);
          condition_.notify_all();
          moving_.store(false);
          return;
        }
      }
    }
  }
  RCLCPP_DEBUG(
    node_handle_->get_logger(), "[ControllerExecution] run() thread exiting normally for %s",
    name_.c_str());
}

}  // namespace navflex_nav
