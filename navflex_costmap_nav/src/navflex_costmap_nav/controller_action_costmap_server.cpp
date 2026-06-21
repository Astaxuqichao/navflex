// New ControllerCostmapServer implementation using NavflexActionBase framework.
// Original version preserved in controller_server_bak.cpp

#include "navflex_costmap_nav/controller_action_costmap_server.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace navflex_costmap_nav {

ControllerCostmapServer::ControllerCostmapServer(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const navflex_utility::RobotInformation::ConstPtr& robot_info,
    const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode(
          "navflex_controller_server", "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               std::string("navflex_controller_server") + ":" +
                   std::string("__node:=navflex_controller_server")})),
      goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
      default_goal_checker_ids_{"goal_checker"},
      default_goal_checker_types_{"nav2_controller::SimpleGoalChecker"},
      lp_loader_("nav2_core", "nav2_core::Controller"),
      default_ids_{"FollowPath"},
      default_types_{"dwb_core::DWBLocalPlanner"},
      costmap_ros_(costmap_ros),
      robot_info_(robot_info),
      name_action_follow_path_("follow_path") {
  RCLCPP_INFO(get_logger(), "[ControllerServer] created action_server=%s",
              name_action_follow_path_.c_str());

  declare_parameter("controller_frequency", 20.0);
  declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("speed_limit_topic", rclcpp::ParameterValue(std::string("speed_limit")));
}

ControllerCostmapServer::~ControllerCostmapServer() {
  if (controller_action_) {
    controller_action_->cancelAll();
  }
  goal_checkers_.clear();
  controllers_.clear();
}

nav2_util::CallbackReturn ControllerCostmapServer::on_configure(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "[ControllerServer] configuring");
  auto node = shared_from_this();

  // --- Goal checkers ---
  get_parameter("goal_checker_plugins", goal_checker_ids_);
  if (goal_checker_ids_ == default_goal_checker_ids_) {
    for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
          node, default_goal_checker_ids_[i] + ".plugin",
          rclcpp::ParameterValue(default_goal_checker_types_[i]));
    }
  }
  goal_checker_ids_concat_.clear();
  goal_checker_types_.resize(goal_checker_ids_.size());
  for (size_t i = 0; i < goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] =
          nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
      nav2_core::GoalChecker::Ptr gc =
          goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(get_logger(), "[ControllerServer] loaded goal_checker id=%s type=%s",
                  goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      gc->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], gc});
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create goal_checker: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }
  for (const auto& id : goal_checker_ids_) {
    goal_checker_ids_concat_ += id + " ";
  }
  RCLCPP_INFO(get_logger(), "[ControllerServer] goal_checkers=[%s]",
              goal_checker_ids_concat_.c_str());

  // --- Controllers ---
  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
          node, default_ids_[i] + ".plugin",
          rclcpp::ParameterValue(default_types_[i]));
    }
  }
  controller_ids_concat_.clear();
  controller_types_.resize(controller_ids_.size());
  for (size_t i = 0; i < controller_ids_.size(); i++) {
    try {
      controller_types_[i] =
          nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr ctrl =
          lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(get_logger(), "[ControllerServer] loaded controller id=%s type=%s",
                  controller_ids_[i].c_str(), controller_types_[i].c_str());
      ctrl->configure(node, controller_ids_[i],
                      costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], ctrl});
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create controller: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }
  for (const auto& id : controller_ids_) {
    controller_ids_concat_ += id + " ";
  }
  RCLCPP_INFO(get_logger(), "[ControllerServer] controllers=[%s]",
              controller_ids_concat_.c_str());

  // --- Publishers ---
  vel_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  current_goal_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/current_goal", 1);

  // --- Speed limit subscription ---
  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
      speed_limit_topic, rclcpp::QoS(10),
      std::bind(&ControllerCostmapServer::speedLimitCallback, this, _1));

  // --- Action handler ---
  controller_action_ = std::make_shared<ControllerAction>(
      node, name_action_follow_path_, robot_info_);

  // --- Dedicated executor (false = not auto-added to node executor) mirroring SimpleActionServer spin_thread=true ---
  action_cb_group_ = node->get_node_base_interface()->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  action_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  action_executor_->add_callback_group(action_cb_group_, node->get_node_base_interface());
  action_executor_thread_ = std::make_unique<nav2_util::NodeThread>(action_executor_);

  // --- rclcpp_action server ---
  action_server_follow_path_ =
      rclcpp_action::create_server<ActionFollowPath>(
          node, name_action_follow_path_,
          std::bind(&ControllerCostmapServer::handleGoalFollowPath, this, _1, _2),
          std::bind(&ControllerCostmapServer::cancelActionFollowPath, this, _1),
          std::bind(&ControllerCostmapServer::callActionFollowPath, this, _1),
          rcl_action_server_get_default_options(),
          action_cb_group_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ControllerCostmapServer::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "[ControllerServer] activating controllers=[%s]",
              controller_ids_concat_.c_str());
  for (auto& [id, ctrl] : controllers_) {
    ctrl->activate();
  }
  vel_publisher_->on_activate();
  current_goal_publisher_->on_activate();

  auto node = shared_from_this();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&ControllerCostmapServer::dynamicParametersCallback, this, _1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ControllerCostmapServer::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating ControllerCostmapServer");
  if (controller_action_) {
    controller_action_->cancelAll();
  }
  for (auto& [id, ctrl] : controllers_) {
    ctrl->deactivate();
  }
  publishZeroVelocity();
  vel_publisher_->on_deactivate();
  current_goal_publisher_->on_deactivate();
  dyn_params_handler_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ControllerCostmapServer::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "Cleaning up ControllerCostmapServer");
  // Break circular ref: NavflexActionBase::node_ and RobotInformation::node_
  // both hold LifecycleNode::SharedPtr back to this server.
  // Must reset action objects first (they hold node_ / robot_info_ refs),
  // then reset costmap and robot_info.
  action_executor_thread_.reset();
  action_executor_.reset();
  controller_action_.reset();
  action_server_follow_path_.reset();
  for (auto& [id, ctrl] : controllers_) {
    ctrl->cleanup();
  }
  controllers_.clear();
  goal_checkers_.clear();
  vel_publisher_.reset();
  current_goal_publisher_.reset();
  speed_limit_sub_.reset();
  costmap_ros_.reset();
  robot_info_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ControllerCostmapServer::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/) {
  RCLCPP_INFO(get_logger(), "Shutting down ControllerCostmapServer");
  return nav2_util::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse ControllerCostmapServer::handleGoalFollowPath(
    const rclcpp_action::GoalUUID& /*uuid*/,
    ActionFollowPath::Goal::ConstSharedPtr goal) {
  const auto& poses = goal->path.poses;
  if (!poses.empty()) {
    const auto& start = poses.front().pose.position;
    const auto& end = poses.back().pose.position;
    RCLCPP_DEBUG(
        get_logger(),
        "Received FollowPath goal: controller_id='%s', goal_checker_id='%s', frame='%s', poses=%zu, start=(%.3f, %.3f), end=(%.3f, %.3f)",
        goal->controller_id.c_str(), goal->goal_checker_id.c_str(),
        goal->path.header.frame_id.c_str(), poses.size(), start.x, start.y,
        end.x, end.y);
  } else {
    RCLCPP_WARN(
        get_logger(),
        "Received FollowPath goal with empty path: controller_id='%s', goal_checker_id='%s', frame='%s'",
        goal->controller_id.c_str(), goal->goal_checker_id.c_str(),
        goal->path.header.frame_id.c_str());
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ControllerCostmapServer::cancelActionFollowPath(
    ServerGoalHandleFollowPathPtr goal_handle) {
  RCLCPP_INFO(get_logger(), "Canceling FollowPath action.");
  if (controller_action_) {
    controller_action_->cancel(goal_handle);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ControllerCostmapServer::callActionFollowPath(
    ServerGoalHandleFollowPathPtr goal_handle) {
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  const auto& goal = goal_handle->get_goal();
  nav_msgs::msg::Path normalized_path = goal->path;

  // Resolve controller plugin
  std::string controller_id;
  if (!findControllerId(goal->controller_id, controller_id)) {
    auto result = std::make_shared<ActionFollowPath::Result>();
    RCLCPP_ERROR(get_logger(), "Requested controller '%s' not found.",
                 goal->controller_id.c_str());
    goal_handle->abort(result);
    return;
  }

  // Resolve goal checker plugin
  std::string goal_checker_id;
  if (!findGoalCheckerId(goal->goal_checker_id, goal_checker_id)) {
    auto result = std::make_shared<ActionFollowPath::Result>();
    RCLCPP_ERROR(get_logger(), "Requested goal checker '%s' not found.",
                 goal->goal_checker_id.c_str());
    goal_handle->abort(result);
    return;
  }

  // Validate plan
  if (normalized_path.poses.empty()) {
    auto result = std::make_shared<ActionFollowPath::Result>();
    RCLCPP_ERROR(get_logger(), "Received an empty path, cannot follow.");
    goal_handle->abort(result);
    return;
  }

  if (normalized_path.header.frame_id.empty()) {
    auto result = std::make_shared<ActionFollowPath::Result>();
    RCLCPP_ERROR(get_logger(), "Received path with empty header.frame_id, cannot follow.");
    goal_handle->abort(result);
    return;
  }

  if (normalized_path.header.stamp.sec == 0 &&
      normalized_path.header.stamp.nanosec == 0) {
    normalized_path.header.stamp = now();
  }

  size_t filled_pose_frames = 0;
  for (auto& pose : normalized_path.poses) {
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = normalized_path.header.frame_id;
      ++filled_pose_frames;
    }
    if (pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) {
      pose.header.stamp = normalized_path.header.stamp;
    }
  }

  if (filled_pose_frames > 0) {
    RCLCPP_WARN(
        get_logger(),
        "FollowPath input path had %zu poses with empty frame_id. Filled with '%s'.",
        filled_pose_frames, normalized_path.header.frame_id.c_str());
  }

  const auto& start = normalized_path.poses.front().pose.position;
  const auto& end = normalized_path.poses.back().pose.position;
  RCLCPP_INFO(
      get_logger(),
      "[FollowPath] goal accepted: controller=%s goal_checker=%s poses=%zu frame=%s start=(%.2f, %.2f) goal=(%.2f, %.2f)",
      controller_id.c_str(), goal_checker_id.c_str(),
      normalized_path.poses.size(), normalized_path.header.frame_id.c_str(),
      start.x, start.y, end.x, end.y);

  // Create execution object and start
  ControllerExecution::Ptr execution =
      newControllerExecution(controller_id, goal_checker_id);
  execution->setNewPlan(normalized_path);

  controller_action_->start(goal_handle, execution);
}

bool ControllerCostmapServer::findControllerId(const std::string& requested,
                                         std::string& resolved) const {
  if (controllers_.find(requested) != controllers_.end()) {
    resolved = requested;
    return true;
  }
  if (requested.empty() && controllers_.size() == 1) {
    RCLCPP_WARN_ONCE(get_logger(),
                     "No controller_id in goal; using the only loaded "
                     "controller: %s",
                     controller_ids_concat_.c_str());
    resolved = controllers_.begin()->first;
    return true;
  }
  RCLCPP_ERROR(get_logger(),
               "Controller '%s' not found. Available: %s",
               requested.c_str(), controller_ids_concat_.c_str());
  return false;
}

bool ControllerCostmapServer::findGoalCheckerId(const std::string& requested,
                                          std::string& resolved) const {
  if (goal_checkers_.find(requested) != goal_checkers_.end()) {
    resolved = requested;
    return true;
  }
  if (requested.empty() && goal_checkers_.size() == 1) {
    RCLCPP_WARN_ONCE(get_logger(),
                     "No goal_checker_id in goal; using the only loaded "
                     "goal checker: %s",
                     goal_checker_ids_concat_.c_str());
    resolved = goal_checkers_.begin()->first;
    return true;
  }
  RCLCPP_ERROR(get_logger(),
               "GoalChecker '%s' not found. Available: %s",
               requested.c_str(), goal_checker_ids_concat_.c_str());
  return false;
}

ControllerExecution::Ptr ControllerCostmapServer::newControllerExecution(
    const std::string& controller_id,
    const std::string& goal_checker_id) {
  auto node = shared_from_this();
  return std::make_shared<ControllerExecution>(
      controller_id,
      controllers_.at(controller_id),
      goal_checkers_.at(goal_checker_id).get(),
      robot_info_,
      node,
      vel_publisher_,
      current_goal_publisher_);
}

void ControllerCostmapServer::speedLimitCallback(
    const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
  for (auto& [id, ctrl] : controllers_) {
    ctrl->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
}

void ControllerCostmapServer::publishZeroVelocity() {
  geometry_msgs::msg::Twist zero;
  if (vel_publisher_ && vel_publisher_->is_activated()) {
    vel_publisher_->publish(zero);
  }
}

rcl_interfaces::msg::SetParametersResult ControllerCostmapServer::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> /*parameters*/) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace navflex_costmap_nav
