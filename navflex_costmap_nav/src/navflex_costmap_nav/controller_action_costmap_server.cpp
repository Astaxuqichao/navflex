// New ControllerCostmapServer implementation using NavflexActionBase framework.
// Original version preserved in controller_server_bak.cpp

#include "navflex_costmap_nav/controller_action_costmap_server.hpp"

#include <cmath>
#include <limits>

#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace navflex_costmap_nav
{

namespace
{

class ActionGoalChecker : public nav2_core::GoalChecker
{
public:
  ActionGoalChecker(double xy_goal_tolerance, double yaw_goal_tolerance)
  : xy_goal_tolerance_(xy_goal_tolerance),
    yaw_goal_tolerance_(yaw_goal_tolerance),
    xy_goal_tolerance_sq_(xy_goal_tolerance * xy_goal_tolerance) {}

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &) override
  {
    const double dx = query_pose.position.x - goal_pose.position.x;
    const double dy = query_pose.position.y - goal_pose.position.y;
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
      return false;
    }

    const double dyaw = angles::shortest_angular_distance(
      tf2::getYaw(query_pose.orientation),
      tf2::getYaw(goal_pose.orientation));
    return std::fabs(dyaw) <= yaw_goal_tolerance_;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override
  {
    const double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = invalid_field;
    pose_tolerance.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

    vel_tolerance.linear.x = invalid_field;
    vel_tolerance.linear.y = invalid_field;
    vel_tolerance.linear.z = invalid_field;
    vel_tolerance.angular.x = invalid_field;
    vel_tolerance.angular.y = invalid_field;
    vel_tolerance.angular.z = invalid_field;

    return true;
  }

private:
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double xy_goal_tolerance_sq_;
};

}  // namespace

ControllerCostmapServer::ControllerCostmapServer(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  const navflex_utility::RobotInformation::ConstPtr & robot_info,
  const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode(
    "navflex_controller_server", "",
    rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r",
        std::string("navflex_controller_server") + ":" +
        std::string("__node:=navflex_controller_server")})),
  lp_loader_("nav2_core", "nav2_core::Controller"),
  default_ids_{"FollowPath"},
  default_types_{"dwb_core::DWBLocalPlanner"},
  costmap_ros_(costmap_ros),
  robot_info_(robot_info),
  name_action_follow_path_("follow_path")
{
  RCLCPP_INFO(
    get_logger(), "[ControllerServer] created action_server=%s",
    name_action_follow_path_.c_str());

  declare_parameter("controller_frequency", 20.0);
  declare_parameter("controller_plugins", default_ids_);
  declare_parameter("default_xy_goal_tolerance", 0.25);
  declare_parameter("default_yaw_goal_tolerance", 0.25);
  declare_parameter("speed_limit_topic", rclcpp::ParameterValue(std::string("speed_limit")));
  declare_parameter("cmd_vel_topic", rclcpp::ParameterValue(std::string("cmd_vel_nav")));
}

ControllerCostmapServer::~ControllerCostmapServer()
{
  if (controller_action_) {
    controller_action_->cancelAll();
  }
  controllers_.clear();
}

nav2_util::CallbackReturn ControllerCostmapServer::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "[ControllerServer] configuring");
  auto node = shared_from_this();

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
      RCLCPP_INFO(
        get_logger(), "[ControllerServer] loaded controller id=%s type=%s",
        controller_ids_[i].c_str(), controller_types_[i].c_str());
      ctrl->configure(
        node, controller_ids_[i],
        costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], ctrl});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create controller: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }
  for (const auto & id : controller_ids_) {
    controller_ids_concat_ += id + " ";
  }
  RCLCPP_INFO(
    get_logger(), "[ControllerServer] controllers=[%s]",
    controller_ids_concat_.c_str());

  // --- Publishers ---
  std::string cmd_vel_topic;
  get_parameter("cmd_vel_topic", cmd_vel_topic);
  vel_publisher_ =
    create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);
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
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(
    get_logger(), "[ControllerServer] activating controllers=[%s]",
    controller_ids_concat_.c_str());
  for (auto & [id, ctrl] : controllers_) {
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
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ControllerCostmapServer");
  if (controller_action_) {
    controller_action_->cancelAll();
  }
  for (auto & [id, ctrl] : controllers_) {
    ctrl->deactivate();
  }
  publishZeroVelocity();
  vel_publisher_->on_deactivate();
  current_goal_publisher_->on_deactivate();
  dyn_params_handler_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ControllerCostmapServer::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up ControllerCostmapServer");
  // Break circular ref: NavflexActionBase::node_ and RobotInformation::node_
  // both hold LifecycleNode::SharedPtr back to this server.
  // Must reset action objects first (they hold node_ / robot_info_ refs),
  // then reset costmap and robot_info.
  action_executor_thread_.reset();
  action_executor_.reset();
  controller_action_.reset();
  action_server_follow_path_.reset();
  for (auto & [id, ctrl] : controllers_) {
    ctrl->cleanup();
  }
  controllers_.clear();
  vel_publisher_.reset();
  current_goal_publisher_.reset();
  speed_limit_sub_.reset();
  costmap_ros_.reset();
  robot_info_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ControllerCostmapServer::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down ControllerCostmapServer");
  return nav2_util::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse ControllerCostmapServer::handleGoalFollowPath(
  const rclcpp_action::GoalUUID & /*uuid*/,
  ActionFollowPath::Goal::ConstSharedPtr goal)
{
  const auto & poses = goal->path.poses;
  if (!poses.empty()) {
    const auto & start = poses.front().pose.position;
    const auto & end = poses.back().pose.position;
    RCLCPP_DEBUG(
      get_logger(),
      "Received FollowPath goal: controller_id='%s', frame='%s', poses=%zu, start=(%.3f, %.3f), end=(%.3f, %.3f), tolerance=(%.3f, %.3f)",
      goal->controller_id.c_str(),
      goal->path.header.frame_id.c_str(), poses.size(), start.x, start.y,
      end.x, end.y, goal->xy_goal_tolerance, goal->yaw_goal_tolerance);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Received FollowPath goal with empty path: controller_id='%s', frame='%s'",
      goal->controller_id.c_str(),
      goal->path.header.frame_id.c_str());
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ControllerCostmapServer::cancelActionFollowPath(
  ServerGoalHandleFollowPathPtr goal_handle)
{
  RCLCPP_INFO(get_logger(), "Canceling FollowPath action.");
  if (controller_action_) {
    controller_action_->cancel(goal_handle);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ControllerCostmapServer::callActionFollowPath(
  ServerGoalHandleFollowPathPtr goal_handle)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  const auto & goal = goal_handle->get_goal();
  nav_msgs::msg::Path normalized_path = goal->path;

  // Resolve controller plugin
  std::string controller_id;
  if (!findControllerId(goal->controller_id, controller_id)) {
    auto result = std::make_shared<ActionFollowPath::Result>();
    result->outcome = ActionFollowPath::Result::INVALID_PLUGIN;
    result->message = "Requested controller plugin was not found";
    RCLCPP_ERROR(
      get_logger(), "Requested controller '%s' not found.",
      goal->controller_id.c_str());
    goal_handle->abort(result);
    return;
  }

  // Validate plan
  if (normalized_path.poses.empty()) {
    auto result = std::make_shared<ActionFollowPath::Result>();
    result->outcome = ActionFollowPath::Result::INVALID_PATH;
    result->message = "Received an empty path, cannot follow";
    RCLCPP_ERROR(get_logger(), "Received an empty path, cannot follow.");
    goal_handle->abort(result);
    return;
  }

  if (normalized_path.header.frame_id.empty()) {
    auto result = std::make_shared<ActionFollowPath::Result>();
    result->outcome = ActionFollowPath::Result::INVALID_PATH;
    result->message = "Received path with empty header.frame_id, cannot follow";
    RCLCPP_ERROR(get_logger(), "Received path with empty header.frame_id, cannot follow.");
    goal_handle->abort(result);
    return;
  }

  if (normalized_path.header.stamp.sec == 0 &&
    normalized_path.header.stamp.nanosec == 0)
  {
    normalized_path.header.stamp = now();
  }

  size_t filled_pose_frames = 0;
  for (auto & pose : normalized_path.poses) {
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

  const auto & start = normalized_path.poses.front().pose.position;
  const auto & end = normalized_path.poses.back().pose.position;
  const double start_yaw =
    tf2::getYaw(normalized_path.poses.front().pose.orientation);
  const double goal_yaw =
    tf2::getYaw(normalized_path.poses.back().pose.orientation);
  double default_xy_goal_tolerance = 0.25;
  double default_yaw_goal_tolerance = 0.25;
  get_parameter("default_xy_goal_tolerance", default_xy_goal_tolerance);
  get_parameter("default_yaw_goal_tolerance", default_yaw_goal_tolerance);

  const bool use_default_goal_tolerance =
    goal->xy_goal_tolerance == 0.0f && goal->yaw_goal_tolerance == 0.0f;
  const double xy_goal_tolerance = use_default_goal_tolerance ?
    default_xy_goal_tolerance : goal->xy_goal_tolerance;
  const double yaw_goal_tolerance = use_default_goal_tolerance ?
    default_yaw_goal_tolerance : goal->yaw_goal_tolerance;

  RCLCPP_INFO(
    get_logger(),
    "[FollowPath] calling controller plugin id=%s poses=%zu frame=%s "
    "start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f) "
    "tolerance=(%.3f, %.3f)%s",
    controller_id.c_str(),
    normalized_path.poses.size(), normalized_path.header.frame_id.c_str(),
    start.x, start.y, start_yaw, end.x, end.y, goal_yaw,
    xy_goal_tolerance, yaw_goal_tolerance,
    use_default_goal_tolerance ? " from defaults" : " from action goal");

  // Create execution object and start
  auto goal_checker = std::make_shared<ActionGoalChecker>(
    xy_goal_tolerance, yaw_goal_tolerance);
  ControllerExecution::Ptr execution =
    newControllerExecution(
    controller_id, goal_checker, xy_goal_tolerance, yaw_goal_tolerance);
  execution->setNewPlan(normalized_path);

  controller_action_->start(goal_handle, execution);
}

bool ControllerCostmapServer::findControllerId(
  const std::string & requested,
  std::string & resolved) const
{
  if (controllers_.find(requested) != controllers_.end()) {
    resolved = requested;
    return true;
  }
  if (requested.empty() && controllers_.size() == 1) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "No controller_id in goal; using the only loaded "
      "controller: %s",
      controller_ids_concat_.c_str());
    resolved = controllers_.begin()->first;
    return true;
  }
  RCLCPP_ERROR(
    get_logger(),
    "Controller '%s' not found. Available: %s",
    requested.c_str(), controller_ids_concat_.c_str());
  return false;
}

ControllerExecution::Ptr ControllerCostmapServer::newControllerExecution(
  const std::string & controller_id,
  const nav2_core::GoalChecker::Ptr & goal_checker,
  double xy_goal_tolerance,
  double yaw_goal_tolerance)
{
  auto node = shared_from_this();
  return std::make_shared<ControllerExecution>(
    controller_id,
    controllers_.at(controller_id),
    goal_checker,
    xy_goal_tolerance,
    yaw_goal_tolerance,
    robot_info_,
    node,
    vel_publisher_,
    current_goal_publisher_);
}

void ControllerCostmapServer::speedLimitCallback(
  const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  for (auto & [id, ctrl] : controllers_) {
    ctrl->setSpeedLimit(msg->speed_limit, msg->percentage);
  }
}

void ControllerCostmapServer::publishZeroVelocity()
{
  geometry_msgs::msg::Twist zero;
  if (vel_publisher_ && vel_publisher_->is_activated()) {
    vel_publisher_->publish(zero);
  }
}

rcl_interfaces::msg::SetParametersResult ControllerCostmapServer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter>/*parameters*/)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace navflex_costmap_nav
