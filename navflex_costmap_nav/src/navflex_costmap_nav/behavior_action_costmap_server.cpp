// New BehaviorCostmapServer implementation using NavflexActionBase framework.
// Original version preserved in behavior_server_bak.cpp

#include "navflex_costmap_nav/behavior_action_costmap_server.hpp"

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace navflex_costmap_nav
{

BehaviorCostmapServer::BehaviorCostmapServer(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_ros,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_ros,
  const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode(
    "navflex_behavior_server", "",
    rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r",
        std::string("navflex_behavior_server") + ":" +
        std::string("__node:=navflex_behavior_server")})),
  plugin_loader_("nav2_core", "nav2_core::Behavior"),
  default_ids_{"cmd_behavior"},
  default_types_{"navflex_cmdbehavior/CmdBehavior"},
  global_costmap_ros_(global_costmap_ros),
  local_costmap_ros_(local_costmap_ros),
  name_action_behavior_("behavior_action")
{
  declare_parameter("behavior_plugins", default_ids_);
}

BehaviorCostmapServer::~BehaviorCostmapServer()
{
  if (behavior_action_) {
    behavior_action_->cancelAll();
  }
  behaviors_.clear();
}

nav2_util::CallbackReturn BehaviorCostmapServer::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring BehaviorCostmapServer");
  auto node = shared_from_this();

  get_parameter("behavior_plugins", behavior_ids_);
  if (behavior_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  behavior_types_.resize(behavior_ids_.size());

  if (!loadBehaviorPlugins()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Create dedicated callback group (false = not auto-added to node executor)
  // and spin it in its own thread, mirroring SimpleActionServer spin_thread=true.
  // This fully isolates the execute callback from goal/cancel service responses.
  action_cb_group_ = node->get_node_base_interface()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  action_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  action_executor_->add_callback_group(action_cb_group_, node->get_node_base_interface());
  action_executor_thread_ = std::make_unique<nav2_util::NodeThread>(action_executor_);

  // Create action handler (robot_info not needed for behavior; pass nullptr)
  behavior_action_ = std::make_shared<BehaviorAction>(
    node, name_action_behavior_, nullptr);

  // Create rclcpp_action server
  action_server_ = rclcpp_action::create_server<ActionDummyBehavior>(
    node, name_action_behavior_,
    std::bind(&BehaviorCostmapServer::handleGoalDummyBehavior, this, _1, _2),
    std::bind(&BehaviorCostmapServer::cancelActionDummyBehavior, this, _1),
    std::bind(&BehaviorCostmapServer::callActionDummyBehavior, this, _1),
    rcl_action_server_get_default_options(),
    action_cb_group_);

  return nav2_util::CallbackReturn::SUCCESS;
}

bool BehaviorCostmapServer::loadBehaviorPlugins()
{
  auto node = shared_from_this();
  for (size_t i = 0; i < behavior_ids_.size(); i++) {
    behavior_types_[i] =
      nav2_util::get_plugin_type_param(node, behavior_ids_[i]);
    try {
      RCLCPP_INFO(
        get_logger(), "Creating behavior plugin %s of type %s",
        behavior_ids_[i].c_str(), behavior_types_[i].c_str());
      nav2_core::Behavior::Ptr behavior =
        plugin_loader_.createUniqueInstance(behavior_types_[i]);
      behavior->configure(
        node, behavior_ids_[i], global_costmap_ros_,
        local_costmap_ros_);
      behaviors_.insert({behavior_ids_[i], behavior});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create behavior %s of type %s: %s",
        behavior_ids_[i].c_str(), behavior_types_[i].c_str(),
        ex.what());
      return false;
    }
  }
  behavior_ids_concat_.clear();
  for (const auto & id : behavior_ids_) {
    behavior_ids_concat_ += id + " ";
  }
  RCLCPP_INFO(
    get_logger(), "Behavior Server has %s behaviors available.",
    behavior_ids_concat_.c_str());
  return true;
}

nav2_util::CallbackReturn BehaviorCostmapServer::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating BehaviorCostmapServer");
  for (auto & [id, beh] : behaviors_) {
    beh->activate();
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BehaviorCostmapServer::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating BehaviorCostmapServer");
  if (behavior_action_) {
    behavior_action_->cancelAll();
  }
  for (auto & [id, beh] : behaviors_) {
    beh->deactivate();
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BehaviorCostmapServer::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up BehaviorCostmapServer");
  action_executor_thread_.reset();
  action_executor_.reset();
  behavior_action_.reset();
  action_server_.reset();
  for (auto & [id, beh] : behaviors_) {
    beh->cleanup();
  }
  behaviors_.clear();
  global_costmap_ros_.reset();
  local_costmap_ros_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BehaviorCostmapServer::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down BehaviorCostmapServer");
  return nav2_util::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse BehaviorCostmapServer::handleGoalDummyBehavior(
  const rclcpp_action::GoalUUID & /*uuid*/,
  ActionDummyBehavior::Goal::ConstSharedPtr /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BehaviorCostmapServer::cancelActionDummyBehavior(
  ServerGoalHandleDummyBehaviorPtr goal_handle)
{
  RCLCPP_INFO(get_logger(), "Canceling DummyBehavior action.");
  if (behavior_action_) {
    behavior_action_->cancel(goal_handle);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BehaviorCostmapServer::callActionDummyBehavior(
  ServerGoalHandleDummyBehaviorPtr goal_handle)
{
  const auto & goal = goal_handle->get_goal();
  const std::string & behavior_name = goal->behavior;

  if (behaviors_.find(behavior_name) == behaviors_.end()) {
    auto result = std::make_shared<ActionDummyBehavior::Result>();
    result->outcome = ActionDummyBehavior::Result::INVALID_PLUGIN;
    result->message = "Behavior '" + behavior_name + "' not found. Available: " +
      behavior_ids_concat_;
    result->used_plugin = behavior_name;
    result->total_elapsed_time = rclcpp::Duration(0, 0);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(
    get_logger(), "[BehaviorServer] calling behavior plugin id=%s",
    behavior_name.c_str());

  BehaviorExecution::Ptr execution = newBehaviorExecution(behavior_name);
  execution->setCommand(goal->command.data);
  behavior_action_->start(goal_handle, execution);
}

BehaviorExecution::Ptr BehaviorCostmapServer::newBehaviorExecution(
  const std::string & behavior_name)
{
  auto node = shared_from_this();
  return std::make_shared<BehaviorExecution>(
    behavior_name,
    behaviors_.at(behavior_name),
    nullptr,     // behaviors don't need RobotInformation
    node);
}

}  // namespace navflex_costmap_nav
