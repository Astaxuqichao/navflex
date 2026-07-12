// Implementation of PlannerCostmapServer
// Manages global path planning with pluggable planner algorithms

// Standard library includes
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// ROS2 and Nav2 includes
#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "navflex_costmap_nav/planner_action_costmap_server.hpp"

// Using declarations
using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using std::placeholders::_2;

namespace navflex_costmap_nav
{

PlannerCostmapServer::PlannerCostmapServer(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  const navflex_utility::RobotInformation::ConstPtr & robot_info,
  const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode(
    "navflex_planner_server", "",
    rclcpp::NodeOptions().arguments(
      {"--ros-args", "-r",
        std::string("navflex_planner_server") + ":" +
        std::string("__node:=") +
        std::string("navflex_planner_server")})),
  name_action_get_path_("compute_path_to_pose"),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"nav2_navfn_planner/NavfnPlanner"},
  costmap_(nullptr),
  costmap_ros_(costmap_ros),
  robot_info_(robot_info)
{
  RCLCPP_INFO(
    get_logger(), "[PlannerServer] created action_server=%s",
    name_action_get_path_.c_str());

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }
}

PlannerCostmapServer::~PlannerCostmapServer()
{
  /*
   * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
   * never called.
   */
  planners_.clear();
}

/**
 * Lifecycle callback: Configure the planner server and load plugins
 *
 * **Purpose:**
 * Initialize planner server during ROS2 lifecycle configure phase. This is
 * where all persistent resources are allocated, plugins are loaded, and the
 * action server is set up.
 *
 * **Initialization Steps:**
 * 1. Get costmap from costmap_ros and extract costmap dimensions
 * 2. Retrieve TF buffer from costmap_ros for coordinate transformations
 * 3. Load global planner plugins based on configured plugin IDs:
 *    - Read planner_ids from parameters
 *    - For each plugin ID, load the corresponding plugin type
 *    - Call configure() on each loaded plugin
 *    - Store in planners_ map keyed by plugin ID
 * 4. Create PlannerAction instance for handling action requests
 * 5. Create rclcpp_action server for handling compute_path_to_pose actions
 * 6. Set up goal/cancel/feedback callbacks for action server
 *
 * **Plugin System:**
 * Uses pluginlib to dynamically load nav2_core::GlobalPlanner implementations:
 * - NavfnPlanner (default)
 * - SmacPlanner
 * - ThetaStarPlanner
 * - Other custom planners implementing nav2_core::GlobalPlanner interface
 *
 * **Error Handling:**
 * - Plugin loading failures are logged as FATAL but don't stop server startup
 * - Missing costmap data logs warnings but continues
 *
 * **Thread Context:** Main ROS2 lifecycle thread
 *
 * @param state ROS2 lifecycle state (passed by lifecycle manager, unused)
 * @return SUCCESS if configuration completed, FAILURE otherwise
 *
 * **Implementation Notes:**
 * - Costmap and TF resources come from shared costmap_ros object
 * - All plugins are configured with same TF buffer and costmap
 * - Action server callbacks are bound to PlannerCostmapServer methods
 */
nav2_util::CallbackReturn PlannerCostmapServer::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "[PlannerServer] configuring");

  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d", costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());
  planner_ids_concat_.clear();

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] =
        nav2_util::get_plugin_type_param(node, planner_ids_[i]);
      nav2_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "[PlannerServer] loaded planner id=%s type=%s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create global planner. Exception: %s", ex.what());
      // return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(), "[PlannerServer] ready planners=[%s]",
    planner_ids_concat_.c_str());

  planner_action_ =
    std::make_shared<PlannerAction>(node, name_action_get_path_, robot_info_);

  // Dedicated executor (false = not auto-added to node executor) mirroring
  // SimpleActionServer spin_thread=true. Reentrant + two threads keeps the
  // goal response/cancel callbacks responsive while an accepted goal is being
  // handed off to the planner execution thread.
  action_cb_group_ = node->get_node_base_interface()->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant, false);
  action_executor_ =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), 2);
  action_executor_->add_callback_group(action_cb_group_, node->get_node_base_interface());
  action_executor_thread_ =
    std::make_unique<std::thread>(
    [executor = action_executor_]() {
      executor->spin();
    });

  action_server_get_path_ptr_ = rclcpp_action::create_server<ActionToPose>(
    node, name_action_get_path_,
    std::bind(
      &navflex_costmap_nav::PlannerCostmapServer::handleGoalGetPath, this,
      _1, _2),
    std::bind(
      &navflex_costmap_nav::PlannerCostmapServer::cancelActionGetPath, this,
      _1),
    std::bind(
      &navflex_costmap_nav::PlannerCostmapServer::callActionGetPath, this,
      _1),
    rcl_action_server_get_default_options(),
    action_cb_group_);

  return nav2_util::CallbackReturn::SUCCESS;
}

/**
 * Lifecycle callback: Activate the planner server
 *
 * **Purpose:**
 * Transition the planner server from inactive to active state. Enables all
 * subscriptions, publishers, action servers, and plugin planners.
 *
 * **Activation Steps:**
 * 1. Activate plan publisher for RViz visualization
 * 2. Activate action servers (single pose and multiple poses)
 * 3. Activate all loaded planner plugins
 * 4. Register dynamic parameter callback for runtime reconfiguration
 *
 * **State Management:**
 * After activation:
 * - Server can receive and process planning requests
 * - All plugins can perform planning computations
 * - Plan visualization is published to RViz
 * - Action server accepts new goals
 *
 * **Dynamic Parameters:**
 * Registers a callback that allows runtime updates to:
 * - Expected planner frequency
 * - Plugin-specific parameters
 * - Planner selection and weights
 *
 * **Thread Context:** Main ROS2 lifecycle thread
 *
 * @param state ROS2 lifecycle state (passed by lifecycle manager, unused)
 * @return SUCCESS if activation completed, FAILURE on plugin activation errors
 *
 * **Implementation Notes:**
 * - All planners activate with same state (all succeed or all fail together)
 * - Dynamic parameters handler registered using bind
 * - Publishers/subscriptions only active when parent ROS node is active
 */
nav2_util::CallbackReturn PlannerCostmapServer::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(
    get_logger(), "[PlannerServer] activating planners=[%s]",
    planner_ids_concat_.c_str());

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();

  // Add callback for dynamic parameters
  // dyn_params_handler_ = node->add_on_set_parameters_callback(
  //     std::bind(&PlannerCostmapServer::dynamicParametersCallback, this, _1));

  return nav2_util::CallbackReturn::SUCCESS;
}

/**
 * Lifecycle callback: Deactivate the planner server
 *
 * **Purpose:**
 * Transition the planner server from active to inactive state. Disables all
 * action servers and plugin planners while maintaining internal state.
 *
 * **Deactivation Steps:**
 * 1. Deactivate action servers (stops accepting new requests)
 * 2. Deactivate plan publisher
 * 3. Deactivate all loaded planner plugins
 * 4. Unregister dynamic parameter callback
 *
 * **State Management:**
 * After deactivation:
 * - Server cannot process new planning requests
 * - In-flight requests are allowed to complete
 * - Plugin state is preserved for later reactivation
 * - Costmap remains active (may be shared with other servers)
 *
 * **Thread Safety:**
 * Note: Costmap may be deactivated externally via preshutdown callbacks,
 * which could happen out-of-order with this server's deactivation.
 *
 * **Thread Context:** Main ROS2 lifecycle thread
 *
 * @param state ROS2 lifecycle state (passed by lifecycle manager, unused)
 * @return SUCCESS if deactivation completed
 *
 * **Implementation Notes:**
 * - Deactivation is lightweight (plugins retain state)
 * - Dynamic params handler explicitly reset to release callback
 * - Action servers stop accepting new requests immediately
 */
nav2_util::CallbackReturn PlannerCostmapServer::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Planner server");
  /*
   * The costmap is also a lifecycle node, so it may have already fired
   * on_deactivate via rcl preshutdown cb. Despite the rclcpp docs saying
   * on_shutdown callbacks fire in the order added, the preshutdown callbacks
   * clearly don't per se, due to using an unordered_set iteration. Once this
   * issue is resolved, we can maybe make a stronger ordering assumption:
   * https://github.com/ros2/rclcpp/issues/2096
   */

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

/**
 * Lifecycle callback: Clean up the planner server
 *
 * **Purpose:**
 * Perform final cleanup during ROS2 lifecycle shutdown. Release all allocated
 * resources and clear plugin instances.
 *
 * **Cleanup Steps:**
 * 1. Reset action server shared pointers (triggers teardown)
 * 2. Reset plan publisher
 * 3. Reset TF buffer reference
 * 4. Clear planner plugin instances (via planners_.clear())
 *
 * **Resource Cleanup:**
 * Releasing shared pointers triggers:
 * - Action server thread shutdown
 * - Publisher callback thread cleanup
 * - TF buffer listener cleanup
 * - Plugin destructor invocation
 *
 * **Thread Context:** Main ROS2 lifecycle thread
 *
 * @param state ROS2 lifecycle state (passed by lifecycle manager, unused)
 * @return SUCCESS after cleanup completion
 *
 * **Implementation Notes:**
 * - Order matters: action servers cleaned before publishers
 * - planners_.clear() handled by destructor as well (backstop)
 * - Costmap cleanup handled by costmap_ros
 * - Reset order ensures no lingering references between components
 */
nav2_util::CallbackReturn PlannerCostmapServer::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Planner server");

  // Break the circular reference: rclcpp_action server holds
  // NodeBaseInterface::SharedPtr back to this node, preventing destruction
  // unless we explicitly reset it here.
  action_server_get_path_ptr_.reset();
  if (action_executor_) {
    action_executor_->cancel();
  }
  if (action_executor_thread_ && action_executor_thread_->joinable()) {
    action_executor_thread_->join();
  }
  action_executor_thread_.reset();
  action_executor_.reset();
  planner_action_.reset();
  tf_.reset();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }

  planners_.clear();
  costmap_ros_.reset();
  costmap_ = nullptr;
  robot_info_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

/**
 * Lifecycle callback: Shutdown the planner server
 *
 * **Purpose:**
 * Final shutdown hook called after cleanup. Allows any final logging or
 * emergency resource cleanup before process exit.
 *
 * **Implementation:**
 * Currently just logs shutdown status. Actual cleanup is handled by
 * on_cleanup() which is called before shutdown in the lifecycle sequence.
 *
 * **Thread Context:** Main ROS2 lifecycle thread
 *
 * @param state ROS2 lifecycle state (unused)
 * @return SUCCESS after shutdown
 */
nav2_util::CallbackReturn PlannerCostmapServer::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down Planner server");
  return nav2_util::CallbackReturn::SUCCESS;
}

/**
 * Handle incoming path planning goal request
 *
 * **Purpose:**
 * ROS2 action server callback for goal reception. Decides whether to accept
 * the planning request from a client.
 *
 * **Behavior:**
 * - Accepts all incoming planning goals immediately
 * - Actual validation happens in callActionGetPath()
 * - UUID is logged but not used for uniqueness enforcement
 *
 * **Thread Context:** Action server executor thread
 *
 * @param uuid ROS2 goal UUID assigned by action server
 * @param goal Goal message containing target pose and planner selection
 * @return ACCEPT_AND_EXECUTE to process goal
 */
rclcpp_action::GoalResponse PlannerCostmapServer::handleGoalGetPath(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  ActionToPose::Goal::ConstSharedPtr goal)
{
  const auto now = this->now();
  RCLCPP_INFO(
    get_logger(),
    "[PlannerServer] handle_goal enter t=%ld.%09ld planner_id=%s use_start=%s",
    static_cast<long>(now.seconds()),
    static_cast<long>(now.nanoseconds() % 1000000000),
    goal ? goal->planner_id.c_str() : "<null>",
    goal && goal->use_start ? "true" : "false");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // accept all goal
}

/**
 * Execute path planning action callback
 *
 * **Purpose:**
 * Main action execution handler. Called when a planning goal is accepted by
 * the action server. Coordinates planner plugin selection, execution thread
 * management, and result reporting.
 *
 * **Execution Flow:**
 * 1. Validate that planner plugins are loaded
 * 2. Extract requested planner name from goal (or use first available)
 * 3. Look up planner plugin by name in planners_ map
 * 4. Create PlannerExecution thread wrapper for the selected plugin
 * 5. Delegate to NavflexActionBase for actual execution management
 * 6. Return result to action client
 *
 * **Planner Selection:**
   * - If goal.planner is empty: uses the first configured loaded planner
 * - If goal.planner is specified: looks up by name in planners_ map
 * - Returns INTERNAL_ERROR if planner not found or no plugins loaded
 *
 * **Error Handling:**
 * - No plugins loaded: Aborts with INTERNAL_ERROR
 * - Invalid planner name: Aborts with INTERNAL_ERROR
 * - Planner null pointer: Aborts with INTERNAL_ERROR
 *
 * **Thread Context:** Action server callback thread (not main node thread)
 *
 * **Threading Model:**
 * 1. This callback runs in action server thread
 * 2. Creates PlannerExecution with dedicated execution thread
 * 3. NavflexActionBase manages the execution thread lifecycle
 * 4. Action server monitors completion and cancellation
 *
 * @param goal_handle ROS2 goal handle for this planning request
 *
 * **Implementation Notes:**
 * - Planner ptr null check prevents crashes from invalid plugin pointers
   * - Empty planner_id selection follows planner_plugins configuration order
 */
void PlannerCostmapServer::callActionGetPath(ServerGoalHandleGetPathPtr goal_handle)
{
  const auto execute_enter_time = this->now();
  const auto execute_enter_steady = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    get_logger(),
    "[PlannerServer] execute_callback enter t=%ld.%09ld",
    static_cast<long>(execute_enter_time.seconds()),
    static_cast<long>(execute_enter_time.nanoseconds() % 1000000000));

  const ActionToPose::Goal & goal = *goal_handle->get_goal();
  ActionToPose::Result::SharedPtr result =
    std::make_shared<ActionToPose::Result>();
  // If no planner plugins were loaded, this indicates a system initialization
  // failure. Abort the action since planning cannot proceed.
  if (planners_.empty()) {
    RCLCPP_FATAL_STREAM(
      rclcpp::get_logger(name_action_get_path_),
      "Internal Error: No plugin loaded");

    // Abort the action and return an error result to the client
    goal_handle->abort(result);
    return;
  }

  // Pointer to the planner plugin that will be used for path generation
  nav2_core::GlobalPlanner::Ptr planner_ptr = nullptr;
  std::string selected_planner_id;

  // If no planner name is specified in the goal, use the first configured
  // planner ID that was loaded successfully. Do not depend on unordered_map
  // iteration order.
  if (goal.planner_id.empty()) {
    for (const auto & planner_id : planner_ids_) {
      auto it = planners_.find(planner_id);
      if (it != planners_.end() && it->second) {
        selected_planner_id = it->first;
        planner_ptr = it->second;
        break;
      }
    }
    if (!planner_ptr) {
      RCLCPP_FATAL_STREAM(
        rclcpp::get_logger(name_action_get_path_),
        "Internal Error: No configured planner plugin is loaded");
      goal_handle->abort(result);
      return;
    }
  } else {
    // Look up the planner plugin by the name provided in the goal
    auto it = planners_.find(goal.planner_id);

    // If the planner does not exist or the plugin pointer is invalid,
    // abort the action with an internal error
    if (it == planners_.end() || !it->second) {
      RCLCPP_FATAL_STREAM(
        rclcpp::get_logger(name_action_get_path_),
        "Internal Error: No plugin loaded with name "
          << goal.planner_id);

      goal_handle->abort(result);
      return;
    }

    // Use the requested planner plugin
    selected_planner_id = it->first;
    planner_ptr = it->second;
  }

  RCLCPP_INFO(
    get_logger(),
    "[PlannerServer] calling planner plugin id=%s requested_id=%s",
    selected_planner_id.c_str(),
    goal.planner_id.empty() ? "<empty>" : goal.planner_id.c_str());

  navflex_costmap_nav::PlannerExecution::Ptr planner_execution =
    newPlannerExecution(selected_planner_id, planner_ptr);

  // start another planning action
  const auto start_before_steady = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    get_logger(),
    "[PlannerServer] PlannerAction::start begin id=%s elapsed_since_execute_ms=%.3f",
    selected_planner_id.c_str(),
    std::chrono::duration<double, std::milli>(
      start_before_steady - execute_enter_steady).count());

  planner_action_->start(goal_handle, planner_execution);

  const auto start_after_steady = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    get_logger(),
    "[PlannerServer] PlannerAction::start returned id=%s start_call_ms=%.3f total_execute_cb_ms=%.3f",
    selected_planner_id.c_str(),
    std::chrono::duration<double, std::milli>(
      start_after_steady - start_before_steady).count(),
    std::chrono::duration<double, std::milli>(
      start_after_steady - execute_enter_steady).count());
}

rclcpp_action::CancelResponse PlannerCostmapServer::cancelActionGetPath(
  ServerGoalHandleGetPathPtr goal_handle)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(name_action_get_path_),
    "Cancel action \"get_path\"");
  return rclcpp_action::CancelResponse::
         ACCEPT; // returning ACCEPT here will change the goal_handle state via
                 // rclcpp_action code. The planner_action reacts on that change
                 // and will stop execution and cancel the action
}

navflex_costmap_nav::PlannerExecution::Ptr
PlannerCostmapServer::newPlannerExecution(
  const std::string & plugin_name,
  const nav2_core::GlobalPlanner::Ptr & plugin_ptr)
{
  auto node = shared_from_this();
  return std::make_shared<navflex_costmap_nav::PlannerExecution>(
    plugin_name, plugin_ptr, robot_info_, node);
}

}  // namespace navflex_costmap_nav
