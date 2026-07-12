#include "navflex_nav/rogmap_nav/navflex_rogmap_nav.hpp"
#include "nav2_util/node_utils.hpp"

#include <algorithm>
#include <vector>

namespace navflex_nav
{

RogMapNavNode::RogMapNavNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("navflex_rogmap_nav", "", options)
{
  RCLCPP_INFO(get_logger(), "[Navflex] RogMapNavNode created");
}

RogMapNavNode::~RogMapNavNode()
{
  planner_server_thread_.reset();
  planner_server_.reset();
  controller_server_thread_.reset();
  controller_server_.reset();
  behavior_server_thread_.reset();
  behavior_server_.reset();

  rog_map_thread_.reset();
  rog_map_.reset();
  RCLCPP_INFO(get_logger(), "RogMapNavNode destroyed.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RogMapNavNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "[Navflex] configuring ROG map and action servers");

  auto node = shared_from_this();
  double tf_timeout_s;

  nav2_util::declare_parameter_if_not_declared(node, "tf_timeout", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "global_frame",
    rclcpp::ParameterValue(std::string("map")));
  nav2_util::declare_parameter_if_not_declared(
    node, "robot_frame",
    rclcpp::ParameterValue(std::string("base_link")));
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_topic",
    rclcpp::ParameterValue(std::string("odom")));

  node->get_parameter("tf_timeout", tf_timeout_s);
  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("robot_frame", robot_frame_);

  rog_map_ = std::make_shared<navflex_rog_map::RogMapROS>(
    "rog_map", std::string{get_namespace()}, "rog_map");
  rog_map_->configure();
  tf_listener_ptr_ = rog_map_->getTfBuffer();

  robot_info_ = std::make_shared<navflex_utility::RobotInformation>(
    node, tf_listener_ptr_, global_frame_, robot_frame_,
    rclcpp::Duration::from_seconds(tf_timeout_s),
    node->get_parameter("odom_topic").as_string());
  rog_map_thread_ = std::make_unique<nav2_util::NodeThread>(rog_map_);

  controller_server_ =
    std::make_shared<navflex_nav::ControllerRogMapServer>(rog_map_, robot_info_);
  controller_server_->configure();
  controller_server_thread_ =
    std::make_unique<nav2_util::NodeThread>(controller_server_);

  planner_server_ =
    std::make_shared<navflex_nav::PlannerRogMapServer>(rog_map_, robot_info_);
  planner_server_->configure();
  planner_server_thread_ =
    std::make_unique<nav2_util::NodeThread>(planner_server_);

  behavior_server_ = std::make_shared<navflex_nav::RecoveryRogMapServer>(
    rog_map_, rog_map_);
  behavior_server_->configure();
  behavior_server_thread_ =
    std::make_unique<nav2_util::NodeThread>(behavior_server_);

  RCLCPP_INFO(
    get_logger(),
    "[Navflex] configured: global_frame=%s robot_frame=%s odom_topic=%s",
    global_frame_.c_str(), robot_frame_.c_str(),
    node->get_parameter("odom_topic").as_string().c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
         CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RogMapNavNode::on_activate(const rclcpp_lifecycle::State &)
{
  rog_map_->activate();
  planner_server_->activate();
  controller_server_->activate();
  behavior_server_->activate();
  createBond();

  // Keep the existing Navflex query service names for client compatibility.
  check_point_srv_ = create_service<nav2_msgs::srv::CheckPoint>(
    "check_point_cost",
    std::bind(
      &RogMapNavNode::checkPointCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  check_pose_srv_ = create_service<nav2_msgs::srv::CheckPose>(
    "check_pose_cost",
    std::bind(
      &RogMapNavNode::checkPoseCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  check_path_srv_ = create_service<nav2_msgs::srv::CheckPath>(
    "check_path_cost",
    std::bind(
      &RogMapNavNode::checkPathCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "[Navflex] active: planner, controller, behavior, costmaps");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
         CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RogMapNavNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  planner_server_->deactivate();
  controller_server_->deactivate();
  behavior_server_->deactivate();

  // Remove costmap query services
  check_point_srv_.reset();
  check_pose_srv_.reset();
  check_path_srv_.reset();

  rog_map_->deactivate();

  destroyBond();
  RCLCPP_INFO(get_logger(), "RogMapNavNode deactivated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
         CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RogMapNavNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Step 1: stop NodeThread (joins the spin thread) before calling
  // cleanup() on the server.  Calling cleanup() while the thread is
  // still spinning causes use-after-free on publishers / subscribers.
  planner_server_thread_.reset();
  planner_server_->cleanup();
  planner_server_.reset();

  controller_server_thread_.reset();
  controller_server_->cleanup();
  controller_server_.reset();

  behavior_server_thread_.reset();
  behavior_server_->cleanup();
  behavior_server_.reset();

  // Step 2: stop costmap threads before cleaning up the costmap nodes.
  rog_map_thread_.reset();
  rog_map_->cleanup();
  // Must reset these shared_ptrs so their ROS nodes are destroyed before
  // on_configure creates new ones with the same names.  Without this,
  // the old Costmap2DROS nodes survive (held by tf_listener_ptr_ /
  // robot_info_), causing duplicate-node conflicts on the next configure.
  robot_info_.reset();
  tf_listener_ptr_.reset();
  rog_map_.reset();
  RCLCPP_INFO(get_logger(), "RogMapNavNode cleaned up.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
         CallbackReturn::SUCCESS;
}

} // namespace navflex_nav


// costmap query service implementations
namespace navflex_nav
{

// ── helpers ─────────────────────────────────────────────────────────────────

std::shared_ptr<navflex_rog_map::RogMapROS>
RogMapNavNode::selectCostmap(uint8_t costmap_id)
{
  using Req = nav2_msgs::srv::CheckPoint::Request;
  (void)costmap_id;
  return rog_map_;
}

uint8_t RogMapNavNode::occupancyToState(navflex_rog_map::OccupancyState state)
{
  using Res = nav2_msgs::srv::CheckPoint::Response;
  if (state == navflex_rog_map::OccupancyState::OCCUPIED) {
    return Res::LETHAL;
  }
  if (state == navflex_rog_map::OccupancyState::UNKNOWN) {
    return Res::UNKNOWN;
  }
  return Res::FREE;
}

// ── CheckPoint ───────────────────────────────────────────────────────────────

void RogMapNavNode::checkPointCallback(
  const std::shared_ptr<nav2_msgs::srv::CheckPoint::Request> request,
  std::shared_ptr<nav2_msgs::srv::CheckPoint::Response> response)
{
  using Res = nav2_msgs::srv::CheckPoint::Response;

  auto rog_map_ros = selectCostmap(request->costmap);

  // Transform point into costmap frame
  geometry_msgs::msg::PointStamped pt_in = request->point;
  geometry_msgs::msg::PointStamped pt_out;
  try {
    pt_out = tf_listener_ptr_->transform(
      pt_in, rog_map_ros->getGlobalFrameID(),
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "CheckPoint TF failed: %s", ex.what());
    response->state = Res::UNKNOWN;
    response->cost = 0;
    return;
  }

  const auto map = rog_map_ros->getRogMap();
  navflex_rog_map::Index3D index;
  if (!map->worldToGrid(pt_out.point, index)) {
    response->state = Res::OUTSIDE;
    response->cost = 0;
    return;
  }

  response->state = occupancyToState(map->localState(pt_out.point));
  response->cost = static_cast<uint32_t>(map->probability(index) * 100.0F);
}

// ── CheckPose ────────────────────────────────────────────────────────────────

void RogMapNavNode::checkPoseCallback(
  const std::shared_ptr<nav2_msgs::srv::CheckPose::Request> request,
  std::shared_ptr<nav2_msgs::srv::CheckPose::Response> response)
{
  using Res = nav2_msgs::srv::CheckPose::Response;

  auto rog_map_ros = selectCostmap(request->costmap);

  // Resolve pose: either use current robot pose or the one in the request
  geometry_msgs::msg::PoseStamped pose_in;
  if (request->current_pose) {
    pose_in.header.frame_id = robot_frame_;
    pose_in.header.stamp = now();
    pose_in.pose.orientation.w = 1.0;
  } else {
    pose_in = request->pose;
  }

  // Transform to costmap frame
  geometry_msgs::msg::PoseStamped pose_out;
  try {
    pose_out = tf_listener_ptr_->transform(
      pose_in, rog_map_ros->getGlobalFrameID(),
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "CheckPose TF failed: %s", ex.what());
    response->state = Res::UNKNOWN;
    response->cost = 0;
    return;
  }

  const auto map = rog_map_ros->getRogMap();
  navflex_rog_map::Index3D index;
  if (!map->worldToGrid(pose_out.pose.position, index)) {
    response->state = Res::OUTSIDE;
    response->cost = 0;
    return;
  }
  response->state = map->isCollisionFree(pose_out.pose.position, request->safety_dist) ?
    occupancyToState(map->localState(pose_out.pose.position)) : Res::LETHAL;
  response->cost = static_cast<uint32_t>(map->probability(index) * 100.0F);
}

// ── CheckPath ────────────────────────────────────────────────────────────────

void RogMapNavNode::checkPathCallback(
  const std::shared_ptr<nav2_msgs::srv::CheckPath::Request> request,
  std::shared_ptr<nav2_msgs::srv::CheckPath::Response> response)
{
  using Res = nav2_msgs::srv::CheckPath::Response;

  auto rog_map_ros = selectCostmap(request->costmap);
  const auto map = rog_map_ros->getRogMap();
  uint8_t worst_state = Res::FREE;
  uint32_t last_checked = 0;
  uint32_t total_cost = 0;

  const auto & poses = request->path.poses;
  for (uint32_t i = 0; i < static_cast<uint32_t>(poses.size()); ++i) {
    // Skip poses as configured
    if (request->skip_poses > 0 && i > 0 && (i % (request->skip_poses + 1)) != 0) {
      continue;
    }

    last_checked = i;

    geometry_msgs::msg::PoseStamped pose_out;
    try {
      pose_out = tf_listener_ptr_->transform(
        poses[i], rog_map_ros->getGlobalFrameID(),
        tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &) {
      worst_state = std::max(worst_state, static_cast<uint8_t>(Res::UNKNOWN));
      continue;
    }
    navflex_rog_map::Index3D index;
    if (!map->worldToGrid(pose_out.pose.position, index)) {
      worst_state = std::max(worst_state, static_cast<uint8_t>(Res::OUTSIDE));
      continue;
    }
    const bool collision_free = request->path_cells_only ||
      map->isCollisionFree(pose_out.pose.position, request->safety_dist);
    const uint8_t cell_state = collision_free ?
      occupancyToState(map->localState(pose_out.pose.position)) : Res::LETHAL;
    total_cost += static_cast<uint32_t>(map->probability(index) * 100.0F);
    worst_state = std::max(worst_state, cell_state);

    if (request->return_on > 0 && worst_state >= request->return_on) {
      break;
    }
  }

  response->last_checked = last_checked;
  response->state = worst_state;
  response->cost = total_cost;
}

} // namespace navflex_nav

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navflex_nav::RogMapNavNode)
