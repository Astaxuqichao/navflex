#include "navflex_costmap_nav/navflex_costmap_nav.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "tf2/utils.h"

#include <vector>

namespace navflex_costmap_nav
{

CostmapNavNode::CostmapNavNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("navflex_costmap_nav", "", options)
{
  RCLCPP_INFO(get_logger(), "[Navflex] CostmapNavNode created");
}

CostmapNavNode::~CostmapNavNode()
{
  planner_server_thread_.reset();
  planner_server_.reset();
  controller_server_thread_.reset();
  controller_server_.reset();
  behavior_server_thread_.reset();
  behavior_server_.reset();

  global_costmap_thread_.reset();
  local_costmap_thread_.reset();
  global_costmap_.reset();
  local_costmap_.reset();
  RCLCPP_INFO(get_logger(), "CostmapNavNode destroyed.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CostmapNavNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "[Navflex] configuring costmaps and action servers");

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

  global_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");

  local_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap");

  global_costmap_->configure();
  local_costmap_->configure();
  tf_listener_ptr_ = global_costmap_->getTfBuffer();

  robot_info_ = std::make_shared<navflex_utility::RobotInformation>(
    node, tf_listener_ptr_, global_frame_, robot_frame_,
    rclcpp::Duration::from_seconds(tf_timeout_s),
    node->get_parameter("odom_topic").as_string());
  global_costmap_thread_ =
    std::make_unique<nav2_util::NodeThread>(global_costmap_);
  local_costmap_thread_ =
    std::make_unique<nav2_util::NodeThread>(local_costmap_);

  controller_server_ =
    std::make_shared<navflex_costmap_nav::ControllerCostmapServer>(local_costmap_, robot_info_);
  controller_server_->configure();
  controller_server_thread_ =
    std::make_unique<nav2_util::NodeThread>(controller_server_);

  planner_server_ =
    std::make_shared<navflex_costmap_nav::PlannerCostmapServer>(global_costmap_, robot_info_);
  planner_server_->configure();
  planner_server_thread_ =
    std::make_unique<nav2_util::NodeThread>(planner_server_);

  behavior_server_ = std::make_shared<navflex_costmap_nav::BehaviorCostmapServer>(
    global_costmap_, local_costmap_);
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
CostmapNavNode::on_activate(const rclcpp_lifecycle::State &)
{
  global_costmap_->activate();
  local_costmap_->activate();
  planner_server_->activate();
  controller_server_->activate();
  behavior_server_->activate();
  createBond();

  // Advertise costmap query services
  check_point_srv_ = create_service<nav2_msgs::srv::CheckPoint>(
    "check_point_cost",
    std::bind(
      &CostmapNavNode::checkPointCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  check_pose_srv_ = create_service<nav2_msgs::srv::CheckPose>(
    "check_pose_cost",
    std::bind(
      &CostmapNavNode::checkPoseCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  check_path_srv_ = create_service<nav2_msgs::srv::CheckPath>(
    "check_path_cost",
    std::bind(
      &CostmapNavNode::checkPathCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "[Navflex] active: planner, controller, behavior, costmaps");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
         CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CostmapNavNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  planner_server_->deactivate();
  controller_server_->deactivate();
  behavior_server_->deactivate();

  // Remove costmap query services
  check_point_srv_.reset();
  check_pose_srv_.reset();
  check_path_srv_.reset();

  global_costmap_->deactivate();
  local_costmap_->deactivate();

  destroyBond();
  RCLCPP_INFO(get_logger(), "CostmapNavNode deactivated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
         CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CostmapNavNode::on_cleanup(const rclcpp_lifecycle::State &)
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
  global_costmap_thread_.reset();
  local_costmap_thread_.reset();
  global_costmap_->cleanup();
  local_costmap_->cleanup();
  // Must reset these shared_ptrs so their ROS nodes are destroyed before
  // on_configure creates new ones with the same names.  Without this,
  // the old Costmap2DROS nodes survive (held by tf_listener_ptr_ /
  // robot_info_), causing duplicate-node conflicts on the next configure.
  robot_info_.reset();
  tf_listener_ptr_.reset();
  global_costmap_.reset();
  local_costmap_.reset();
  RCLCPP_INFO(get_logger(), "CostmapNavNode cleaned up.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
         CallbackReturn::SUCCESS;
}

} // namespace navflex_costmap_nav


// costmap query service implementations
namespace navflex_costmap_nav
{

// ── helpers ─────────────────────────────────────────────────────────────────

std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
CostmapNavNode::selectCostmap(uint8_t costmap_id)
{
  using Req = nav2_msgs::srv::CheckPoint::Request;
  if (costmap_id == Req::LOCAL_COSTMAP) {
    return local_costmap_;
  }
  return global_costmap_;  // default: global
}

uint8_t CostmapNavNode::cellCostToState(unsigned char cost)
{
  using Res = nav2_msgs::srv::CheckPoint::Response;
  if (cost == nav2_costmap_2d::NO_INFORMATION) {return Res::UNKNOWN;}
  if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {return Res::LETHAL;}
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {return Res::INSCRIBED;}
  return Res::FREE;
}

// ── CheckPoint ───────────────────────────────────────────────────────────────

void CostmapNavNode::checkPointCallback(
  const std::shared_ptr<nav2_msgs::srv::CheckPoint::Request> request,
  std::shared_ptr<nav2_msgs::srv::CheckPoint::Response> response)
{
  using Res = nav2_msgs::srv::CheckPoint::Response;

  auto costmap_ros = selectCostmap(request->costmap);

  // Transform point into costmap frame
  geometry_msgs::msg::PointStamped pt_in = request->point;
  geometry_msgs::msg::PointStamped pt_out;
  try {
    pt_out = tf_listener_ptr_->transform(
      pt_in, costmap_ros->getGlobalFrameID(),
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "CheckPoint TF failed: %s", ex.what());
    response->state = Res::UNKNOWN;
    response->cost = 0;
    return;
  }

  auto * costmap = costmap_ros->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  unsigned int mx, my;
  if (!costmap->worldToMap(pt_out.point.x, pt_out.point.y, mx, my)) {
    response->state = Res::OUTSIDE;
    response->cost = 0;
    return;
  }

  unsigned char cost = costmap->getCost(mx, my);
  response->state = cellCostToState(cost);
  response->cost = static_cast<uint32_t>(cost);
}

// ── CheckPose ────────────────────────────────────────────────────────────────

void CostmapNavNode::checkPoseCallback(
  const std::shared_ptr<nav2_msgs::srv::CheckPose::Request> request,
  std::shared_ptr<nav2_msgs::srv::CheckPose::Response> response)
{
  using Res = nav2_msgs::srv::CheckPose::Response;

  auto costmap_ros = selectCostmap(request->costmap);

  // Resolve pose: either use current robot pose or the one in the request
  geometry_msgs::msg::PoseStamped pose_in;
  if (request->current_pose) {
    if (!costmap_ros->getRobotPose(pose_in)) {
      RCLCPP_WARN(get_logger(), "CheckPose: could not get robot pose");
      response->state = Res::UNKNOWN;
      response->cost = 0;
      return;
    }
  } else {
    pose_in = request->pose;
  }

  // Transform to costmap frame
  geometry_msgs::msg::PoseStamped pose_out;
  try {
    pose_out = tf_listener_ptr_->transform(
      pose_in, costmap_ros->getGlobalFrameID(),
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "CheckPose TF failed: %s", ex.what());
    response->state = Res::UNKNOWN;
    response->cost = 0;
    return;
  }

  double x = pose_out.pose.position.x;
  double y = pose_out.pose.position.y;
  double yaw = tf2::getYaw(pose_out.pose.orientation);

  // Build footprint padded by safety_dist
  auto footprint = costmap_ros->getRobotFootprint();
  nav2_costmap_2d::padFootprint(footprint, request->safety_dist);

  auto * costmap = costmap_ros->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  // Use FootprintCollisionChecker to query cost at this pose
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  double raw_cost = checker.footprintCostAtPose(x, y, yaw, footprint);

  if (raw_cost < 0) {
    // Negative means collision (lethal) or partially outside
    response->state = Res::LETHAL;
    response->cost = static_cast<uint32_t>(nav2_costmap_2d::LETHAL_OBSTACLE);
    return;
  }

  auto cell_cost = static_cast<unsigned char>(raw_cost);
  response->state = cellCostToState(cell_cost);

  // Apply optional cost multipliers to produce weighted total cost
  double weighted = static_cast<double>(cell_cost);
  if (request->lethal_cost_mult > 0.0f && cell_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    weighted *= request->lethal_cost_mult;
  } else if (request->inscrib_cost_mult > 0.0f &&
    cell_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    weighted *= request->inscrib_cost_mult;
  } else if (request->unknown_cost_mult > 0.0f &&
    cell_cost == nav2_costmap_2d::NO_INFORMATION)
  {
    weighted *= request->unknown_cost_mult;
  }
  response->cost = static_cast<uint32_t>(weighted);
}

// ── CheckPath ────────────────────────────────────────────────────────────────

void CostmapNavNode::checkPathCallback(
  const std::shared_ptr<nav2_msgs::srv::CheckPath::Request> request,
  std::shared_ptr<nav2_msgs::srv::CheckPath::Response> response)
{
  using Res = nav2_msgs::srv::CheckPath::Response;

  auto costmap_ros = selectCostmap(request->costmap);
  auto footprint = costmap_ros->getRobotFootprint();
  nav2_costmap_2d::padFootprint(footprint, request->safety_dist);

  struct PathPoseCheck
  {
    uint32_t index;
    geometry_msgs::msg::PoseStamped pose;
  };

  std::vector<PathPoseCheck> transformed_poses;
  transformed_poses.reserve(request->path.poses.size());

  uint8_t worst_state = Res::FREE;
  uint32_t last_checked = 0;

  const auto & poses = request->path.poses;
  for (uint32_t i = 0; i < static_cast<uint32_t>(poses.size()); ++i) {
    // Skip poses as configured
    if (request->skip_poses > 0 && i > 0 && (i % (request->skip_poses + 1)) != 0) {
      continue;
    }

    last_checked = i;

    try {
      auto pose_out = tf_listener_ptr_->transform(
        poses[i], costmap_ros->getGlobalFrameID(),
        tf2::durationFromSec(0.1));
      transformed_poses.push_back({i, pose_out});
    } catch (const tf2::TransformException &) {
      worst_state = std::max(worst_state, static_cast<uint8_t>(Res::UNKNOWN));
      continue;
    }
  }

  auto * costmap = costmap_ros->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);

  uint32_t total_cost = 0;
  for (const auto & transformed : transformed_poses) {
    last_checked = transformed.index;
    const auto & pose_out = transformed.pose;

    double x = pose_out.pose.position.x;
    double y = pose_out.pose.position.y;
    double yaw = tf2::getYaw(pose_out.pose.orientation);

    double raw_cost;
    if (request->path_cells_only) {
      // Only check the cell directly under the pose, ignore footprint
      unsigned int mx, my;
      if (!costmap->worldToMap(x, y, mx, my)) {
        worst_state = std::max(worst_state, static_cast<uint8_t>(Res::OUTSIDE));
        total_cost += nav2_costmap_2d::LETHAL_OBSTACLE;
        if (request->return_on > 0 && worst_state >= request->return_on) {break;}
        continue;
      }
      raw_cost = static_cast<double>(costmap->getCost(mx, my));
    } else {
      raw_cost = checker.footprintCostAtPose(x, y, yaw, footprint);
    }

    uint8_t cell_state;
    unsigned char cell_cost;
    if (raw_cost < 0) {
      cell_state = Res::LETHAL;
      cell_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    } else {
      cell_cost = static_cast<unsigned char>(raw_cost);
      cell_state = cellCostToState(cell_cost);
    }

    // Apply multipliers
    double weighted = static_cast<double>(cell_cost);
    if (request->lethal_cost_mult > 0.0f && cell_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      weighted *= request->lethal_cost_mult;
    } else if (request->inscrib_cost_mult > 0.0f &&
      cell_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      weighted *= request->inscrib_cost_mult;
    } else if (request->unknown_cost_mult > 0.0f &&
      cell_cost == nav2_costmap_2d::NO_INFORMATION)
    {
      weighted *= request->unknown_cost_mult;
    }

    total_cost += static_cast<uint32_t>(weighted);
    worst_state = std::max(worst_state, cell_state);

    if (request->return_on > 0 && worst_state >= request->return_on) {
      break;
    }
  }

  response->last_checked = last_checked;
  response->state = worst_state;
  response->cost = total_cost;
}

} // namespace navflex_costmap_nav

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navflex_costmap_nav::CostmapNavNode)
