#include "navflex_frontier_planner/fael_frontier_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <unordered_map>
#include <utility>

#include "rclcpp/exceptions.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "ufo/map/point_cloud.h"

namespace navflex_frontier_planner
{

namespace
{
double sqr(double value) {return value * value;}

template<typename PublisherT, typename MessageT>
bool safePublish(
  const PublisherT & publisher,
  const MessageT & message,
  const rclcpp::Logger & logger,
  const char * topic_label)
{
  try {
    publisher->publish(message);
    return true;
  } catch (const rclcpp::exceptions::RCLError & ex) {
    RCLCPP_WARN(
      logger,
      "Skipping frontier debug publish on %s after RCL publish failure: %s",
      topic_label, ex.what());
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      logger,
      "Skipping frontier debug publish on %s after exception: %s",
      topic_label, ex.what());
  }
  return false;
}

struct GridCell
{
  int x;
  int y;

  bool operator==(const GridCell & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct GridCellHash
{
  std::size_t operator()(const GridCell & cell) const
  {
    const auto hx = std::hash<int>{}(cell.x);
    const auto hy = std::hash<int>{}(cell.y);
    return hx ^ (hy + 0x9e3779b9u + (hx << 6) + (hx >> 2));
  }
};

GridCell toGridCell(const Point3 & point, double cell_size)
{
  return GridCell{
    static_cast<int>(std::floor(point.x / cell_size)),
    static_cast<int>(std::floor(point.y / cell_size))};
}

std::vector<std::size_t> radiusCandidates(
  const std::unordered_map<GridCell, std::vector<std::size_t>, GridCellHash> & grid,
  const Point3 & point,
  double radius,
  double cell_size)
{
  std::vector<std::size_t> candidates;
  const auto center = toGridCell(point, cell_size);
  const int reach = std::max(1, static_cast<int>(std::ceil(radius / cell_size)));
  for (int dx = -reach; dx <= reach; ++dx) {
    for (int dy = -reach; dy <= reach; ++dy) {
      const auto it = grid.find(GridCell{center.x + dx, center.y + dy});
      if (it == grid.end()) {
        continue;
      }
      candidates.insert(candidates.end(), it->second.begin(), it->second.end());
    }
  }
  return candidates;
}

double cross2D(const Point3 & a, const Point3 & b, const Point3 & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool pointOnSegment2D(const Point3 & a, const Point3 & b, const Point3 & point)
{
  constexpr double epsilon = 1e-6;
  return std::abs(cross2D(a, b, point)) <= epsilon &&
         point.x >= std::min(a.x, b.x) - epsilon &&
         point.x <= std::max(a.x, b.x) + epsilon &&
         point.y >= std::min(a.y, b.y) - epsilon &&
         point.y <= std::max(a.y, b.y) + epsilon;
}

bool segmentsIntersect2D(
  const Point3 & a,
  const Point3 & b,
  const Point3 & c,
  const Point3 & d)
{
  constexpr double epsilon = 1e-6;
  const double ab_c = cross2D(a, b, c);
  const double ab_d = cross2D(a, b, d);
  const double cd_a = cross2D(c, d, a);
  const double cd_b = cross2D(c, d, b);
  if (((ab_c > epsilon && ab_d < -epsilon) || (ab_c < -epsilon && ab_d > epsilon)) &&
    ((cd_a > epsilon && cd_b < -epsilon) || (cd_a < -epsilon && cd_b > epsilon)))
  {
    return true;
  }
  return (std::abs(ab_c) <= epsilon && pointOnSegment2D(a, b, c)) ||
         (std::abs(ab_d) <= epsilon && pointOnSegment2D(a, b, d)) ||
         (std::abs(cd_a) <= epsilon && pointOnSegment2D(c, d, a)) ||
         (std::abs(cd_b) <= epsilon && pointOnSegment2D(c, d, b));
}

geometry_msgs::msg::PoseStamped makePose(
  const Point3 & point,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = point.x;
  pose.pose.position.y = point.y;
  pose.pose.position.z = point.z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

void updatePathOrientations(nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    return;
  }

  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & current = path.poses[i].pose.position;
    geometry_msgs::msg::Point target;
    if (i + 1 < path.poses.size()) {
      target = path.poses[i + 1].pose.position;
    } else {
      target = path.poses[i - 1].pose.position;
    }

    double dx = target.x - current.x;
    double dy = target.y - current.y;
    if (i + 1 == path.poses.size()) {
      dx = current.x - target.x;
      dy = current.y - target.y;
    }
    if (std::hypot(dx, dy) < 1e-6) {
      continue;
    }

    const double yaw = std::atan2(dy, dx);
    auto & orientation = path.poses[i].pose.orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = std::sin(yaw * 0.5);
    orientation.w = std::cos(yaw * 0.5);
  }
}

std::mutex g_shared_maps_mutex;
std::weak_ptr<FaelFrontierCore::SharedMapState> g_shared_map;
}  // namespace

double Point3::distanceXY(const Point3 & other) const
{
  return std::hypot(x - other.x, y - other.y);
}

double Point3::distance(const Point3 & other) const
{
  return std::sqrt(sqr(x - other.x) + sqr(y - other.y) + sqr(z - other.z));
}

void FaelFrontierCore::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const std::string shared_parameter_prefix = "frontier_shared_config.";
  auto declare_if_missing = [&](const std::string & param, const auto & value) {
      const auto parameter_name = shared_parameter_prefix + param;
      if (node->has_parameter(parameter_name)) {
        return;
      }
      node->declare_parameter(parameter_name, rclcpp::ParameterValue(value));
    };
  auto get_parameter = [&](const std::string & param, auto & value) {
      node->get_parameter(shared_parameter_prefix + param, value);
    };

  declare_if_missing("frame_id", frame_id_);
  declare_if_missing("point_cloud_topic", point_cloud_topic_);
  declare_if_missing("visualization_topic_prefix", visualization_topic_prefix_);
  declare_if_missing("resolution", resolution_);
  declare_if_missing("depth_levels", depth_levels_);
  declare_if_missing("insert_depth", insert_depth_);
  declare_if_missing("insert_discrete", insert_discrete_);
  declare_if_missing("simple_ray_casting", simple_ray_casting_);
  declare_if_missing("early_stopping", early_stopping_);
  declare_if_missing("publish_map_clouds", publish_map_clouds_);
  declare_if_missing("map_publish_period", map_publish_period_);
  declare_if_missing("max_range", max_range_);
  declare_if_missing("sample_dist", sample_dist_);
  declare_if_missing("local_range", local_range_);
  declare_if_missing("candidate_visibility_range", candidate_visibility_range_);
  declare_if_missing("reuse_cached_candidates", reuse_cached_candidates_);
  declare_if_missing("cache_robot_move_threshold", cache_robot_move_threshold_);
  declare_if_missing("candidate_recompute_period", candidate_recompute_period_);
  declare_if_missing("frontier_attach_grid_size", frontier_attach_grid_size_);
  declare_if_missing("global_frontier_revalidate_max_cells", global_frontier_revalidate_max_cells_);
  declare_if_missing("frontier_visibility_max_viewpoints", frontier_visibility_max_viewpoints_);
  declare_if_missing("viewpoint_gain_threshold", viewpoint_gain_threshold_);
  declare_if_missing("min_frontier_area", min_frontier_area_);
  declare_if_missing("candidate_separation", candidate_separation_);
  declare_if_missing("frontier_distance_weight", frontier_distance_weight_);
  declare_if_missing("min_candidate_count", min_candidate_count_);
  declare_if_missing("max_candidate_count", max_candidate_count_);
  declare_if_missing("frontier_gain", frontier_gain_);
  declare_if_missing("unknown_gain_range", unknown_gain_range_);
  declare_if_missing("unknown_gain_step", unknown_gain_step_);
  declare_if_missing("min_unknown_gain", min_unknown_gain_);
  declare_if_missing("distance_weight", distance_weight_);
  declare_if_missing("visited_radius", visited_radius_);
  declare_if_missing("visited_penalty", visited_penalty_);
  declare_if_missing("known_gain_penalty", known_gain_penalty_);
  declare_if_missing("min_candidate_dist", min_candidate_dist_);
  declare_if_missing("min_robot_frontier_dist", min_robot_frontier_dist_);
  declare_if_missing("robot_clear_radius", robot_clear_radius_);
  declare_if_missing("unknown_clear_radius", unknown_clear_radius_);
  declare_if_missing("viewpoint_free_z_min", viewpoint_free_z_min_);
  declare_if_missing("viewpoint_free_z_max", viewpoint_free_z_max_);
  declare_if_missing("viewpoint_free_z_step", viewpoint_free_z_step_);
  declare_if_missing("sensor_height", sensor_height_);
  declare_if_missing("frontier_slope_deg", frontier_slope_deg_);
  declare_if_missing("viewpoint_slope_deg", viewpoint_slope_deg_);
  declare_if_missing("topology_enabled", topology_enabled_);
  declare_if_missing("topology_initialization_period", topology_initialization_period_);
  declare_if_missing("topology_update_distance", topology_update_distance_);
  declare_if_missing("topology_update_radius", topology_update_radius_);
  declare_if_missing("topology_node_spacing", topology_node_spacing_);
  declare_if_missing("topology_local_node_spacing", topology_local_node_spacing_);
  declare_if_missing("topology_min_clearance", topology_min_clearance_);
  declare_if_missing("topology_local_min_clearance", topology_local_min_clearance_);
  declare_if_missing("topology_max_clearance", topology_max_clearance_);
  declare_if_missing("topology_connection_radius", topology_connection_radius_);
  declare_if_missing("topology_attach_radius", topology_attach_radius_);
  declare_if_missing("topology_z_tolerance", topology_z_tolerance_);
  declare_if_missing("topology_initial_min_nodes", topology_initial_min_nodes_);
  declare_if_missing("topology_max_samples", topology_max_samples_);
  declare_if_missing("topology_max_nodes", topology_max_nodes_);
  declare_if_missing("topology_max_neighbors", topology_max_neighbors_);

  get_parameter("frame_id", frame_id_);
  get_parameter("point_cloud_topic", point_cloud_topic_);
  get_parameter("visualization_topic_prefix", visualization_topic_prefix_);
  get_parameter("resolution", resolution_);
  get_parameter("depth_levels", depth_levels_);
  get_parameter("insert_depth", insert_depth_);
  get_parameter("insert_discrete", insert_discrete_);
  get_parameter("simple_ray_casting", simple_ray_casting_);
  get_parameter("early_stopping", early_stopping_);
  get_parameter("publish_map_clouds", publish_map_clouds_);
  get_parameter("map_publish_period", map_publish_period_);
  get_parameter("max_range", max_range_);
  get_parameter("sample_dist", sample_dist_);
  get_parameter("local_range", local_range_);
  get_parameter("candidate_visibility_range", candidate_visibility_range_);
  get_parameter("reuse_cached_candidates", reuse_cached_candidates_);
  get_parameter("cache_robot_move_threshold", cache_robot_move_threshold_);
  get_parameter("candidate_recompute_period", candidate_recompute_period_);
  get_parameter("frontier_attach_grid_size", frontier_attach_grid_size_);
  get_parameter("global_frontier_revalidate_max_cells", global_frontier_revalidate_max_cells_);
  get_parameter("frontier_visibility_max_viewpoints", frontier_visibility_max_viewpoints_);
  get_parameter("viewpoint_gain_threshold", viewpoint_gain_threshold_);
  get_parameter("min_frontier_area", min_frontier_area_);
  get_parameter("candidate_separation", candidate_separation_);
  get_parameter("frontier_distance_weight", frontier_distance_weight_);
  get_parameter("min_candidate_count", min_candidate_count_);
  get_parameter("max_candidate_count", max_candidate_count_);
  get_parameter("frontier_gain", frontier_gain_);
  get_parameter("unknown_gain_range", unknown_gain_range_);
  get_parameter("unknown_gain_step", unknown_gain_step_);
  get_parameter("min_unknown_gain", min_unknown_gain_);
  get_parameter("distance_weight", distance_weight_);
  get_parameter("visited_radius", visited_radius_);
  get_parameter("visited_penalty", visited_penalty_);
  get_parameter("known_gain_penalty", known_gain_penalty_);
  get_parameter("min_candidate_dist", min_candidate_dist_);
  get_parameter("min_robot_frontier_dist", min_robot_frontier_dist_);
  get_parameter("robot_clear_radius", robot_clear_radius_);
  get_parameter("unknown_clear_radius", unknown_clear_radius_);
  get_parameter("viewpoint_free_z_min", viewpoint_free_z_min_);
  get_parameter("viewpoint_free_z_max", viewpoint_free_z_max_);
  get_parameter("viewpoint_free_z_step", viewpoint_free_z_step_);
  get_parameter("sensor_height", sensor_height_);
  get_parameter("frontier_slope_deg", frontier_slope_deg_);
  get_parameter("viewpoint_slope_deg", viewpoint_slope_deg_);
  get_parameter("topology_enabled", topology_enabled_);
  get_parameter("topology_initialization_period", topology_initialization_period_);
  get_parameter("topology_update_distance", topology_update_distance_);
  get_parameter("topology_update_radius", topology_update_radius_);
  get_parameter("topology_node_spacing", topology_node_spacing_);
  get_parameter("topology_local_node_spacing", topology_local_node_spacing_);
  get_parameter("topology_min_clearance", topology_min_clearance_);
  get_parameter("topology_local_min_clearance", topology_local_min_clearance_);
  get_parameter("topology_max_clearance", topology_max_clearance_);
  get_parameter("topology_connection_radius", topology_connection_radius_);
  get_parameter("topology_attach_radius", topology_attach_radius_);
  get_parameter("topology_z_tolerance", topology_z_tolerance_);
  get_parameter("topology_initial_min_nodes", topology_initial_min_nodes_);
  get_parameter("topology_max_samples", topology_max_samples_);
  get_parameter("topology_max_nodes", topology_max_nodes_);
  get_parameter("topology_max_neighbors", topology_max_neighbors_);

  map_state_ = getSharedMapState(resolution_, depth_levels_);

  bool owns_shared_map = false;
  {
    std::lock_guard<std::mutex> lock(map_state_->mutex);
    if (!map_state_->cloud_sub) {
      owns_shared_map = true;
      map_state_->owner_name = name_;
      map_state_->cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FaelFrontierCore::pointCloudCallback, this, std::placeholders::_1));
    }
  }
  auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
  const std::string topic_prefix =
    visualization_topic_prefix_.empty() ? name_ : visualization_topic_prefix_;
  candidate_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    topic_prefix + "/candidates", latched_qos);
  topology_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    topic_prefix + "/topology_map", rclcpp::SystemDefaultsQoS());
  selected_candidate_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_prefix + "/selected_candidate", latched_qos);
  occupied_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    topic_prefix + "/ufomap_occupied_cloud", rclcpp::SystemDefaultsQoS());
  free_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    topic_prefix + "/ufomap_free_cloud", rclcpp::SystemDefaultsQoS());
  if (owns_shared_map) {
    const auto period = std::chrono::milliseconds(
      static_cast<int64_t>(std::max(0.1, topology_initialization_period_) * 1000.0));
    topology_initialization_timer_ = node->create_wall_timer(
      period, std::bind(&FaelFrontierCore::topologyInitializationTimerCallback, this));
  }
  last_map_publish_time_ = clock_->now();

  RCLCPP_INFO(
    logger_,
    "[%s] configured FAEL UFOMap frontier selector: cloud=%s target_frame=%s "
    "resolution=%.3f depth_levels=%d insert_depth=%d shared_owner=%s",
    name_.c_str(), point_cloud_topic_.c_str(), frame_id_.c_str(), resolution_,
    depth_levels_, insert_depth_, map_state_->owner_name.c_str());
}

std::shared_ptr<FaelFrontierCore::SharedMapState> FaelFrontierCore::getSharedMapState(
  double resolution,
  int depth_levels)
{
  std::lock_guard<std::mutex> lock(g_shared_maps_mutex);
  if (auto existing = g_shared_map.lock()) {
    return existing;
  }

  auto state = std::make_shared<SharedMapState>();
  state->map = std::make_unique<ufo::map::OccupancyMap>(resolution, depth_levels, true);
  state->map->enableChangeDetection(true);
  g_shared_map = state;
  return state;
}

void FaelFrontierCore::activate()
{
  if (candidate_pub_) {
    candidate_pub_->on_activate();
  }
  if (topology_pub_) {
    topology_pub_->on_activate();
  }
  if (selected_candidate_pub_) {
    selected_candidate_pub_->on_activate();
  }
  if (occupied_map_pub_) {
    occupied_map_pub_->on_activate();
  }
  if (free_map_pub_) {
    free_map_pub_->on_activate();
  }
  topologyInitializationTimerCallback();
}

void FaelFrontierCore::deactivate()
{
  if (candidate_pub_) {
    candidate_pub_->on_deactivate();
  }
  if (topology_pub_) {
    topology_pub_->on_deactivate();
  }
  if (selected_candidate_pub_) {
    selected_candidate_pub_->on_deactivate();
  }
  if (occupied_map_pub_) {
    occupied_map_pub_->on_deactivate();
  }
  if (free_map_pub_) {
    free_map_pub_->on_deactivate();
  }
}

void FaelFrontierCore::cleanup()
{
  topology_initialization_timer_.reset();
  map_state_.reset();
  candidate_pub_.reset();
  topology_pub_.reset();
  selected_candidate_pub_.reset();
  occupied_map_pub_.reset();
  free_map_pub_.reset();
}

void FaelFrontierCore::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!tf_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_msg;
  try {
    transform_msg = tf_->lookupTransform(
      frame_id_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[%s] waiting for TF %s -> %s: %s",
      name_.c_str(), msg->header.frame_id.c_str(), frame_id_.c_str(), ex.what());
    return;
  }

  const auto & t = transform_msg.transform.translation;
  const auto & q = transform_msg.transform.rotation;
  Point3 sensor{t.x, t.y, t.z};

  tf2::Quaternion sensor_q(q.x, q.y, q.z, q.w);
  tf2::Transform sensor_tf(sensor_q, tf2::Vector3(t.x, t.y, t.z));

  std::lock_guard<std::mutex> lock(map_state_->mutex);
  map_state_->latest_sensor_position = sensor;
  map_state_->has_latest_sensor_position = true;
  ufo::map::PointCloud cloud;
  cloud.reserve(static_cast<std::size_t>(msg->width) * msg->height);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    const tf2::Vector3 local_point(*iter_x, *iter_y, *iter_z);
    const tf2::Vector3 world_point = sensor_tf * local_point;
    cloud.push_back(ufo::map::Point3(world_point.x(), world_point.y(), world_point.z()));
  }

  const ufo::map::Point3 sensor_origin(sensor.x, sensor.y, sensor.z);
  if (insert_discrete_) {
    map_state_->map->insertPointCloudDiscrete(
      sensor_origin, cloud, max_range_, insert_depth_, simple_ray_casting_,
      static_cast<unsigned int>(std::max(0, early_stopping_)), false);
  } else {
    map_state_->map->insertPointCloud(
      sensor_origin, cloud, max_range_, insert_depth_, simple_ray_casting_,
      static_cast<unsigned int>(std::max(0, early_stopping_)), false);
  }

  const int clear_radius = std::max(
    1, static_cast<int>(std::ceil(
      robot_clear_radius_ / resolution_)));
  for (int dx = -clear_radius; dx <= clear_radius; ++dx) {
    for (int dy = -clear_radius; dy <= clear_radius; ++dy) {
      for (int dz = -clear_radius; dz <= clear_radius; ++dz) {
        const auto point = ufo::map::Point3(
          sensor.x + static_cast<double>(dx) * resolution_,
          sensor.y + static_cast<double>(dy) * resolution_,
          sensor.z + static_cast<double>(dz) * resolution_);
        if (Point3{point.x(), point.y(), point.z()}.distance(sensor) <= robot_clear_radius_) {
          map_state_->map->setOccupancy(
            point, map_state_->map->getClampingThresMin(),
            insert_depth_);
        }
      }
    }
  }

  bool map_changed = false;
  for (auto it = map_state_->map->changesBegin(); it != map_state_->map->changesEnd(); ++it) {
    map_state_->changed_cell_codes.insert(*it);
    map_changed = true;
  }
  if (map_changed) {
    ++map_state_->map_revision;
  }
  map_state_->map->resetChangeDetection();
  map_state_->topology_plane_z = sensor.z;
  const bool initialize_from_accumulated_map =
    map_state_->initialized_from_accumulated_map == false;
  const bool topology_updated = updateTopologyMap(
    sensor, initialize_from_accumulated_map);
  if (topology_updated) {
    publishTopology(map_state_->candidates, sensor, nullptr, false);
    if (topology_pub_ && topology_pub_->is_activated() && map_state_->has_topology) {
      map_state_->startup_topology_published = true;
    }
  }
  publishMapClouds();
}

void FaelFrontierCore::topologyInitializationTimerCallback()
{
  if (!map_state_) {
    return;
  }
  std::lock_guard<std::mutex> lock(map_state_->mutex);
  if (!map_state_->has_latest_sensor_position || map_state_->map_revision == 0) {
    return;
  }

  const bool needs_initial_generation =
    map_state_->initialized_from_accumulated_map == false &&
    (map_state_->has_topology_update_origin == false ||
    map_state_->topology_map_revision != map_state_->map_revision);
  if (needs_initial_generation) {
    updateTopologyMap(map_state_->latest_sensor_position, true);
  }

  if (map_state_->has_topology && map_state_->startup_topology_published == false &&
    topology_pub_ && topology_pub_->is_activated())
  {
    publishTopology(
      map_state_->candidates, map_state_->latest_sensor_position, nullptr, false);
    map_state_->startup_topology_published = true;
  }
}

ufo::map::Point3 FaelFrontierCore::toUfoPoint(const Point3 & point) const
{
  return ufo::map::Point3(point.x, point.y, point.z);
}

Point3 FaelFrontierCore::fromUfoPoint(const ufo::map::Point3 & point) const
{
  return Point3{point.x(), point.y(), point.z()};
}

Point3 FaelFrontierCore::codeToPoint(const ufo::map::Code & code) const
{
  return fromUfoPoint(map_state_->map->toCoord(code.toKey(), code.getDepth()));
}

bool FaelFrontierCore::isNearOccupied(const Point3 & point, double radius) const
{
  const int cells = std::max(1, static_cast<int>(std::ceil(radius / resolution_)));
  for (int dx = -cells; dx <= cells; ++dx) {
    for (int dy = -cells; dy <= cells; ++dy) {
      for (int dz = -cells; dz <= cells; ++dz) {
        const auto p = ufo::map::Point3(
          point.x + static_cast<double>(dx) * resolution_,
          point.y + static_cast<double>(dy) * resolution_,
          point.z + static_cast<double>(dz) * resolution_);
        if (map_state_->map->isOccupied(
            p,
            insert_depth_) && fromUfoPoint(p).distance(point) <= radius)
        {
          return true;
        }
      }
    }
  }
  return false;
}

bool FaelFrontierCore::isNearUnknown(const Point3 & point, double radius) const
{
  const int cells = std::max(1, static_cast<int>(std::ceil(radius / resolution_)));
  for (int dx = -cells; dx <= cells; ++dx) {
    for (int dy = -cells; dy <= cells; ++dy) {
      for (int dz = -cells; dz <= cells; ++dz) {
        const auto p = ufo::map::Point3(
          point.x + static_cast<double>(dx) * resolution_,
          point.y + static_cast<double>(dy) * resolution_,
          point.z + static_cast<double>(dz) * resolution_);
        if (map_state_->map->isUnknown(
            p,
            insert_depth_) && fromUfoPoint(p).distance(point) <= radius)
        {
          return true;
        }
      }
    }
  }
  return false;
}

bool FaelFrontierCore::hasFreeVoxelNearHeight(const Point3 & point) const
{
  const double z_min = std::min(viewpoint_free_z_min_, viewpoint_free_z_max_);
  const double z_max = std::max(viewpoint_free_z_min_, viewpoint_free_z_max_);
  const double z_step = std::max(resolution_, viewpoint_free_z_step_);
  for (double dz = z_min; dz <= z_max + 1e-6; dz += z_step) {
    Point3 sample{point.x, point.y, point.z + dz};
    if (map_state_->map->isFree(toUfoPoint(sample), insert_depth_) &&
      !isNearOccupied(sample, robot_clear_radius_))
    {
      return true;
    }
  }
  return false;
}

bool FaelFrontierCore::isFrontier(const ufo::map::Code & code) const
{
  if (!map_state_->map->isFree(code)) {
    return false;
  }

  const auto center = map_state_->map->toCoord(code.toKey(), code.getDepth());
  const ufo::map::Code neighbors[4] = {
    map_state_->map->toCode(center.x() - resolution_, center.y(), center.z(), code.getDepth()),
    map_state_->map->toCode(center.x() + resolution_, center.y(), center.z(), code.getDepth()),
    map_state_->map->toCode(center.x(), center.y() - resolution_, center.z(), code.getDepth()),
    map_state_->map->toCode(center.x(), center.y() + resolution_, center.z(), code.getDepth())};

  for (const auto & neighbor : neighbors) {
    if (map_state_->map->isUnknown(neighbor)) {
      return true;
    }
  }
  return false;
}

bool FaelFrontierCore::isCollisionFree2D(const Point3 & from, const Point3 & to) const
{
  const double distance = from.distanceXY(to);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / resolution_)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    Point3 sample{
      from.x + (to.x - from.x) * t,
      from.y + (to.y - from.y) * t,
      to.z};
    if (isNearOccupied(sample, robot_clear_radius_)) {
      return false;
    }
  }
  return true;
}

bool FaelFrontierCore::isKnownFree2D(const Point3 & from, const Point3 & to) const
{
  const double distance = from.distanceXY(to);
  const double step_size = std::max(resolution_ * 0.5, 0.1);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / step_size)));
  const double plane_z = map_state_->topology_plane_z;
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    Point3 sample{
      from.x + (to.x - from.x) * t,
      from.y + (to.y - from.y) * t,
      plane_z};
    if (!map_state_->map->isFree(toUfoPoint(sample), insert_depth_) ||
      isNearOccupied(sample, robot_clear_radius_))
    {
      return false;
    }
  }
  return true;
}

bool FaelFrontierCore::isFrontierVisible(
  const Point3 & viewpoint,
  const Point3 & frontier) const
{
  return map_state_->map->isCollisionFree(
    toUfoPoint(viewpoint), toUfoPoint(frontier), true, insert_depth_);
}

double FaelFrontierCore::topologyClearance(const Point3 & point) const
{
  const double max_clearance = std::max(topology_min_clearance_, topology_max_clearance_);
  const int cells = std::max(1, static_cast<int>(std::ceil(max_clearance / resolution_)));
  double clearance = max_clearance;
  for (int dx = -cells; dx <= cells; ++dx) {
    for (int dy = -cells; dy <= cells; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const double distance = std::hypot(dx * resolution_, dy * resolution_);
      if (distance >= clearance || distance > max_clearance) {
        continue;
      }
      const ufo::map::Point3 sample(
        point.x + dx * resolution_, point.y + dy * resolution_, point.z);
      if (!map_state_->map->isFree(sample, insert_depth_)) {
        clearance = distance;
      }
    }
  }
  return clearance;
}

bool FaelFrontierCore::updateTopologyMap(
  const Point3 & current,
  bool initialize_from_accumulated_map)
{
  if (!topology_enabled_) {
    map_state_->topology_nodes.clear();
    map_state_->last_topology_path.clear();
    map_state_->has_topology = false;
    map_state_->has_topology_update_origin = false;
    map_state_->initialized_from_accumulated_map = false;
    return false;
  }

  Point3 update_center{current.x, current.y, map_state_->topology_plane_z};
  const double update_distance = std::max(resolution_, topology_update_distance_);
  const double motion_since_update = map_state_->has_topology_update_origin ?
    update_center.distanceXY(map_state_->topology_update_origin) : 0.0;
  const bool force_initial_update =
    initialize_from_accumulated_map &&
    map_state_->initialized_from_accumulated_map == false;
  if (map_state_->has_topology_update_origin && map_state_->has_topology &&
    motion_since_update < update_distance && force_initial_update == false)
  {
    return false;
  }

  const auto started = std::chrono::steady_clock::now();
  const auto now = clock_->now();
  const double local_priority_radius = std::max(
    topology_update_radius_, topology_node_spacing_ * 2.0);
  const double update_radius = force_initial_update ?
    std::numeric_limits<double>::infinity() :
    local_priority_radius;
  const double node_spacing = std::max(resolution_, topology_node_spacing_);
  const double connection_radius = std::max(node_spacing, topology_connection_radius_);
  const std::size_t old_node_count = map_state_->topology_nodes.size();

  std::vector<std::size_t> old_to_new(old_node_count, old_node_count);
  std::vector<TopologyNode> nodes;
  nodes.reserve(static_cast<std::size_t>(std::max(1, topology_max_nodes_)));
  for (std::size_t i = 0; i < old_node_count; ++i) {
    const auto & old_node = map_state_->topology_nodes[i];
    if (old_node.point.distanceXY(update_center) <= update_radius) {
      continue;
    }
    old_to_new[i] = nodes.size();
    nodes.push_back(TopologyNode{old_node.point, old_node.clearance, {}});
  }
  const std::size_t retained_node_count = nodes.size();
  const auto retained_nodes_done = std::chrono::steady_clock::now();

  auto edge_crosses_graph = [&](std::size_t from, std::size_t to) {
      for (std::size_t i = 0; i < nodes.size(); ++i) {
        for (const auto neighbor : nodes[i].neighbors) {
          if (neighbor <= i || neighbor >= nodes.size()) {
            continue;
          }
          if (i == from || i == to || neighbor == from || neighbor == to) {
            continue;
          }
          if (segmentsIntersect2D(
              nodes[from].point, nodes[to].point,
              nodes[i].point, nodes[neighbor].point))
          {
            return true;
          }
        }
      }
      return false;
    };

  struct EdgeCandidate
  {
    double distance;
    std::size_t from;
    std::size_t to;
  };
  std::vector<EdgeCandidate> retained_edges;
  for (std::size_t i = 0; i < old_node_count; ++i) {
    if (old_to_new[i] == old_node_count) {
      continue;
    }
    for (const auto neighbor : map_state_->topology_nodes[i].neighbors) {
      if (neighbor <= i || neighbor >= old_node_count ||
        old_to_new[neighbor] == old_node_count)
      {
        continue;
      }
      const auto from = old_to_new[i];
      const auto to = old_to_new[neighbor];
      const double distance = nodes[from].point.distanceXY(nodes[to].point);
      if (distance <= connection_radius) {
        retained_edges.push_back(EdgeCandidate{distance, from, to});
      }
    }
  }
  std::sort(
    retained_edges.begin(), retained_edges.end(),
    [](const EdgeCandidate & lhs, const EdgeCandidate & rhs) {
      return lhs.distance < rhs.distance;
    });
  const std::size_t max_neighbors = static_cast<std::size_t>(
    std::max(1, topology_max_neighbors_));
  for (const auto & edge : retained_edges) {
    if (nodes[edge.from].neighbors.size() >= max_neighbors ||
      nodes[edge.to].neighbors.size() >= max_neighbors ||
      !isKnownFree2D(nodes[edge.from].point, nodes[edge.to].point) ||
      edge_crosses_graph(edge.from, edge.to))
    {
      continue;
    }
    nodes[edge.from].neighbors.push_back(edge.to);
    nodes[edge.to].neighbors.push_back(edge.from);
  }
  const auto retained_edges_done = std::chrono::steady_clock::now();

  const double sample_spacing = std::max(resolution_, topology_node_spacing_ * 0.5);
  const double local_sample_spacing = std::max(
    resolution_, topology_local_node_spacing_ * 0.5);
  std::unordered_map<GridCell, Point3, GridCellHash> free_samples;
  std::unordered_map<GridCell, Point3, GridCellHash> local_free_samples;
  free_samples.reserve(static_cast<std::size_t>(std::max(1, topology_max_samples_)));
  local_free_samples.reserve(512);

  for (auto it = map_state_->map->beginLeaves(false, true, false, false, insert_depth_),
    it_end = map_state_->map->endLeaves(); it != it_end; ++it)
  {
    const auto center = it.getCenter();
    Point3 sample{center.x(), center.y(), map_state_->topology_plane_z};
    const double distance_to_robot = sample.distanceXY(update_center);
    if (distance_to_robot > update_radius ||
      std::abs(center.z() - map_state_->topology_plane_z) > topology_z_tolerance_)
    {
      continue;
    }
    if (!map_state_->map->isFree(toUfoPoint(sample), insert_depth_)) {
      continue;
    }
    if (force_initial_update && distance_to_robot <= local_priority_radius) {
      local_free_samples.emplace(toGridCell(sample, local_sample_spacing), sample);
      continue;
    }
    const auto sample_cell = toGridCell(sample, sample_spacing);
    if (free_samples.size() < static_cast<std::size_t>(std::max(1, topology_max_samples_))) {
      free_samples.emplace(sample_cell, sample);
    }
    if (force_initial_update == false &&
      free_samples.size() >= static_cast<std::size_t>(std::max(1, topology_max_samples_)))
    {
      break;
    }
  }
  const auto sampling_done = std::chrono::steady_clock::now();

  std::vector<TopologyNode> bubbles;
  bubbles.reserve(free_samples.size() + local_free_samples.size());
  for (const auto & entry : local_free_samples) {
    const double clearance = topologyClearance(entry.second);
    if (clearance + 1e-6 >= topology_local_min_clearance_) {
      bubbles.push_back(TopologyNode{entry.second, clearance, {}});
    }
  }
  for (const auto & entry : free_samples) {
    const double clearance = topologyClearance(entry.second);
    if (clearance + 1e-6 >= topology_min_clearance_) {
      bubbles.push_back(TopologyNode{entry.second, clearance, {}});
    }
  }
  std::sort(
    bubbles.begin(), bubbles.end(),
    [&](const TopologyNode & lhs, const TopologyNode & rhs) {
      if (force_initial_update) {
        const bool lhs_local = lhs.point.distanceXY(update_center) <= local_priority_radius;
        const bool rhs_local = rhs.point.distanceXY(update_center) <= local_priority_radius;
        if (lhs_local != rhs_local) {
          return lhs_local;
        }
      }
      return lhs.clearance > rhs.clearance;
    });
  const auto clearance_done = std::chrono::steady_clock::now();

  const std::size_t new_node_start = nodes.size();
  for (const auto & bubble : bubbles) {
    const bool local_node = bubble.point.distanceXY(update_center) <= local_priority_radius;
    const double separation = local_node ?
      std::max(resolution_, topology_local_node_spacing_) : node_spacing;
    bool covered = false;
    for (const auto & node : nodes) {
      if (bubble.point.distanceXY(node.point) < separation) {
        covered = true;
        break;
      }
    }
    if (!covered) {
      nodes.push_back(bubble);
      if (nodes.size() >= static_cast<std::size_t>(std::max(1, topology_max_nodes_))) {
        break;
      }
    }
  }
  bool robot_anchor_added = false;
  double nearest_robot_node = std::numeric_limits<double>::infinity();
  for (const auto & node : nodes) {
    nearest_robot_node = std::min(
      nearest_robot_node, node.point.distanceXY(update_center));
  }
  const std::size_t max_topology_nodes = static_cast<std::size_t>(
    std::max(1, topology_max_nodes_));
  if (force_initial_update) {
    nodes.erase(
      std::remove_if(
        nodes.begin(), nodes.end(),
        [&](const TopologyNode & node) {
          return node.point.distanceXY(update_center) < resolution_ * 0.5;
        }),
      nodes.end());
    if (nodes.size() >= max_topology_nodes) {
      nodes.pop_back();
    }
    nodes.push_back(
      TopologyNode{
        update_center, std::max(robot_clear_radius_, topology_local_min_clearance_), {}});
    robot_anchor_added = true;
  } else if (nearest_robot_node > std::max(resolution_, topology_local_node_spacing_) &&
    map_state_->map->isFree(toUfoPoint(update_center), insert_depth_) &&
    nodes.size() < max_topology_nodes)
  {
    nodes.push_back(
      TopologyNode{
        update_center, std::max(robot_clear_radius_, topology_local_min_clearance_), {}});
    robot_anchor_added = true;
  }
  const auto node_selection_done = std::chrono::steady_clock::now();

  std::unordered_map<GridCell, std::vector<std::size_t>, GridCellHash> node_grid;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    node_grid[toGridCell(nodes[i].point, connection_radius)].push_back(i);
  }
  std::vector<EdgeCandidate> new_edges;
  for (std::size_t i = new_node_start; i < nodes.size(); ++i) {
    for (const auto j : radiusCandidates(
        node_grid, nodes[i].point, connection_radius, connection_radius))
    {
      if (j == i || (j >= new_node_start && j < i)) {
        continue;
      }
      const double distance = nodes[i].point.distanceXY(nodes[j].point);
      if (distance <= connection_radius) {
        new_edges.push_back(EdgeCandidate{distance, i, j});
      }
    }
  }
  std::sort(
    new_edges.begin(), new_edges.end(),
    [](const EdgeCandidate & lhs, const EdgeCandidate & rhs) {
      return lhs.distance < rhs.distance;
    });
  const auto edge_candidates_done = std::chrono::steady_clock::now();
  for (const auto & edge : new_edges) {
    if (nodes[edge.from].neighbors.size() >= max_neighbors ||
      nodes[edge.to].neighbors.size() >= max_neighbors ||
      !isKnownFree2D(nodes[edge.from].point, nodes[edge.to].point) ||
      edge_crosses_graph(edge.from, edge.to))
    {
      continue;
    }
    nodes[edge.from].neighbors.push_back(edge.to);
    nodes[edge.to].neighbors.push_back(edge.from);
  }
  const auto edge_connection_done = std::chrono::steady_clock::now();

  std::size_t edge_count = 0;
  for (const auto & node : nodes) {
    edge_count += node.neighbors.size();
  }
  map_state_->topology_nodes = std::move(nodes);
  map_state_->last_topology_path.clear();
  map_state_->topology_map_revision = map_state_->map_revision;
  map_state_->topology_stamp = now;
  map_state_->topology_update_origin = update_center;
  map_state_->has_topology_update_origin = true;
  map_state_->has_topology = !map_state_->topology_nodes.empty();
  if (initialize_from_accumulated_map) {
    map_state_->startup_topology_published = false;
  }
  if (initialize_from_accumulated_map &&
    map_state_->topology_nodes.size() >=
    static_cast<std::size_t>(std::max(1, topology_initial_min_nodes_)))
  {
    map_state_->initialized_from_accumulated_map = true;
  }

  RCLCPP_INFO(
    logger_,
    "[%s] incrementally updated topology after %.2fm motion: radius=%.2fm retained=%zu "
    "removed=%zu added=%zu nodes=%zu edges=%zu revision=%lu total=%.2fms "
    "phases_ms{retain_nodes=%.2f retained_edges=%.2f free_sample=%.2f clearance=%.2f "
    "node_select=%.2f edge_candidates=%.2f new_edge_connect=%.2f finalize=%.2f} "
    "counts{free_samples=%zu local_priority_samples=%zu bubbles=%zu robot_anchor=%s "
    "retained_edge_candidates=%zu new_edge_candidates=%zu}",
    name_.c_str(), motion_since_update,
    update_radius, retained_node_count, old_node_count - retained_node_count,
    map_state_->topology_nodes.size() - retained_node_count,
    map_state_->topology_nodes.size(),
    edge_count / 2, static_cast<unsigned long>(map_state_->map_revision),
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count(),
    std::chrono::duration<double, std::milli>(retained_nodes_done - started).count(),
    std::chrono::duration<double, std::milli>(retained_edges_done - retained_nodes_done).count(),
    std::chrono::duration<double, std::milli>(sampling_done - retained_edges_done).count(),
    std::chrono::duration<double, std::milli>(clearance_done - sampling_done).count(),
    std::chrono::duration<double, std::milli>(node_selection_done - clearance_done).count(),
    std::chrono::duration<double, std::milli>(edge_candidates_done - node_selection_done).count(),
    std::chrono::duration<double, std::milli>(edge_connection_done - edge_candidates_done).count(),
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - edge_connection_done).count(),
    free_samples.size(), local_free_samples.size(), bubbles.size(),
    robot_anchor_added ? "added" : "existing", retained_edges.size(), new_edges.size());
  return true;
}

std::vector<Point3> FaelFrontierCore::searchTopologyPath(
  const Point3 & start,
  const Point3 & goal) const
{
  const auto started = std::chrono::steady_clock::now();
  const auto & nodes = map_state_->topology_nodes;
  if (nodes.empty()) {
    RCLCPP_WARN(logger_, "[%s] topology A*: graph is empty", name_.c_str());
    return {};
  }

  const std::size_t start_index = nodes.size();
  const std::size_t goal_index = nodes.size() + 1;
  const std::size_t graph_size = nodes.size() + 2;
  std::vector<std::vector<std::pair<std::size_t, double>>> adjacency(graph_size);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (const auto neighbor : nodes[i].neighbors) {
      adjacency[i].emplace_back(neighbor, nodes[i].point.distanceXY(nodes[neighbor].point));
    }
  }
  const auto adjacency_done = std::chrono::steady_clock::now();

  auto is_start_connection_free = [&](const Point3 & from, const Point3 & to) {
      const double distance = from.distanceXY(to);
      const double step_size = std::max(resolution_ * 0.5, 0.1);
      const int steps = std::max(1, static_cast<int>(std::ceil(distance / step_size)));
      const double plane_z = map_state_->topology_plane_z;
      for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        Point3 sample{
          from.x + (to.x - from.x) * t,
          from.y + (to.y - from.y) * t,
          plane_z};
        if (isNearOccupied(sample, robot_clear_radius_)) {
          return false;
        }
        // The first cloud may not have marked the robot footprint free yet. Allow that
        // short unknown prefix while keeping the rest of the attachment in known space.
        if (sample.distanceXY(from) > robot_clear_radius_ &&
          !map_state_->map->isFree(toUfoPoint(sample), insert_depth_))
        {
          return false;
        }
      }
      return true;
    };

  auto attach_virtual_node = [&](
    std::size_t index, const Point3 & point, bool is_start,
    std::size_t & nearby_count, std::size_t & blocked_count)
    {
      std::vector<std::pair<double, std::size_t>> candidates;
      for (std::size_t i = 0; i < nodes.size(); ++i) {
        const double distance = point.distanceXY(nodes[i].point);
        if (distance <= topology_attach_radius_) {
          candidates.emplace_back(distance, i);
        }
      }
      std::sort(candidates.begin(), candidates.end());
      nearby_count = candidates.size();
      int attached = 0;
      for (const auto & candidate : candidates) {
        if (attached >= std::max(1, topology_max_neighbors_)) {
          break;
        }
        const bool connection_free = is_start ?
          is_start_connection_free(point, nodes[candidate.second].point) :
          isKnownFree2D(point, nodes[candidate.second].point);
        if (!connection_free) {
          ++blocked_count;
          continue;
        }
        adjacency[index].emplace_back(candidate.second, candidate.first);
        adjacency[candidate.second].emplace_back(index, candidate.first);
        ++attached;
      }
    };
  std::size_t start_nearby = 0;
  std::size_t start_blocked = 0;
  std::size_t goal_nearby = 0;
  std::size_t goal_blocked = 0;
  attach_virtual_node(start_index, start, true, start_nearby, start_blocked);
  attach_virtual_node(goal_index, goal, false, goal_nearby, goal_blocked);
  if (start.distanceXY(goal) <= topology_attach_radius_ &&
    is_start_connection_free(start, goal))
  {
    const double distance = start.distanceXY(goal);
    adjacency[start_index].emplace_back(goal_index, distance);
    adjacency[goal_index].emplace_back(start_index, distance);
  }
  const auto attachment_done = std::chrono::steady_clock::now();
  if (adjacency[start_index].empty() || adjacency[goal_index].empty()) {
    RCLCPP_WARN(
      logger_,
      "[%s] topology A* attachment failed: nodes=%zu start_edges=%zu goal_edges=%zu "
      "start_nearby=%zu start_blocked=%zu goal_nearby=%zu goal_blocked=%zu "
      "phases_ms{adjacency=%.2f attach=%.2f total=%.2f}",
      name_.c_str(), nodes.size(), adjacency[start_index].size(), adjacency[goal_index].size(),
      start_nearby, start_blocked, goal_nearby, goal_blocked,
      std::chrono::duration<double, std::milli>(adjacency_done - started).count(),
      std::chrono::duration<double, std::milli>(attachment_done - adjacency_done).count(),
      std::chrono::duration<double, std::milli>(attachment_done - started).count());
    return {};
  }

  auto point_at = [&](std::size_t index) -> const Point3 & {
      if (index == start_index) {
        return start;
      }
      if (index == goal_index) {
        return goal;
      }
      return nodes[index].point;
    };
  using QueueItem = std::pair<double, std::size_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  std::vector<double> g_score(graph_size, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(graph_size, graph_size);
  std::vector<bool> closed(graph_size, false);
  g_score[start_index] = 0.0;
  open.emplace(start.distanceXY(goal), start_index);
  std::size_t expanded_nodes = 0;

  while (!open.empty()) {
    const auto current_index = open.top().second;
    open.pop();
    if (closed[current_index]) {
      continue;
    }
    if (current_index == goal_index) {
      break;
    }
    closed[current_index] = true;
    ++expanded_nodes;
    for (const auto & edge : adjacency[current_index]) {
      if (closed[edge.first]) {
        continue;
      }
      const double tentative = g_score[current_index] + edge.second;
      if (tentative >= g_score[edge.first]) {
        continue;
      }
      parent[edge.first] = current_index;
      g_score[edge.first] = tentative;
      const double heuristic = point_at(edge.first).distanceXY(goal) * 1.001;
      open.emplace(tentative + heuristic, edge.first);
    }
  }
  const auto search_done = std::chrono::steady_clock::now();
  if (!std::isfinite(g_score[goal_index])) {
    RCLCPP_WARN(
      logger_,
      "[%s] topology A* no path: nodes=%zu expanded=%zu start_edges=%zu goal_edges=%zu "
      "phases_ms{adjacency=%.2f attach=%.2f search=%.2f total=%.2f}",
      name_.c_str(), nodes.size(), expanded_nodes,
      adjacency[start_index].size(), adjacency[goal_index].size(),
      std::chrono::duration<double, std::milli>(adjacency_done - started).count(),
      std::chrono::duration<double, std::milli>(attachment_done - adjacency_done).count(),
      std::chrono::duration<double, std::milli>(search_done - attachment_done).count(),
      std::chrono::duration<double, std::milli>(search_done - started).count());
    return {};
  }

  std::vector<Point3> path;
  for (std::size_t index = goal_index; index != graph_size; index = parent[index]) {
    path.push_back(point_at(index));
    if (index == start_index) {
      break;
    }
  }
  if (path.empty() || path.back().distanceXY(start) > 1e-6) {
    return {};
  }
  std::reverse(path.begin(), path.end());
  const auto finished = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    logger_,
    "[%s] topology A* success: nodes=%zu expanded=%zu waypoints=%zu cost=%.2f "
    "start_edges=%zu goal_edges=%zu phases_ms{adjacency=%.2f attach=%.2f search=%.2f "
    "backtrack=%.2f total=%.2f}",
    name_.c_str(), nodes.size(), expanded_nodes, path.size(), g_score[goal_index],
    adjacency[start_index].size(), adjacency[goal_index].size(),
    std::chrono::duration<double, std::milli>(adjacency_done - started).count(),
    std::chrono::duration<double, std::milli>(attachment_done - adjacency_done).count(),
    std::chrono::duration<double, std::milli>(search_done - attachment_done).count(),
    std::chrono::duration<double, std::milli>(finished - search_done).count(),
    std::chrono::duration<double, std::milli>(finished - started).count());
  return path;
}

bool FaelFrontierCore::isViewpointConnectionFree(const Point3 & from, const Point3 & to) const
{
  return isCollisionFree2D(from, to);
}

double FaelFrontierCore::unknownGainBeyondFrontier(
  const Point3 & viewpoint,
  const Point3 & frontier) const
{
  const double dx = frontier.x - viewpoint.x;
  const double dy = frontier.y - viewpoint.y;
  const double dz = frontier.z - viewpoint.z;
  const double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (norm <= 1e-6) {
    return 0.0;
  }

  const double ux = dx / norm;
  const double uy = dy / norm;
  const double uz = dz / norm;
  const double step = std::max(resolution_, unknown_gain_step_);
  double gain = 0.0;
  for (double dist = step; dist <= unknown_gain_range_; dist += step) {
    Point3 sample{
      frontier.x + ux * dist,
      frontier.y + uy * dist,
      frontier.z + uz * dist};
    if (map_state_->map->isOccupied(toUfoPoint(sample), insert_depth_)) {
      break;
    }
    if (map_state_->map->isUnknown(toUfoPoint(sample), insert_depth_)) {
      gain += 1.0;
    }
  }
  return gain;
}

void FaelFrontierCore::frontierSearch(const Point3 & current)
{
  map_state_->known_cell_codes.insert(
    map_state_->changed_cell_codes.begin(), map_state_->changed_cell_codes.end());
  findLocalFrontiers(current);
  updateGlobalFrontiers(current);
  map_state_->changed_cell_codes.clear();
}

void FaelFrontierCore::findLocalFrontiers(const Point3 & current)
{
  map_state_->local_frontier_cells.clear();
  const double slope_limit = std::tan(frontier_slope_deg_ * M_PI / 180.0);
  FrontierSet search_codes = map_state_->changed_cell_codes;
  for (const auto & code : map_state_->changed_cell_codes) {
    const auto center = map_state_->map->toCoord(code.toKey(), code.getDepth());
    search_codes.insert(
      map_state_->map->toCode(
        center.x() - resolution_, center.y(), center.z(),
        code.getDepth()));
    search_codes.insert(
      map_state_->map->toCode(
        center.x() + resolution_, center.y(), center.z(),
        code.getDepth()));
    search_codes.insert(
      map_state_->map->toCode(
        center.x(), center.y() - resolution_, center.z(),
        code.getDepth()));
    search_codes.insert(
      map_state_->map->toCode(
        center.x(), center.y() + resolution_, center.z(),
        code.getDepth()));
  }

  for (const auto & code : search_codes) {
    if (!isFrontier(code)) {
      continue;
    }

    const auto point = codeToPoint(code);
    const double dist_xy = point.distanceXY(current);
    if (dist_xy <= min_robot_frontier_dist_) {
      continue;
    }
    if (dist_xy > 1e-6 && std::fabs(point.z - current.z) / dist_xy < slope_limit) {
      map_state_->local_frontier_cells.insert(code);
    }
  }
}

void FaelFrontierCore::updateGlobalFrontiers(const Point3 & current)
{
  FrontierSet updated_frontiers;
  const double slope_limit = std::tan(frontier_slope_deg_ * M_PI / 180.0);
  int revalidated = 0;

  for (const auto & code : map_state_->global_frontier_cells) {
    const auto point = codeToPoint(code);
    const double dist_xy = point.distanceXY(current);
    if (dist_xy < max_range_ + 1.0) {
      if (global_frontier_revalidate_max_cells_ > 0 &&
        revalidated >= global_frontier_revalidate_max_cells_)
      {
        updated_frontiers.insert(code);
        continue;
      }
      ++revalidated;
      if (dist_xy > 1e-6 &&
        std::fabs(point.z - current.z) / dist_xy < slope_limit &&
        isFrontier(code))
      {
        updated_frontiers.insert(code);
      }
    } else {
      updated_frontiers.insert(code);
    }
  }

  updated_frontiers.insert(
    map_state_->local_frontier_cells.begin(), map_state_->local_frontier_cells.end());
  map_state_->global_frontier_cells = std::move(updated_frontiers);
}

std::vector<Point3> FaelFrontierCore::getGlobalFrontiers() const
{
  std::vector<Point3> frontiers;
  frontiers.reserve(map_state_->global_frontier_cells.size());
  for (const auto & code : map_state_->global_frontier_cells) {
    frontiers.push_back(codeToPoint(code));
  }
  return frontiers;
}

std::vector<Point3> FaelFrontierCore::compactFrontiersForAttachment(
  const std::vector<Point3> & frontiers,
  const Point3 & current) const
{
  const double cell_size = std::max(resolution_, frontier_attach_grid_size_);
  std::unordered_map<GridCell, Point3, GridCellHash> representatives;
  representatives.reserve(frontiers.size());

  for (const auto & frontier : frontiers) {
    if (frontier.distanceXY(current) > candidate_visibility_range_ + local_range_) {
      continue;
    }
    const auto cell = toGridCell(frontier, cell_size);
    const auto existing = representatives.find(cell);
    if (existing == representatives.end() ||
      frontier.distanceXY(current) < existing->second.distanceXY(current))
    {
      representatives[cell] = frontier;
    }
  }

  std::vector<Point3> compacted;
  compacted.reserve(representatives.size());
  for (const auto & item : representatives) {
    compacted.push_back(item.second);
  }
  return compacted;
}

std::vector<Point3> FaelFrontierCore::sampleViewpoints(const Point3 & current) const
{
  std::vector<Point3> viewpoints;
  last_viewpoint_debug_ = ViewpointDebug{};
  const int steps = std::max(1, static_cast<int>(std::ceil(local_range_ / sample_dist_)));
  for (int ix = -steps; ix <= steps; ++ix) {
    for (int iy = -steps; iy <= steps; ++iy) {
      Point3 sample{
        current.x + static_cast<double>(ix) * sample_dist_,
        current.y + static_cast<double>(iy) * sample_dist_,
        current.z + sensor_height_};
      ++last_viewpoint_debug_.sampled;
      const double current_distance = sample.distanceXY(current);
      if (current_distance > local_range_) {
        ++last_viewpoint_debug_.outside_range;
        continue;
      }
      if (!hasFreeVoxelNearHeight(sample)) {
        ++last_viewpoint_debug_.not_free;
        continue;
      }
      if (isNearOccupied(sample, robot_clear_radius_)) {
        ++last_viewpoint_debug_.near_occupied;
        continue;
      }
      if (isNearUnknown(sample, unknown_clear_radius_)) {
        ++last_viewpoint_debug_.near_unknown;
        continue;
      }
      if (current_distance < min_candidate_dist_) {
        ++last_viewpoint_debug_.too_close;
        continue;
      }
      if (!isViewpointConnectionFree(current, sample)) {
        ++last_viewpoint_debug_.collision;
        continue;
      }
      if (current_distance < local_range_) {
        viewpoints.push_back(sample);
      }
    }
  }
  return viewpoints;
}

std::vector<Candidate> FaelFrontierCore::attachFrontiers(
  const std::vector<Point3> & viewpoints,
  const std::vector<Point3> & frontiers,
  const Point3 & current) const
{
  const auto started = std::chrono::steady_clock::now();
  std::size_t cache_hits = 0;
  std::size_t nearby_pairs = 0;
  std::size_t visibility_checks = 0;
  std::size_t blocked_rays = 0;
  std::unordered_map<std::size_t, std::vector<Point3>> attached;
  const double slope_limit = std::tan(viewpoint_slope_deg_ * M_PI / 180.0);
  const double frontier_cell_area =
    std::max(resolution_ * resolution_, frontier_attach_grid_size_ * frontier_attach_grid_size_);
  const double attach_range = std::max(
    resolution_,
    std::min(candidate_visibility_range_, max_range_ - 0.5));

  std::vector<Point3> representative_points;
  representative_points.reserve(viewpoints.size() + map_state_->candidates.size());
  for (const auto & viewpoint : viewpoints) {
    bool near_old_candidate = false;
    for (const auto & candidate : map_state_->candidates) {
      if (viewpoint.distanceXY(candidate.point) < std::max(1.0, sample_dist_)) {
        near_old_candidate = true;
        break;
      }
    }
    if (!near_old_candidate) {
      representative_points.push_back(viewpoint);
    }
  }
  for (const auto & candidate : map_state_->candidates) {
    if (candidate.point.distanceXY(current) < local_range_ + attach_range &&
      map_state_->map->isFree(toUfoPoint(candidate.point), insert_depth_))
    {
      representative_points.push_back(candidate.point);
    }
  }
  if (representative_points.empty()) {
    representative_points = viewpoints;
  }

  const double grid_size = std::max(sample_dist_, attach_range * 0.25);
  std::unordered_map<GridCell, std::vector<std::size_t>, GridCellHash> viewpoint_grid;
  viewpoint_grid.reserve(representative_points.size());
  for (std::size_t i = 0; i < representative_points.size(); ++i) {
    viewpoint_grid[toGridCell(representative_points[i], grid_size)].push_back(i);
  }

  auto representative_index = [&](const Point3 & point) {
      for (std::size_t i = 0; i < representative_points.size(); ++i) {
        if (representative_points[i].distanceXY(point) < resolution_) {
          return i;
        }
      }
      representative_points.push_back(point);
      return representative_points.size() - 1;
    };

  std::unordered_map<ufo::map::Code, Point3, ufo::map::Code::Hash> next_frontiers_viewpoints;
  next_frontiers_viewpoints.reserve(frontiers.size());

  for (const auto & frontier : frontiers) {
    const auto frontier_code = map_state_->map->toCode(
      frontier.x, frontier.y, frontier.z, insert_depth_);
    const auto old = map_state_->frontiers_viewpoints.find(frontier_code);
    if (old != map_state_->frontiers_viewpoints.end()) {
      const double old_distance = frontier.distanceXY(old->second);
      if (old_distance > 1e-6 && old_distance < attach_range &&
        std::fabs(frontier.z - old->second.z) / old_distance < slope_limit &&
        map_state_->map->isFree(toUfoPoint(old->second), insert_depth_))
      {
        ++visibility_checks;
        if (isFrontierVisible(old->second, frontier)) {
          const auto old_idx = representative_index(old->second);
          next_frontiers_viewpoints[frontier_code] = old->second;
          attached[old_idx].push_back(frontier);
          ++cache_hits;
          continue;
        }
        ++blocked_rays;
      }
    }

    std::optional<std::size_t> best_idx;
    const auto nearby_viewpoints =
      radiusCandidates(viewpoint_grid, frontier, attach_range, grid_size);
    std::vector<std::pair<double, std::size_t>> ordered_viewpoints;
    ordered_viewpoints.reserve(nearby_viewpoints.size());
    for (const auto i : nearby_viewpoints) {
      const auto & viewpoint = representative_points[i];
      const double dist_xy = frontier.distanceXY(viewpoint);
      if (dist_xy >= attach_range || dist_xy <= 1e-6) {
        continue;
      }
      if (std::fabs(frontier.z - viewpoint.z) / dist_xy >= slope_limit) {
        continue;
      }
      ordered_viewpoints.emplace_back(dist_xy, i);
    }
    std::sort(ordered_viewpoints.begin(), ordered_viewpoints.end());
    nearby_pairs += ordered_viewpoints.size();
    int checked_viewpoints = 0;
    for (const auto & ordered : ordered_viewpoints) {
      if (checked_viewpoints >= std::max(1, frontier_visibility_max_viewpoints_)) {
        break;
      }
      ++checked_viewpoints;
      ++visibility_checks;
      if (!isFrontierVisible(representative_points[ordered.second], frontier)) {
        ++blocked_rays;
        continue;
      }
      best_idx = ordered.second;
      break;
    }

    if (best_idx) {
      next_frontiers_viewpoints[frontier_code] = representative_points[*best_idx];
      attached[*best_idx].push_back(frontier);
    }
  }
  map_state_->frontiers_viewpoints = std::move(next_frontiers_viewpoints);
  const auto association_done = std::chrono::steady_clock::now();

  std::vector<Candidate> candidates;
  for (const auto & item : attached) {
    const auto & frontier_set = item.second;
    const double frontier_area = static_cast<double>(frontier_set.size()) * frontier_cell_area;
    if (frontier_area < min_frontier_area_) {
      continue;
    }

    if (item.first >= representative_points.size()) {
      continue;
    }
    const Point3 viewpoint = representative_points[item.first];
    double information_gain = 0.0;
    for (const auto & frontier : frontier_set) {
      const double frontier_distance = std::max(resolution_, frontier.distanceXY(viewpoint));
      const double distance_decay = std::max(
        0.25, 1.0 / (1.0 + frontier_distance_weight_ * frontier_distance));
      const double unknown_gain = unknownGainBeyondFrontier(viewpoint, frontier);
      if (unknown_gain < min_unknown_gain_) {
        continue;
      }
      information_gain += frontier_gain_ * unknown_gain * distance_decay;
    }
    if (information_gain < viewpoint_gain_threshold_) {
      information_gain = std::max(
        information_gain,
        frontier_gain_ * frontier_area);
    }
    if (information_gain < viewpoint_gain_threshold_) {
      continue;
    }

    double visited_penalty = 0.0;
    for (const auto & visited : map_state_->visited_positions) {
      if (viewpoint.distanceXY(visited) < visited_radius_) {
        visited_penalty = visited_penalty_;
        break;
      }
    }

    double known_penalty = 0.0;
    const int known_cells = std::max(1, static_cast<int>(std::ceil(visited_radius_ / resolution_)));
    for (int dx = -known_cells; dx <= known_cells; ++dx) {
      for (int dy = -known_cells; dy <= known_cells; ++dy) {
        Point3 sample{
          viewpoint.x + static_cast<double>(dx) * resolution_,
          viewpoint.y + static_cast<double>(dy) * resolution_,
          viewpoint.z};
        if (sample.distanceXY(viewpoint) > visited_radius_) {
          continue;
        }
        if (map_state_->map->isFree(toUfoPoint(sample), insert_depth_)) {
          known_penalty += known_gain_penalty_;
        }
      }
    }

    Candidate candidate;
    candidate.point = viewpoint;
    candidate.frontiers = frontier_set;
    candidate.score = information_gain - distance_weight_ * candidate.point.distanceXY(current) -
      visited_penalty - known_penalty;
    candidates.push_back(candidate);
  }
  const auto scoring_done = std::chrono::steady_clock::now();

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.score > b.score;});

  std::vector<Candidate> filtered;
  filtered.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    bool too_close_to_better = false;
    for (const auto & selected : filtered) {
      if (candidate.point.distanceXY(selected.point) < candidate_separation_) {
        too_close_to_better = true;
        break;
      }
    }
    if (!too_close_to_better) {
      filtered.push_back(candidate);
      if (max_candidate_count_ > 0 &&
        filtered.size() >= static_cast<std::size_t>(max_candidate_count_))
      {
        break;
      }
    }
  }

  if (min_candidate_count_ > 0 &&
    filtered.size() < static_cast<std::size_t>(min_candidate_count_))
  {
    for (const auto & candidate : candidates) {
      const auto duplicate = std::find_if(
        filtered.begin(), filtered.end(),
        [&](const Candidate & selected) {
          return candidate.point.distanceXY(selected.point) < resolution_;
        });
      if (duplicate != filtered.end()) {
        continue;
      }
      filtered.push_back(candidate);
      if (filtered.size() >= static_cast<std::size_t>(min_candidate_count_)) {
        break;
      }
      if (max_candidate_count_ > 0 &&
        filtered.size() >= static_cast<std::size_t>(max_candidate_count_))
      {
        break;
      }
    }
  }

  const auto finished = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    logger_,
    "[%s] FAEL-style frontier attachment: frontiers=%zu viewpoints=%zu attached_groups=%zu "
    "candidates=%zu kept=%zu cache_hits=%zu nearby_pairs=%zu visibility_checks=%zu "
    "blocked_rays=%zu phases_ms{associate_raycast=%.2f score=%.2f filter=%.2f total=%.2f}",
    name_.c_str(), frontiers.size(), representative_points.size(), attached.size(),
    candidates.size(), filtered.size(), cache_hits, nearby_pairs, visibility_checks, blocked_rays,
    std::chrono::duration<double, std::milli>(association_done - started).count(),
    std::chrono::duration<double, std::milli>(scoring_done - association_done).count(),
    std::chrono::duration<double, std::milli>(finished - scoring_done).count(),
    std::chrono::duration<double, std::milli>(finished - started).count());

  return filtered;
}

std::vector<Candidate> FaelFrontierCore::selectCandidates(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & requested_goal,
  bool force_refresh)
{
  (void) requested_goal;
  const auto total_start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(map_state_->mutex);
  Point3 current{start.pose.position.x, start.pose.position.y, start.pose.position.z};
  const auto duplicate_visit = std::find_if(
    map_state_->visited_positions.begin(), map_state_->visited_positions.end(),
    [&](const Point3 & visited) {
      return current.distanceXY(visited) < std::max(resolution_, visited_radius_ * 0.5);
    });
  if (duplicate_visit == map_state_->visited_positions.end()) {
    map_state_->visited_positions.push_back(current);
  }
  updateTopologyMap(current, true);
  const bool can_reuse_by_map_revision =
    map_state_->has_cached_candidates &&
    map_state_->candidates_map_revision == map_state_->map_revision;
  const bool can_reuse_by_age =
    map_state_->has_cached_candidates &&
    candidate_recompute_period_ > 0.0 &&
    (clock_->now() - map_state_->candidates_stamp).seconds() <= candidate_recompute_period_;
  const bool can_reuse_by_motion =
    map_state_->has_cached_candidates &&
    current.distanceXY(map_state_->candidates_origin) <= cache_robot_move_threshold_;
  if (!force_refresh && reuse_cached_candidates_ && can_reuse_by_motion &&
    (can_reuse_by_map_revision || can_reuse_by_age))
  {
    const Candidate * selected =
      map_state_->candidates.empty() ? nullptr : &map_state_->candidates.front();
    publishCandidates(map_state_->candidates, selected);
    publishTopology(map_state_->candidates, current, selected, false);
    if (selected && selected_candidate_pub_ && selected_candidate_pub_->is_activated()) {
      std_msgs::msg::Header header;
      header.stamp = clock_->now();
      header.frame_id = frame_id_;
      safePublish(
        selected_candidate_pub_, makePose(
          selected->point,
          header), logger_, "selected_candidate");
    }
    RCLCPP_INFO(
      logger_,
      "[%s] reused cached frontier candidates: count=%zu map_revision=%lu age=%.3fs motion=%.2fm",
      name_.c_str(), map_state_->candidates.size(),
      static_cast<unsigned long>(map_state_->map_revision),
      (clock_->now() - map_state_->candidates_stamp).seconds(),
      current.distanceXY(map_state_->candidates_origin));
    return map_state_->candidates;
  }

  const auto frontier_start = std::chrono::steady_clock::now();
  frontierSearch(current);
  const auto frontier_done = std::chrono::steady_clock::now();
  const auto frontiers = getGlobalFrontiers();
  const auto get_frontiers_done = std::chrono::steady_clock::now();
  const auto attach_frontiers = compactFrontiersForAttachment(frontiers, current);
  const auto compact_done = std::chrono::steady_clock::now();
  const auto viewpoints = sampleViewpoints(current);
  const auto viewpoints_done = std::chrono::steady_clock::now();
  auto candidates = attachFrontiers(viewpoints, attach_frontiers, current);
  const auto attach_done = std::chrono::steady_clock::now();
  map_state_->candidates = candidates;
  map_state_->candidates_stamp = clock_->now();
  map_state_->candidates_owner = name_;
  map_state_->candidates_origin = current;
  map_state_->candidates_map_revision = map_state_->map_revision;
  map_state_->has_cached_candidates = true;

  RCLCPP_INFO(
    logger_,
    "[%s] candidate refresh timing ms: frontier=%.2f get_frontiers=%.2f compact=%.2f viewpoints=%.2f "
    "attach=%.2f total=%.2f counts{frontiers=%zu viewpoints=%zu candidates=%zu "
    "attach_frontiers=%zu}",
    name_.c_str(),
    std::chrono::duration<double, std::milli>(frontier_done - frontier_start).count(),
    std::chrono::duration<double, std::milli>(get_frontiers_done - frontier_done).count(),
    std::chrono::duration<double, std::milli>(compact_done - get_frontiers_done).count(),
    std::chrono::duration<double, std::milli>(viewpoints_done - compact_done).count(),
    std::chrono::duration<double, std::milli>(attach_done - viewpoints_done).count(),
    std::chrono::duration<double, std::milli>(attach_done - total_start).count(),
    frontiers.size(), viewpoints.size(), candidates.size(), attach_frontiers.size());

  if (candidates.empty()) {
    publishCandidates(candidates, nullptr);
    publishTopology(candidates, current, nullptr, false);
    RCLCPP_WARN(
      logger_,
      "[%s] no FAEL frontier candidate, global_frontiers=%zu local_frontiers=%zu "
      "attach_frontiers=%zu viewpoints=%zu known_voxels=%zu map_revision=%lu sampled=%zu "
      "reject{range=%zu not_free=%zu "
      "occupied=%zu unknown=%zu too_close=%zu collision=%zu}",
      name_.c_str(), frontiers.size(), map_state_->local_frontier_cells.size(),
      attach_frontiers.size(), viewpoints.size(),
      map_state_->known_cell_codes.size(), static_cast<unsigned long>(map_state_->map_revision),
      last_viewpoint_debug_.sampled,
      last_viewpoint_debug_.outside_range, last_viewpoint_debug_.not_free,
      last_viewpoint_debug_.near_occupied, last_viewpoint_debug_.near_unknown,
      last_viewpoint_debug_.too_close, last_viewpoint_debug_.collision);
    return {};
  }

  publishCandidates(candidates, &candidates.front());
  publishTopology(candidates, current, &candidates.front(), false);
  if (selected_candidate_pub_ && selected_candidate_pub_->is_activated()) {
    std_msgs::msg::Header header;
    header.stamp = clock_->now();
    header.frame_id = frame_id_;
    safePublish(
      selected_candidate_pub_, makePose(
        candidates.front().point, header), logger_, "selected_candidate");
  }
  RCLCPP_INFO(
    logger_, "[%s] selected candidate x=%.2f y=%.2f z=%.2f score=%.2f attached_frontiers=%zu",
    name_.c_str(), candidates.front().point.x, candidates.front().point.y,
    candidates.front().point.z, candidates.front().score, candidates.front().frontiers.size());
  return candidates;
}

std::optional<Candidate> FaelFrontierCore::selectCandidate(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & requested_goal)
{
  auto candidates = selectCandidates(start, requested_goal);
  if (candidates.empty()) {
    return std::nullopt;
  }
  return candidates.front();
}

nav_msgs::msg::Path FaelFrontierCore::makeCandidatePath(
  const geometry_msgs::msg::PoseStamped & start,
  const Candidate & candidate) const
{
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.header.frame_id = frame_id_;
  path.poses.push_back(start);
  path.poses.back().header.frame_id = frame_id_;
  path.poses.push_back(makePose(candidate.point, path.header));
  return path;
}

nav_msgs::msg::Path FaelFrontierCore::makeCandidatePath(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<Candidate> & candidates) const
{
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.header.frame_id = frame_id_;
  for (auto it = candidates.rbegin(); it != candidates.rend(); ++it) {
    path.poses.push_back(makePose(it->point, path.header));
  }
  return path;
}

nav_msgs::msg::Path FaelFrontierCore::makeAStarPath(
  const geometry_msgs::msg::PoseStamped & start,
  const Candidate & candidate)
{
  const auto started = std::chrono::steady_clock::now();
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.header.frame_id = frame_id_;

  std::lock_guard<std::mutex> lock(map_state_->mutex);
  const auto lock_acquired = std::chrono::steady_clock::now();
  const Point3 start_point{start.pose.position.x, start.pose.position.y, start.pose.position.z};
  Point3 target = candidate.point;
  target.z = start.pose.position.z;
  const bool topology_updated = updateTopologyMap(start_point, true);
  const auto topology_done = std::chrono::steady_clock::now();

  Point3 graph_start = start_point;
  Point3 graph_target = target;
  graph_start.z = map_state_->topology_plane_z;
  graph_target.z = map_state_->topology_plane_z;
  const auto topology_path = searchTopologyPath(graph_start, graph_target);
  const auto search_done = std::chrono::steady_clock::now();
  map_state_->last_topology_path = topology_path;
  if (topology_path.empty()) {
    RCLCPP_WARN(
      logger_,
      "[%s] makeAStarPath failed: topology_updated=%s phases_ms{lock_wait=%.2f "
      "topology_update=%.2f graph_search=%.2f total=%.2f}",
      name_.c_str(), topology_updated ? "true" : "false",
      std::chrono::duration<double, std::milli>(lock_acquired - started).count(),
      std::chrono::duration<double, std::milli>(topology_done - lock_acquired).count(),
      std::chrono::duration<double, std::milli>(search_done - topology_done).count(),
      std::chrono::duration<double, std::milli>(search_done - started).count());
    return path;
  }

  const double interpolation_step = std::max(resolution_, 0.2);
  path.poses.push_back(makePose(start_point, path.header));
  for (std::size_t segment = 1; segment < topology_path.size(); ++segment) {
    const auto & from = topology_path[segment - 1];
    const auto & to = topology_path[segment];
    const double segment_length = from.distanceXY(to);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(segment_length / interpolation_step)));
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      Point3 interpolated{
        from.x + (to.x - from.x) * ratio,
        from.y + (to.y - from.y) * ratio,
        start_point.z};
      path.poses.push_back(makePose(interpolated, path.header));
    }
  }

  updatePathOrientations(path);
  const auto finished = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    logger_,
    "[%s] makeAStarPath success: topology_updated=%s topology_waypoints=%zu path_poses=%zu "
    "phases_ms{lock_wait=%.2f topology_update=%.2f graph_search=%.2f interpolate=%.2f "
    "total=%.2f}",
    name_.c_str(), topology_updated ? "true" : "false", topology_path.size(), path.poses.size(),
    std::chrono::duration<double, std::milli>(lock_acquired - started).count(),
    std::chrono::duration<double, std::milli>(topology_done - lock_acquired).count(),
    std::chrono::duration<double, std::milli>(search_done - topology_done).count(),
    std::chrono::duration<double, std::milli>(finished - search_done).count(),
    std::chrono::duration<double, std::milli>(finished - started).count());
  return path;
}

void FaelFrontierCore::publishSelection(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<Candidate> & candidates,
  const Candidate & selected) const
{
  std::lock_guard<std::mutex> lock(map_state_->mutex);
  const Point3 current{start.pose.position.x, start.pose.position.y, start.pose.position.z};
  publishCandidates(candidates, &selected);
  publishTopology(candidates, current, &selected, true);
  if (selected_candidate_pub_ && selected_candidate_pub_->is_activated()) {
    std_msgs::msg::Header header;
    header.stamp = clock_->now();
    header.frame_id = frame_id_;
    safePublish(
      selected_candidate_pub_, makePose(
        selected.point,
        header), logger_, "selected_candidate");
  }
}

void FaelFrontierCore::publishCandidates(
  const std::vector<Candidate> & candidates,
  const Candidate * selected) const
{
  if (!candidate_pub_ || !candidate_pub_->is_activated()) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker points;
  points.header.frame_id = frame_id_;
  points.header.stamp = clock_->now();
  points.ns = name_ + "_candidates";
  points.id = 0;
  points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points.action = visualization_msgs::msg::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.scale.x = 0.35;
  points.scale.y = 0.35;
  points.scale.z = 0.35;
  points.color.r = 0.8f;
  points.color.g = 0.2f;
  points.color.b = 1.0f;
  points.color.a = 1.0f;

  for (const auto & candidate : candidates) {
    geometry_msgs::msg::Point point;
    point.x = candidate.point.x;
    point.y = candidate.point.y;
    point.z = candidate.point.z;
    points.points.push_back(point);
  }
  markers.markers.push_back(points);

  if (selected) {
    visualization_msgs::msg::Marker marker;
    marker.header = points.header;
    marker.ns = name_ + "_selected_candidate";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = selected->point.x;
    marker.pose.position.y = selected->point.y;
    marker.pose.position.z = selected->point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.7;
    marker.scale.y = 0.7;
    marker.scale.z = 0.7;
    marker.color.r = 1.0f;
    marker.color.g = 0.8f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0f;
    markers.markers.push_back(marker);
  }

  safePublish(candidate_pub_, markers, logger_, "candidates");
}

void FaelFrontierCore::publishTopology(
  const std::vector<Candidate> & candidates,
  const Point3 & current,
  const Candidate * selected,
  bool publish_best_path) const
{
  if (!topology_pub_ || !topology_pub_->is_activated()) {
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = frame_id_;
  header.stamp = clock_->now();

  visualization_msgs::msg::MarkerArray markers;

  auto make_marker = [&](int id, const std::string & ns, int type) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = ns;
      marker.id = id;
      marker.type = type;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      return marker;
    };

  auto to_msg_point = [](const Point3 & point) {
      geometry_msgs::msg::Point msg;
      msg.x = point.x;
      msg.y = point.y;
      msg.z = point.z;
      return msg;
    };

  visualization_msgs::msg::Marker robot = make_marker(
    0, "fael_current_position", visualization_msgs::msg::Marker::SPHERE);
  robot.pose.position = to_msg_point(current);
  robot.scale.x = 0.45;
  robot.scale.y = 0.45;
  robot.scale.z = 0.45;
  robot.color.r = 0.1f;
  robot.color.g = 0.4f;
  robot.color.b = 1.0f;
  robot.color.a = 1.0f;
  markers.markers.push_back(robot);

  visualization_msgs::msg::Marker viewpoints = make_marker(
    1, "fael_viewpoints", visualization_msgs::msg::Marker::SPHERE_LIST);
  viewpoints.scale.x = 0.36;
  viewpoints.scale.y = 0.36;
  viewpoints.scale.z = 0.36;
  viewpoints.color.r = 1.0f;
  viewpoints.color.g = 0.5f;
  viewpoints.color.b = 1.0f;
  viewpoints.color.a = 1.0f;

  visualization_msgs::msg::Marker frontier_links = make_marker(
    2, "fael_viewpoint_frontier_links", visualization_msgs::msg::Marker::LINE_LIST);
  frontier_links.scale.x = 0.035;
  frontier_links.color.r = 0.5f;
  frontier_links.color.g = 0.5f;
  frontier_links.color.b = 0.1f;
  frontier_links.color.a = 0.35f;

  visualization_msgs::msg::Marker frontier_cells = make_marker(
    3, "fael_attached_frontiers", visualization_msgs::msg::Marker::CUBE_LIST);
  frontier_cells.scale.x = resolution_;
  frontier_cells.scale.y = resolution_;
  frontier_cells.scale.z = resolution_;
  frontier_cells.color.r = 0.1f;
  frontier_cells.color.g = 0.9f;
  frontier_cells.color.b = 0.1f;
  frontier_cells.color.a = 0.8f;

  visualization_msgs::msg::Marker graph_nodes = make_marker(
    4, "fael_topology_graph_nodes", visualization_msgs::msg::Marker::POINTS);
  graph_nodes.scale.x = 0.18;
  graph_nodes.scale.y = 0.18;
  graph_nodes.color.r = 0.1f;
  graph_nodes.color.g = 0.8f;
  graph_nodes.color.b = 1.0f;
  graph_nodes.color.a = 1.0f;

  visualization_msgs::msg::Marker graph_edges = make_marker(
    5, "fael_topology_graph_edges", visualization_msgs::msg::Marker::LINE_LIST);
  graph_edges.scale.x = 0.045;
  graph_edges.color.r = 0.1f;
  graph_edges.color.g = 0.65f;
  graph_edges.color.b = 0.9f;
  graph_edges.color.a = 0.65f;

  visualization_msgs::msg::Marker selected_marker = make_marker(
    6, "fael_selected_viewpoint", visualization_msgs::msg::Marker::SPHERE);
  selected_marker.action = selected ? visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  selected_marker.scale.x = 0.7;
  selected_marker.scale.y = 0.7;
  selected_marker.scale.z = 0.7;
  selected_marker.color.r = 1.0f;
  selected_marker.color.g = 0.85f;
  selected_marker.color.b = 0.05f;
  selected_marker.color.a = 1.0f;

  visualization_msgs::msg::Marker best_path = make_marker(
    7, "fael_best_topology_path", visualization_msgs::msg::Marker::LINE_STRIP);
  best_path.action = (publish_best_path && selected) ? visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  best_path.scale.x = 0.12;
  best_path.color.r = 1.0f;
  best_path.color.g = 0.2f;
  best_path.color.b = 0.1f;
  best_path.color.a = 1.0f;

  for (const auto & candidate : candidates) {
    const auto candidate_point = to_msg_point(candidate.point);
    viewpoints.points.push_back(candidate_point);

    for (const auto & frontier : candidate.frontiers) {
      const auto frontier_point = to_msg_point(frontier);
      frontier_links.points.push_back(candidate_point);
      frontier_links.points.push_back(frontier_point);
      frontier_cells.points.push_back(frontier_point);
    }
  }

  if (selected) {
    selected_marker.pose.position = to_msg_point(selected->point);
  }

  for (std::size_t i = 0; i < map_state_->topology_nodes.size(); ++i) {
    graph_nodes.points.push_back(to_msg_point(map_state_->topology_nodes[i].point));
    for (const auto neighbor : map_state_->topology_nodes[i].neighbors) {
      if (neighbor <= i || neighbor >= map_state_->topology_nodes.size()) {
        continue;
      }
      graph_edges.points.push_back(to_msg_point(map_state_->topology_nodes[i].point));
      graph_edges.points.push_back(to_msg_point(map_state_->topology_nodes[neighbor].point));
    }
  }

  if (publish_best_path && selected && !map_state_->last_topology_path.empty()) {
    for (auto point : map_state_->last_topology_path) {
      point.z = current.z;
      best_path.points.push_back(to_msg_point(point));
    }
  }

  markers.markers.push_back(viewpoints);
  markers.markers.push_back(frontier_links);
  markers.markers.push_back(frontier_cells);
  markers.markers.push_back(graph_nodes);
  markers.markers.push_back(graph_edges);
  markers.markers.push_back(selected_marker);
  markers.markers.push_back(best_path);

  safePublish(topology_pub_, markers, logger_, "topology_map");
}

void FaelFrontierCore::publishMapClouds()
{
  if (!publish_map_clouds_ || !occupied_map_pub_ || !free_map_pub_ ||
    !occupied_map_pub_->is_activated() || !free_map_pub_->is_activated())
  {
    return;
  }

  const auto now = clock_->now();
  if ((now - last_map_publish_time_).seconds() < map_publish_period_) {
    return;
  }
  last_map_publish_time_ = now;

  std_msgs::msg::Header header;
  header.stamp = now;
  header.frame_id = frame_id_;

  auto build_cloud = [&](bool occupied, bool free) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = header;
      cloud.height = 1;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");

      std::vector<ufo::map::Point3> points;
      for (auto it = map_state_->map->beginLeaves(occupied, free, false, false, insert_depth_),
        it_end = map_state_->map->endLeaves(); it != it_end; ++it)
      {
        points.push_back(it.getCenter());
      }

      modifier.resize(points.size());
      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
      for (const auto & point : points) {
        *iter_x = static_cast<float>(point.x());
        *iter_y = static_cast<float>(point.y());
        *iter_z = static_cast<float>(point.z());
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
      cloud.width = static_cast<uint32_t>(points.size());
      cloud.row_step = cloud.point_step * cloud.width;
      return cloud;
    };

  safePublish(occupied_map_pub_, build_cloud(true, false), logger_, "ufomap_occupied_cloud");
  safePublish(free_map_pub_, build_cloud(false, true), logger_, "ufomap_free_cloud");
}

}  // namespace navflex_frontier_planner
