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

std::mutex g_shared_maps_mutex;
std::map<std::string, std::weak_ptr<FaelFrontierCore::SharedMapState>> g_shared_maps;
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
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
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

  auto declare_if_missing = [&](const std::string & param, const auto & value) {
      if (!node->has_parameter(name_ + "." + param)) {
        node->declare_parameter(name_ + "." + param, rclcpp::ParameterValue(value));
      }
    };

  declare_if_missing("frame_id", frame_id_);
  declare_if_missing("point_cloud_topic", point_cloud_topic_);
  declare_if_missing("visualization_topic_prefix", visualization_topic_prefix_);
  declare_if_missing("shared_map", shared_map_);
  declare_if_missing("shared_map_key", shared_map_key_);
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
  declare_if_missing("road_graph_dist", road_graph_dist_);
  declare_if_missing("road_graph_connectable_num", road_graph_connectable_num_);
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

  node->get_parameter(name_ + ".frame_id", frame_id_);
  node->get_parameter(name_ + ".point_cloud_topic", point_cloud_topic_);
  node->get_parameter(name_ + ".visualization_topic_prefix", visualization_topic_prefix_);
  node->get_parameter(name_ + ".shared_map", shared_map_);
  node->get_parameter(name_ + ".shared_map_key", shared_map_key_);
  node->get_parameter(name_ + ".resolution", resolution_);
  node->get_parameter(name_ + ".depth_levels", depth_levels_);
  node->get_parameter(name_ + ".insert_depth", insert_depth_);
  node->get_parameter(name_ + ".insert_discrete", insert_discrete_);
  node->get_parameter(name_ + ".simple_ray_casting", simple_ray_casting_);
  node->get_parameter(name_ + ".early_stopping", early_stopping_);
  node->get_parameter(name_ + ".publish_map_clouds", publish_map_clouds_);
  node->get_parameter(name_ + ".map_publish_period", map_publish_period_);
  node->get_parameter(name_ + ".max_range", max_range_);
  node->get_parameter(name_ + ".sample_dist", sample_dist_);
  node->get_parameter(name_ + ".local_range", local_range_);
  node->get_parameter(name_ + ".candidate_visibility_range", candidate_visibility_range_);
  node->get_parameter(name_ + ".reuse_cached_candidates", reuse_cached_candidates_);
  node->get_parameter(name_ + ".cache_robot_move_threshold", cache_robot_move_threshold_);
  node->get_parameter(name_ + ".candidate_recompute_period", candidate_recompute_period_);
  node->get_parameter(name_ + ".frontier_attach_grid_size", frontier_attach_grid_size_);
  node->get_parameter(name_ + ".global_frontier_revalidate_max_cells", global_frontier_revalidate_max_cells_);
  node->get_parameter(name_ + ".road_graph_dist", road_graph_dist_);
  node->get_parameter(name_ + ".road_graph_connectable_num", road_graph_connectable_num_);
  node->get_parameter(name_ + ".viewpoint_gain_threshold", viewpoint_gain_threshold_);
  node->get_parameter(name_ + ".min_frontier_area", min_frontier_area_);
  node->get_parameter(name_ + ".candidate_separation", candidate_separation_);
  node->get_parameter(name_ + ".frontier_distance_weight", frontier_distance_weight_);
  node->get_parameter(name_ + ".min_candidate_count", min_candidate_count_);
  node->get_parameter(name_ + ".max_candidate_count", max_candidate_count_);
  node->get_parameter(name_ + ".frontier_gain", frontier_gain_);
  node->get_parameter(name_ + ".unknown_gain_range", unknown_gain_range_);
  node->get_parameter(name_ + ".unknown_gain_step", unknown_gain_step_);
  node->get_parameter(name_ + ".min_unknown_gain", min_unknown_gain_);
  node->get_parameter(name_ + ".distance_weight", distance_weight_);
  node->get_parameter(name_ + ".visited_radius", visited_radius_);
  node->get_parameter(name_ + ".visited_penalty", visited_penalty_);
  node->get_parameter(name_ + ".known_gain_penalty", known_gain_penalty_);
  node->get_parameter(name_ + ".min_candidate_dist", min_candidate_dist_);
  node->get_parameter(name_ + ".min_robot_frontier_dist", min_robot_frontier_dist_);
  node->get_parameter(name_ + ".robot_clear_radius", robot_clear_radius_);
  node->get_parameter(name_ + ".unknown_clear_radius", unknown_clear_radius_);
  node->get_parameter(name_ + ".viewpoint_free_z_min", viewpoint_free_z_min_);
  node->get_parameter(name_ + ".viewpoint_free_z_max", viewpoint_free_z_max_);
  node->get_parameter(name_ + ".viewpoint_free_z_step", viewpoint_free_z_step_);
  node->get_parameter(name_ + ".sensor_height", sensor_height_);
  node->get_parameter(name_ + ".frontier_slope_deg", frontier_slope_deg_);
  node->get_parameter(name_ + ".viewpoint_slope_deg", viewpoint_slope_deg_);

  const std::string map_key = shared_map_ ? shared_map_key_ : name_;
  map_state_ = getSharedMapState(map_key, resolution_, depth_levels_);

  {
    std::lock_guard<std::mutex> lock(map_state_->mutex);
    if (!map_state_->cloud_sub) {
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
  last_map_publish_time_ = clock_->now();

  RCLCPP_INFO(
    logger_,
    "[%s] configured FAEL UFOMap frontier selector: cloud=%s target_frame=%s "
    "resolution=%.3f depth_levels=%d insert_depth=%d shared_map=%s key=%s owner=%s",
    name_.c_str(), point_cloud_topic_.c_str(), frame_id_.c_str(), resolution_,
    depth_levels_, insert_depth_, shared_map_ ? "true" : "false", map_key.c_str(),
    map_state_->owner_name.c_str());
}

std::shared_ptr<FaelFrontierCore::SharedMapState> FaelFrontierCore::getSharedMapState(
  const std::string & key,
  double resolution,
  int depth_levels)
{
  std::lock_guard<std::mutex> lock(g_shared_maps_mutex);
  if (auto existing = g_shared_maps[key].lock()) {
    return existing;
  }

  auto state = std::make_shared<SharedMapState>();
  state->map = std::make_unique<ufo::map::OccupancyMap>(resolution, depth_levels, true);
  state->map->enableChangeDetection(true);
  g_shared_maps[key] = state;
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

  const int clear_radius = std::max(1, static_cast<int>(std::ceil(robot_clear_radius_ / resolution_)));
  for (int dx = -clear_radius; dx <= clear_radius; ++dx) {
    for (int dy = -clear_radius; dy <= clear_radius; ++dy) {
      for (int dz = -clear_radius; dz <= clear_radius; ++dz) {
        const auto point = ufo::map::Point3(
          sensor.x + static_cast<double>(dx) * resolution_,
          sensor.y + static_cast<double>(dy) * resolution_,
          sensor.z + static_cast<double>(dz) * resolution_);
        if (Point3{point.x(), point.y(), point.z()}.distance(sensor) <= robot_clear_radius_) {
          map_state_->map->setOccupancy(point, map_state_->map->getClampingThresMin(), insert_depth_);
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
  publishMapClouds();
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
        if (map_state_->map->isOccupied(p, insert_depth_) && fromUfoPoint(p).distance(point) <= radius) {
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
        if (map_state_->map->isUnknown(p, insert_depth_) && fromUfoPoint(p).distance(point) <= radius) {
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
  map_state_->known_cell_codes.insert(map_state_->changed_cell_codes.begin(), map_state_->changed_cell_codes.end());
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
    search_codes.insert(map_state_->map->toCode(center.x() - resolution_, center.y(), center.z(), code.getDepth()));
    search_codes.insert(map_state_->map->toCode(center.x() + resolution_, center.y(), center.z(), code.getDepth()));
    search_codes.insert(map_state_->map->toCode(center.x(), center.y() - resolution_, center.z(), code.getDepth()));
    search_codes.insert(map_state_->map->toCode(center.x(), center.y() + resolution_, center.z(), code.getDepth()));
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

  updated_frontiers.insert(map_state_->local_frontier_cells.begin(), map_state_->local_frontier_cells.end());
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
    if (frontier.distanceXY(current) > max_range_ * 1.5 &&
        !map_state_->frontiers_viewpoints.empty())
    {
      const auto old = map_state_->frontiers_viewpoints.find(frontier_code);
      if (old != map_state_->frontiers_viewpoints.end()) {
        const auto old_idx = representative_index(old->second);
        next_frontiers_viewpoints[frontier_code] = old->second;
        attached[old_idx].push_back(frontier);
        continue;
      }
    }

    double best_dist = std::numeric_limits<double>::max();
    std::optional<std::size_t> best_idx;
    const auto nearby_viewpoints =
      radiusCandidates(viewpoint_grid, frontier, attach_range, grid_size);
    for (const auto i : nearby_viewpoints) {
      const auto & viewpoint = representative_points[i];
      const double dist_xy = frontier.distanceXY(viewpoint);
      if (dist_xy >= attach_range || dist_xy >= best_dist || dist_xy <= 1e-6) {
        continue;
      }
      if (std::fabs(frontier.z - viewpoint.z) / dist_xy >= slope_limit) {
        continue;
      }
      if (!isViewpointConnectionFree(viewpoint, frontier)) {
        continue;
      }
      best_dist = dist_xy;
      best_idx = i;
    }

    if (best_idx) {
      next_frontiers_viewpoints[frontier_code] = representative_points[*best_idx];
      attached[*best_idx].push_back(frontier);
    }
  }
  map_state_->frontiers_viewpoints = std::move(next_frontiers_viewpoints);

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

  if (min_candidate_count_ > 0 && filtered.size() < static_cast<std::size_t>(min_candidate_count_)) {
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

  return filtered;
}

RoadGraph FaelFrontierCore::buildRoadGraph(
  const Point3 & current,
  const std::vector<Candidate> & candidates) const
{
  RoadGraph graph;
  auto add_node = [&](const Point3 & point, double min_interval) {
      for (const auto & existing : graph.nodes) {
        if (point.distanceXY(existing) < min_interval) {
          return;
        }
      }
      graph.nodes.push_back(point);
    };

  add_node(current, 0.0);

  const int steps = std::max(1, static_cast<int>(std::ceil(local_range_ / sample_dist_)));
  const double min_interval = std::max(sample_dist_, 1.0);
  for (int ix = -steps; ix <= steps; ++ix) {
    for (int iy = -steps; iy <= steps; ++iy) {
      Point3 sample{
        current.x + static_cast<double>(ix) * sample_dist_,
        current.y + static_cast<double>(iy) * sample_dist_,
        current.z + sensor_height_};
      if (sample.distanceXY(current) > local_range_) {
        continue;
      }
      if (!hasFreeVoxelNearHeight(sample) ||
          isNearOccupied(sample, robot_clear_radius_) ||
          isNearUnknown(sample, unknown_clear_radius_))
      {
        continue;
      }
      add_node(sample, min_interval);
    }
  }

  for (const auto & candidate : candidates) {
    add_node(candidate.point, resolution_);
  }

  graph.edges.resize(graph.nodes.size());
  const double grid_size = std::max(resolution_, road_graph_dist_);
  std::unordered_map<GridCell, std::vector<std::size_t>, GridCellHash> node_grid;
  node_grid.reserve(graph.nodes.size());
  for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
    node_grid[toGridCell(graph.nodes[i], grid_size)].push_back(i);
  }

  for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
    std::vector<std::pair<double, std::size_t>> neighbors;
    const auto nearby_nodes =
      radiusCandidates(node_grid, graph.nodes[i], road_graph_dist_, grid_size);
    for (const auto j : nearby_nodes) {
      if (j <= i) {
        continue;
      }
      const double dist = graph.nodes[i].distanceXY(graph.nodes[j]);
      if (dist > road_graph_dist_) {
        continue;
      }
      neighbors.emplace_back(dist, j);
    }
    std::sort(neighbors.begin(), neighbors.end());

    int connected = 0;
    for (const auto & neighbor : neighbors) {
      const auto j = neighbor.second;
      graph.edges[i].push_back(j);
      graph.edges[j].push_back(i);
      ++connected;
      if (road_graph_connectable_num_ > 0 && connected >= road_graph_connectable_num_) {
        break;
      }
    }
  }

  return graph;
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
    const Candidate * selected = map_state_->candidates.empty() ? nullptr : &map_state_->candidates.front();
    publishCandidates(map_state_->candidates, selected);
    publishTopology(map_state_->candidates, current, selected, false);
    if (selected && selected_candidate_pub_ && selected_candidate_pub_->is_activated()) {
      std_msgs::msg::Header header;
      header.stamp = clock_->now();
      header.frame_id = frame_id_;
      selected_candidate_pub_->publish(makePose(selected->point, header));
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
  auto road_graph = buildRoadGraph(current, candidates);
  const auto graph_done = std::chrono::steady_clock::now();
  map_state_->candidates = candidates;
  map_state_->candidates_stamp = clock_->now();
  map_state_->candidates_owner = name_;
  map_state_->candidates_origin = current;
  map_state_->candidates_map_revision = map_state_->map_revision;
  map_state_->has_cached_candidates = true;
  map_state_->road_graph = road_graph;

  RCLCPP_INFO(
    logger_,
    "[%s] candidate refresh timing ms: frontier=%.2f get_frontiers=%.2f compact=%.2f viewpoints=%.2f "
    "attach=%.2f road_graph=%.2f total=%.2f counts{frontiers=%zu viewpoints=%zu candidates=%zu "
    "attach_frontiers=%zu road_nodes=%zu} road_graph_collision_check=false",
    name_.c_str(),
    std::chrono::duration<double, std::milli>(frontier_done - frontier_start).count(),
    std::chrono::duration<double, std::milli>(get_frontiers_done - frontier_done).count(),
    std::chrono::duration<double, std::milli>(compact_done - get_frontiers_done).count(),
    std::chrono::duration<double, std::milli>(viewpoints_done - compact_done).count(),
    std::chrono::duration<double, std::milli>(attach_done - viewpoints_done).count(),
    std::chrono::duration<double, std::milli>(graph_done - attach_done).count(),
    std::chrono::duration<double, std::milli>(graph_done - total_start).count(),
    frontiers.size(), viewpoints.size(), candidates.size(), attach_frontiers.size(),
    road_graph.nodes.size());

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
    selected_candidate_pub_->publish(makePose(candidates.front().point, header));
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
  const Candidate & candidate) const
{
  std::lock_guard<std::mutex> lock(map_state_->mutex);
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.header.frame_id = frame_id_;
  const Point3 start_point{start.pose.position.x, start.pose.position.y, start.pose.position.z};
  auto points = shortestPathRoadGraph(start_point, candidate, map_state_->road_graph);
  for (const auto & point : points) {
    path.poses.push_back(makePose(point, path.header));
  }
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
    selected_candidate_pub_->publish(makePose(selected.point, header));
  }
}

std::vector<Point3> FaelFrontierCore::shortestPathRoadGraph(
  const Point3 & start,
  const Candidate & goal,
  const RoadGraph & graph) const
{
  if (graph.nodes.empty()) {
    return {};
  }

  auto nearest_node = [&](const Point3 & point) {
      std::size_t best_index = 0;
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
        const double distance = graph.nodes[i].distanceXY(point);
        if (distance < best_distance) {
          best_distance = distance;
          best_index = i;
        }
      }
      return best_index;
    };

  const auto start_index = nearest_node(start);
  const auto goal_index = nearest_node(goal.point);
  RCLCPP_DEBUG(
    logger_,
    "[%s] topology path snap: start_node=%zu dist=%.2f goal_node=%zu dist=%.2f graph_nodes=%zu",
    name_.c_str(), start_index, graph.nodes[start_index].distanceXY(start),
    goal_index, graph.nodes[goal_index].distanceXY(goal.point), graph.nodes.size());

  struct Node
  {
    std::size_t index;
    double f;
    double g;
    bool operator<(const Node & other) const {return f > other.f;}
  };

  std::priority_queue<Node> open;
  std::vector<double> g_score(graph.nodes.size(), std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(graph.nodes.size(), graph.nodes.size());
  g_score[start_index] = 0.0;
  open.push(Node{start_index, graph.nodes[start_index].distanceXY(graph.nodes[goal_index]), 0.0});

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    if (current.index == goal_index) {
      std::vector<Point3> path;
      for (std::size_t idx = goal_index; idx != graph.nodes.size(); idx = parent[idx]) {
        path.push_back(graph.nodes[idx]);
        if (idx == start_index) {
          break;
        }
      }
      std::reverse(path.begin(), path.end());
      if (path.empty() || path.front().distanceXY(start) > resolution_) {
        path.insert(path.begin(), start);
      } else {
        path.front() = start;
      }
      if (path.empty() || path.back().distanceXY(goal.point) > resolution_) {
        path.push_back(goal.point);
      } else {
        path.back() = goal.point;
      }
      RCLCPP_DEBUG(
        logger_,
        "[%s] topology path solved: poses=%zu start_node=%zu goal_node=%zu",
        name_.c_str(), path.size(), start_index, goal_index);
      return path;
    }

    if (current.g > g_score[current.index]) {
      continue;
    }

    if (current.index >= graph.edges.size()) {
      continue;
    }

    for (const auto next : graph.edges[current.index]) {
      if (next >= graph.nodes.size()) {
        continue;
      }
      const double tentative = current.g + graph.nodes[current.index].distanceXY(graph.nodes[next]);
      if (tentative < g_score[next]) {
        parent[next] = current.index;
        g_score[next] = tentative;
        open.push(Node{
          next,
          tentative + graph.nodes[next].distanceXY(graph.nodes[goal_index]),
          tentative});
      }
    }
  }

  return {};
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

  candidate_pub_->publish(markers);
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
  graph_nodes.scale.x = 0.14;
  graph_nodes.scale.y = 0.14;
  graph_nodes.color.r = 1.0f;
  graph_nodes.color.g = 0.0f;
  graph_nodes.color.b = 0.0f;
  graph_nodes.color.a = 1.0f;

  visualization_msgs::msg::Marker graph_edges = make_marker(
    5, "fael_topology_graph_edges", visualization_msgs::msg::Marker::LINE_LIST);
  graph_edges.scale.x = 0.055;
  graph_edges.color.r = 0.1f;
  graph_edges.color.g = 0.5f;
  graph_edges.color.b = 1.0f;
  graph_edges.color.a = 0.35f;

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

  const auto & road_graph = map_state_->road_graph;
  for (const auto & node : road_graph.nodes) {
    graph_nodes.points.push_back(to_msg_point(node));
  }
  if (graph_nodes.points.empty()) {
    graph_nodes.points.push_back(to_msg_point(current));
  }

  for (std::size_t i = 0; i < road_graph.edges.size() && i < road_graph.nodes.size(); ++i) {
    for (const auto j : road_graph.edges[i]) {
      if (j >= road_graph.nodes.size() || j < i) {
        continue;
      }
      graph_edges.points.push_back(to_msg_point(road_graph.nodes[i]));
      graph_edges.points.push_back(to_msg_point(road_graph.nodes[j]));
    }
  }

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

  if (publish_best_path && selected) {
    const auto path_points = shortestPathRoadGraph(current, *selected, road_graph);
    for (const auto & point : path_points) {
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

  topology_pub_->publish(markers);
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

  occupied_map_pub_->publish(build_cloud(true, false));
  free_map_pub_->publish(build_cloud(false, true));
}

}  // namespace navflex_frontier_planner
