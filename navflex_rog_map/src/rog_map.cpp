// Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong
// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "navflex_rog_map/rog_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>
#include <stdexcept>

#include "rog_map/rog_map.h"
#include "pcl/io/pcd_io.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

#ifdef logit
#undef logit
#endif

namespace navflex_rog_map
{
class OfficialRogMap : public rog_map::ROGMap
{
public:
  void initialize(const RogMapConfig & config, RogMap * owner)
  {
    owner_ = owner;
    cfg_.resolution = config.local_resolution;
    cfg_.inflation_resolution = config.inflation_resolution;
    cfg_.inflation_step = config.inflation_step;
    cfg_.unk_inflation_en = config.unknown_inflation;
    cfg_.unk_inflation_step = config.unknown_inflation_step;
    cfg_.unk_thresh = config.unknown_threshold;
    // Filtering is performed before constructing the temporary PCL cloud.
    cfg_.point_filt_num = 1;
    cfg_.batch_update_size = config.batch_update_size;
    cfg_.p_hit = config.hit_probability;
    cfg_.p_miss = config.miss_probability;
    cfg_.p_min = config.min_probability;
    cfg_.p_max = config.max_probability;
    cfg_.p_occ = config.occupied_threshold;
    cfg_.p_free = config.miss_probability;
    cfg_.l_hit = std::log(cfg_.p_hit / (1.0F - cfg_.p_hit));
    cfg_.l_miss = std::log(cfg_.p_miss / (1.0F - cfg_.p_miss));
    cfg_.l_min = std::log(cfg_.p_min / (1.0F - cfg_.p_min));
    cfg_.l_max = std::log(cfg_.p_max / (1.0F - cfg_.p_max));
    cfg_.l_occ = std::log(cfg_.p_occ / (1.0F - cfg_.p_occ));
    cfg_.l_free = std::log(cfg_.p_free / (1.0F - cfg_.p_free));
    cfg_.raycast_range_min = config.ray_min_range;
    cfg_.raycast_range_max = config.ray_max_range;
    cfg_.sqr_raycast_range_min = config.ray_min_range * config.ray_min_range;
    cfg_.sqr_raycast_range_max = config.ray_max_range * config.ray_max_range;
    cfg_.map_size_d = rog_map::Vec3f(
      config.local_size_x, config.local_size_y, config.local_size_z);
    cfg_.local_update_box_d = cfg_.map_size_d;
    cfg_.map_sliding_en = true;
    cfg_.map_sliding_thresh = config.map_sliding_threshold;
    cfg_.fix_map_origin.setZero();
    cfg_.virtual_ground_height = config.virtual_ground_height;
    cfg_.virtual_ceil_height = config.virtual_ceiling_height;
    cfg_.frontier_extraction_en = config.frontier_extraction;
    cfg_.esdf_en = config.enable_esdf;
    cfg_.esdf_update_interval = config.esdf_update_interval;
    cfg_.esdf_resolution = config.local_resolution;
    cfg_.esdf_local_update_box = cfg_.map_size_d;
    cfg_.visualization_range = cfg_.map_size_d;
    cfg_.frame_id = config.frame_id;
    cfg_.ros_callback_en = false;
    cfg_.load_pcd_en = false;
    cfg_.resetMapSize();
    init();
  }

  double esdfDistance(const rog_map::Vec3f & point) const
  {return esdf_map_->getDistance(point);}

  void esdfDistanceAndGradient(
    const Eigen::Vector3d & point, double & distance, Eigen::Vector3d & gradient)
  {
    esdf_map_->evaluateEDT(point, distance);
    esdf_map_->evaluateFirstGrad(point, gradient);
  }

  void esdfSecondGradient(const Eigen::Vector3d & point, Eigen::Vector3d & gradient)
  {esdf_map_->evaluateSecondGrad(point, gradient);}

  void loadOccupiedCloud(const rog_map::PointCloud & cloud)
  {
    updateOccPointCloud(cloud);
    if (cfg_.esdf_en) {
      esdf_map_->updateESDF3D(getLocalMapOrigin());
    }
  }

protected:
  bool shouldUpdateProbMap(const rog_map::PclPoint & point) const override
  {
    return owner_->shouldUpdateLocal(static_cast<unsigned int>(point.intensity));
  }

  bool shouldObserveRaycast(const rog_map::PclPoint & point) const override
  {
    geometry_msgs::msg::Point endpoint;
    endpoint.x = point.x;
    endpoint.y = point.y;
    endpoint.z = point.z;
    return owner_->shouldObserveGlobal(
      static_cast<unsigned int>(point.intensity), endpoint);
  }

  bool beginRaycastObservation(const rog_map::Vec3f & endpoint, bool update_hit) override
  {
    geometry_msgs::msg::Point point;
    point.x = endpoint.x();
    point.y = endpoint.y();
    point.z = endpoint.z();
    return owner_->beginGlobalRay(point, update_hit);
  }

  void observeRaycastPoint(const rog_map::Vec3f & sample, bool is_hit) override
  {
    geometry_msgs::msg::Point point;
    point.x = sample.x();
    point.y = sample.y();
    point.z = sample.z();
    owner_->observeGlobalRayPoint(point, is_hit);
  }

  void endRaycastObservation() override
  {
    owner_->endGlobalRay();
  }

  const double getSystemWalltimeNow() override
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

private:
  RogMap * owner_{nullptr};
};

namespace
{
constexpr int64_t kOffset = 1 << 20;
constexpr int64_t kMask = (1 << 21) - 1;
double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}
}  // namespace

RogMap::RogMap(RogMapConfig config)
: config_(std::move(config))
{
  if (config_.global_resolution <= 0.0 || config_.local_resolution <= 0.0) {
    throw std::invalid_argument("ROG map resolutions must be positive");
  }
  const auto valid_probability = [](double value) {return value > 0.0 && value < 1.0;};
  if (!valid_probability(config_.hit_probability) ||
    !valid_probability(config_.miss_probability) ||
    !valid_probability(config_.occupied_threshold))
  {
    throw std::invalid_argument("ROG map probabilities must be between zero and one");
  }
  if (config_.hit_probability <= config_.occupied_threshold) {
    throw std::invalid_argument(
            "hit_probability must be greater than occupied_threshold so one hit can create an "
            "occupied cell");
  }
  if (config_.point_filter_num <= 0 || config_.global_point_filter_num <= 0 ||
    config_.batch_update_size <= 0 || config_.esdf_update_interval <= 0 ||
    config_.map_sliding_threshold < 0.0)
  {
    throw std::invalid_argument("ROG map filter, batch, and sliding parameters are invalid");
  }
  if (config_.load_pcd && config_.pcd_file.empty()) {
    throw std::invalid_argument("pcd_file must be set when load_pcd is enabled");
  }
  if (config_.load_pcd && config_.pcd_frame != config_.frame_id) {
    throw std::invalid_argument(
            "pcd_frame must match global_frame; PCD points are loaded directly");
  }
  hit_log_ = logit(config_.hit_probability);
  miss_log_ = logit(config_.miss_probability);
  min_log_ = logit(config_.min_probability);
  max_log_ = logit(config_.max_probability);
  occupied_log_ = logit(config_.occupied_threshold);
  global_endpoint_voxels_.reserve(4096);
  official_map_ = std::make_unique<OfficialRogMap>();
  official_map_->initialize(config_, this);
  if (config_.load_pcd) {
    pcl::PointCloud<pcl::PointXYZ> pcd_cloud;
    if (pcl::io::loadPCDFile(config_.pcd_file, pcd_cloud) < 0) {
      throw std::runtime_error("Failed to load PCD map: " + config_.pcd_file);
    }
    rog_map::PointCloud cloud;
    cloud.reserve(pcd_cloud.size());
    for (const auto & sample : pcd_cloud) {
      if (!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.z)) {
        continue;
      }
      rog_map::PclPoint rog_sample;
      rog_sample.x = sample.x;
      rog_sample.y = sample.y;
      rog_sample.z = sample.z;
      rog_sample.intensity = 0.0F;
      cloud.push_back(rog_sample);
      geometry_msgs::msg::Point point;
      point.x = sample.x; point.y = sample.y; point.z = sample.z;
      Index3D index;
      if (!worldToGrid(point, index)) {
        continue;
      }
      global_cells_[key(index)].log_odds = max_log_;
      ++loaded_pcd_points_;
    }
    official_map_->loadOccupiedCloud(cloud);
    ++revision_;
  }
}

RogMap::~RogMap() = default;

double RogMap::logit(double value) {return std::log(value / (1.0 - value));}
int64_t RogMap::key(const Index3D & i)
{
  return ((static_cast<int64_t>(i.x) + kOffset) << 42) |
         ((static_cast<int64_t>(i.y) + kOffset) << 21) |
         (static_cast<int64_t>(i.z) + kOffset);
}
std::string RogMap::frameId() const {return config_.frame_id;}
double RogMap::resolution() const {return config_.global_resolution;}
double RogMap::localResolution() const {return config_.local_resolution;}
Bounds3D RogMap::bounds() const {return config_.global_bounds;}
Bounds3D RogMap::localBounds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  const auto origin = official_map_->getLocalMapOrigin();
  const auto size = official_map_->getLocalMapSize();
  return {origin.x() - size.x() * 0.5, origin.y() - size.y() * 0.5,
    origin.z() - size.z() * 0.5, origin.x() + size.x() * 0.5,
    origin.y() + size.y() * 0.5, origin.z() + size.z() * 0.5};
}
uint64_t RogMap::revision() const {return revision_.load();}

bool RogMap::worldToGrid(
  const geometry_msgs::msg::Point & p, Index3D & i) const
{
  const auto & b = config_.global_bounds;
  if (p.x < b.min_x || p.y < b.min_y || p.z < b.min_z ||
    p.x >= b.max_x || p.y >= b.max_y || p.z >= b.max_z)
  {return false;}
  i.x = std::floor((p.x - b.min_x) / config_.global_resolution);
  i.y = std::floor((p.y - b.min_y) / config_.global_resolution);
  i.z = std::floor((p.z - b.min_z) / config_.global_resolution);
  return true;
}
geometry_msgs::msg::Point RogMap::gridToWorld(const Index3D & i) const
{
  geometry_msgs::msg::Point p; const auto & b = config_.global_bounds;
  p.x = b.min_x + (i.x + 0.5) * config_.global_resolution;
  p.y = b.min_y + (i.y + 0.5) * config_.global_resolution;
  p.z = b.min_z + (i.z + 0.5) * config_.global_resolution; return p;
}
float RogMap::probability(const Index3D & i) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_); auto it = global_cells_.find(key(i));
  if (it == global_cells_.end()) {return -1.0F;}
  return 1.0F / (1.0F + std::exp(-it->second.log_odds));
}
OccupancyState RogMap::state(const Index3D & i) const
{
  float p = probability(i); if (p < 0.0F) {return OccupancyState::UNKNOWN;}
  return p >= config_.occupied_threshold ? OccupancyState::OCCUPIED : OccupancyState::FREE;
}

OccupancyState RogMap::localState(const geometry_msgs::msg::Point & point) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  const auto type = official_map_->getGridType(rog_map::Vec3f(point.x, point.y, point.z));
  if (type == super_utils::OCCUPIED) {return OccupancyState::OCCUPIED;}
  if (type == super_utils::UNKNOWN) {return OccupancyState::UNKNOWN;}
  return OccupancyState::FREE;
}

OccupancyState RogMap::inflatedState(const geometry_msgs::msg::Point & point) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  const auto type = official_map_->getInfGridType(rog_map::Vec3f(point.x, point.y, point.z));
  if (type == super_utils::OCCUPIED) {return OccupancyState::OCCUPIED;}
  if (type == super_utils::UNKNOWN) {return OccupancyState::UNKNOWN;}
  return OccupancyState::FREE;
}

bool RogMap::isFrontier(const geometry_msgs::msg::Point & point) const
{
  if (!config_.frontier_extraction) {
    return false;
  }
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return official_map_->isFrontier(rog_map::Vec3f(point.x, point.y, point.z));
}

bool RogMap::shouldUpdateLocal(unsigned int flags) const
{
  return (flags & 1U) != 0U;
}

bool RogMap::shouldObserveGlobal(
  unsigned int flags, const geometry_msgs::msg::Point & endpoint) const
{
  if ((flags & 2U) == 0U) {
    return false;
  }
  const double range = distance(global_sensor_origin_, endpoint);
  return range >= config_.ray_min_range && range <= config_.ray_max_range;
}

bool RogMap::beginGlobalRay(
  const geometry_msgs::msg::Point & endpoint, bool update_hit)
{
  Index3D endpoint_index;
  has_current_global_hit_ = update_hit && worldToGrid(endpoint, endpoint_index);
  if (has_current_global_hit_) {
    current_global_hit_key_ = key(endpoint_index);
    if (!global_endpoint_voxels_.insert(current_global_hit_key_).second) {
      has_current_global_hit_ = false;
      return false;
    }
  }
  has_last_global_ray_key_ = false;
  return true;
}

void RogMap::observeGlobalRayPoint(
  const geometry_msgs::msg::Point & point, bool is_hit)
{
  Index3D index;
  if (!worldToGrid(point, index)) {
    return;
  }
  const int64_t cell_key = key(index);
  if (!is_hit &&
    ((has_current_global_hit_ && cell_key == current_global_hit_key_) ||
    (has_last_global_ray_key_ && cell_key == last_global_ray_key_)))
  {
    return;
  }
  auto & cell = global_cells_[cell_key];
  cell.log_odds = std::clamp(
    cell.log_odds + (is_hit ? hit_log_ : miss_log_), min_log_, max_log_);
  last_global_ray_key_ = cell_key;
  has_last_global_ray_key_ = true;
}

void RogMap::endGlobalRay()
{
  has_current_global_hit_ = false;
  has_last_global_ray_key_ = false;
}

void RogMap::update(
  const geometry_msgs::msg::Point & sensor_origin,
  const std::vector<geometry_msgs::msg::Point> & points)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  global_sensor_origin_ = sensor_origin;
  global_endpoint_voxels_.clear();
  rog_map::PointCloud cloud;
  const auto minimum_filter = std::min(
    config_.point_filter_num, config_.global_point_filter_num);
  cloud.reserve((points.size() + minimum_filter - 1) / minimum_filter);
  for (std::size_t index = 0; index < points.size(); ++index) {
    const bool update_local = index % config_.point_filter_num == 0;
    const bool update_global = index % config_.global_point_filter_num == 0;
    if (!update_local && !update_global) {
      continue;
    }
    const auto & point = points[index];
    rog_map::PclPoint sample;
    sample.x = point.x;
    sample.y = point.y;
    sample.z = point.z;
    sample.intensity = static_cast<float>(
      (update_local ? 1U : 0U) | (update_global ? 2U : 0U));
    cloud.push_back(sample);
  }
  super_utils::Pose pose;
  pose.first = rog_map::Vec3f(sensor_origin.x, sensor_origin.y, sensor_origin.z);
  pose.second = super_utils::Quatf::Identity();
  official_map_->updateMap(cloud, pose);
  ++revision_;
}

void RogMap::update(const RogMapInput & input)
{
  update(input.sensor_origin, input.points);
}

bool RogMap::distanceAt(const geometry_msgs::msg::Point & p, double & value) const
{
  if (!config_.enable_esdf) {
    value = std::numeric_limits<double>::infinity();
    return false;
  }
  std::shared_lock<std::shared_mutex> lock(mutex_);
  value = official_map_->esdfDistance(rog_map::Vec3f(p.x, p.y, p.z));
  return std::isfinite(value);
}
bool RogMap::distanceAndGradientAt(
  const geometry_msgs::msg::Point & p, double & value, geometry_msgs::msg::Vector3 & gradient) const
{
  if (!config_.enable_esdf) {
    value = std::numeric_limits<double>::infinity();
    gradient = geometry_msgs::msg::Vector3();
    return false;
  }
  std::shared_lock<std::shared_mutex> lock(mutex_);
  Eigen::Vector3d eigen_gradient;
  official_map_->esdfDistanceAndGradient(Eigen::Vector3d(p.x, p.y, p.z), value, eigen_gradient);
  gradient.x = eigen_gradient.x(); gradient.y = eigen_gradient.y(); gradient.z = eigen_gradient.z();
  return std::isfinite(value) && eigen_gradient.allFinite();
}

bool RogMap::secondGradientAt(
  const geometry_msgs::msg::Point & point, geometry_msgs::msg::Vector3 & gradient) const
{
  if (!config_.enable_esdf) {
    gradient = geometry_msgs::msg::Vector3();
    return false;
  }
  std::shared_lock<std::shared_mutex> lock(mutex_);
  Eigen::Vector3d result;
  official_map_->esdfSecondGradient(Eigen::Vector3d(point.x, point.y, point.z), result);
  gradient.x = result.x(); gradient.y = result.y(); gradient.z = result.z();
  return result.allFinite();
}

bool RogMap::isCollisionFree(const geometry_msgs::msg::Point & p, double radius) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  const rog_map::Vec3f point(p.x, p.y, p.z);
  if (official_map_->isOccupiedInflate(point) ||
    (config_.unknown_inflation && official_map_->isUnknownInflate(point)))
  {
    return false;
  }
  if (!config_.enable_esdf) {
    return true;
  }
  return official_map_->esdfDistance(point) > radius;
}

bool RogMap::isCollisionFree(
  const geometry_msgs::msg::Pose & pose, const Footprint3D & footprint) const
{
  validateFootprint(footprint);
  tf2::Quaternion rotation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  if (rotation.length2() < 1e-12) {
    rotation.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    rotation.normalize();
  }
  const tf2::Transform transform(
    rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  const auto worldPoint = [&transform](const geometry_msgs::msg::Vector3 & local) {
      const tf2::Vector3 world = transform * tf2::Vector3(local.x, local.y, local.z);
      geometry_msgs::msg::Point point;
      point.x = world.x();
      point.y = world.y();
      point.z = world.z();
      return point;
    };
  if (footprint.type == FootprintType::SPHERE) {
    return isCollisionFree(
      worldPoint(footprint.offset), footprint.radius + footprint.safety_margin);
  }
  if (footprint.type == FootprintType::DOUBLE_SPHERE) {
    return isCollisionFree(
      worldPoint(footprint.front_sphere.offset),
      footprint.front_sphere.radius + footprint.safety_margin) &&
           isCollisionFree(
      worldPoint(footprint.rear_sphere.offset),
      footprint.rear_sphere.radius + footprint.safety_margin);
  }
  const double step = std::max(0.02, config_.local_resolution * 0.5);
  if (footprint.type == FootprintType::CYLINDER) {
    const int samples = std::max(1, static_cast<int>(std::ceil(footprint.height / step)));
    for (int index = 0; index <= samples; ++index) {
      geometry_msgs::msg::Vector3 local = footprint.offset;
      local.z += -0.5 * footprint.height + footprint.height * index / samples;
      if (!isCollisionFree(
          worldPoint(local), footprint.radius + footprint.safety_margin))
      {
        return false;
      }
    }
    return true;
  }

  const int count_x = std::max(1, static_cast<int>(std::ceil(footprint.size.x / step)));
  const int count_y = std::max(1, static_cast<int>(std::ceil(footprint.size.y / step)));
  const int count_z = std::max(1, static_cast<int>(std::ceil(footprint.size.z / step)));
  for (int x = 0; x <= count_x; ++x) {
    for (int y = 0; y <= count_y; ++y) {
      for (int z = 0; z <= count_z; ++z) {
        geometry_msgs::msg::Vector3 local = footprint.offset;
        local.x += -0.5 * footprint.size.x + footprint.size.x * x / count_x;
        local.y += -0.5 * footprint.size.y + footprint.size.y * y / count_y;
        local.z += -0.5 * footprint.size.z + footprint.size.z * z / count_z;
        if (!isCollisionFree(worldPoint(local), footprint.safety_margin)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool RogMap::raycastFree(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b,
  double radius) const
{
  double length = distance(a, b);
  int count =
    std::max(1, static_cast<int>(std::ceil(length / (config_.local_resolution * 0.5))));
  for (int i = 0;
    i <= count; ++i)
  {
    double t = static_cast<double>(i) / count;geometry_msgs::msg::Point p;
    p.x = a.x + t * (b.x - a.x);p.y = a.y + t * (b.y - a.y);p.z = a.z + t * (b.z - a.z);
    if (!isCollisionFree(p, radius)) {
      return false;
    }
  }
  return true;
}

bool RogMap::raycastFree(
  const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & end,
  const Footprint3D & footprint) const
{
  const double length = distance(start.position, end.position);
  const int count = std::max(
    1, static_cast<int>(std::ceil(length / (config_.local_resolution * 0.5))));
  tf2::Quaternion start_rotation(
    start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
  tf2::Quaternion end_rotation(
    end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w);
  if (start_rotation.length2() < 1e-12) {
    start_rotation.setValue(0.0, 0.0, 0.0, 1.0);
  }
  if (end_rotation.length2() < 1e-12) {
    end_rotation.setValue(0.0, 0.0, 0.0, 1.0);
  }
  start_rotation.normalize();
  end_rotation.normalize();
  for (int index = 0; index <= count; ++index) {
    const double ratio = static_cast<double>(index) / count;
    geometry_msgs::msg::Pose sample;
    sample.position.x = start.position.x + ratio * (end.position.x - start.position.x);
    sample.position.y = start.position.y + ratio * (end.position.y - start.position.y);
    sample.position.z = start.position.z + ratio * (end.position.z - start.position.z);
    const tf2::Quaternion rotation = start_rotation.slerp(end_rotation, ratio);
    sample.orientation.x = rotation.x();
    sample.orientation.y = rotation.y();
    sample.orientation.z = rotation.z();
    sample.orientation.w = rotation.w();
    if (!isCollisionFree(sample, footprint)) {
      return false;
    }
  }
  return true;
}

sensor_msgs::msg::PointCloud2 RogMap::occupiedCloud() const
{
  sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id = config_.frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(
    1,
    "xyz");
  std::vector<geometry_msgs::msg::Point> points;
  {std::shared_lock<std::shared_mutex> lock(mutex_);for (const auto & item : global_cells_) {
      if (item.second.log_odds < occupied_log_) {
        continue;
      }
      int64_t raw = item.first;
      Index3D i{static_cast<int>((raw >>
        42) & kMask) - static_cast<int>(kOffset),
        static_cast<int>((raw >> 21) & kMask) - static_cast<int>(kOffset),
        static_cast<int>(raw & kMask) - static_cast<int>(kOffset)};
      points.push_back(gridToWorld(i));
    }
  }
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto & p : points) {
    *x = p.x;*y = p.y;*z = p.z;++x;++y;++z;
  }
  return cloud;
}
sensor_msgs::msg::PointCloud2 RogMap::localOccupiedCloud() const
{
  sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id = config_.frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(
    1,
    "xyz");
  std::shared_lock<std::shared_mutex> lock(mutex_);
  const auto origin = official_map_->getLocalMapOrigin();
  const auto size = official_map_->getLocalMapSize();
  super_utils::vec_E<rog_map::Vec3f> occupied;
  official_map_->boxSearch(
    origin - size * 0.5, origin + size * 0.5, super_utils::OCCUPIED, occupied);
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(occupied.size());
  for (const auto & point : occupied) {
    geometry_msgs::msg::Point output;output.x = point.x();output.y = point.y();output.z = point.z();
    points.push_back(output);
  }
  modifier.resize(points.size());sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(
    cloud,
    "y"),
  iz(cloud, "z");for (const auto & p : points) {
    *ix = p.x;*iy = p.y;*iz = p.z;++ix;++iy;++iz;
  }
  return cloud;
}
void RogMap::reset()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  global_cells_.clear();
  ++revision_;
}
}  // namespace navflex_rog_map
