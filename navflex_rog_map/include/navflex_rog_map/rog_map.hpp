// Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong
// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef NAVFLEX_ROG_MAP__ROG_MAP_HPP_
#define NAVFLEX_ROG_MAP__ROG_MAP_HPP_

#include <atomic>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "navflex_rog_map/footprint.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace navflex_rog_map
{

class OfficialRogMap;

enum class OccupancyState : uint8_t {FREE = 0, OCCUPIED = 1, UNKNOWN = 2};
struct Index3D {int x{0}; int y{0}; int z{0};};
struct Bounds3D
{
  double min_x{0.0}; double min_y{0.0}; double min_z{0.0};
  double max_x{0.0}; double max_y{0.0}; double max_z{0.0};
};
struct RogMapInput
{
  geometry_msgs::msg::Point sensor_origin;
  std::vector<geometry_msgs::msg::Point> points;
  uint64_t stamp_nanoseconds{0};
};
struct RogMapConfig
{
  std::string frame_id{"map"}; double global_resolution{0.4}; double local_resolution{0.1};
  Bounds3D global_bounds{-50.0, -50.0, -5.0, 50.0, 50.0, 15.0};
  double local_size_x{12.0}; double local_size_y{12.0}; double local_size_z{6.0};
  double hit_probability{0.70}; double miss_probability{0.40}; double occupied_threshold{0.70};
  double min_probability{0.12}; double max_probability{0.97};
  double ray_min_range{0.3}; double ray_max_range{15.0}; double esdf_max_distance{5.0};
  double inflation_resolution{0.2}; int inflation_step{2};
  bool unknown_inflation{false}; int unknown_inflation_step{1};
  double unknown_threshold{0.7}; int point_filter_num{2}; int batch_update_size{1};
  double virtual_ground_height{-5.0}; double virtual_ceiling_height{15.0};
};

class RogMap
{
public:
  using Ptr = std::shared_ptr<RogMap>;
  using ConstPtr = std::shared_ptr<const RogMap>;
  explicit RogMap(RogMapConfig config);
  ~RogMap();
  void update(const RogMapInput & input);
  void update(const geometry_msgs::msg::Point &, const std::vector<geometry_msgs::msg::Point> &);
  void reset();
  std::string frameId() const; double resolution() const; double localResolution() const;
  Bounds3D bounds() const; Bounds3D localBounds() const; uint64_t revision() const;
  bool worldToGrid(const geometry_msgs::msg::Point &, Index3D &) const;
  geometry_msgs::msg::Point gridToWorld(const Index3D &) const;
  OccupancyState state(const Index3D &) const; float probability(const Index3D &) const;
  OccupancyState localState(const geometry_msgs::msg::Point &) const;
  OccupancyState inflatedState(const geometry_msgs::msg::Point &) const;
  bool isFrontier(const geometry_msgs::msg::Point &) const;
  bool isCollisionFree(const geometry_msgs::msg::Point &, double radius) const;
  bool isCollisionFree(
    const geometry_msgs::msg::Pose &, const Footprint3D & footprint) const;
  bool raycastFree(
    const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &,
    double radius) const;
  bool raycastFree(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const Footprint3D & footprint) const;
  bool distanceAt(const geometry_msgs::msg::Point &, double &) const;
  bool distanceAndGradientAt(
    const geometry_msgs::msg::Point &, double &,
    geometry_msgs::msg::Vector3 &) const;
  bool secondGradientAt(
    const geometry_msgs::msg::Point &, geometry_msgs::msg::Vector3 &) const;
  sensor_msgs::msg::PointCloud2 occupiedCloud() const;
  sensor_msgs::msg::PointCloud2 localOccupiedCloud() const;

private:
  struct Cell {float log_odds{0.0F};};
  static int64_t key(const Index3D &); static double logit(double);
  void updateGlobalRay(const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &);
  RogMapConfig config_; mutable std::shared_mutex mutex_;
  std::unordered_map<int64_t, Cell> global_cells_;
  float hit_log_{0.0F}; float miss_log_{0.0F}; float min_log_{0.0F};
  float max_log_{0.0F}; float occupied_log_{0.0F}; std::atomic<uint64_t> revision_{0};
  std::unique_ptr<OfficialRogMap> official_map_;
};
}  // namespace navflex_rog_map
#endif  // NAVFLEX_ROG_MAP__ROG_MAP_HPP_
