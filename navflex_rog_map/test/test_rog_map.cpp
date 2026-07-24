// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "navflex_rog_map/rog_map.hpp"

namespace navflex_rog_map
{
namespace
{

RogMapConfig makeConfig()
{
  RogMapConfig config;
  config.global_resolution = 0.5;
  config.global_bounds = {-10.0, -10.0, -5.0, 10.0, 10.0, 5.0};
  config.local_resolution = 0.25;
  config.local_size_x = 2.0;
  config.local_size_y = 2.0;
  config.local_size_z = 2.0;
  config.hit_probability = 0.8;
  config.miss_probability = 0.4;
  config.occupied_threshold = 0.7;
  config.ray_min_range = 0.1;
  config.ray_max_range = 8.0;
  config.esdf_max_distance = 5.0;
  config.inflation_resolution = 0.5;
  config.inflation_step = 1;
  config.point_filter_num = 1;
  config.global_point_filter_num = 1;
  config.batch_update_size = 1;
  config.map_sliding_threshold = 0.5;
  config.frontier_extraction = false;
  config.enable_esdf = true;
  config.esdf_update_interval = 1;
  config.virtual_ground_height = -5.0;
  config.virtual_ceiling_height = 5.0;
  return config;
}

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

TEST(RogMapGlobalIntegration, ConnectsOccupancyCollisionAndEsdf)
{
  RogMap map(makeConfig());
  const auto origin = point(0.25, 0.25, 0.25);
  const auto local_obstacle = point(0.75, 0.25, 0.25);
  const auto global_obstacle = point(3.25, 0.25, 0.25);
  map.update(origin, std::vector<geometry_msgs::msg::Point>{
    local_obstacle, global_obstacle});

  EXPECT_EQ(map.globalState(global_obstacle), OccupancyState::OCCUPIED);
  EXPECT_EQ(map.globalState(point(1.25, 0.25, 0.25)), OccupancyState::FREE);
  EXPECT_EQ(map.globalState(point(4.25, 1.25, 0.25)), OccupancyState::UNKNOWN);
  EXPECT_EQ(
    map.globalInflatedState(point(2.75, 0.25, 0.25)),
    OccupancyState::OCCUPIED);

  Footprint3D footprint;
  footprint.type = FootprintType::SPHERE;
  footprint.radius = 0.1;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position = global_obstacle;
  EXPECT_FALSE(map.isGlobalCollisionFree(pose, footprint));
  pose.position = point(1.75, 0.25, 0.25);
  EXPECT_TRUE(map.isGlobalCollisionFree(pose, footprint));
  pose.position = point(4.25, 1.25, 0.25);
  EXPECT_FALSE(map.isGlobalCollisionFree(pose, footprint, true));
  EXPECT_TRUE(map.isGlobalCollisionFree(pose, footprint, false));

  double distance = 0.0;
  geometry_msgs::msg::Vector3 gradient;
  EXPECT_TRUE(map.distanceAndGradientAt(point(2.25, 0.25, 0.25), distance, gradient));
  EXPECT_NEAR(distance, 1.0, 1e-6);
  EXPECT_NEAR(gradient.x, -1.0, 1e-6);
  EXPECT_NEAR(gradient.y, 0.0, 1e-6);
  EXPECT_NEAR(gradient.z, 0.0, 1e-6);
}

TEST(RogMapGlobalIntegration, ResetClearsBothMapLayers)
{
  RogMap map(makeConfig());
  const auto origin = point(0.25, 0.25, 0.25);
  const auto local_obstacle = point(0.75, 0.25, 0.25);
  map.update(origin, std::vector<geometry_msgs::msg::Point>{local_obstacle});

  EXPECT_EQ(map.globalState(local_obstacle), OccupancyState::OCCUPIED);
  EXPECT_EQ(map.localState(local_obstacle), OccupancyState::OCCUPIED);

  map.reset();

  EXPECT_EQ(map.globalState(local_obstacle), OccupancyState::UNKNOWN);
  EXPECT_NE(map.localState(local_obstacle), OccupancyState::OCCUPIED);
  EXPECT_EQ(map.loadedPcdPointCount(), 0U);
}

}  // namespace
}  // namespace navflex_rog_map
