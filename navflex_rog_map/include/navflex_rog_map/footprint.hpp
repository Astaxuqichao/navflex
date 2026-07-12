// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef NAVFLEX_ROG_MAP__FOOTPRINT_HPP_
#define NAVFLEX_ROG_MAP__FOOTPRINT_HPP_

#include <string>

#include "geometry_msgs/msg/vector3.hpp"

namespace navflex_rog_map
{

enum class FootprintType : uint8_t {SPHERE, CYLINDER, BOX, DOUBLE_SPHERE};

struct Sphere3D
{
  geometry_msgs::msg::Vector3 offset;
  double radius{0.35};
};

struct Footprint3D
{
  FootprintType type{FootprintType::SPHERE};
  geometry_msgs::msg::Vector3 offset;
  double radius{0.35};
  double height{0.7};
  geometry_msgs::msg::Vector3 size;
  Sphere3D front_sphere;
  Sphere3D rear_sphere;
  double safety_margin{0.0};
};

FootprintType footprintTypeFromString(const std::string & type);
std::string toString(FootprintType type);
void validateFootprint(const Footprint3D & footprint);

}  // namespace navflex_rog_map

#endif  // NAVFLEX_ROG_MAP__FOOTPRINT_HPP_
