// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "navflex_rog_map/footprint.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace navflex_rog_map
{

FootprintType footprintTypeFromString(const std::string & type)
{
  std::string normalized = type;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char value) {return static_cast<char>(std::tolower(value));});
  if (normalized == "sphere") {
    return FootprintType::SPHERE;
  }
  if (normalized == "cylinder") {
    return FootprintType::CYLINDER;
  }
  if (normalized == "box") {
    return FootprintType::BOX;
  }
  if (normalized == "double_sphere" || normalized == "dual_sphere") {
    return FootprintType::DOUBLE_SPHERE;
  }
  throw std::invalid_argument(
          "footprint_type must be sphere, cylinder, box, or double_sphere");
}

std::string toString(FootprintType type)
{
  switch (type) {
    case FootprintType::SPHERE:
      return "sphere";
    case FootprintType::CYLINDER:
      return "cylinder";
    case FootprintType::BOX:
      return "box";
    case FootprintType::DOUBLE_SPHERE:
      return "double_sphere";
  }
  return "unknown";
}

void validateFootprint(const Footprint3D & footprint)
{
  if (footprint.safety_margin < 0.0) {
    throw std::invalid_argument("footprint safety_margin must be non-negative");
  }
  if (footprint.type == FootprintType::SPHERE && footprint.radius <= 0.0) {
    throw std::invalid_argument("sphere footprint radius must be positive");
  }
  if (footprint.type == FootprintType::CYLINDER &&
    (footprint.radius <= 0.0 || footprint.height <= 0.0))
  {
    throw std::invalid_argument("cylinder footprint radius and height must be positive");
  }
  if (footprint.type == FootprintType::BOX &&
    (footprint.size.x <= 0.0 || footprint.size.y <= 0.0 || footprint.size.z <= 0.0))
  {
    throw std::invalid_argument("box footprint size must contain three positive values");
  }
  if (footprint.type == FootprintType::DOUBLE_SPHERE &&
    (footprint.front_sphere.radius <= 0.0 || footprint.rear_sphere.radius <= 0.0))
  {
    throw std::invalid_argument("double_sphere radii must be positive");
  }
}

}  // namespace navflex_rog_map
