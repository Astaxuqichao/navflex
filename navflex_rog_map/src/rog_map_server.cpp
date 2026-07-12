// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <memory>

#include "navflex_rog_map/rog_map_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navflex_rog_map::RogMapROS>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
