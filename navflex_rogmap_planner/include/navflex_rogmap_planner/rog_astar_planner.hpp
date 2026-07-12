// Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong
// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef NAVFLEX_ROGMAP_PLANNER__ROG_ASTAR_PLANNER_HPP_
#define NAVFLEX_ROGMAP_PLANNER__ROG_ASTAR_PLANNER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "navflex_rogmap_core/global_planner.hpp"

namespace navflex_rogmap_planner
{

class RogAStarPlanner : public navflex_rogmap_core::GlobalPlanner
{
public:
  RogAStarPlanner() = default;
  ~RogAStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    navflex_rog_map::RogMap::ConstPtr map) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    navflex_rogmap_core::Trajectory3D & trajectory,
    std::string & message) override;
  bool cancel() override;

private:
  enum class NodeState : uint8_t {UNDEFINED, OPEN, CLOSED};
  enum class Heuristic : int {DIAGONAL = 0, MANHATTAN = 1, EUCLIDEAN = 2};

  struct GridIndex
  {
    int x{0};
    int y{0};
    int z{0};
  };

  struct GridNode
  {
    GridIndex index;
    double total_score{0.0};
    double distance_score{0.0};
    GridNode * parent{nullptr};
    uint32_t round{0};
    NodeState state{NodeState::UNDEFINED};
  };

  struct NodeComparator
  {
    bool operator()(const GridNode * first, const GridNode * second) const;
  };

  bool isStateValid(const geometry_msgs::msg::Point & point) const;
  bool insideSearchMap(const GridIndex & index) const;
  GridIndex positionToIndex(const geometry_msgs::msg::Point & point) const;
  geometry_msgs::msg::Point indexToPosition(const GridIndex & index) const;
  size_t localAddress(const GridIndex & index) const;
  double heuristic(const GridIndex & first, const GridIndex & second) const;
  void buildTrajectory(
    GridNode * goal_node,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    navflex_rogmap_core::Trajectory3D & trajectory) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  navflex_rog_map::RogMap::ConstPtr map_;
  navflex_rog_map::Footprint3D footprint_;
  std::string name_;
  std::vector<GridNode> nodes_;
  GridIndex search_center_;
  int size_x_{100};
  int size_y_{100};
  int size_z_{50};
  double resolution_{0.2};
  double inverse_resolution_{5.0};
  double max_planning_time_{1.0};
  bool allow_diagonal_{true};
  bool use_inflated_map_{true};
  bool unknown_as_obstacle_{false};
  Heuristic heuristic_type_{Heuristic::EUCLIDEAN};
  uint32_t round_{0};
  std::atomic_bool cancel_requested_{false};
  bool active_{false};
};

}  // namespace navflex_rogmap_planner

#endif  // NAVFLEX_ROGMAP_PLANNER__ROG_ASTAR_PLANNER_HPP_
