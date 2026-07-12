// Copyright (c) 2024 navflex_bt_nodes
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_behavior_tree/bt_cancel_action_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace navflex_bt_nodes
{

/**
 * @brief BT action node that calls the follow_path action server
 *        (nav2_msgs::action::FollowPath).
 *
 * Mirrors the ControllerAction / ControllerCostmapServer in navflex_costmap_nav.
 * The server name "follow_path" matches the string registered in
 * ControllerCostmapServer (name_action_follow_path_).
 *
 * BT node ID: NavflexExePathAction
 *
 * Outcome codes (nav2_msgs::action::FollowPath::Result):
 *   SUCCESS       = 0
 *   FAILURE       = 100
 *   CANCELED      = 101
 *   NO_VALID_CMD  = 102
 *   PAT_EXCEEDED  = 103
 *   COLLISION     = 104
 *   OSCILLATION   = 105
 *   ROBOT_STUCK   = 106
 *   MISSED_GOAL   = 107
 *   MISSED_PATH   = 108
 *   BLOCKED_GOAL  = 109
 *   BLOCKED_PATH  = 110
 *   INVALID_PATH  = 111
 *   TF_ERROR      = 112
 *   NOT_INITIALIZED = 113
 *   INVALID_PLUGIN = 114
 *   INTERNAL_ERROR = 115
 *   OUT_OF_MAP    = 116
 *   MAP_ERROR     = 117
 *   STOPPED       = 118
 *
 * BT XML example:
 * @code{.xml}
 *   <NavflexExePathAction
 *       path="{path}"
 *       controller_id="FollowPath"
 *       xy_goal_tolerance="0.0"
 *       yaw_goal_tolerance="0.0"
 *       outcome="{controller_outcome}"
 *       message="{controller_message}"/>
 * @endcode
 */
class ExePathAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  using Action = nav2_msgs::action::FollowPath;

  ExePathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_wait_for_result(
    std::shared_ptr<const Action::Feedback> feedback) override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = providedBasicPorts({});

    // ── Inputs (aligned with FollowPath.action goal) ──────────────────
    ports.insert(
      BT::InputPort<nav_msgs::msg::Path>(
        "path", "Global path to follow"));
    ports.insert(
      BT::InputPort<std::string>(
        "controller_id", "FollowPath", "Controller plugin ID"));
    ports.insert(
      BT::InputPort<float>(
        "xy_goal_tolerance", 0.0f,
        "XY goal tolerance in meters. 0.0 uses controller defaults"));
    ports.insert(
      BT::InputPort<float>(
        "yaw_goal_tolerance", 0.0f,
        "Yaw goal tolerance in radians. 0.0 uses controller defaults"));

    // ── Outputs ───────────────────────────────────────────────────────
    ports.insert(
      BT::OutputPort<uint32_t>(
        "outcome",
        "Result outcome code (0=SUCCESS, 100=FAILURE, 101=CANCELED, ...)"));
    ports.insert(
      BT::OutputPort<std::string>(
        "message", "Human-readable result message"));
    ports.insert(
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "final_pose", "Final robot pose reported by the controller"));
    ports.insert(
      BT::OutputPort<float>(
        "dist_to_goal", "Distance to goal at completion"));
    ports.insert(
      BT::OutputPort<float>(
        "angle_to_goal", "Angle to goal at completion"));

    // ── Feedback (updated every BT tick while running) ─────────────────
    ports.insert(
      BT::OutputPort<float>(
        "feedback_distance_to_goal", "Live distance-to-goal from controller feedback"));
    ports.insert(
      BT::OutputPort<float>(
        "feedback_speed", "Live robot speed from controller feedback"));
    ports.insert(
      BT::OutputPort<uint32_t>(
        "feedback_outcome", "Live outcome code from latest controller cycle"));
    ports.insert(
      BT::OutputPort<std::string>(
        "feedback_message", "Live message from latest controller cycle"));

    return ports;
  }

private:
  void clearOutputs();
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief BT cancel node for the follow_path action server.
 *
 * BT node ID: NavflexCancelExePath
 *
 * BT XML example:
 * @code{.xml}
 *   <NavflexCancelExePath server_name="follow_path"/>
 * @endcode
 */
class CancelExePathAction
  : public nav2_behavior_tree::BtCancelActionNode<nav2_msgs::action::FollowPath>
{
public:
  CancelExePathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }
};

}  // namespace navflex_bt_nodes
