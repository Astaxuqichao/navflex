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
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace navflex_bt_nodes
{

/**
 * @brief BT action node that calls the compute_path_to_pose action server.
 *
 * Input ports align exactly with the ComputePathToPose.action goal definition.
 * Output ports extend the nav2 default with `outcome` (uint8) and `message`
 * (string) for fine-grained error handling in downstream BT nodes.
 *
 * BT node ID: NavflexGetPathAction
 *
 * Outcome codes (nav2_msgs::action::ComputePathToPose::Result):
 *   SUCCESS       = 0
 *   FAILURE       = 50
 *   CANCELED      = 51
 *   INVALID_START = 52
 *   INVALID_GOAL  = 53
 *   BLOCKED_START = 54
 *   BLOCKED_GOAL  = 55
 *   NO_PATH_FOUND = 56
 *   PAT_EXCEEDED  = 57
 *   EMPTY_PATH    = 58
 *   TF_ERROR      = 59
 *   NOT_INITIALIZED = 60
 *
 * BT XML example:
 * @code{.xml}
 *   <NavflexGetPathAction
 *       goal="{goal}"
 *       planner_id="GridBased"
 *       path="{path}"
 *       outcome="{planner_outcome}"
 *       message="{planner_message}"/>
 * @endcode
 */
class GetPathAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathToPose>
{
public:
  using Action = nav2_msgs::action::ComputePathToPose;

  GetPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = providedBasicPorts({});

    // ── Inputs (aligned with ComputePathToPose.action goal) ──────────────
    ports.insert(
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Destination pose to plan to"));
    ports.insert(
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "start", "Override start pose (only used when use_start=true)"));
    ports.insert(
      BT::InputPort<std::string>(
        "planner_id", "GridBased", "Planner plugin ID"));
    ports.insert(
      BT::InputPort<bool>(
        "use_start", "If true, use the start port instead of current robot pose"));
    ports.insert(
      BT::InputPort<double>(
        "tolerance", "Goal tolerance in metres"));

    // ── Outputs ───────────────────────────────────────────────────────────
    ports.insert(
      BT::OutputPort<nav_msgs::msg::Path>(
        "path", "Computed global path"));
    ports.insert(
      BT::OutputPort<uint8_t>(
        "outcome", "Result outcome code (0=SUCCESS, 50=FAILURE, 51=CANCELED, ...)"));
    ports.insert(
      BT::OutputPort<std::string>(
        "message", "Human-readable result message"));

    return ports;
  }

private:
  void clearOutputs();
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief BT cancel node for the compute_path_to_pose action server.
 *
 * BT node ID: NavflexCancelGetPath
 *
 * BT XML example:
 * @code{.xml}
 *   <NavflexCancelGetPath server_name="compute_path_to_pose"/>
 * @endcode
 */
class CancelGetPathAction
  : public nav2_behavior_tree::BtCancelActionNode<nav2_msgs::action::ComputePathToPose>
{
public:
  CancelGetPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }
};

}  // namespace navflex_bt_nodes
