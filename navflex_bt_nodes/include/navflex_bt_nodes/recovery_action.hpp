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
#include "nav2_msgs/action/dummy_behavior.hpp"
#include "nav2_behavior_tree/bt_cancel_action_node.hpp"

namespace navflex_bt_nodes
{

/**
 * @brief BT action node that calls the behavior_action action server
 *        (nav2_msgs::action::DummyBehavior).
 *
 * Mirrors the BehaviorAction / BehaviorCostmapServer in navflex_costmap_nav.
 * The server name "behavior_action" matches the string registered in
 * BehaviorCostmapServer (name_action_behavior_).
 *
 * The DummyBehavior goal carries:
 *   - behavior (string): name of the recovery behavior to run
 *   - command  (std_msgs/String): free-form command forwarded to the plugin
 *
 * BT node ID: NavflexRecoveryAction
 *
 * Outcome codes (nav2_msgs::action::DummyBehavior::Result):
 *   SUCCESS          = 0
 *   FAILURE          = 150
 *   CANCELED         = 151
 *   PAT_EXCEEDED     = 152
 *   TF_ERROR         = 153
 *   NOT_INITIALIZED  = 154
 *   INVALID_PLUGIN   = 155
 *   INTERNAL_ERROR   = 156
 *   STOPPED          = 157
 *   IMPASSABLE       = 158
 *
 * BT XML example:
 * @code{.xml}
 *   <NavflexRecoveryAction
 *       behavior="spin"
 *       command=""
 *       outcome="{recovery_outcome}"
 *       message="{recovery_message}"/>
 * @endcode
 */
class RecoveryAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::DummyBehavior>
{
public:
  using Action = nav2_msgs::action::DummyBehavior;

  RecoveryAction(
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

    // ── Inputs (aligned with DummyBehavior.action goal) ───────────────
    ports.insert(
      BT::InputPort<std::string>(
        "behavior", "spin", "Name of the recovery behavior plugin to execute"));
    ports.insert(
      BT::InputPort<std::string>(
        "command", "", "Free-form command string forwarded to the behavior plugin"));

    // ── Outputs ───────────────────────────────────────────────────────
    ports.insert(
      BT::OutputPort<uint32_t>(
        "outcome",
        "Result outcome code (0=SUCCESS, 150=FAILURE, 151=CANCELED, ...)"));
    ports.insert(
      BT::OutputPort<std::string>(
        "message", "Human-readable result message"));
    ports.insert(
      BT::OutputPort<std::string>(
        "used_plugin", "Name of the behavior plugin that was actually used"));

    return ports;
  }

private:
  void clearOutputs();
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief BT cancel node for the behavior_action action server.
 *
 * BT node ID: NavflexCancelRecovery
 *
 * BT XML example:
 * @code{.xml}
 *   <NavflexCancelRecovery server_name="behavior_action"/>
 * @endcode
 */
class CancelRecoveryAction
  : public nav2_behavior_tree::BtCancelActionNode<nav2_msgs::action::DummyBehavior>
{
public:
  CancelRecoveryAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }
};

}  // namespace navflex_bt_nodes
