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

#include "navflex_bt_nodes/recovery_action.hpp"

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/string.hpp"

namespace navflex_bt_nodes
{

using Action = nav2_msgs::action::DummyBehavior;

// ── RecoveryAction ────────────────────────────────────────────────────────────

RecoveryAction::RecoveryAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void RecoveryAction::on_tick()
{
  getInput("behavior", goal_.behavior);

  std::string cmd_str;
  getInput("command", cmd_str);
  goal_.command.data = cmd_str;
}

BT::NodeStatus RecoveryAction::on_success()
{
  setOutput("outcome", result_.result->outcome);
  setOutput("message", result_.result->message);
  setOutput("used_plugin", result_.result->used_plugin);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RecoveryAction::on_aborted()
{
  clearOutputs();
  setOutput("outcome", static_cast<uint32_t>(Action::Result::FAILURE));
  setOutput("message", std::string("Recovery aborted"));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RecoveryAction::on_cancelled()
{
  clearOutputs();
  setOutput("outcome", static_cast<uint32_t>(Action::Result::CANCELED));
  setOutput("message", std::string("Recovery cancelled"));
  return BT::NodeStatus::SUCCESS;
}

void RecoveryAction::halt()
{
  clearOutputs();
  BtActionNode::halt();
}

void RecoveryAction::clearOutputs()
{
  setOutput("outcome", static_cast<uint32_t>(Action::Result::FAILURE));
  setOutput("message", std::string(""));
  setOutput("used_plugin", std::string(""));
}

// ── CancelRecoveryAction ──────────────────────────────────────────────────────

CancelRecoveryAction::CancelRecoveryAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtCancelActionNode<Action>(xml_tag_name, action_name, conf)
{
}

}  // namespace navflex_bt_nodes

// ── Plugin registration ───────────────────────────────────────────────────────
BT_REGISTER_NODES(factory)
{
  // NavflexRecoveryAction
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<navflex_bt_nodes::RecoveryAction>(
          name, "behavior_action", config);
      };
    factory.registerBuilder<navflex_bt_nodes::RecoveryAction>(
      "NavflexRecoveryAction", builder);
  }

  // NavflexCancelRecovery
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<navflex_bt_nodes::CancelRecoveryAction>(
          name, "behavior_action", config);
      };
    factory.registerBuilder<navflex_bt_nodes::CancelRecoveryAction>(
      "NavflexCancelRecovery", builder);
  }
}
