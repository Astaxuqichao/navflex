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

#include "navflex_bt_nodes/get_path_action.hpp"

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace navflex_bt_nodes
{

using Action = nav2_msgs::action::ComputePathToPose;

GetPathAction::GetPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void GetPathAction::on_tick()
{
  getInput("goal", goal_.goal);
  getInput("planner_id", goal_.planner_id);
  getInput("tolerance", goal_.tolerance);

  bool use_start = false;
  getInput("use_start", use_start);
  goal_.use_start = use_start;
  if (use_start) {
    getInput("start", goal_.start);
  }
}

BT::NodeStatus GetPathAction::on_success()
{
  setOutput("path", result_.result->path);
  setOutput("outcome", static_cast<uint8_t>(Action::Result::SUCCESS));
  setOutput("message", std::string("Path computed successfully"));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetPathAction::on_aborted()
{
  clearOutputs();
  setOutput("outcome", static_cast<uint8_t>(Action::Result::FAILURE));
  setOutput("message", std::string("Planning aborted: no path found"));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GetPathAction::on_cancelled()
{
  clearOutputs();
  setOutput("outcome", static_cast<uint8_t>(Action::Result::CANCELED));
  setOutput("message", std::string("Planning cancelled"));
  return BT::NodeStatus::SUCCESS;
}

void GetPathAction::halt()
{
  clearOutputs();
  BtActionNode::halt();
}

void GetPathAction::clearOutputs()
{
  setOutput("path", nav_msgs::msg::Path{});
  setOutput("outcome", static_cast<uint8_t>(Action::Result::FAILURE));
  setOutput("message", std::string(""));
}

// ── CancelGetPathAction ───────────────────────────────────────────────────────

CancelGetPathAction::CancelGetPathAction(
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
  // NavflexGetPathAction
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<navflex_bt_nodes::GetPathAction>(
          name, "compute_path_to_pose", config);
      };
    factory.registerBuilder<navflex_bt_nodes::GetPathAction>(
      "NavflexGetPathAction", builder);
  }

  // NavflexCancelGetPath
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<navflex_bt_nodes::CancelGetPathAction>(
          name, "compute_path_to_pose", config);
      };
    factory.registerBuilder<navflex_bt_nodes::CancelGetPathAction>(
      "NavflexCancelGetPath", builder);
  }
}
