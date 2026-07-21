// Copyright (c) 2026 navflex_wm_bt_nodes
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

#include "navflex_wm_bt_nodes/evaluate_plan_action.hpp"
#include "navflex_wm_bt_nodes/path_end_pose.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace navflex_wm_bt_nodes
{

namespace
{

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::string to_lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

// Is `verdict` listed in the comma-separated `pass_verdicts` set? Case- and
// whitespace-insensitive, so "Approve, needs_confirmation" parses as expected.
bool verdict_passes(const std::string & verdict, const std::string & pass_verdicts)
{
  const std::string needle = to_lower(trim(verdict));
  if (needle.empty()) {
    return false;
  }
  std::string token;
  for (std::size_t i = 0; i <= pass_verdicts.size(); ++i) {
    if (i == pass_verdicts.size() || pass_verdicts[i] == ',') {
      if (to_lower(trim(token)) == needle) {
        return true;
      }
      token.clear();
    } else {
      token.push_back(pass_verdicts[i]);
    }
  }
  return false;
}

}  // namespace

EvaluatePlanAction::EvaluatePlanAction(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<navflex_world_model::srv::EvaluatePlan>(
    service_node_name, conf, "navflex_world_model/evaluate")
{
}

void EvaluatePlanAction::on_tick()
{
  getInput("goal", request_->goal);
  getInput("instruction", request_->instruction);
  getInput("dry_run", request_->dry_run);

  // frame_num is int32 in the srv; read into a plain int and narrow explicitly
  // so the port type never has to match the message field type exactly.
  int frame_num = 0;
  getInput("frame_num", frame_num);
  request_->frame_num = static_cast<int32_t>(frame_num);

  // task_json is for the instruction-driven task server, not exploration.
  request_->task_json = "";
}

BT::NodeStatus EvaluatePlanAction::on_completion(
  std::shared_ptr<navflex_world_model::srv::EvaluatePlan::Response> response)
{
  setOutput("verdict", response->verdict);
  setOutput("confidence", response->confidence);
  setOutput("reason", response->reason);

  std::string pass_verdicts = "approve";
  getInput("pass_verdicts", pass_verdicts);
  const bool approved = verdict_passes(response->verdict, pass_verdicts);

  // `success` reports whether the gate itself ran, not whether the plan is
  // safe. A gate that failed to run (unavailable) fails closed to whatever
  // `unavailable_verdict` the world-model node was configured with (default
  // needs_confirmation) — which, under a strict pass set, correctly vetoes.
  RCLCPP_INFO(
    node_->get_logger(),
    "[EvaluatePlan] verdict=%s confidence=%.2f (gate_ran=%s) -> %s | %s",
    response->verdict.c_str(), response->confidence,
    response->success ? "yes" : "no",
    approved ? "SUCCESS(execute)" : "FAILURE(veto)",
    response->reason.c_str());

  return approved ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace navflex_wm_bt_nodes

// ── Plugin registration ───────────────────────────────────────────────────────
// Loaded at runtime via BehaviorTreeEngine / bt_navigator's plugin_lib_names
// (NOT pluginlib). BT_REGISTER_NODES self-registers when the .so is dlopen'd.
// This whole package is ONE shared library, so it must have exactly ONE
// BT_REGISTER_NODES — both node types are registered here.
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navflex_wm_bt_nodes::EvaluatePlanAction>(
    "NavflexEvaluatePlanAction");
  factory.registerNodeType<navflex_wm_bt_nodes::PathEndPose>(
    "NavflexPathEndPose");
}
