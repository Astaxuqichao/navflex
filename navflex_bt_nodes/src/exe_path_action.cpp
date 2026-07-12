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

#include "navflex_bt_nodes/exe_path_action.hpp"

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace navflex_bt_nodes
{

using Action = nav2_msgs::action::FollowPath;

ExePathAction::ExePathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void ExePathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  getInput("xy_goal_tolerance", goal_.xy_goal_tolerance);
  getInput("yaw_goal_tolerance", goal_.yaw_goal_tolerance);
}

void ExePathAction::on_wait_for_result(
  std::shared_ptr<const Action::Feedback> feedback)
{
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  if (goal_.path != new_path && new_path != nav_msgs::msg::Path()) {
    goal_.path = new_path;
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  float new_xy_goal_tolerance = 0.0f;
  getInput("xy_goal_tolerance", new_xy_goal_tolerance);

  if (goal_.xy_goal_tolerance != new_xy_goal_tolerance) {
    goal_.xy_goal_tolerance = new_xy_goal_tolerance;
    goal_updated_ = true;
  }

  float new_yaw_goal_tolerance = 0.0f;
  getInput("yaw_goal_tolerance", new_yaw_goal_tolerance);

  if (goal_.yaw_goal_tolerance != new_yaw_goal_tolerance) {
    goal_.yaw_goal_tolerance = new_yaw_goal_tolerance;
    goal_updated_ = true;
  }

  if (feedback) {
    setOutput("feedback_distance_to_goal", feedback->distance_to_goal);
    setOutput("feedback_speed", feedback->speed);
    setOutput("feedback_outcome", feedback->outcome);
    setOutput("feedback_message", feedback->message);
  }
}

BT::NodeStatus ExePathAction::on_success()
{
  setOutput("outcome", result_.result->outcome);
  setOutput("message", result_.result->message);
  setOutput("final_pose", result_.result->final_pose);
  setOutput("dist_to_goal", result_.result->dist_to_goal);
  setOutput("angle_to_goal", result_.result->angle_to_goal);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExePathAction::on_aborted()
{
  clearOutputs();
  setOutput("outcome", static_cast<uint32_t>(Action::Result::FAILURE));
  setOutput("message", std::string("ExePath aborted: controller failed"));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ExePathAction::on_cancelled()
{
  clearOutputs();
  setOutput("outcome", static_cast<uint32_t>(Action::Result::CANCELED));
  setOutput("message", std::string("ExePath cancelled"));
  return BT::NodeStatus::SUCCESS;
}

void ExePathAction::halt()
{
  clearOutputs();
  BtActionNode::halt();
}

void ExePathAction::clearOutputs()
{
  setOutput("outcome", static_cast<uint32_t>(Action::Result::FAILURE));
  setOutput("message", std::string(""));
  setOutput("final_pose", geometry_msgs::msg::PoseStamped{});
  setOutput("dist_to_goal", 0.0f);
  setOutput("angle_to_goal", 0.0f);
}

// ── CancelExePathAction ───────────────────────────────────────────────────────

CancelExePathAction::CancelExePathAction(
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
  // NavflexExePathAction
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<navflex_bt_nodes::ExePathAction>(
          name, "follow_path", config);
      };
    factory.registerBuilder<navflex_bt_nodes::ExePathAction>(
      "NavflexExePathAction", builder);
  }

  // NavflexCancelExePath
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<navflex_bt_nodes::CancelExePathAction>(
          name, "follow_path", config);
      };
    factory.registerBuilder<navflex_bt_nodes::CancelExePathAction>(
      "NavflexCancelExePath", builder);
  }
}
