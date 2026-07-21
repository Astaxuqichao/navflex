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

#pragma once

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "navflex_world_model/srv/evaluate_plan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace navflex_wm_bt_nodes
{

/**
 * @brief BT service node that gates a candidate viewpoint through the navflex
 *        forward-dynamics world model + critic.
 *
 * BT node ID: NavflexEvaluatePlanAction
 * Wraps:      navflex_world_model::srv::EvaluatePlan  (service: navflex_world_model/evaluate)
 *
 * The world-model node takes ONE goal, plans to it with compute_path_to_pose
 * (but never executes), converts that plan into the camera trajectory the robot
 * would fly, rolls a forward-dynamics world model along it, and asks a VLM
 * critic whether the imagined future is safe / traversable. It returns a
 * verdict: approve | reject | needs_confirmation | unavailable.
 *
 * This node maps that verdict onto BT control flow:
 *   verdict IN pass_verdicts  -> SUCCESS  (execute the plan)
 *   otherwise                 -> FAILURE  (veto; caller re-selects next cycle)
 *
 * It is NOT a viewpoint ranker. Ranking over N candidates stays in the
 * FrontierAStar planner (cheap, geometric); this node is the expensive
 * per-candidate go/no-go on the planner's top pick. A single rollout costs
 * seconds to minutes and pins tens of GB of GPU, so gating every candidate is
 * infeasible — the propose(cheap)/verify(expensive) cascade is the only
 * computationally viable shape. See docs/world_model_viewpoint_gate.md.
 *
 * ── IMPORTANT: server_timeout ────────────────────────────────────────────────
 * BtServiceNode treats `server_timeout` as a HARD cap on the whole service call
 * and returns FAILURE (a spurious veto) once it elapses. A real rollout takes
 * seconds to minutes, so the XML MUST set a large server_timeout (e.g. 900000
 * ms). While waiting the node returns RUNNING, so the tree keeps ticking and
 * the robot simply holds position until the verdict arrives.
 *
 * ── IMPORTANT: the world model + critic must be configured ────────────────────
 * With the default backend='null' / critic='null', the null critic NEVER
 * approves, so every gate call vetoes and the robot never executes any frontier
 * (exploration freezes). Launch navflex_world_model with a real backend
 * (lingbot) and critic (claude / openai_compat). See docs/world_model_viewpoint_gate.md.
 *
 * BT XML example:
 * @code{.xml}
 *   <NavflexEvaluatePlanAction
 *       goal="{candidate_goal}"
 *       pass_verdicts="approve"
 *       verdict="{wm_verdict}"
 *       confidence="{wm_confidence}"
 *       reason="{wm_reason}"
 *       service_name="navflex_world_model/evaluate"
 *       server_timeout="900000"/>
 * @endcode
 */
class EvaluatePlanAction
  : public nav2_behavior_tree::BtServiceNode<navflex_world_model::srv::EvaluatePlan>
{
public:
  EvaluatePlanAction(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    // The critic (not the world model) reads `instruction`. For autonomous
    // exploration there is no task, so frame it as a safety/traversability
    // question. Deliberately NOT a task description — the world model's own
    // prompt stays task-free on purpose (see navflex_world_model_node.py).
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "goal",
          "Candidate viewpoint to gate: position + observation yaw "
          "(the endpoint of the FrontierAStar path)"),
        BT::InputPort<std::string>(
          "instruction",
          "机器人正在自主探索一栋室内住宅。请判断：沿这条路径前往该观测视点是否"
          "安全、可通行——不会撞墙或家具、不会跌落、通路顺畅。只有明确安全时才批准。",
          "Instruction handed to the critic (a safety/traversability question)"),
        BT::InputPort<int>(
          "frame_num", 0,
          "World-model rollout frame count (0 = auto-size from the plan)"),
        BT::InputPort<bool>(
          "dry_run", false,
          "Produce the rollout but skip the critic (returns needs_confirmation)"),
        BT::InputPort<std::string>(
          "pass_verdicts", "approve",
          "Comma-separated verdicts that count as SUCCESS. "
          "'approve' = strict (only clearly-safe plans run). "
          "'approve,needs_confirmation' = lenient (only a hard 'reject' vetoes)."),
        BT::OutputPort<std::string>(
          "verdict", "approve | reject | needs_confirmation | unavailable"),
        BT::OutputPort<double>(
          "confidence", "Critic confidence in [0, 1]"),
        BT::OutputPort<std::string>(
          "reason", "Human-readable reason for the verdict")
      });
  }

  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<navflex_world_model::srv::EvaluatePlan::Response> response) override;
};

}  // namespace navflex_wm_bt_nodes
