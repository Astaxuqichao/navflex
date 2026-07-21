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

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace navflex_wm_bt_nodes
{

/**
 * @brief Synchronous BT node: extract a path's final pose as a PoseStamped.
 *
 * BT node ID: NavflexPathEndPose
 *
 * Exists so the world-model gate can read the selected viewpoint WITHOUT
 * modifying the stock NavflexGetPathAction. For the FrontierAStar planner the
 * path's last pose is the chosen viewpoint AND its observation yaw
 * (makeAStarPath overrides the endpoint orientation with the candidate yaw), so
 * this hands the gate exactly the arrival the robot will drive to.
 *
 * Pure blackboard I/O — no ROS. FAILURE if there is no path or it is empty
 * (nothing to gate); the caller treats that like a planning miss.
 *
 * BT XML example:
 * @code{.xml}
 *   <NavflexPathEndPose path="{frontier_path}" goal="{candidate_goal}"/>
 * @endcode
 */
class PathEndPose : public BT::SyncActionNode
{
public:
  PathEndPose(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>(
        "path", "Path whose final pose is the selected viewpoint"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Final pose of the path (viewpoint position + observation yaw)")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace navflex_wm_bt_nodes
