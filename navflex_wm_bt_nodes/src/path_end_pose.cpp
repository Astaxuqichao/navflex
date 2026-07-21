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

#include "navflex_wm_bt_nodes/path_end_pose.hpp"

#include <string>

namespace navflex_wm_bt_nodes
{

PathEndPose::PathEndPose(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus PathEndPose::tick()
{
  nav_msgs::msg::Path path;
  if (!getInput("path", path)) {
    return BT::NodeStatus::FAILURE;   // no path on the blackboard
  }
  if (path.poses.empty()) {
    return BT::NodeStatus::FAILURE;   // empty path -> no viewpoint to gate
  }

  geometry_msgs::msg::PoseStamped goal = path.poses.back();
  // Per-pose headers are frequently blank; fall back to the path frame so the
  // gate's goal always carries a valid frame_id (the world model needs it).
  if (goal.header.frame_id.empty()) {
    goal.header = path.header;
  }
  setOutput("goal", goal);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace navflex_wm_bt_nodes
