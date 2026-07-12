#pragma once

#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/check_point.hpp"
#include "nav2_msgs/srv/check_pose.hpp"
#include "nav2_msgs/srv/check_path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace navflex_bt_nodes
{

class CheckPointService : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::CheckPoint>
{
public:
  CheckPointService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PointStamped>("point", "Point to check"),
        BT::InputPort<uint8_t>(
          "costmap", nav2_msgs::srv::CheckPoint::Request::GLOBAL_COSTMAP,
          "Costmap id: 1 local, 2 global"),
        BT::OutputPort<uint8_t>("state", "FREE/INSCRIBED/LETHAL/UNKNOWN/OUTSIDE"),
        BT::OutputPort<uint32_t>("cost", "Cell cost at point")
      });
  }

  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<nav2_msgs::srv::CheckPoint::Response> response) override;
};

class CheckPoseService : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::CheckPose>
{
public:
  CheckPoseService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Pose to check"),
        BT::InputPort<float>("safety_dist", 0.0f, "Footprint safety padding (m)"),
        BT::InputPort<float>("lethal_cost_mult", 0.0f, "Multiplier for lethal cells"),
        BT::InputPort<float>("inscrib_cost_mult", 0.0f, "Multiplier for inscribed cells"),
        BT::InputPort<float>("unknown_cost_mult", 0.0f, "Multiplier for unknown cells"),
        BT::InputPort<uint8_t>(
          "costmap", nav2_msgs::srv::CheckPose::Request::GLOBAL_COSTMAP,
          "Costmap id: 1 local, 2 global"),
        BT::InputPort<bool>(
          "current_pose", false,
          "Check current robot pose instead of input pose"),
        BT::OutputPort<uint8_t>("state", "FREE/INSCRIBED/LETHAL/UNKNOWN/OUTSIDE"),
        BT::OutputPort<uint32_t>("cost", "Accumulated pose cost")
      });
  }

  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<nav2_msgs::srv::CheckPose::Response> response) override;
};

class CheckPathService : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::CheckPath>
{
public:
  CheckPathService(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to check"),
        BT::InputPort<float>("safety_dist", 0.0f, "Footprint safety padding (m)"),
        BT::InputPort<float>("lethal_cost_mult", 0.0f, "Multiplier for lethal cells"),
        BT::InputPort<float>("inscrib_cost_mult", 0.0f, "Multiplier for inscribed cells"),
        BT::InputPort<float>("unknown_cost_mult", 0.0f, "Multiplier for unknown cells"),
        BT::InputPort<uint8_t>(
          "costmap", nav2_msgs::srv::CheckPath::Request::GLOBAL_COSTMAP,
          "Costmap id: 1 local, 2 global"),
        BT::InputPort<uint8_t>("return_on", 0, "Abort when state >= return_on (0 disables)"),
        BT::InputPort<uint8_t>("skip_poses", 0, "Skip N poses between checks"),
        BT::InputPort<bool>("path_cells_only", false, "Ignore footprint and check only path cells"),
        BT::OutputPort<uint32_t>("last_checked", "Index of last checked pose"),
        BT::OutputPort<uint8_t>("state", "Worst state on path"),
        BT::OutputPort<uint32_t>("cost", "Accumulated path cost")
      });
  }

  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<nav2_msgs::srv::CheckPath::Response> response) override;
};

}  // namespace navflex_bt_nodes
