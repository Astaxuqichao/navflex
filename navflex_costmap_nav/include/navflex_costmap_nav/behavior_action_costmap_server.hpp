#pragma once
// New BehaviorCostmapServer using NavflexActionBase framework.
// Original (SimpleActionServer-based) version preserved in behavior_server_bak.hpp

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_core/behavior.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/dummy_behavior.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/server.hpp"
#include "tf2_ros/buffer.h"
#include "navflex_base/behavior_action.h"
#include "navflex_base/behavior_execution.h"

namespace navflex_costmap_nav
{

/**
 * @class BehaviorCostmapServer
 * @brief Lifecycle node hosting behavior (recovery) plugins, exposing a
 *        DummyBehavior action server using the NavflexActionBase framework.
 */
class BehaviorCostmapServer : public nav2_util::LifecycleNode
{
public:
  using BehaviorMap =
    std::unordered_map<std::string, nav2_core::Behavior::Ptr>;

  using ActionDummyBehavior = nav2_msgs::action::DummyBehavior;
  using ServerGoalHandleDummyBehavior =
    rclcpp_action::ServerGoalHandle<ActionDummyBehavior>;
  using ServerGoalHandleDummyBehaviorPtr =
    std::shared_ptr<ServerGoalHandleDummyBehavior>;

  /**
   * @brief Constructor
   * @param global_costmap_ros  Global costmap (passed to behavior plugins)
   * @param local_costmap_ros   Local costmap (passed to behavior plugins)
   * @param options             ROS2 node options
   */
  explicit BehaviorCostmapServer(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_ros,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_ros,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~BehaviorCostmapServer();

protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

private:
  bool loadBehaviorPlugins();

  rclcpp_action::GoalResponse handleGoalDummyBehavior(
    const rclcpp_action::GoalUUID & uuid,
    ActionDummyBehavior::Goal::ConstSharedPtr goal);

  rclcpp_action::CancelResponse cancelActionDummyBehavior(
    ServerGoalHandleDummyBehaviorPtr goal_handle);

  void callActionDummyBehavior(ServerGoalHandleDummyBehaviorPtr goal_handle);

  BehaviorExecution::Ptr newBehaviorExecution(const std::string & behavior_name);

  // Plugins
  pluginlib::ClassLoader<nav2_core::Behavior> plugin_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> behavior_ids_;
  std::vector<std::string> behavior_types_;
  std::string behavior_ids_concat_;
  BehaviorMap behaviors_;

  // Costmaps
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_ros_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_ros_;

  // Action server
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr action_executor_;
  std::unique_ptr<nav2_util::NodeThread> action_executor_thread_;
  rclcpp_action::Server<ActionDummyBehavior>::SharedPtr action_server_;

  // Action handler
  std::shared_ptr<BehaviorAction> behavior_action_;
  std::string name_action_behavior_;
};

}  // namespace navflex_costmap_nav
