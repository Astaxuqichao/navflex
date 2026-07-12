#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/blackboard.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

namespace navflex_autonomous_exploration_bt
{

class ExplorationBtNavigator : public rclcpp::Node
{
public:
  ExplorationBtNavigator()
  : rclcpp::Node("exploration_bt_navigator"), cancel_requested_(false), active_(false)
  {
    const auto share_dir = ament_index_cpp::get_package_share_directory(
      "navflex_autonomous_exploration_bt");

    declare_parameter("bt_xml", share_dir + "/behavior_trees/fael_frontier_exploration.xml");
    declare_parameter("start_topic", "exploration/start");
    declare_parameter("stop_topic", "exploration/stop");
    declare_parameter("goal_frame", "map");
    declare_parameter("goal_x", 0.0);
    declare_parameter("goal_y", 0.0);
    declare_parameter("goal_yaw", 0.0);
    declare_parameter("bt_loop_duration", 10);
    declare_parameter("default_server_timeout", 1000);
    declare_parameter("wait_for_service_timeout", 1000);
    declare_parameter(
      "plugin_lib_names", std::vector<std::string>{
        "nav2_clear_costmap_service_bt_node",
        "nav2_goal_updated_condition_bt_node",
        "nav2_rate_controller_bt_node",
        "nav2_recovery_node_bt_node",
        "nav2_round_robin_node_bt_node",
        "nav2_pipeline_sequence_bt_node",
        "navflex_get_path_action",
        "navflex_exe_path_action",
        "navflex_recovery_action"});

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      get_parameter("start_topic").as_string(), 10,
      std::bind(&ExplorationBtNavigator::onStart, this, std::placeholders::_1));
    stop_sub_ = create_subscription<std_msgs::msg::Empty>(
      get_parameter("stop_topic").as_string(), 10,
      std::bind(&ExplorationBtNavigator::onStop, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Exploration BT navigator ready: start=%s stop=%s",
      get_parameter("start_topic").as_string().c_str(),
      get_parameter("stop_topic").as_string().c_str());
  }

  ~ExplorationBtNavigator() override
  {
    requestStop();
    joinWorker();
  }

private:
  void onStart(const std_msgs::msg::Empty::SharedPtr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
      RCLCPP_WARN(get_logger(), "Exploration BT is already running; ignoring start trigger");
      return;
    }

    if (worker_.joinable()) {
      worker_.join();
    }

    cancel_requested_.store(false);
    active_ = true;
    worker_ = std::thread(&ExplorationBtNavigator::runTree, this);
  }

  void onStop(const std_msgs::msg::Empty::SharedPtr)
  {
    if (!active_.load()) {
      RCLCPP_WARN(get_logger(), "Exploration BT is not running; ignoring stop trigger");
      return;
    }

    RCLCPP_INFO(get_logger(), "Stop requested for exploration BT");
    requestStop();
  }

  void requestStop()
  {
    cancel_requested_.store(true);
  }

  void joinWorker()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void runTree()
  {
    RCLCPP_INFO(get_logger(), "Starting exploration BT");

    try {
      const auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();
      nav2_behavior_tree::BehaviorTreeEngine bt(plugin_lib_names);
      auto blackboard = BT::Blackboard::create();

      blackboard->set<rclcpp::Node::SharedPtr>("node", shared_from_this());
      blackboard->set<std::chrono::milliseconds>(
        "bt_loop_duration",
        std::chrono::milliseconds(get_parameter("bt_loop_duration").as_int()));
      blackboard->set<std::chrono::milliseconds>(
        "server_timeout",
        std::chrono::milliseconds(get_parameter("default_server_timeout").as_int()));
      blackboard->set<std::chrono::milliseconds>(
        "wait_for_service_timeout",
        std::chrono::milliseconds(get_parameter("wait_for_service_timeout").as_int()));
      blackboard->set<int>("number_recoveries", 0);
      blackboard->set<geometry_msgs::msg::PoseStamped>("goal", makeDummyGoal());

      auto tree = bt.createTreeFromFile(get_parameter("bt_xml").as_string(), blackboard);
      const auto loop_duration = std::chrono::milliseconds(
        get_parameter(
          "bt_loop_duration").as_int());
      auto status = bt.run(
        &tree,
        []() {},
        [this]() {return cancel_requested_.load();},
        loop_duration);

      if (status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Exploration BT finished successfully");
      } else if (status == nav2_behavior_tree::BtStatus::CANCELED) {
        RCLCPP_INFO(get_logger(), "Exploration BT canceled");
      } else {
        RCLCPP_ERROR(get_logger(), "Exploration BT failed");
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Exploration BT exception: %s", ex.what());
    }

    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    cancel_requested_.store(false);
  }

  geometry_msgs::msg::PoseStamped makeDummyGoal()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = get_parameter("goal_frame").as_string();
    pose.pose.position.x = get_parameter("goal_x").as_double();
    pose.pose.position.y = get_parameter("goal_y").as_double();
    const auto yaw = get_parameter("goal_yaw").as_double();
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    return pose;
  }

  std::mutex mutex_;
  std::atomic_bool cancel_requested_;
  std::atomic_bool active_;
  std::thread worker_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
};

}  // namespace navflex_autonomous_exploration_bt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navflex_autonomous_exploration_bt::ExplorationBtNavigator>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
