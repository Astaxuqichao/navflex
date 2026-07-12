// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef NAVFLEX_ROG_MAP__ROG_MAP_ROS_HPP_
#define NAVFLEX_ROG_MAP__ROG_MAP_ROS_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "navflex_rog_map/rog_map.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navflex_rog_map
{

class RogMapROS : public nav2_util::LifecycleNode
{
public:
  explicit RogMapROS(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  explicit RogMapROS(const std::string & name);
  RogMapROS(
    const std::string & name, const std::string & parent_namespace,
    const std::string & local_namespace);
  ~RogMapROS() override;

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void start();
  void stop();
  void pause();
  void resume();
  void resetMap();

  std::shared_ptr<RogMap> getMap() const {return map_;}
  RogMap::Ptr getRogMap() const {return map_;}
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() const {return tf_buffer_;}
  std::string getGlobalFrameID() const {return global_frame_;}
  std::string getBaseFrameID() const {return robot_base_frame_;}
  std::string getName() const {return map_name_;}
  bool isCurrent() const {return current_.load();}
  bool isStopped() const {return stopped_.load();}

private:
  void declareParameters();
  RogMapConfig loadConfig();
  void createRosInterfaces();
  void destroyRosInterfaces();
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void publishMaps();

  std::string map_name_;
  std::string parent_namespace_;
  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
  std::string cloud_topic_{"/scan_cloud"};
  double transform_tolerance_{0.3};
  double publish_frequency_{1.0};

  std::shared_ptr<RogMap> map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::atomic_bool stopped_{true};
  std::atomic_bool paused_{false};
  std::atomic_bool current_{false};
};

}  // namespace navflex_rog_map

#endif  // NAVFLEX_ROG_MAP__ROG_MAP_ROS_HPP_
