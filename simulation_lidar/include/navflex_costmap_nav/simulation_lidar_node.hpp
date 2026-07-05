#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <string>


namespace autolabor_simulation
{

class SimulationLidar : public rclcpp::Node
{
public:
  explicit SimulationLidar(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  // ===== 主流程 =====
  void timerCallback();

  // ===== 激光 =====
  void initLaserScan(sensor_msgs::msg::LaserScan & scan);
  void generateFrame(sensor_msgs::msg::LaserScan & scan);
  void publishPointCloud2(const sensor_msgs::msg::LaserScan & scan);

  // ===== 位姿 =====
  bool getPose(double & x, double & y, double & yaw);

  // ===== Ray casting =====
  void raycast(float sx, float sy, double theta, float & ex, float & ey);
  bool isOccupied(float wx, float wy);

  // ===== 回调 =====
  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  // ===== 工具 =====
  double gaussian(double mu, double sigma);

private:

  // pub/sub
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> static_broadcaster_;
  

  // tf2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // data
  nav_msgs::msg::OccupancyGrid local_map_;
  nav_msgs::msg::Odometry odom_msg_;

  std::mutex map_mutex_;
  // params
  double min_angle_, max_angle_, step_;
  double min_dis_, max_dis_;
  double noise_;
  double rate_;
  double cloud_wall_height_;
  double cloud_wall_step_;
  int point_size_;
  bool use_topic_odom_;
  bool use_sim_time_;

  std::string odom_topic_;
  std::string stage_map_topic_;
  std::string global_frame_;
  std::string lidar_frame_;
};

}  // namespace autolabor_simulation
