// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "navflex_rog_map/rog_map_ros.hpp"

#include <cmath>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace navflex_rog_map
{

RogMapROS::RogMapROS(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("rog_map", "", options), map_name_("rog_map")
{
  declareParameters();
}

RogMapROS::RogMapROS(const std::string & name)
: RogMapROS(name, "/", name) {}

RogMapROS::RogMapROS(
  const std::string & name, const std::string & parent_namespace,
  const std::string & local_namespace)
: nav2_util::LifecycleNode(
    name, "", rclcpp::NodeOptions().arguments({
    "--ros-args", "-r", "__ns:=" + nav2_util::add_namespaces(parent_namespace, local_namespace),
    "--ros-args", "-r", name + ":__node:=" + name
  })),
  map_name_(name), parent_namespace_(parent_namespace)
{
  declareParameters();
}

RogMapROS::~RogMapROS() {destroyRosInterfaces();}

void RogMapROS::declareParameters()
{
  declare_parameter("global_frame", rclcpp::ParameterValue("map"));
  declare_parameter("robot_base_frame", rclcpp::ParameterValue("base_link"));
  declare_parameter("point_cloud_topic", rclcpp::ParameterValue("/scan_cloud"));
  declare_parameter("point_cloud_frame", rclcpp::ParameterValue("base_link"));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  declare_parameter("publish_frequency", rclcpp::ParameterValue(1.0));
  declare_parameter("update_frequency", rclcpp::ParameterValue(5.0));
  declare_parameter("global_resolution", rclcpp::ParameterValue(0.4));
  declare_parameter("global_min", rclcpp::ParameterValue(std::vector<double>{-50.0, -50.0, -5.0}));
  declare_parameter("global_max", rclcpp::ParameterValue(std::vector<double>{50.0, 50.0, 15.0}));
  declare_parameter("local_resolution", rclcpp::ParameterValue(0.1));
  declare_parameter("local_size_x", rclcpp::ParameterValue(12.0));
  declare_parameter("local_size_y", rclcpp::ParameterValue(12.0));
  declare_parameter("local_size_z", rclcpp::ParameterValue(6.0));
  declare_parameter("hit_probability", rclcpp::ParameterValue(0.75));
  declare_parameter("miss_probability", rclcpp::ParameterValue(0.40));
  declare_parameter("occupied_threshold", rclcpp::ParameterValue(0.70));
  declare_parameter("ray_min_range", rclcpp::ParameterValue(0.3));
  declare_parameter("ray_max_range", rclcpp::ParameterValue(15.0));
  declare_parameter("esdf_max_distance", rclcpp::ParameterValue(5.0));
  declare_parameter("inflation_resolution", rclcpp::ParameterValue(0.2));
  declare_parameter("inflation_step", rclcpp::ParameterValue(2));
  declare_parameter("unknown_inflation", rclcpp::ParameterValue(false));
  declare_parameter("unknown_inflation_step", rclcpp::ParameterValue(1));
  declare_parameter("unknown_threshold", rclcpp::ParameterValue(0.7));
  declare_parameter("point_filter_num", rclcpp::ParameterValue(2));
  declare_parameter("global_point_filter_num", rclcpp::ParameterValue(2));
  declare_parameter("batch_update_size", rclcpp::ParameterValue(2));
  declare_parameter("map_sliding_threshold", rclcpp::ParameterValue(1.0));
  declare_parameter("frontier_extraction", rclcpp::ParameterValue(true));
  declare_parameter("enable_esdf", rclcpp::ParameterValue(true));
  declare_parameter("esdf_update_interval", rclcpp::ParameterValue(2));
  declare_parameter("load_pcd", rclcpp::ParameterValue(false));
  declare_parameter("pcd_file", rclcpp::ParameterValue(""));
  declare_parameter("pcd_frame", rclcpp::ParameterValue("map"));
  declare_parameter("virtual_ground_height", rclcpp::ParameterValue(-5.0));
  declare_parameter("virtual_ceiling_height", rclcpp::ParameterValue(15.0));
}

RogMapConfig RogMapROS::loadConfig()
{
  RogMapConfig config;get_parameter("global_frame", global_frame_);
  get_parameter("robot_base_frame", robot_base_frame_);get_parameter(
    "point_cloud_topic",
    cloud_topic_);
  get_parameter("point_cloud_frame", point_cloud_frame_);
  get_parameter("transform_tolerance", transform_tolerance_);get_parameter(
    "publish_frequency",
    publish_frequency_);
  get_parameter("update_frequency", update_frequency_);
  config.frame_id = global_frame_;get_parameter("global_resolution", config.global_resolution);
  get_parameter("local_resolution", config.local_resolution);get_parameter(
    "local_size_x",
    config.local_size_x);
  get_parameter("local_size_y", config.local_size_y);get_parameter(
    "local_size_z",
    config.local_size_z);
  get_parameter("hit_probability", config.hit_probability);
  get_parameter("miss_probability", config.miss_probability);
  get_parameter("occupied_threshold", config.occupied_threshold);
  get_parameter("ray_min_range", config.ray_min_range);get_parameter(
    "ray_max_range",
    config.ray_max_range);
  get_parameter("esdf_max_distance", config.esdf_max_distance);
  get_parameter("inflation_resolution", config.inflation_resolution);
  get_parameter("inflation_step", config.inflation_step);
  get_parameter("unknown_inflation", config.unknown_inflation);
  get_parameter("unknown_inflation_step", config.unknown_inflation_step);
  get_parameter("unknown_threshold", config.unknown_threshold);
  get_parameter("point_filter_num", config.point_filter_num);
  get_parameter("global_point_filter_num", config.global_point_filter_num);
  get_parameter("batch_update_size", config.batch_update_size);
  get_parameter("map_sliding_threshold", config.map_sliding_threshold);
  get_parameter("frontier_extraction", config.frontier_extraction);
  get_parameter("enable_esdf", config.enable_esdf);
  get_parameter("esdf_update_interval", config.esdf_update_interval);
  get_parameter("load_pcd", config.load_pcd);
  get_parameter("pcd_file", config.pcd_file);
  get_parameter("pcd_frame", config.pcd_frame);
  get_parameter("virtual_ground_height", config.virtual_ground_height);
  get_parameter("virtual_ceiling_height", config.virtual_ceiling_height);
  auto minimum = get_parameter("global_min").as_double_array();auto maximum = get_parameter(
    "global_max").as_double_array();
  if (minimum.size() != 3 || maximum.size() != 3) {
    throw std::invalid_argument("global_min and global_max must contain three values");
  }
  config.global_bounds = {minimum[0], minimum[1], minimum[2], maximum[0], maximum[1], maximum[2]};
  return config;
}

nav2_util::CallbackReturn RogMapROS::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", map_name_.c_str());
  try {
    map_ = std::make_shared<RogMap>(loadConfig());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "ROG map configuration failed: %s", error.what());
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (map_->loadedPcdPointCount() > 0) {
    RCLCPP_INFO(
      get_logger(), "Loaded %zu PCD map points in frame %s",
      map_->loadedPcdPointCount(), global_frame_.c_str());
  }
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  global_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "global_occupied", rclcpp::QoS(
      1).transient_local());
  local_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "local_occupied", rclcpp::QoS(
      1).transient_local());
  stopped_ = true;paused_ = false;current_ = map_->loadedPcdPointCount() > 0;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RogMapROS::on_activate(const rclcpp_lifecycle::State &)
{
  global_pub_->on_activate();local_pub_->on_activate();start();createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn RogMapROS::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop();global_pub_->on_deactivate();local_pub_->on_deactivate();destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn RogMapROS::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop();destroyRosInterfaces();map_.reset();callback_group_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn RogMapROS::on_shutdown(const rclcpp_lifecycle::State &)
{stop();destroyRosInterfaces();map_.reset();return nav2_util::CallbackReturn::SUCCESS;}

void RogMapROS::createRosInterfaces()
{
  if (cloud_sub_) {return;} rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RogMapROS::cloudCallback, this, std::placeholders::_1), options);
  if (publish_frequency_ > 0.0) {
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_frequency_),
      std::bind(&RogMapROS::publishMaps, this), callback_group_);
  }
}
void RogMapROS::destroyRosInterfaces()
{
  publish_timer_.reset();cloud_sub_.reset();tf_listener_.reset();tf_buffer_.reset();
  global_pub_.reset();local_pub_.reset();
}
void RogMapROS::start() {paused_ = false;stopped_ = false;createRosInterfaces();}
void RogMapROS::stop() {stopped_ = true;current_ = false;publish_timer_.reset();cloud_sub_.reset();}
void RogMapROS::pause() {paused_ = true;current_ = false;}
void RogMapROS::resume() {paused_ = false;}
void RogMapROS::resetMap() {if (map_) {map_->reset();current_ = false;}}

void RogMapROS::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  if (stopped_ || paused_ || !map_) {return;} geometry_msgs::msg::TransformStamped transform;
  const uint64_t stamp_nanoseconds = rclcpp::Time(message->header.stamp).nanoseconds();
  if (update_frequency_ > 0.0 && last_update_stamp_nanoseconds_ > 0 &&
    stamp_nanoseconds > last_update_stamp_nanoseconds_)
  {
    const uint64_t minimum_period = static_cast<uint64_t>(1.0e9 / update_frequency_);
    if (stamp_nanoseconds - last_update_stamp_nanoseconds_ < minimum_period) {
      return;
    }
  }
  const std::string & source_frame =
    point_cloud_frame_.empty() ? message->header.frame_id : point_cloud_frame_;
  try {
    transform = tf_buffer_->lookupTransform(
      global_frame_, source_frame, message->header.stamp,
      rclcpp::Duration::from_seconds(transform_tolerance_));
  } catch (const tf2::TransformException & error) {
    current_ = false;RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(), 2000, "ROG map cloud transform %s -> %s failed: %s",
      source_frame.c_str(), global_frame_.c_str(), error.what());return;
  }
  const auto & q = transform.transform.rotation;double x = q.x, y = q.y, z = q.z, w = q.w;
  double r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - z * w), r02 = 2 * (x * z + y * w);
  double r10 = 2 * (x * y + z * w), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - x * w);
  double r20 = 2 * (x * z - y * w), r21 = 2 * (y * z + x * w), r22 = 1 - 2 * (x * x + y * y);
  RogMapInput input;input.sensor_origin.x = transform.transform.translation.x;
  input.sensor_origin.y = transform.transform.translation.y;
  input.sensor_origin.z = transform.transform.translation.z;
  input.stamp_nanoseconds = stamp_nanoseconds;input.points.reserve(
    message->width * message->height);
  sensor_msgs::PointCloud2ConstIterator<float> ix(*message, "x"), iy(*message, "y"), iz(
    *message,
    "z");
  for (; ix != ix.end(); ++ix, ++iy, ++iz) {
    if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
      continue;
    }
    geometry_msgs::msg::Point point;
    point.x = r00 * *ix + r01 * *iy + r02 * *iz + input.sensor_origin.x;
    point.y = r10 * *ix + r11 * *iy + r12 * *iz + input.sensor_origin.y;
    point.z = r20 * *ix + r21 * *iy + r22 * *iz + input.sensor_origin.z;input.points.push_back(
      point);
  }
  map_->update(input);last_update_stamp_nanoseconds_ = stamp_nanoseconds;current_ = true;
}

void RogMapROS::publishMaps()
{
  if (stopped_ || paused_ || !current_ || !map_) {
    return;
  }
  const bool publish_global = global_pub_->get_subscription_count() > 0 ||
    global_pub_->get_intra_process_subscription_count() > 0;
  const bool publish_local = local_pub_->get_subscription_count() > 0 ||
    local_pub_->get_intra_process_subscription_count() > 0;
  if (!publish_global && !publish_local) {
    return;
  }
  const auto stamp = now();
  if (publish_global) {
    auto global = map_->occupiedCloud();
    global.header.stamp = stamp;
    global_pub_->publish(global);
  }
  if (publish_local) {
    auto local = map_->localOccupiedCloud();
    local.header.stamp = stamp;
    local_pub_->publish(local);
  }
}

}  // namespace navflex_rog_map

RCLCPP_COMPONENTS_REGISTER_NODE(navflex_rog_map::RogMapROS)
