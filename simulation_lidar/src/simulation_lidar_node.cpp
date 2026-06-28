#include "navflex_costmap_nav/simulation_lidar_node.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <limits>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>





using std::placeholders::_1;

namespace autolabor_simulation
{

SimulationLidar::SimulationLidar(const rclcpp::NodeOptions & options)
: Node("simulation_lidar_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{

  declare_parameter("min_angle", -M_PI);
  declare_parameter("max_angle",  M_PI);
  declare_parameter("min_distance", 0.15);
  declare_parameter("max_distance", 6.0);
  declare_parameter("noise", 0.0);
  declare_parameter("size", 400);
  declare_parameter("rate", 10.0);
  declare_parameter("use_topic_odom", false);

  declare_parameter("odom_topic", "/odom");
  declare_parameter("stage_map_topic", "map");
  declare_parameter("global_frame", "map");
  declare_parameter("lidar_frame", "base_scan");

  get_parameter("use_sim_time", use_sim_time_);
  get_parameter("min_angle", min_angle_);
  get_parameter("max_angle", max_angle_);
  get_parameter("min_distance", min_dis_);
  get_parameter("max_distance", max_dis_);
  get_parameter("noise", noise_);
  get_parameter("size", point_size_);
  get_parameter("rate", rate_);
  get_parameter("use_topic_odom", use_topic_odom_);

  get_parameter("odom_topic", odom_topic_);
  get_parameter("stage_map_topic", stage_map_topic_);
  get_parameter("global_frame", global_frame_);
  get_parameter("lidar_frame", lidar_frame_);

  step_ = (max_angle_ - min_angle_) / (point_size_ - 1);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", sensor_qos);
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", sensor_qos);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local();
  qos.reliable();

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    stage_map_topic_,
    qos,
    std::bind(&SimulationLidar::mapCallback, this, _1));

  if (use_topic_odom_) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&SimulationLidar::odomCallback, this, _1));
  }

  timer_ = create_wall_timer(
    std::chrono::milliseconds((int)(1000.0 / rate_)),
    std::bind(&SimulationLidar::timerCallback, this));

}

void SimulationLidar::timerCallback()
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (local_map_.data.empty()) return;
  sensor_msgs::msg::LaserScan scan;
  initLaserScan(scan);
  generateFrame(scan);
  scan_pub_->publish(scan);
  publishPointCloud2(scan);
}

void SimulationLidar::initLaserScan(sensor_msgs::msg::LaserScan & scan)
{
  // 使用仿真时间
  rclcpp::Time sim_time = this->get_clock()->now();
  scan.header.stamp = sim_time;
  scan.header.frame_id = lidar_frame_;

  scan.angle_min = min_angle_;
  scan.angle_max = max_angle_;
  scan.angle_increment = step_;
  scan.range_min = min_dis_;
  scan.range_max = max_dis_;

  scan.ranges.resize(point_size_);
}

bool SimulationLidar::getPose(double & x, double & y, double & yaw)
{
  try {
    auto tf = tf_buffer_.lookupTransform(
      global_frame_,
      lidar_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.2)); 

    x = tf.transform.translation.x;
    y = tf.transform.translation.y;

    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    yaw = tf2::getYaw(q);
  }
  catch (...) {
    return false;
  }

  return true;
}

void SimulationLidar::generateFrame(sensor_msgs::msg::LaserScan & scan)
{
  double x, y, yaw;
  if (!getPose(x, y, yaw)) 
  {
    RCLCPP_WARN(get_logger(), "TF failed");
    return;
  }
  scan.intensities.resize(point_size_);
  for (int i = 0; i < point_size_; ++i)
  {
    double angle = yaw + min_angle_ + i * step_;
    float ex, ey;
    raycast(x, y, angle, ex, ey);
    float dis = std::hypot(ex - x, ey - y);
    if (dis < min_dis_ || dis >= max_dis_)
    {
      scan.ranges[i] = std::numeric_limits<float>::infinity();
      scan.intensities[i] = 0.0;   //  无效点
    }
    else
    {
      float noisy_dis = dis + (noise_ > 0.0 ? static_cast<float>(gaussian(0.0, noise_)) : 0.0f);
      if (noisy_dis < min_dis_ || noisy_dis > max_dis_)
      {
        scan.ranges[i] = std::numeric_limits<float>::infinity();
        scan.intensities[i] = 0.0;
      }
      else
      {
        scan.ranges[i] = noisy_dis;
        scan.intensities[i] = noisy_dis;   //  用距离做强度
      }
    }
  }
}

void SimulationLidar::raycast(float sx, float sy, double theta, float & ex, float & ey)
{
  float step = local_map_.info.resolution;
  for (float d = 0; d < max_dis_; d += step)
  {
    float wx = sx + d * cos(theta);
    float wy = sy + d * sin(theta);
    if (isOccupied(wx, wy))
    {
      ex = wx;
      ey = wy;
      return;
    }
  }
  ex = sx + max_dis_ * cos(theta);
  ey = sy + max_dis_ * sin(theta);
}

bool SimulationLidar::isOccupied(float wx, float wy)
{
  int mx = (wx - local_map_.info.origin.position.x) / local_map_.info.resolution;
  int my = (wy - local_map_.info.origin.position.y) / local_map_.info.resolution;
  if (mx < 0 || my < 0 ||
      mx >= (int)local_map_.info.width ||
      my >= (int)local_map_.info.height)
    return true;

  int index = my * local_map_.info.width + mx;

  return local_map_.data[index] > 50 || local_map_.data[index] < 0;
}

void SimulationLidar::publishPointCloud2(const sensor_msgs::msg::LaserScan & scan)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = scan.header;
  cloud.height = 1;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  // Count valid points
  std::vector<std::pair<float, float>> pts;
  pts.reserve(point_size_);
  for (int i = 0; i < point_size_; ++i) {
    float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    float angle = scan.angle_min + i * scan.angle_increment;
    pts.emplace_back(r * std::cos(angle), r * std::sin(angle));
  }

  cloud.width = pts.size();
  modifier.resize(pts.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (auto & [px, py] : pts) {
    *iter_x = px;
    *iter_y = py;
    *iter_z = 0.0f;
    ++iter_x; ++iter_y; ++iter_z;
  }

  cloud_pub_->publish(cloud);
}

void SimulationLidar::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  local_map_ = *msg;
  RCLCPP_INFO(this->get_logger(), "map received11");
}

void SimulationLidar::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_msg_ = *msg;
}

double SimulationLidar::gaussian(double mu, double sigma)
{
  return sigma * ((double)rand() / RAND_MAX - 0.5) * 2 + mu;
}


}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<autolabor_simulation::SimulationLidar>();

  rclcpp::spin(node);

  rclcpp::shutdown();
}
