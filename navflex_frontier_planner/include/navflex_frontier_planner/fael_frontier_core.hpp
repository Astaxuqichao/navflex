#ifndef NAVFLEX_FRONTIER_PLANNER__FAEL_FRONTIER_CORE_HPP_
#define NAVFLEX_FRONTIER_PLANNER__FAEL_FRONTIER_CORE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "ufo/map/code.h"
#include "ufo/map/occupancy_map.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace navflex_frontier_planner
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  double distanceXY(const Point3 & other) const;
  double distance(const Point3 & other) const;
};

struct Candidate
{
  Point3 point;
  std::vector<Point3> frontiers;
  double score{0.0};
};

struct TopologyNode
{
  Point3 point;
  double clearance{0.0};
  std::vector<std::size_t> neighbors;
};


class FaelFrontierCore
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  void activate();
  void deactivate();
  void cleanup();

  std::optional<Candidate> selectCandidate(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & requested_goal);

  std::vector<Candidate> selectCandidates(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & requested_goal,
    bool force_refresh = false);

  nav_msgs::msg::Path makeCandidatePath(
    const geometry_msgs::msg::PoseStamped & start,
    const Candidate & candidate) const;

  nav_msgs::msg::Path makeCandidatePath(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<Candidate> & candidates) const;

  nav_msgs::msg::Path makeAStarPath(
    const geometry_msgs::msg::PoseStamped & start,
    const Candidate & candidate);

  void publishSelection(
    const geometry_msgs::msg::PoseStamped & start,
    const std::vector<Candidate> & candidates,
    const Candidate & selected) const;

  std::string frameId() const {return frame_id_;}

private:
  using FrontierSet = std::unordered_set<ufo::map::Code, ufo::map::Code::Hash>;

public:
  struct SharedMapState
  {
    mutable std::mutex mutex;
    std::unique_ptr<ufo::map::OccupancyMap> map;
    FrontierSet changed_cell_codes;
    FrontierSet known_cell_codes;
    FrontierSet local_frontier_cells;
    FrontierSet global_frontier_cells;
    std::vector<Candidate> candidates;
    rclcpp::Time candidates_stamp;
    std::string candidates_owner;
    std::uint64_t map_revision{0};
    std::uint64_t candidates_map_revision{0};
    Point3 candidates_origin;
    bool has_cached_candidates{false};
    std::unordered_map<ufo::map::Code, Point3, ufo::map::Code::Hash> frontiers_viewpoints;
    std::vector<Point3> visited_positions;
    std::vector<TopologyNode> topology_nodes;
    std::vector<Point3> last_topology_path;
    rclcpp::Time topology_stamp;
    std::uint64_t topology_map_revision{0};
    double topology_plane_z{0.0};
    Point3 topology_update_origin;
    bool has_topology_update_origin{false};
    bool initialized_from_accumulated_map{false};
    Point3 latest_sensor_position;
    bool has_latest_sensor_position{false};
    bool startup_topology_published{false};
    bool has_topology{false};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    std::string owner_name;
  };

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void topologyInitializationTimerCallback();
  std::shared_ptr<SharedMapState> getSharedMapState(
    double resolution,
    int depth_levels);

  ufo::map::Point3 toUfoPoint(const Point3 & point) const;
  Point3 fromUfoPoint(const ufo::map::Point3 & point) const;
  Point3 codeToPoint(const ufo::map::Code & code) const;
  bool isNearOccupied(const Point3 & point, double radius) const;
  bool isNearUnknown(const Point3 & point, double radius) const;
  bool hasFreeVoxelNearHeight(const Point3 & point) const;
  bool isFrontier(const ufo::map::Code & code) const;
  bool isCollisionFree2D(const Point3 & from, const Point3 & to) const;
  bool isKnownFree2D(const Point3 & from, const Point3 & to) const;
  bool isFrontierVisible(const Point3 & viewpoint, const Point3 & frontier) const;
  bool isViewpointConnectionFree(const Point3 & from, const Point3 & to) const;
  double topologyClearance(const Point3 & point) const;
  bool updateTopologyMap(
    const Point3 & current,
    bool initialize_from_accumulated_map = false);
  std::vector<Point3> searchTopologyPath(
    const Point3 & start,
    const Point3 & goal) const;
  double unknownGainBeyondFrontier(const Point3 & viewpoint, const Point3 & frontier) const;

  void frontierSearch(const Point3 & current);
  void findLocalFrontiers(const Point3 & current);
  void updateGlobalFrontiers(const Point3 & current);
  std::vector<Point3> getGlobalFrontiers() const;
  std::vector<Point3> compactFrontiersForAttachment(
    const std::vector<Point3> & frontiers,
    const Point3 & current) const;
  std::vector<Point3> sampleViewpoints(const Point3 & current) const;
  std::vector<Candidate> attachFrontiers(
    const std::vector<Point3> & viewpoints,
    const std::vector<Point3> & frontiers,
    const Point3 & current) const;
  void publishCandidates(
    const std::vector<Candidate> & candidates,
    const Candidate * selected) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("FaelFrontierCore")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    candidate_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    topology_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    selected_candidate_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_map_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_map_pub_;
  rclcpp::TimerBase::SharedPtr topology_initialization_timer_;

  std::shared_ptr<SharedMapState> map_state_;

  std::string name_;
  std::string frame_id_{"map"};
  std::string point_cloud_topic_{"point_cloud"};
  std::string visualization_topic_prefix_{"frontier_exploration"};
  double resolution_{0.4};
  int depth_levels_{16};
  int insert_depth_{0};
  bool insert_discrete_{true};
  bool simple_ray_casting_{false};
  int early_stopping_{0};
  bool publish_map_clouds_{true};
  double map_publish_period_{1.0};
  double max_range_{12.0};
  double sample_dist_{1.0};
  double local_range_{12.0};
  double candidate_visibility_range_{11.5};
  bool reuse_cached_candidates_{true};
  double cache_robot_move_threshold_{1.0};
  double candidate_recompute_period_{1.0};
  double frontier_attach_grid_size_{0.4};
  int global_frontier_revalidate_max_cells_{5000};
  int frontier_visibility_max_viewpoints_{8};
  double viewpoint_gain_threshold_{2.0};
  double min_frontier_area_{0.05};
  double candidate_separation_{1.0};
  double frontier_distance_weight_{0.0};
  int min_candidate_count_{8};
  int max_candidate_count_{10};
  double frontier_gain_{100.0};
  double unknown_gain_range_{1.5};
  double unknown_gain_step_{0.2};
  double min_unknown_gain_{0.0};
  double distance_weight_{0.1};
  double visited_radius_{1.5};
  double visited_penalty_{1000.0};
  double known_gain_penalty_{0.02};
  double min_candidate_dist_{0.5};
  double min_robot_frontier_dist_{0.6};
  double robot_clear_radius_{0.3};
  double unknown_clear_radius_{0.0};
  double viewpoint_free_z_min_{0.0};
  double viewpoint_free_z_max_{0.8};
  double viewpoint_free_z_step_{0.1};
  double sensor_height_{0.45};
  double frontier_slope_deg_{89.0};
  double viewpoint_slope_deg_{15.0};
  bool topology_enabled_{true};
  double topology_initialization_period_{0.5};
  double topology_update_distance_{1.0};
  double topology_update_radius_{6.0};
  double topology_node_spacing_{1.2};
  double topology_local_node_spacing_{0.6};
  double topology_min_clearance_{0.35};
  double topology_local_min_clearance_{0.2};
  double topology_max_clearance_{2.4};
  double topology_connection_radius_{2.5};
  double topology_attach_radius_{5.0};
  double topology_z_tolerance_{0.6};
  int topology_initial_min_nodes_{8};
  int topology_max_samples_{6000};
  int topology_max_nodes_{1200};
  int topology_max_neighbors_{4};
  rclcpp::Time last_map_publish_time_;

  void publishMapClouds();
  void publishTopology(
    const std::vector<Candidate> & candidates,
    const Point3 & current,
    const Candidate * selected,
    bool publish_best_path = false) const;

  struct ViewpointDebug
  {
    std::size_t sampled{0};
    std::size_t outside_range{0};
    std::size_t not_free{0};
    std::size_t near_occupied{0};
    std::size_t near_unknown{0};
    std::size_t too_close{0};
    std::size_t collision{0};
  };

  mutable ViewpointDebug last_viewpoint_debug_;
};

}  // namespace navflex_frontier_planner

#endif  // NAVFLEX_FRONTIER_PLANNER__FAEL_FRONTIER_CORE_HPP_
