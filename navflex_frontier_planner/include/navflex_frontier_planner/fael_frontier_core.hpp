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
  // 【视点朝向】= 从视点指向它所认领前沿的【质心】。
  //
  // 为什么必须有:
  //   激光滤波后只有 ±45° 的前视锥(h_angle)。视点的意义是"站在这里能看到那些前沿",
  //   但【只有朝向前沿时才真的看得见】。
  //   而 updatePathOrientations() 把路径每个位姿的朝向设为【沿路径切线】——
  //   末点的朝向 = 到达时的【行进方向】, 完全可能【背对】它本该观测的前沿。
  //   再加上 default_yaw_goal_tolerance 会强制这个朝向, 机器人就会认认真真地
  //   朝着【错误的方向】停下 -> 看不到前沿 -> 前沿消不掉 -> 下轮又选同一个视点 -> 死循环。
  //
  // 用法: makeAStarPath() 用它覆盖路径【最后一个位姿】的朝向(而非路径切线)。
  double yaw{0.0};
  bool has_yaw{false};
  std::vector<Point3> frontiers;
  // 【信息增益】—— score 的正项, 未扣距离代价和惩罚。
  // 单独存一份是为了【标定 distance_weight】: 分数是 gain - w*dist - penalties,
  // 只看 score 无法反推出 gain 和 w*dist 各占多少。见 calibration_log_enabled。
  double gain{0.0};
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
    // 【不可解前沿黑名单】
    // 有些 unknown 是物理上永远观测不到的(家具内部/沙发底下/柜子里/墙缝)。
    // 它们与自由空间的交界永远满足 isFrontier(), 且背后确实有 unknown 体积
    // (unknownGainBeyondFrontier > 0), 所以规划器会一直认为"值得去" ->
    // 机器人跑过去看不到 -> 前沿依旧 -> 无限循环。
    // frontier_attempts: 机器人在近距离(unresolvable_frontier_probe_dist)看过它、
    //                    但它仍是前沿的次数(每个规划周期最多 +1)。
    // unresolvable_frontier_cells: 次数超阈值后拉黑, 从此不再产生候选。
    std::unordered_map<ufo::map::Code, int, ufo::map::Code::Hash> frontier_attempts;
    FrontierSet unresolvable_frontier_cells;
    // ★计数必须与 BT 的重规划频率【解耦】★
    // updateGlobalFrontiers 只在 selectCandidates 里被调用，而后者由探索 BT 的
    // RateController 驱动 —— 用户随时可能把 hz 从 0.33 改成 10（快 30 倍）。
    // 若按"每次调用 +1"计数，max_attempts=5 就会从 15 秒变成 0.5 秒，
    // 机器人会在半秒内把身边所有还没来得及观测的前沿全部拉黑，探索直接崩溃。
    // 所以这里记住上次计数的时刻，只有真正过了 probe_period 秒才 +1。
    rclcpp::Time last_unresolvable_count_time;
    bool has_unresolvable_count_time{false};
    // 【目标承诺 / 防横跳】
    // FrontierAStar 每个规划周期都用 force_refresh=true 全量重算候选并选第一名
    // (frontier_astar_planner.cpp:65)，缓存类参数(reuse_cached_candidates 等)对它
    // 完全不生效 —— 也就是【零迟滞】。
    // 而地图 10Hz 在变、unknownGainBeyondFrontier 的射线穿过的体素在变 -> 增益在抖；
    // 距离项又压不住(distance_weight×dist 比增益小几个数量级) ->
    // 探索后期两个增益相近的口袋会让排名反复翻转 -> 机器人原地横跳。
    // 这里记住上一轮真正选中的目标，下一轮若它仍然有效就继续咬住它，
    // 除非新的最佳【明显】更好(超出 switch_margin)才切换。
    // 存【完整的 Candidate】(含 yaw)，而不只是一个点 —— 因为"到了位置但还没转到朝向"时
    // 需要把它原样再返回一次，让机器人把转向做完。
    Candidate committed_candidate;
    bool has_committed_target{false};
    // 「转向未完成」保护的计时起点（超时后放弃锁定，避免死锁）
    rclcpp::Time observation_hold_start;
    bool observation_hold_active{false};
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

  // 【全局前沿兜底】局部视点认领不到任何前沿时(candidates 为空)的最后一招。
  // 核心洞察: 远处前沿【认领不到】(认领要求 isFrontierVisible 视线可达),
  //           但【走得到】(拓扑图是全局连通的)。
  // 于是绕开"认领", 直接把远处前沿当导航目标, 走近了局部逻辑自然接管。
  std::vector<Candidate> selectGlobalFallbackCandidates(const Point3 & current) const;
  // 在 frontier 周围找一个"站得住"的点作为视点(复用 sampleViewpoints 的判据)。
  bool findStandableViewpointNear(
    const Point3 & frontier,
    const Point3 & current,
    Point3 & out) const;

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

  // ---- 不可解前沿黑名单 (见 SharedMapState::unresolvable_frontier_cells) ----
  bool unresolvable_frontier_enabled_{true};
  // 机器人离前沿多近才算"我尽力看过了"。
  double unresolvable_frontier_probe_dist_{2.0};
  // 两次计数之间至少要间隔多少【秒】。这一项让黑名单与 BT 的重规划频率解耦：
  // 无论 RateController 是 0.33Hz 还是 10Hz，计数都最多 probe_period 秒一次。
  double unresolvable_frontier_probe_period_{3.0};
  // 累计多少次近距离观测未果就拉黑。
  // 实际耗时 = max_attempts x probe_period = 5 x 3.0 = 15 秒（与 BT 频率无关）。
  int unresolvable_frontier_max_attempts_{5};

  // ---- 拓扑图的【连通性间隙】—— 与 robot_clear_radius 解耦 ----
  // 为什么必须独立成一个参数:
  //   isKnownFree2D()(建边 / 目标接入) 和 is_start_connection_free()(起点接入)
  //   原本都用 robot_clear_radius(0.35) 做间隙检查 -> 要求通道宽 >= 2x0.35 = 0.70m。
  //   但 robot_clear_radius 是【三重复用】的参数, 动不得:
  //     · 强制清空球半径 -> 必须 >= 滤波 min_range(0.3), 否则周身留"未知壳"(永久伪前沿)
  //     · 视点避障判据   -> 调小会让视点贴墙
  //   而机器人的内接半径只有 0.19m(footprint 0.56x0.38) -> 实际只需通道 >= 0.38m。
  //   => 0.38 ~ 0.70m 的通道: 机器人过得去, 拓扑图却连不上(house2 厨房窄道就是这种)。
  // 语义: 拓扑边/接入线上每一点, 该半径内不得有障碍。
  //       它只是【路径引导】的间隙 —— 真正的避障由 MPPI 用 costmap + 精确 footprint 完成,
  //       所以可以比 robot_clear_radius 小, 贴着内接半径给一点余量即可。
  // 默认 0.35 = 保持原行为(向后兼容)。
  double topology_edge_clearance_{0.35};

  // ---- 目标承诺 / 防横跳 (见 SharedMapState::committed_target) ----
  bool target_commitment_enabled_{true};
  // 本轮候选离上轮目标多近，才算"同一个目标"（候选每轮重新采样，点不会完全重合）。
  double target_commitment_match_radius_{1.0};
  // 只有当新的最佳分数超出【已承诺目标】这个【相对比例】时才切换。
  // 用相对比例而非绝对值：information_gain 的量级随前沿数量剧烈变化(几百~几千)，
  // 绝对阈值没法通用。0.25 = "新目标要好过 25% 才值得掉头"。
  double target_commitment_switch_margin_{0.25};

  // ---- 「转向未完成」保护 ----
  // 治的病: 机器人到达视点后【还没转到目标朝向】, 5 秒后 RateController 就重规划了。
  //   此时机器人离原视点只有 ~0.2m < min_candidate_dist(0.8) -> 原视点【不再被采样】
  //   -> 目标承诺找不到它 -> 选一个全新的候选 -> NavflexExePathAction 运行中替换目标
  //   -> 机器人【放弃转向】直奔新目标 -> 朝向机制形同虚设(白改了)。
  // 解法: 位置已到、朝向未到 时, 把【上一轮的候选原样返回】, 锁住目标直到转完。
  //   控制器的 isGoalReached 本来就要求 xy 和 yaw 都到位才算完成, 所以只要目标不被
  //   换掉, 它自己会把转向做完。
  bool observation_hold_enabled_{true};
  // 离视点多近算"位置已到"。应 >= 控制器的 default_xy_goal_tolerance(0.20)，
  // 否则机器人已被判定到点、控制器在原地转, 这里却还以为没到, 白白放行重规划。
  double observation_hold_xy_tol_{0.30};
  // 朝向误差多小算"转到了"。应与控制器的 default_yaw_goal_tolerance(0.35) 一致。
  double observation_hold_yaw_tol_{0.35};
  // 最长锁多久(秒)。防止朝向永远转不到(被卡住/MPPI 转不动)时死锁在这里。
  double observation_hold_timeout_{8.0};

  // ---- 全局前沿兜底 ----
  bool global_fallback_enabled_{true};
  // 最多尝试几个最近的全局前沿(每个都要跑一次拓扑 A*, 不能太多)。
  int global_fallback_max_targets_{5};
  // 在前沿周围多大范围内找"站得住"的视点。
  double global_fallback_viewpoint_radius_{1.5};
  double robot_clear_radius_{0.3};
  double unknown_clear_radius_{0.0};
  double viewpoint_free_z_min_{0.0};
  double viewpoint_free_z_max_{0.8};
  double viewpoint_free_z_step_{0.1};
  double sensor_height_{0.45};
  double frontier_slope_deg_{89.0};
  double viewpoint_slope_deg_{15.0};
  // ★【视点的水平视场角】(度, 全宽)。雷达做了水平 FOV 裁剪时必须设成裁剪后的宽度。
  //   viewpoint_slope_deg_ 已经约束了【垂直】方向 (|dz|/dxy < tan(slope))，
  //   但水平方向此前【毫无约束】——一个视点可以认领 360° 上的所有前沿，
  //   而机器人站在那儿一次只看得见 ±(fov/2)。→ 增益被严重高估、视点排序失真。
  //   360.0 = 不裁剪 (全向雷达, 保持旧行为)。
  double viewpoint_horizontal_fov_deg_{360.0};
  // 打开后每轮把每个候选的 (gain, dist) 打成 "[CAL] gain=... dist=..." 一行,
  // 供离线标定 distance_weight 用。默认关 —— 打开会明显增加日志量。
  bool calibration_log_enabled_{false};
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
