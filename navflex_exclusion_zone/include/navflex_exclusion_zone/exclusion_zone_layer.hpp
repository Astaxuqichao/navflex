// Copyright (c) 2024 NavFlex
// SPDX-License-Identifier: Apache-2.0
//
// 电子禁区 Costmap Layer 插件
//
// 支持多多边形禁区（world 坐标），将禁区内所有格标记为 LETHAL_OBSTACLE，
// 兼容滚动地图（rolling window）和非滚动地图（static window）。
//
// ┌──────────────────────────────────────────────────────────────────┐
// │  ROS2 接口                                                        │
// ├──────────────────────────────────────────────────────────────────┤
// │  订阅  /add_exclusion_zone                                       │
// │        geometry_msgs/msg/PolygonStamped  —— 追加一个禁区           │
// │  订阅  /clear_exclusion_zones                                     │
// │        std_msgs/msg/Empty               —— 清除全部禁区            │
// ├──────────────────────────────────────────────────────────────────┤
// │  ROS2 参数（在 costmap 的 YAML 中配置）                            │
// │    {layer_name}.enabled  bool  default: true                     │
// └──────────────────────────────────────────────────────────────────┘

#pragma once

#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

namespace navflex_exclusion_zone
{

/**
 * @class ExclusionZoneLayer
 * @brief nav2_costmap_2d 插件：多边形电子禁区
 *
 * 设计要点
 * --------
 * 继承 CostmapLayer（有 costmap_[] 私有 buffer）：
 *
 * ● 非滚动地图（static window）
 *     地图格坐标固定。禁区变化时用多边形测试重建 buffer（昂贵，仅一次），
 *     之后每帧用 updateWithMax 把 buffer 覆写到 master_grid（廉价）。
 *     这样即使 obstacle_layer 的射线清除把 master_grid 里的格清掉，
 *     下一帧我们的 updateWithMax 依然能把 LETHAL 写回去。
 *
 * ● 滚动地图（rolling window）
 *     地图原点随机器人平移，格坐标每帧都变，无法复用 buffer。
 *     每帧直接对 master_grid 做点在多边形测试并写入。
 */
class ExclusionZoneLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ExclusionZoneLayer();
  ~ExclusionZoneLayer() override;

  // ---- nav2_costmap_2d::Layer 纯虚/虚函数 ----
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}

  /// 地图尺寸/分辨率变化时同步 buffer 大小
  void matchSize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  // ---- 回调 ----
  void cbAddZone(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void cbClearZones(const std_msgs::msg::Empty::SharedPtr msg);

  // ---- 工具 ----

  /// 射线法：判断点 (px, py) 是否在多边形内部
  static bool pointInPolygon(
    double px, double py,
    const std::vector<geometry_msgs::msg::Point32> & poly);

  /// 根据当前 zones_ 重新计算总包围盒（调用前需已持锁）
  void recalcBoundsLocked();

  /// 非滚动地图：把 zones_ 全量写入 costmap_[] buffer（调用前需已持锁）
  void rebuildBufferLocked();

  // ---- ROS 句柄 ----
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr add_zone_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_zone_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ---- 状态 ----
  mutable std::mutex mutex_;

  /// 所有禁区多边形的顶点列表（world 坐标，单位：米）
  std::vector<std::vector<geometry_msgs::msg::Point32>> zones_;

  /// 全部禁区的合并 AABB（world 坐标）
  double bbox_min_x_{std::numeric_limits<double>::max()};
  double bbox_min_y_{std::numeric_limits<double>::max()};
  double bbox_max_x_{std::numeric_limits<double>::lowest()};
  double bbox_max_y_{std::numeric_limits<double>::lowest()};

  /// 非滚动地图：zones_ 变化后需要重建 costmap_[] buffer
  bool need_recalc_{false};
};

}  // namespace navflex_exclusion_zone
