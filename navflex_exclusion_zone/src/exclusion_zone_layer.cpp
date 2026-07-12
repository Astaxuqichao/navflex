// Copyright (c) 2024 NavFlex
// SPDX-License-Identifier: Apache-2.0

#include "navflex_exclusion_zone/exclusion_zone_layer.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/empty.hpp"

// 向 pluginlib 注册插件
PLUGINLIB_EXPORT_CLASS(
  navflex_exclusion_zone::ExclusionZoneLayer,
  nav2_costmap_2d::Layer)

namespace navflex_exclusion_zone
{

ExclusionZoneLayer::ExclusionZoneLayer() = default;
ExclusionZoneLayer::~ExclusionZoneLayer() = default;

// ---------------------------------------------------------------------------
// 生命周期
// ---------------------------------------------------------------------------

void ExclusionZoneLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("[ExclusionZoneLayer] Failed to lock lifecycle node.");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);

  RCLCPP_INFO(
    node->get_logger(),
    "[ExclusionZoneLayer] '%s' initializing (enabled=%s).",
    name_.c_str(), enabled_ ? "true" : "false");

  const std::string add_topic = "/add_exclusion_zone";
  add_zone_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    add_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&ExclusionZoneLayer::cbAddZone, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "[ExclusionZoneLayer] Subscribe -> %s", add_topic.c_str());

  const std::string clear_topic = "/clear_exclusion_zones";
  clear_zone_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    clear_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&ExclusionZoneLayer::cbClearZones, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "[ExclusionZoneLayer] Subscribe -> %s", clear_topic.c_str());

  // 注册动态参数回调，支持运行时 ros2 param set
  param_cb_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() == name_ + ".enabled") {
          enabled_ = p.as_bool();
          // 重新启用时强制重建 buffer，使禁区在下一个 update 周期立即生效
          if (enabled_) {
            std::lock_guard<std::mutex> lock(mutex_);
            need_recalc_ = true;
          }
          if (auto n = node_.lock()) {
            RCLCPP_INFO(
              n->get_logger(),
              "[ExclusionZoneLayer] '%s' enabled -> %s",
              name_.c_str(), enabled_ ? "true" : "false");
          }
        }
      }
      return result;
    });
}

void ExclusionZoneLayer::activate() {}
void ExclusionZoneLayer::deactivate() {}

// ---------------------------------------------------------------------------
// matchSize
//
// 地图尺寸/分辨率变化时由框架调用，同步 costmap_[] buffer 大小。
// ---------------------------------------------------------------------------

void ExclusionZoneLayer::matchSize()
{
  CostmapLayer::matchSize();
  std::lock_guard<std::mutex> lock(mutex_);
  need_recalc_ = true;
}

void ExclusionZoneLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  zones_.clear();
  bbox_min_x_ = bbox_min_y_ = std::numeric_limits<double>::max();
  bbox_max_x_ = bbox_max_y_ = std::numeric_limits<double>::lowest();
  if (costmap_) {
    std::fill(
      costmap_, costmap_ + size_x_ * size_y_,
      nav2_costmap_2d::NO_INFORMATION);
  }
  need_recalc_ = false;
}

// ---------------------------------------------------------------------------
// 话题 / 服务回调
// ---------------------------------------------------------------------------

void ExclusionZoneLayer::cbAddZone(
  const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  if (msg->polygon.points.size() < 3) {
    if (auto node = node_.lock()) {
      RCLCPP_WARN(
        node->get_logger(),
        "[ExclusionZoneLayer] Polygon has < 3 points -- ignored.");
    }
    return;
  }

  // 将多边形从消息坐标系变换到 costmap 的 global frame（如 map / odom）
  const std::string target_frame = layered_costmap_->getGlobalFrameID();
  const std::string source_frame =
    msg->header.frame_id.empty() ? target_frame : msg->header.frame_id;

  std::vector<geometry_msgs::msg::Point32> transformed_points;
  transformed_points.reserve(msg->polygon.points.size());

  if (source_frame == target_frame) {
    // 同一坐标系，直接使用
    transformed_points.assign(msg->polygon.points.begin(), msg->polygon.points.end());
  } else {
    // 逐点做 TF 变换
    try {
      // 等待变换关系最多 0.5 s
      geometry_msgs::msg::TransformStamped tf_stamped =
        tf_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.5));

      for (const auto & p : msg->polygon.points) {
        geometry_msgs::msg::PointStamped src_pt, dst_pt;
        src_pt.header.frame_id = source_frame;
        src_pt.header.stamp = msg->header.stamp;
        src_pt.point.x = static_cast<double>(p.x);
        src_pt.point.y = static_cast<double>(p.y);
        src_pt.point.z = static_cast<double>(p.z);

        tf2::doTransform(src_pt, dst_pt, tf_stamped);

        geometry_msgs::msg::Point32 tp;
        tp.x = static_cast<float>(dst_pt.point.x);
        tp.y = static_cast<float>(dst_pt.point.y);
        tp.z = static_cast<float>(dst_pt.point.z);
        transformed_points.push_back(tp);
      }
    } catch (const tf2::TransformException & ex) {
      if (auto node = node_.lock()) {
        RCLCPP_WARN(
          node->get_logger(),
          "[ExclusionZoneLayer] TF transform %s->%s failed: %s. Zone ignored.",
          source_frame.c_str(), target_frame.c_str(), ex.what());
      }
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    zones_.emplace_back(std::move(transformed_points));
    recalcBoundsLocked();
    need_recalc_ = true;
  }

  if (auto node = node_.lock()) {
    RCLCPP_INFO(
      node->get_logger(),
      "[ExclusionZoneLayer] Zone #%zu added (%zu vertices) [%s -> %s].",
      zones_.size(), msg->polygon.points.size(),
      source_frame.c_str(), target_frame.c_str());
  }
}

void ExclusionZoneLayer::cbClearZones(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  reset();
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[ExclusionZoneLayer] All zones cleared.");
  }
}

// ---------------------------------------------------------------------------
// updateBounds
//
// 只要禁区非空就每帧提交 AABB：
// 确保 updateCosts 每帧运行，覆盖障碍层射线清除带来的影响。
// ---------------------------------------------------------------------------

void ExclusionZoneLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(mutex_);
  if (zones_.empty()) {return;}

  *min_x = std::min(*min_x, bbox_min_x_);
  *min_y = std::min(*min_y, bbox_min_y_);
  *max_x = std::max(*max_x, bbox_max_x_);
  *max_y = std::max(*max_y, bbox_max_y_);
}

// ---------------------------------------------------------------------------
// updateCosts
//
// 非滚动地图（static window）：
//   禁区变化时重建 costmap_[] buffer（昂贵，仅一次），
//   之后每帧用 updateWithMax 把 buffer 覆写 master_grid（廉价）。
//   updateWithMax：buffer=NO_INFORMATION 跳过，buffer=LETHAL(254) 写 max。
//
// 滚动地图（rolling window）：
//   地图原点每帧平移，buffer 失效，每帧对 master_grid 直接做点在多边形测试。
// ---------------------------------------------------------------------------

void ExclusionZoneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(mutex_);
  if (zones_.empty()) {return;}

  if (!layered_costmap_->isRolling()) {
    // 非滚动：buffer 路径
    if (need_recalc_) {
      rebuildBufferLocked();
    }
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  } else {
    // 滚动：每帧直接写 master_grid
    double wx{0.0}, wy{0.0};
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        master_grid.mapToWorld(
          static_cast<unsigned int>(i),
          static_cast<unsigned int>(j),
          wx, wy);
        for (const auto & zone : zones_) {
          if (pointInPolygon(wx, wy, zone)) {
            master_grid.setCost(
              static_cast<unsigned int>(i),
              static_cast<unsigned int>(j),
              nav2_costmap_2d::LETHAL_OBSTACLE);
            break;
          }
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 私有工具
// ---------------------------------------------------------------------------

bool ExclusionZoneLayer::pointInPolygon(
  double px, double py,
  const std::vector<geometry_msgs::msg::Point32> & poly)
{
  if (poly.size() < 3) {return false;}

  bool inside = false;
  const std::size_t n = poly.size();
  std::size_t j = n - 1;

  for (std::size_t i = 0; i < n; ++i) {
    const double xi = static_cast<double>(poly[i].x);
    const double yi = static_cast<double>(poly[i].y);
    const double xj = static_cast<double>(poly[j].x);
    const double yj = static_cast<double>(poly[j].y);

    if (((yi > py) != (yj > py)) &&
      (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
    j = i;
  }
  return inside;
}

void ExclusionZoneLayer::recalcBoundsLocked()
{
  bbox_min_x_ = bbox_min_y_ = std::numeric_limits<double>::max();
  bbox_max_x_ = bbox_max_y_ = std::numeric_limits<double>::lowest();

  for (const auto & zone : zones_) {
    for (const auto & pt : zone) {
      bbox_min_x_ = std::min(bbox_min_x_, static_cast<double>(pt.x));
      bbox_min_y_ = std::min(bbox_min_y_, static_cast<double>(pt.y));
      bbox_max_x_ = std::max(bbox_max_x_, static_cast<double>(pt.x));
      bbox_max_y_ = std::max(bbox_max_y_, static_cast<double>(pt.y));
    }
  }
}

// ---------------------------------------------------------------------------
// rebuildBufferLocked  （非滚动地图，mutex_ 须已持有）
//
// 清空 costmap_[] 为 NO_INFORMATION，然后在禁区 AABB 内做点在多边形测试，
// 命中格写 LETHAL_OBSTACLE。完成后清除 need_recalc_。
// ---------------------------------------------------------------------------

void ExclusionZoneLayer::rebuildBufferLocked()
{
  if (!costmap_) {return;}

  std::fill(
    costmap_, costmap_ + size_x_ * size_y_,
    nav2_costmap_2d::NO_INFORMATION);

  if (zones_.empty()) {
    need_recalc_ = false;
    return;
  }

  // 世界坐标 AABB -> 格坐标范围（clamp 到地图边界）
  unsigned int i0 = 0, j0 = 0, i1 = size_x_, j1 = size_y_;
  {
    unsigned int mx, my;
    if (worldToMap(bbox_min_x_, bbox_min_y_, mx, my)) {
      i0 = mx; j0 = my;
    }
    if (worldToMap(bbox_max_x_, bbox_max_y_, mx, my)) {
      i1 = std::min(mx + 1u, size_x_);
      j1 = std::min(my + 1u, size_y_);
    }
  }

  double wx, wy;
  for (unsigned int j = j0; j < j1; ++j) {
    for (unsigned int i = i0; i < i1; ++i) {
      mapToWorld(i, j, wx, wy);
      for (const auto & zone : zones_) {
        if (pointInPolygon(wx, wy, zone)) {
          costmap_[j * size_x_ + i] = nav2_costmap_2d::LETHAL_OBSTACLE;
          break;
        }
      }
    }
  }

  need_recalc_ = false;
}

}  // namespace navflex_exclusion_zone
