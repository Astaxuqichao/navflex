#!/usr/bin/env python3
"""
ufomap_viz_crop.py —— 【纯可视化】用的 UFO 地图高度裁剪节点

问题：
    frontier planner 的 UFO 地图在插入点云时【完全没有高度裁剪】
    (frontier_tools.cpp:495-501, 所有有限值的点都塞进地图)，
    所以天花板也在地图里。在 RViz 里俯视时天花板会把整个房间盖住。
    而 RViz 的 PointCloud2 显示【没有内置的高度过滤】
    (AxisColor 的 Min/Max Value 只影响颜色, 不影响可见性)。

做法：
    订阅 UFO 占据/自由点云 -> 按 z 裁掉天花板(和地面以下的杂点) -> 重发到新话题。
    RViz 订阅新话题即可。

⚠️ 这个节点【只影响可视化】:
    · 不修改 UFO 地图本身
    · 不影响前沿检测 / 视点选择 / 路径规划
    · 不需要重编 navflex (直接 python3 运行)

点云的 frame_id = "map" (frontier_shared_config.frame_id)，
所以 z 就是【真实离地高度】，直接按 z 裁即可。

用法（在 nav 容器里，与导航栈同一套 RMW/DOMAIN 环境）：

    source /workspace/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp    # MATRiX 场景; 本地仿真则不要设
    export ROS_DOMAIN_ID=89                    # MATRiX 场景; 本地仿真则不要设

    python3 /workspace/navflex/navflex_bringup/scripts/ufomap_viz_crop.py \
        --ros-args -p z_max:=1.6

    # z_max:=2 和 z_max:=2.0 都能用 (本脚本用 dynamic_typing 兼容了 int/float)

然后在 RViz 里把那个 PointCloud2 的 Topic 改成:
    /frontier_exploration/ufomap_occupied_cloud_cropped

上下都裁, 只留【机器人视角下真正有意义的一层】:

    z > z_max  天花板/吊灯  -> 俯视时把整个房间盖住
    z < z_min  地面         -> 一大片连续的占据体素, 同样糊住室内结构
    ------------------------ 只保留中间这一层: 墙面 + 家具 + 障碍

参数(z_min/z_max 都是 map 系的【绝对离地高度】, 因为点云 frame_id="map"):

    z_min  (默认 0.15) 低于此高度的点丢弃 —— 主要是【切掉地面】。
                       这个值跟着 frontier_shared_config.resolution 走:
                           res 0.15 -> 地面体素中心 ≈ 0.075 -> z_min 0.20
                           res 0.10 -> 地面体素中心 ≈ 0.050 -> z_min 0.15  ← 当前
                       原则: 切在地面第一层体素【上方】, 又不误伤矮家具。
                       想把地面看回来: z_min:=-0.3
                       地面还是有残留(地不平/里程计漂移): 提到 0.20~0.25

    z_max  (默认 1.60) 高于此高度的点丢弃 —— 切掉天花板。
                       xgb 站立时 base_link 高 ~0.33, 雷达离地 ~0.51。
                       1.6 保留墙面/家具, 切掉 2.4m 左右的天花板。
                       房间矮就调到 1.4; 想看吊灯/门框上沿就调到 2.0。

    in_topic  (string) 输入话题, 默认 /frontier_exploration/ufomap_occupied_cloud
    out_topic (string) 输出话题, 默认 <in_topic>_cropped

    (z_min/z_max 传整数或小数都行, 如 z_max:=2 与 z_max:=2.0 等价)
"""

import struct
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import PointCloud2, PointField


class UfomapVizCrop(Node):
    def __init__(self):
        super().__init__('ufomap_viz_crop')

        # ROS 2 的参数是强类型的: 若把默认值声明成 DOUBLE(1.60),
        # 命令行传 `-p z_max:=2` 会被解析成 INTEGER -> 直接抛
        # InvalidParameterTypeException。用 dynamic_typing 允许 int/float 都能传,
        # 再统一 float() 转换 —— 这样 `z_max:=2` 和 `z_max:=2.0` 都能用。
        any_type = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('z_min', 0.15, any_type)
        self.declare_parameter('z_max', 1.60, any_type)
        self.declare_parameter(
            'in_topic', '/frontier_exploration/ufomap_occupied_cloud')
        self.declare_parameter('out_topic', '')

        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value or (in_topic + '_cropped')

        if self.z_min >= self.z_max:
            raise ValueError(
                f'z_min({self.z_min}) 必须小于 z_max({self.z_max})，否则会裁掉所有点')

        # UFO 点云是每 map_publish_period(1.0s) 重发的【全量快照】，
        # 用 Best Effort + depth 1 即可，不需要可靠传输堆积。
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.cb, qos)

        self.get_logger().info(
            f'ufomap_viz_crop: {in_topic} -> {out_topic}  '
            f'(keep {self.z_min:.2f} <= z <= {self.z_max:.2f})')

    def cb(self, msg: PointCloud2):
        # 找 x/y/z 三个字段的偏移量（不假设它们是 0/4/8，UFO 的点云可能带别的字段）
        offs = {f.name: f.offset for f in msg.fields}
        if not {'x', 'y', 'z'} <= offs.keys():
            self.get_logger().warn_once('点云缺少 x/y/z 字段，原样透传')
            self.pub.publish(msg)
            return

        ox, oy, oz = offs['x'], offs['y'], offs['z']
        step = msg.point_step
        data = msg.data
        n = len(data) // step if step else 0

        kept = bytearray()
        for i in range(n):
            base = i * step
            z = struct.unpack_from('<f', data, base + oz)[0]
            if self.z_min <= z <= self.z_max:
                kept += data[base:base + step]

        out = PointCloud2()
        out.header = msg.header          # frame_id = map，保持不变
        out.height = 1
        out.width = len(kept) // step if step else 0
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = step
        out.row_step = len(kept)
        out.data = bytes(kept)
        out.is_dense = msg.is_dense
        self.pub.publish(out)


def main():
    rclpy.init(args=sys.argv)
    node = UfomapVizCrop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C / SIGTERM 时安静退出，不要吐一屏 traceback
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
