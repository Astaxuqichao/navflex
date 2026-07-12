# navflex_bringup

`navflex_bringup` 是 Navflex 导航栈的统一启动包，集中维护启动文件、地图、参数和 RViz 配置。它支持两类仿真：

- 本地轻量仿真：`omni_fake_node` + `simulation_lidar`
- TurtleBot3 Waffle Pi + OpenMANIPULATOR-X Gazebo 仿真

导航栈本身由 `navflex_nav`、`bt_navigator`、`velocity_smoother` 和 lifecycle manager 组成。

## 依赖

### 系统环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Nav2 Humble
- BehaviorTree.CPP v3

常用系统依赖：

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-imu-sensor-broadcaster
```

### 工作区源码依赖

Navflex 必须使用个人维护的 Nav2 Humble 分支和开源 STVL。在工作区根目录执行：

```bash
git clone --branch humble --single-branch \
  https://github.com/Astaxuqichao/navigation2.git navigation2
git clone --branch humble --single-branch \
  https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git \
  spatio_temporal_voxel_layer
git clone https://github.com/Astaxuqichao/navflex.git navflex
```

不能用 apt 安装的标准 Nav2 替代 `Astaxuqichao/navigation2`，因为 Navflex 使用了
扩展后的 `nav2_msgs/action/FollowPath.action`。当前工作区需要包含这些源码包：

| 路径 | 作用 |
| --- | --- |
| `navflex` | Navflex 核心包、BT 节点、仿真节点和 bringup |
| `navigation2` | `Astaxuqichao/navigation2` 的 `humble` 分支，包含改动后的 `FollowPath.action` |
| `spatio_temporal_voxel_layer` | `SteveMacenski/spatio_temporal_voxel_layer` 的 `humble` 分支 |
| `turtlebot3` | TurtleBot3 基础包 |
| `turtlebot3_manipulation` | TurtleBot3 机械臂相关包 |
| `turtlebot3_simulations` | TurtleBot3 / TB3 manipulation Gazebo 仿真包 |
| `turtlebot3_msgs` | TurtleBot3 消息包 |
| `aws-robomaker-small-house-world-ros2` | AWS Small House Gazebo world、模型和配套地图 |

如果从新工作区准备依赖，建议使用 `rosdep` 补齐系统包：

```bash
cd ~/ros2/nav_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths \
  navigation2 spatio_temporal_voxel_layer navflex \
  --ignore-src -r -y --rosdistro humble
```

## 编译

工作区默认不编译 `navflex/FAEL`。FAEL 是 ROS 1 参考实现，当前 ROS 2
工作区只将其作为算法参考；仓库根目录已放置：

```text
navflex/FAEL/COLCON_IGNORE
```

因此直接执行 `colcon build` 时会自动跳过 FAEL，不需要额外
`--packages-ignore` 参数。如果确实需要单独处理 FAEL，请先移除或临时改名
`navflex/FAEL/COLCON_IGNORE`，并使用对应的 ROS 1/catkin 环境。

因为 `FollowPath.action` 已扩展了 `xy_goal_tolerance` 和 `yaw_goal_tolerance`，修改 action 后需要重编依赖它的包。推荐直接编译整个工作区：

```bash
cd ~/ros2/nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

仅调试 Navflex bringup 时，可选择性编译：

```bash
colcon build --packages-select \
  nav2_msgs nav2_behavior_tree nav2_bt_navigator \
  navflex_nav navflex_bt_nodes navflex_bt_navigator \
  navflex_frontier_planner navflex_bringup omni_fake_node simulation_lidar
source install/setup.bash
```

## 启动方式概览

| 启动文件 | 作用 |
| --- | --- |
| `sim_local_launch.py` | 启动本地轻量仿真，不启动导航栈 |
| `tb3_manipulation_sim_launch.py` | 启动 TB3 manipulation Gazebo 仿真，不启动 Navflex 导航栈 |
| `navflex_bringup_launch.py` | 启动 Navflex 导航栈 |

`navflex_bringup_launch.py` 默认：

- `use_sim_time:=true`
- `chassis_model:=omni`
- `params_file` 为空时按 `chassis_model` 自动选择参数文件

底盘参数选择：

| `chassis_model` | 参数文件 | 适用对象 |
| --- | --- | --- |
| `omni` | `params/nav2_params.yaml` | 全向底盘、本地 `omni_fake_node` |
| `diff` | `params/nav2_params_tb3_diff.yaml` | 差速底盘、TurtleBot3 |

## 本地轻量仿真

本地仿真用于快速验证 Navflex，不依赖 Gazebo。它会启动：

- RViz
- `omni_fake_node`
- `simulation_lidar`
- `map_server`
- `lifecycle_manager_map`
- 静态 TF：`map -> odom`

启动仿真：

```bash
ros2 launch navflex_bringup sim_local_launch.py
```

另开终端启动导航栈：

```bash
source install/setup.bash
ros2 launch navflex_bringup navflex_bringup_launch.py
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map_file` | `maps/map1.yaml` | 地图文件 |
| `rviz_config_file` | `rviz/nav2_default_view.rviz` | RViz 配置 |
| `use_sim_time` | `true` | 使用仿真时间 |
| `autostart` | `true` | 自动激活 `map_server` |

本地轻量仿真没有 Gazebo，因此 `omni_fake_node` 会发布 `/clock`。如果 `use_sim_time:=true` 下 costmap 不更新，先检查：

```bash
ros2 topic echo /clock
ros2 topic hz /scan_cloud
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

## TB3 Manipulation Gazebo 仿真

`tb3_manipulation_sim_launch.py` 启动 TurtleBot3 Waffle Pi + OpenMANIPULATOR-X Gazebo 仿真。默认场景已切换到 AWS RoboMaker Small House World，并加载其 `turtlebot3_waffle_pi` 地图。TB3 是差速底盘，不是全向底盘，导航时应使用 `chassis_model:=diff`。

### 1. 启动 Gazebo 仿真

默认以无头模式启动 Gazebo server，并启动 RViz：

```bash
ros2 launch navflex_bringup tb3_manipulation_sim_launch.py
```

该启动文件会启动：

- `gzserver`
- `robot_state_publisher`
- `spawn_entity.py`
- `map_server`
- `lifecycle_manager_map_server`
- `odom_fake_localization.py`
- RViz
- ros2_control controller spawner

启动文件会自动把 AWS Small House 的资源目录加入 `GAZEBO_RESOURCE_PATH` 和 `GAZEBO_MODEL_PATH`，因此 Gazebo 能直接找到 `small_house.world` 和相关模型。

默认 TF 链路：

```text
map -> odom -> base_footprint -> base_link
```

`odom_fake_localization.py` 会根据 Gazebo `/odom` 发布 `map -> odom`，用于无 AMCL 的仿真测试。RViz 的 `2D Pose Estimate` 可通过 `/initialpose` 重置仿真定位。

### 2. 启动 Navflex 导航栈

另开终端：

```bash
source install/setup.bash
ros2 launch navflex_bringup navflex_bringup_launch.py chassis_model:=diff
```

因为 `use_sim_time` 默认已经是 `true`，通常不需要再显式传入。若要写清楚也可以：

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py \
  use_sim_time:=true \
  chassis_model:=diff
```

### 3. 查看 Gazebo 可视化

默认 `gui:=false`，也就是只启动 `gzserver`。有两种查看方式：

启动时直接打开 Gazebo GUI：

```bash
ros2 launch navflex_bringup tb3_manipulation_sim_launch.py gui:=true
```

如果已经以无头模式启动，可另开终端连接到已有仿真：

```bash
gzclient
```

### 4. 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `world` | `aws_robomaker_small_house_world/worlds/small_house.world` | Gazebo world |
| `map` | `aws_robomaker_small_house_world/maps/turtlebot3_waffle_pi/map.yaml` | map_server 加载的地图 |
| `verbose` | `true` | 是否输出 Gazebo server 详细日志 |
| `gazebo_master_uri` | `http://localhost:11345` | Gazebo master 地址 |
| `gui` | `false` | 是否启动 Gazebo GUI |
| `rviz` | `true` | 是否启动 RViz |
| `rviz_config` | `rviz/nav2_default_view.rviz` | RViz 配置 |
| `spawn_controllers` | `true` | 是否启动 ros2_control controllers |
| `use_sim_time` | `true` | 使用 Gazebo `/clock` |
| `x_pose` / `y_pose` / `yaw` | `0.0` | 初始位姿 |

TB3 差速参数摘要：

| 参数 | 值 |
| --- | --- |
| 驱动类型 | Differential drive |
| 轮距 | `0.287 m` |
| 轮半径 | `0.033 m` |
| 最大线速度 | `0.26 m/s` |
| 最大角速度 | `1.82 rad/s` |
| 导航半径 | `0.22 m` |

TB3 差速参数文件 `params/nav2_params_tb3_diff.yaml` 当前使用 `/scan` 作为 local/global costmap 的障碍观测源，传感器类型为 `LaserScan`。默认目标容差为：

| 参数 | 值 |
| --- | --- |
| `default_xy_goal_tolerance` | `0.08 m` |
| `default_yaw_goal_tolerance` | `0.2 rad` |

## Navflex 导航栈

启动：

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_sim_time` | `true` | 使用仿真时间 |
| `chassis_model` | `omni` | `omni` 或 `diff` |
| `params_file` | empty | 自定义参数文件；为空时按底盘模型选择 |
| `bt_params_file` | `navflex_bt_navigator/params/navflex_bt_navigator.yaml` | BT Navigator 参数 |
| `default_nav_to_pose_bt_xml` | `navflex_bt_navigator/behavior_trees/test_bt_navigator.xml` | 默认行为树 |
| `autostart` | `true` | 自动激活 lifecycle 节点 |
| `use_respawn` | `False` | 非 composition 模式下崩溃重启 `navflex_nav` |
| `use_composition` | `true` | 使用组件容器启动核心节点 |
| `container_name` | `navflex_container` | 组件容器名称 |
| `log_level` | `info` | 日志等级 |
| `use_route_server` | `False` | 是否启动 `nav2_route` |
| `graph_filepath` | `nav2_route/graphs/sample_graph.geojson` | route graph 文件 |
| `namespace` | empty | 顶层 namespace |

启动后包含：

- `navflex_nav`
- `bt_navigator`
- `velocity_smoother`
- `lifecycle_manager_navflex`
- 可选 `route_server`

lifecycle 激活顺序：

1. `navflex_nav`
2. `route_server`，仅 `use_route_server:=True`
3. `velocity_smoother`
4. `bt_navigator`

## GeoJSON 语义标注可视化

`navflex_bringup` 提供 `publish_geojson_marker_array.py`，可将 GeoJSON 中的点、线和区域发布为 RViz `MarkerArray`。默认文件为：

```text
navflex_bringup/maps/aws_small_house_semantics.geojson
```

启动：

```bash
ros2 run navflex_bringup publish_geojson_marker_array.py
```

RViz 中添加 `MarkerArray`，topic 选择：

```text
/geojson_marker_array
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `geojson_file` | `maps/aws_small_house_semantics.geojson` | GeoJSON 文件；不存在时回退到 `params/sample_graph.geojson` |
| `topic` | `/geojson_marker_array` | MarkerArray 发布话题 |
| `frame_id` | `map` | Marker 坐标系 |
| `publish_once` | `false` | 是否只发布一次 |
| `publish_period` | `1.0` | 循环发布周期，单位秒 |
| `line_width` | `0.08` | 线/区域边界宽度 |
| `node_scale` | `0.35` | 点标记尺寸 |
| `z` | `0.03` | 未显式提供 z 坐标时的高度 |
| `show_labels` | `true` | 是否显示文字标签 |

使用自定义语义图：

```bash
ros2 run navflex_bringup publish_geojson_marker_array.py --ros-args \
  -p geojson_file:=/absolute/path/to/semantics.geojson \
  -p frame_id:=map
```

## 行为树和 FollowPath 容差

Navflex 使用自定义 `NavflexExePathAction` 调用 `/follow_path`。`FollowPath.action` 支持：

- `xy_goal_tolerance`
- `yaw_goal_tolerance`

如果这两个字段都为 `0.0`，`navflex_nav` 会使用参数文件中的：

- `default_xy_goal_tolerance`
- `default_yaw_goal_tolerance`

控制器内部仍使用 `nav2_core::GoalChecker`。每个新的 FollowPath goal 会根据 action goal 的容差生成对应的 goal checker。同 controller、同 resolved 容差的新 path 会原地更新，不重启控制器；容差变化时才重建 goal checker 和 controller execution。

调试 `/follow_path` 时，日志会打印 path 起点和终点的 `(x, y, yaw)` 以及最终使用的容差，便于确认行为树传入的目标姿态和参数默认值是否符合预期。

## 常用检查命令

检查节点：

```bash
ros2 node list
ros2 lifecycle nodes
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

检查地图、传感器和 costmap：

```bash
ros2 topic hz /map
ros2 topic hz /scan
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
ros2 topic echo /geojson_marker_array --once
```

检查仿真时间：

```bash
ros2 topic echo /clock
ros2 param get /navflex_nav use_sim_time
ros2 param get /bt_navigator use_sim_time
```

发送导航目标可直接在 RViz 使用 `2D Goal Pose`。

## 常见问题

### `use_sim_time:=true` 时 costmap 不更新

先检查 `/clock` 是否存在：

```bash
ros2 topic echo /clock
```

本地轻量仿真由 `omni_fake_node` 发布 `/clock`；Gazebo 仿真由 Gazebo 发布 `/clock`。如果没有 `/clock`，ROS time 会停在 0，costmap、TF filter 和 BT timeout 都可能异常。

### `bt_navigator` 找不到 action server

确认 `navflex_nav` 已激活，并且 action server 存在：

```bash
ros2 lifecycle get /navflex_nav
ros2 action list | grep -E 'compute_path_to_pose|follow_path|behavior_action'
```

### TB3 导航效果像全向机器人

TB3 是差速底盘，启动导航时必须使用：

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py chassis_model:=diff
```

`diff` 参数会将 MPPI motion model 设置为 `DiffDrive`，并禁用横向速度。

### 已经无头启动 Gazebo，如何看画面

另开终端执行：

```bash
gzclient
```

或者下次启动时使用：

```bash
ros2 launch navflex_bringup tb3_manipulation_sim_launch.py gui:=true
```
