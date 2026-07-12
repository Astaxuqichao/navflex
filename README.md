# navflex

Navflex 是基于 ROS 2 Humble 和 Nav2 的导航扩展包集合。它把 costmap、规划、控制、BT 行为树、自定义恢复行为、本地仿真和 TB3 manipulation Gazebo 仿真整理成一套可直接启动的导航工作区。

更详细的启动参数和排查命令见 [navflex_bringup/README.md](navflex_bringup/README.md)。

## 包功能

| 包名 | 功能 |
| --- | --- |
| `navflex_bringup` | 统一启动包，维护 launch、地图、参数、RViz 配置；提供本地仿真、TB3 manipulation Gazebo 仿真和 Navflex 导航栈启动入口 |
| `navflex_nav` | 核心导航服务端；在一个 lifecycle 节点内管理 global/local costmap，并提供 `/compute_path_to_pose`、`/follow_path`、`/behavior_action` |
| `navflex_bt_navigator` | BT Navigator 封装包，提供 Navflex 默认行为树 XML 和 BT Navigator 参数 |
| `navflex_bt_nodes` | 自定义 BT 节点，包括 GetPath、ExePath、Recovery；`NavflexExePathAction` 支持运行中路径更新和 FollowPath 容差传递 |
| `navflex_cmdbehavior` | Nav2 behavior 插件，提供 `rotate`、`linear`、`wait` 等简单恢复/动作命令 |
| `navflex_exclusion_zone` | Nav2 costmap 电子禁区层插件，可通过话题动态标记禁行区域 |
| `navflex_instruction_server` | 文本指令、语义地图、任务服务和 VLN/VLM 桥接入口 |
| `navflex_utility` | 公共工具库，提供机器人状态、TF/odom 查询和导航辅助函数 |
| `omni_fake_node` | 本地全向虚拟机器人，订阅 `/cmd_vel`，发布 `/odom`、TF 和本地仿真 `/clock` |
| `simulation_lidar` | 基于 OccupancyGrid 地图射线投射的 2D 仿真激光雷达，发布 `/scan` 和 `/scan_cloud` |
| `rcl_logging_spdlog_rotating` | 自定义 ROS 2 spdlog 滚动日志后端 |

## navflex_nav 设计

`navflex_nav` 是 Navflex 的核心导航执行层。它没有直接复用 Nav2 的整套
planner/controller/behavior server 进程组合，而是在一个 `CostmapNavNode`
lifecycle 节点里统一管理 costmap、插件服务器和 action 执行，从而让 Navflex 可以在
同一套生命周期下协调规划、控制、行为恢复和代价地图查询。

顶层结构如下：

```text
CostmapNavNode (nav2_util::LifecycleNode)
├── global_costmap  -> PlannerCostmapServer    -> nav2_core::GlobalPlanner
├── local_costmap   -> ControllerCostmapServer -> nav2_core::Controller
└── global/local    -> BehaviorCostmapServer   -> nav2_core::Behavior
```

核心设计点：

- **统一生命周期**：`CostmapNavNode` 由 `lifecycle_manager` 驱动
  `configure -> activate -> deactivate -> cleanup`，在各阶段统一创建、激活、
  停止和释放 global/local costmap、Planner、Controller、Behavior server。
- **复用 Nav2 插件体系**：全局规划、路径跟踪和行为恢复仍通过
  `nav2_core::GlobalPlanner`、`nav2_core::Controller`、`nav2_core::Behavior`
  插件接口扩展，Navflex 主要负责执行编排和 action 封装。
- **分离 costmap 与执行线程**：global/local costmap 各自运行在独立
  `NodeThread` 中；三个 server 也分别运行在独立 `NodeThread` 中，避免代价地图更新、
  action 接收和长时间执行互相阻塞。
- **Action/Execution 双层抽象**：`NavflexActionBase<Action, Execution>` 负责
  action goal、cancel、并发槽位和结果回传；具体 `PlannerExecution`、
  `ControllerExecution`、`BehaviorExecution` 负责调用插件完成实际任务。
- **并发槽位模型**：action goal 可通过 `concurrency_slot` 分配到 0-255 号槽位。
  同一槽位的新任务会替换旧任务，不同槽位可并行执行，便于后续扩展多任务或分组控制。
- **控制层运行时能力**：`/follow_path` 支持运行中路径更新、到点容差透传，以及卡困检测；
  卡困时会以明确结果码 abort，并在结果消息中给出位移、耗时和阈值。
- **代价地图查询服务**：节点激活后提供 `check_point_cost`、`check_pose_cost`、
  `check_path_cost`，可直接查询点、位姿或路径在 global/local costmap 中的通行状态。
- **路网导航扩展**：可选接入 `nav2_route`，在拓扑图上规划路网路径，再转成
  `nav_msgs/Path` 交给 `FollowPath` 执行。

更完整的类关系、线程模型和回调链见
[navflex_nav/docs/ARCHITECTURE.md](navflex_nav/docs/ARCHITECTURE.md)。

## 获取源码

Navflex 依赖修改过接口的 Nav2，因此必须使用
[`Astaxuqichao/navigation2`](https://github.com/Astaxuqichao/navigation2) 的
`humble` 分支。不要使用 `ros-humble-navigation2` 替代该源码仓库，否则扩展后的
`nav2_msgs/action/FollowPath.action` 会与 Navflex 不兼容。

在一个空的 ROS 2 工作区根目录中拉取三个并列仓库：

```bash
mkdir -p ~/ros2/nav_ws
cd ~/ros2/nav_ws

git clone --branch humble --single-branch \
  https://github.com/Astaxuqichao/navigation2.git navigation2
git clone --branch humble --single-branch \
  https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git \
  spatio_temporal_voxel_layer
git clone https://github.com/Astaxuqichao/navflex.git navflex
```

完成后的必要目录结构为：

```text
nav_ws/
├── navigation2/                   Astaxuqichao 的 Nav2 Humble 分支
├── spatio_temporal_voxel_layer/   开源三维时空体素障碍层
└── navflex/                       Navflex
```

已有工作区可用以下命令检查来源与分支：

```bash
git -C navigation2 remote get-url origin
git -C navigation2 branch --show-current
git -C spatio_temporal_voxel_layer remote get-url origin
git -C spatio_temporal_voxel_layer branch --show-current
```

预期两个分支均为 `humble`，仓库地址分别为：

```text
https://github.com/Astaxuqichao/navigation2.git
https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git
```

## 主要依赖

- ROS 2 Humble
- [`Astaxuqichao/navigation2`](https://github.com/Astaxuqichao/navigation2)，`humble` 分支
- [`SteveMacenski/spatio_temporal_voxel_layer`](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)，`humble` 分支
- TurtleBot3 / TurtleBot3 Manipulation / TurtleBot3 Simulations
- Gazebo Classic 11
- `ros2_control`、`controller_manager`、`xacro`

从新环境准备依赖时，在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths \
  navigation2 spatio_temporal_voxel_layer navflex \
  --ignore-src -r -y --rosdistro humble
```

## 编译

```bash
cd ~/ros2/nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果改过 `navigation2/nav2_msgs/action/FollowPath.action`，需要重编依赖该 action 的包，至少包括：

```bash
colcon build --packages-select \
  nav2_msgs nav2_behavior_tree nav2_bt_navigator \
  navflex_nav navflex_bt_nodes navflex_bt_navigator \
  navflex_bringup
source install/setup.bash
```

## 本地全向仿真快速启动

终端 1，启动本地仿真环境：

```bash
ros2 launch navflex_bringup sim_local_launch.py
```

终端 2，启动 Navflex 导航栈：

```bash
source install/setup.bash
ros2 launch navflex_bringup navflex_bringup_launch.py
```

默认导航参数为 `chassis_model:=omni`，适配 `omni_fake_node`。

## TB3 Manipulation 移动操作仿真

TB3 manipulation 是差速底盘，不是全向底盘。启动导航时必须使用 `chassis_model:=diff`。

终端 1，启动 TB3 manipulation Gazebo 仿真：

```bash
ros2 launch navflex_bringup tb3_manipulation_sim_launch.py
```

默认是无头 Gazebo server，并启动 RViz。需要 Gazebo GUI 时有两种方式：

```bash
ros2 launch navflex_bringup tb3_manipulation_sim_launch.py gui:=true
```

或者在无头仿真已经启动后，另开终端连接：

```bash
gzclient
```

终端 2，启动 Navflex 导航栈并选择 TB3 差速参数：

```bash
source install/setup.bash
ros2 launch navflex_bringup navflex_bringup_launch.py chassis_model:=diff
```

启动后可在 RViz 使用 `2D Goal Pose` 发送导航目标。

常用检查：

```bash
ros2 topic echo /clock
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

## FollowPath 到点容差

Navflex 扩展了 `nav2_msgs/action/FollowPath`：

- `xy_goal_tolerance`
- `yaw_goal_tolerance`

两个值都为 `0.0` 时，`navflex_nav` 使用参数文件中的默认到点精度。控制器内部仍通过 `nav2_core::GoalChecker` 判断是否到点；每个新的 FollowPath goal 会按 action 容差生成对应的 goal checker。

## 文档入口

- Bringup、依赖、TB3 仿真和排查：[navflex_bringup/README.md](navflex_bringup/README.md)
- 核心导航服务端：[navflex_nav/README.md](navflex_nav/README.md)
- 文本/语义任务入口：[navflex_instruction_server/README.md](navflex_instruction_server/README.md)
- 本地全向假机器人：[omni_fake_node/README.md](omni_fake_node/README.md)

## License

Apache-2.0
