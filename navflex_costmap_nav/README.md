# navflex_costmap_nav

NavFlex 导航框架核心包，基于 Nav2 插件体系，提供规划（Planner）、控制（Controller）和行为（Behavior）三个 action server，并支持 nav2_route 路网导航。

## 构建

---

## 启动方式

启动操作请参见上级仓库 `ros2_nav/README.md` 中的“启动方式”章节。

---

## 路网导航（nav2_route）

### 工作原理

`nav2_route` 使用 **Dijkstra 算法**在预定义的拓扑图（GeoJSON 格式）上搜索最优路径，再通过 `FollowPath` 控制器执行：

1. 从 TF 获取机器人当前位置，通过最近邻 BFS 找到图中最近节点
2. Dijkstra 搜索最优节点-边序列，代价由 `DistanceScorer`（距离）和 `DynamicEdgesScorer`（动态障碍）计算
3. 将图搜索结果转换为稠密 `nav_msgs/Path`
4. 交给 `FollowPath` 控制器执行

### RViz 可视化

| Topic | 类型 | 说明 |
|-------|------|------|
| `/route_graph` | `visualization_msgs/MarkerArray` | 路网图（节点+边） |
| `/route_plan` | `nav_msgs/Path` | 最新规划路径 |

两个 topic 均使用 `TRANSIENT_LOCAL` QoS，在 RViz 中订阅时需将 **Durability** 设为 `Transient Local`。

### 交互式导航脚本

启动导航栈后，使用 `route_nav.py` 交互式输入目标点：

```bash
# 需先启动带路网服务器的导航栈
ros2 launch navflex_costmap_nav navigation_launch_test.launch.py use_route_server:=True

# 启动交互式导航客户端
ros2 run navflex_costmap_nav route_nav.py
```

输入格式：
```
输入目标点 (x y [yaw]) 或 q 退出: 2.0 3.0
输入目标点 (x y [yaw]) 或 q 退出: 5.0 1.0 1.57
输入目标点 (x y [yaw]) 或 q 退出: q
```

- `yaw` 单位为弧度，可选，默认 `0.0`
- 输入新目标时若机器人正在执行，自动取消旧任务
- 规划成功后路径发布至 `/route_plan` topic

---

## 测试 action

导航栈启动并激活后，可通过命令行发送 action 目标进行测试。

### Behavior action（行为）

```bash
# 直线行驶 1.0 m
ros2 action send_goal /behavior_action nav2_msgs/action/DummyBehavior \
  "{behavior: 'cmd_behavior', command: {data: 'linear 1.0'}}"

# 旋转 -1.0 rad（约 -57°）
ros2 action send_goal /behavior_action nav2_msgs/action/DummyBehavior \
  "{behavior: 'cmd_behavior', command: {data: 'rotate -1.0'}}"

# 原地等待 2.0 秒
ros2 action send_goal /behavior_action nav2_msgs/action/DummyBehavior \
  "{behavior: 'cmd_behavior', command: {data: 'wait 2.0'}}"
```

### FollowPath action（路径跟踪）

```bash
ros2 action send_goal /follow_path nav2_msgs/action/FollowPath \
  "{path: {header: {frame_id: 'map'}, poses: []}, controller_id: '', goal_checker_id: ''}"
```

### ComputePathToPose action（全局规划）

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}, planner_id: ''}"
```

### ComputeRoute action（路网规划）

```bash
ros2 action send_goal /compute_route nav2_msgs/action/ComputeRoute \
  '{use_poses: true, use_start: false, goal: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 3.0}, orientation: {w: 1.0}}}}'
```

---

## 包结构

```
navflex_costmap_nav/
├── include/navflex_costmap_nav/    # 头文件
├── src/
│   ├── navflex_costmap_nav/        # 主节点和三个服务器实现
│   │   ├── navflex_costmap_nav.cpp
│   │   ├── behavior_action_costmap_server.cpp
│   │   ├── controller_action_costmap_server.cpp
│   │   └── planner_action_costmap_server.cpp
│   ├── navflex_base/               # Action/Execution 基类实现
│   └── main.cpp
├── launch/                         # 启动文件
└── params/                         # 参数文件
```
