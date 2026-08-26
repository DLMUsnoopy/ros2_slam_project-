# P03 自主导航整合

这个包只负责 launch 和参数，把现有算法包通过 topic 与 TF 串起来；没有把多个 ROS 包合并成一个包，也没有修改原始同事包。

## 数据链路

```text
ipb_ros2_sim
  ├─ LaserScan / IMU / JointState
  v
p03_slam_ros ──> /P1_robot0/map + TF(map -> odom -> base_link)
  │
  ├─> p03_frontier_exploration ──> /P1_robot0/goal_pose
  │                                      │
  └──────────────────────────────────────v
                              p03_rviz_route_planner
                                      │
                                      v /planned_route_poses
p03_obstacle_detector ─────────> p03_nmpc_controller
                                      │
                                      v /P1_robot0/cmd_vel
                                  ipb_ros2_sim
```

所有用于规划和控制的位置、路线、障碍物统一在 `map` frame。机器人位置统一从 TF `map -> P1_robot0/base_link` 获得，不再依赖 `odom_gt` 或 `esmt_pose`。

## 编译

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-select \
    p03_slam_core p03_slam_ros ipb_ros2_sim \
    p03_rviz_route_planner p03_obstacle_detector \
    p03_nmpc_controller p03_frontier_exploration \
    p03_autonomy_bringup \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -Dcasadi_DIR=/home/johnny/.local/share/p03-casadi-cpp/lib/cmake/casadi
source install/setup.bash
```

`p03_nmpc_controller` 会把所选 CasADi 库目录写入可执行文件的 RUNPATH，因此本机启动时不再需要手动设置 `LD_LIBRARY_PATH`。

## 模式一：手动目标导航

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch p03_autonomy_bringup manual_navigation.launch.py
```

该模式默认启动仿真、SLAM、A*、障碍物检测和 NMPC，但不启动 Frontier。在 RViz 中用 `2D Goal Pose` 向 `/P1_robot0/goal_pose` 发布目标。

不用 RViz 时：

```bash
ros2 launch p03_autonomy_bringup manual_navigation.launch.py rviz:=false
```

连接已经单独启动的仿真和 SLAM：

```bash
ros2 launch p03_autonomy_bringup manual_navigation.launch.py \
  start_sim:=false start_slam:=false
```

命令行发布一个测试目标：

```bash
ros2 topic pub --once /P1_robot0/goal_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
```

## 模式二：自动 Frontier 探索

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch p03_autonomy_bringup autonomous_exploration.launch.py
```

该模式会额外启动 Frontier。不要同时用 RViz 手动发布目标，否则两个目标源会争抢 `/P1_robot0/goal_pose`。Frontier 等待 NMPC 在 `/P1_robot0/goal_reached` 上报告完成后，才选择下一个目标。

## 验收命令

每条命令开一个新终端，并先执行两条 `source`：

```bash
ros2 topic hz /P1_robot0/map
ros2 run tf2_ros tf2_echo map P1_robot0/base_link
ros2 topic hz /P1_robot0/lidar_map_data
ros2 topic echo --once /planned_route_poses --field header.frame_id
ros2 topic echo /P1_robot0/cmd_vel
ros2 topic echo /P1_robot0/goal_reached
```

预期结果：地图、路线和障碍物数据的 frame 都是 `map`；TF 可查询；运动时 `cmd_vel` 非零；到达后速度归零且 `goal_reached.data` 为 `true`。

## 只提交副本和 bringup

先确认在 `integration` 分支：

```bash
git branch --show-current
git status --short
```

只暂存副本包和新 bringup 包：

```bash
git add \
  src/controller/p03_nmpc_controller \
  src/controller/p03_obstacle_detector \
  src/p03_frontier_exploration \
  src/p03_rviz_route_planner \
  src/p03_autonomy_bringup
git diff --cached --name-only
git commit -m "integrate copied autonomy packages"
git push -u origin integration
```

提交前必须检查 `git diff --cached --name-only`，确保其中没有原始包、`p03_slam_core` 或 `p03_slam_ros`。
