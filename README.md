# Traymover ROS2

Traymover ROS2 是基于 ROS 2 Humble 的机场行李托盘搬运机器人软件栈，运行于 NVIDIA Jetson Orin NX 平台。系统通过串口驱动 STM32 差速底盘，接入 HiPNUC N300Pro IMU 和 Leishen C32 激光雷达，支持底盘控制、键盘遥控、FAST-LIO 建图、PCD 地图保存、2D Nav2 导航、速度模式导航和 3D 点云导航实验。

主要启动入口是交互式脚本：

```bash
cd /home/wheeltec/traymover_ros2
bash scripts/traymover.sh
```

运行前先构建并加载环境：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动菜单

| 选项 | 功能 | 用途 |
| --- | --- | --- |
| `0` | Kill all traymover processes | 清理脚本启动过的底盘、雷达、FAST-LIO、Nav2、RViz 等进程。 |
| `1` | Start chassis | 启动底盘基础链路、描述、RViz、IMU 和 base serial，用于单独检查底盘和状态。 |
| `2` | Start keyboard teleop | 启动键盘遥控节点，向 `/cmd_vel` 发布速度。通常配合选项 `1`、`6`、`8` 使用。 |
| `3` | Start SLAM | 启动 slam_toolbox async 流程，包含底盘、雷达和 EKF。 |
| `4` | Save SLAM map | 保存 slam_toolbox 的 2D 栅格地图到 `src/traymover_robot_nav2/map`。 |
| `5` | Show battery | 从 `/battery_state` 读取电量、温度和充电状态，需要底盘节点已运行。 |
| `6` | Start FAST-LIO | 启动底盘串口、IMU、雷达、FAST-LIO、RViz 和在线 PCD 过滤节点。 |
| `7` | Save filtered FAST-LIO PCD map | 调用 `/save_filtered_map` 保存过滤后的 FAST-LIO PCD。 |
| `8` | Record FAST-LIO rosbag | 录制 `/point_cloud_raw`、`/imu/data_raw`、`/tf_static`、`/tf`，用于离线建图。 |
| `9` | Replay FAST-LIO rosbag offline | 用 `use_sim_time` 回放选项 `8` 的 rosbag，并离线跑 FAST-LIO。 |
| `10` | Start navigation | 当前默认慢速 2D Nav2 导航：选择 PCD，重建运行时 2D 地图，启动底盘、雷达、FAST-LIO+NDT、Nav2 和可选 RViz。 |
| `11` | Start teleop-only | 只启动底盘串口和键盘，不启动雷达、IMU、EKF。 |
| `12` | Clean FAST-LIO PCD | 对 PCD 做 SOR、悬浮柱、DBSCAN 等离线清理，输出 `_clean.pcd`。 |
| `13` | Intersect multiple FAST-LIO PCDs | 多次建图结果做体素多数投票，保留跨 session 稳定出现的点。 |
| `14` | Start 3D point-cloud navigation | 3D 点云导航实验：FAST-LIO + NDT + CMU `terrain_analysis/local_planner`，不走 2D PGM。 |
| `15` | Start navigation with speed mode | 选项 `10` 的扩展版，启动流程相同，但可选择 Nav2 速度配置。 |

## 常用流程

### 手动遥控

1. 选择 `11`，启动底盘串口和键盘。
2. 或选择 `1` 后再选择 `2`，用于同时查看描述和 RViz。

### FAST-LIO 在线建图

1. 选择 `6` 启动 FAST-LIO 建图链路。
2. 另开一次菜单选择 `2`，用键盘遥控机器人移动采图。
3. 地图充分覆盖后选择 `7` 保存过滤后的 PCD。

默认 PCD 保存目录：

```text
src/traymover_robot_slam/FAST_LIO/PCD
```

### FAST-LIO 离线建图

1. 选择 `8` 录制传感器 rosbag，移动机器人采集数据。
2. 选择 `0` 停止录制并让 rosbag 正常落盘。
3. 选择 `9` 回放 rosbag 并离线跑 FAST-LIO。
4. 选择 `7` 保存离线生成的过滤 PCD。

默认 rosbag 目录：

```text
src/traymover_robot_slam/FAST_LIO/rosbag
```

### 默认慢速 Nav2 导航

选择 `10`。脚本会：

1. 从 `src/traymover_robot_slam/FAST_LIO/PCD` 中选择一个 PCD。
2. 使用该 PCD 重新生成运行时 2D 栅格地图到 `/tmp/traymover_nav_runtime`。
3. 启动底盘串口、雷达、点云预处理、FAST-LIO、NDT 定位、Nav2 和可选 RViz。

启动后：

1. 在 RViz 使用 `2D Pose Estimate` 设置机器人当前位置。
2. 等待 NDT 定位收敛，RobotModel 和点云地图对齐。
3. 使用 `2D Goal Pose` 发送导航目标。

默认慢速 Nav2 配置文件：

```text
src/traymover_robot_nav/config/nav2_params.yaml
```

### 速度模式 Nav2 导航

选择 `15`。该选项不修改选项 `10` 的默认慢速配置，只是在同一套启动流程上增加速度模式选择：

| 速度模式 | Nav2 参数文件 | 速度变化 |
| --- | --- | --- |
| `0` | `nav2_params.yaml` | 与选项 `10` 完全相同的默认慢速导航。 |
| `1` | `nav2_params_linear_2x.yaml` | 线速度为当前慢速的 2 倍，原地转向速度不变。 |
| `2` | `nav2_params_linear_2_5x_turn_2x.yaml` | 线速度为当前慢速的 2.5 倍，原地转向速度为当前慢速的 2 倍。 |

当前慢速基准为：

```text
desired_linear_vel: 0.14
rotate_to_heading_angular_vel: 0.45
```

快速模式只通过 `params_file:=...` 切换 Nav2 配置；PCD 选择、2D 地图生成、底盘启动、雷达启动、定位链路和 RViz 操作都与选项 `10` 一致。

### 3D 点云导航实验

选择 `14`。该流程跳过 2D PGM 投影，使用 CMU 的 `terrain_analysis` 和 `local_planner` 直接基于 3D 点云导航，同时仍运行 FAST-LIO + NDT 来保持 map 坐标一致。该选项适合作为 2D Nav2 之外的实验路径。

## 关键配置

| 文件 | 说明 |
| --- | --- |
| `scripts/traymover.sh` | 主启动脚本和菜单入口。 |
| `src/traymover_robot_nav/launch/traymover_nav.launch.py` | 2D Nav2 导航主 launch，选项 `10` 和 `15` 使用。 |
| `src/traymover_robot_nav/config/nav2_params.yaml` | 选项 `10` 和选项 `15` 模式 `0` 的默认慢速 Nav2 参数。 |
| `src/traymover_robot_nav/config/nav2_params_linear_2x.yaml` | 选项 `15` 模式 `1` 的 2 倍线速度参数。 |
| `src/traymover_robot_nav/config/nav2_params_linear_2_5x_turn_2x.yaml` | 选项 `15` 模式 `2` 的 2.5 倍线速度、2 倍转向速度参数。 |
| `src/traymover_robot_nav/config/localization.yaml` | FAST-LIO + NDT 定位参数，包括 `ndt_align_interval_s`。 |
| `src/traymover_robot_nav/launch/traymover_3d_nav.launch.py` | 选项 `14` 的 3D 点云导航 launch。 |

## 快速检查命令

导航启动后可在新终端中检查：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/traymover_ros2/install/setup.bash
ros2 topic info /odom -v
./scripts/check_localization.sh
ros2 action list | grep navigate_to_pose
ros2 topic echo --once /cmd_vel
```

代码或配置修改后建议运行：

```bash
bash -n scripts/traymover.sh
colcon build --packages-select traymover_robot_nav --symlink-install
python3 -m pytest src/traymover_robot_nav/test/test_nav_simple_config.py
colcon test --packages-select traymover_robot_nav
colcon test-result --verbose --test-result-base build/traymover_robot_nav/test_results
```
