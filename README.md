# Traymover ROS2

Traymover ROS2 是基于 ROS 2 Humble 的机场行李托盘搬运机器人软件栈，运行于 NVIDIA Jetson Orin NX 平台。系统通过串口驱动 STM32 差速底盘，接入 HiPNUC N300Pro IMU 和 Leishen C32 激光雷达，支持底盘控制、键盘遥控、FAST-LIO 建图、PCD 地图保存和基于 Nav2 的自主导航。

主要启动入口是交互式脚本：

```bash
cd /home/wheeltec/traymover_ros2
bash scripts/traymover.sh
```

启动菜单中仅保留以下常用选项说明。

## 1. Start chassis

启动底盘基础链路，用于单独验证底盘、IMU、URDF 和 RViz 显示，不启动雷达、EKF 或 SLAM。

脚本内部执行：

```bash
ros2 launch turn_on_traymover_robot turn_on_traymover_robot.launch.py \
    use_lidar:=false use_ekf:=false
```

## 2. Start keyboard teleop

启动键盘遥控节点，向 `/cmd_vel` 发布速度指令。通常需要先运行选项 1，或其他已经启动底盘串口节点的选项。

脚本内部执行：

```bash
ros2 run traymover_robot_keyboard traymover_keyboard
```

## 6. Start FAST-LIO

启动 FAST-LIO 激光惯性里程计和 RViz，用于建图。该选项会启动底盘串口、IMU、雷达、FAST-LIO，以及在线地图过滤节点。FAST-LIO 不使用 STM32 轮式里程计，底盘串口只负责接收 `/cmd_vel` 驱动车体。

建图时可另开一次启动菜单选择选项 2，用键盘遥控机器人移动采图。

## 7. Save filtered FAST-LIO PCD map

保存选项 6 运行过程中生成的过滤后 PCD 地图。使用该选项前需要保持选项 6 正在运行，并让机器人移动到足够区域以积累地图点云。

脚本会调用 `/save_filtered_map` 服务，并将结果保存为 `.pcd` 文件。默认保存目录：

```text
src/traymover_robot_slam/FAST_LIO/PCD
```

## 10. Start navigation

启动自主导航流程。该选项会从已保存的 FAST-LIO PCD 地图中选择一个地图，自动重新生成匹配的 2D 栅格地图，然后启动底盘、雷达、Nav2 和可选 RViz。

使用前需要先通过选项 6 建图，并通过选项 7 保存至少一个 `.pcd` 地图。

启动后常规操作：

1. 在 RViz 中使用 `2D Pose Estimate` 设置机器人当前位置。
2. 等待定位收敛。
3. 使用 `2D Goal Pose` 发送导航目标。
