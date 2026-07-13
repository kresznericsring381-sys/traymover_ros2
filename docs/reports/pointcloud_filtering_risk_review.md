# Traymover 点云滤除体系风险审查

## 结论

当前点云滤除体系可以生成一个经过后处理的 PCD，但还不能认为它已经形成了可靠的实时动态物体滤除闭环。最严重的问题是建图选项 6 中，在线滤波器监听的 `/odom` 与 FAST-LIO 实际发布的 `/Odometry` 不是同一条链路；此时滤波器可能把底盘轮速坐标应用到 `camera_init` 点云上。

此外，在线节点只影响保存的过滤地图，不影响 FAST-LIO 的输入和内部地图，也不直接影响实时定位或 3D 导航点云。因此，滤除后的 PCD 变干净，并不代表 FAST-LIO 里程计、NDT 实时输入和动态避障已经被净化。

## 检查范围

- `src/traymover_robot_nav/scripts/fastlio_online_map_filter.py`
- `src/traymover_robot_nav/scripts/pointcloud_nav_preprocessor.py`
- `src/traymover_robot_nav/scripts/pcd_clean.py`
- `src/traymover_robot_nav/scripts/pcd_intersect.py`
- `scripts/traymover.sh`
- FAST-LIO 的 `laserMapping.cpp`、`preprocess.cpp` 和 `traymover.yaml`
- 导航和定位 launch 文件

## 高风险问题

### 1. 选项 6 的在线后方滤除可能使用了错误的位姿

`fastlio_online_map_filter.py` 默认订阅 `/odom`，`traymover.sh` 的选项 6 和选项 9 也显式传入 `/odom`：

- `fastlio_online_map_filter.py:321`
- `scripts/traymover.sh:269-273`
- `scripts/traymover.sh:325-333`

但 FAST-LIO 源码发布的是 `/Odometry`，并且消息位姿的父坐标系是 `camera_init`、子坐标系是 `body`：

- `FAST_LIO/src/laserMapping.cpp:636-642`
- `FAST_LIO/src/laserMapping.cpp:938-944`

选项 6 启动的底盘节点默认使用 STM32 编码器发布 `/odom`。该 `/odom` 的父坐标系是 `odom`，子坐标系是 `base_footprint`：

- `turn_on_traymover_robot.py:480`
- `turn_on_traymover_robot.py:759-768`
- `config/traymover_robot.yaml:8-15`

选项 6 本身没有把 `/Odometry` remap 到 `/odom`，也没有建立 `odom -> camera_init` 的静态变换。因此，滤波器的计算：

```python
points_body = (points_world - pose.translation) @ pose.rotation_body_to_world
```

可能是在用 `odom` 轮速坐标解释 `camera_init` 点云。因为 `/odom` 有消息，代码不会进入“没有位姿”的降级路径，而是可能在错误位置删除点。

导航用的 `lidar_localization.launch.py` 确实配置了 `/Odometry -> /odom` remap，但这只覆盖导航 launch，不覆盖 `traymover.sh` 的建图选项：

- `src/traymover_robot_nav/launch/lidar_localization.launch.py:83-101`

### 2. 回放模式可能完全关闭后方滤除

选项 8 录制的 topic 列表包含点云、IMU 和 TF，但不包含 `/odom` 或 `/Odometry`：

- `scripts/traymover.sh:341-350`

选项 9 启动 FAST-LIO 时没有给 `/Odometry` 做 remap，过滤器仍然监听 `/odom`。因此回放时很可能收不到位姿。代码在没有位姿或位姿超时后直接返回原始点云：

- `fastlio_online_map_filter.py:170-173`
- `fastlio_online_map_filter.py:418-434`

结果是实时建图和离线回放的后方过滤行为不一致。

### 3. 稳定 voxel 一旦确认不会撤销

默认参数是 `voxel_size=0.15`、`min_observations=3`、`min_observation_span_sec=0.25`：

- `fastlio_online_map_filter.py:324-327`

稳定判定只要求同一 voxel 在多个扫描中出现，并没有要求多个独立机器人位姿或有效的自由空间证据：

- `fastlio_online_map_filter.py:282-286`

`candidate_ttl_sec` 只清理尚未稳定的候选 voxel。已经稳定的 voxel 不会被清除：

- `fastlio_online_map_filter.py:249-260`

所以一个站立超过 0.25 秒的人、慢速移动的设备，或者在错误位姿下重复出现的动态点，都可能永久进入地图。相反，只被短暂看到的真实墙面、细杆或远处结构也可能被丢弃。

### 4. 在线过滤不能修复 FAST-LIO 已经受到的动态干扰

在线过滤器订阅 `/cloud_registered` 后建立自己的地图，不修改 FAST-LIO 的输入、scan matching 或内部 ikd-tree：

- `fastlio_online_map_filter.py:2-14`
- `scripts/traymover.sh:254-256`

因此动态人员仍可能影响 FAST-LIO 位姿。若位姿已经被动态物体拉偏，后续 Python 过滤只能删除部分点，不能恢复静态结构的正确位置。

同时，实时消费链路没有统一使用过滤地图：

- 2D 定位使用 `/point_cloud_localization`，它只是从原始点云剔除一个后方矩形。
- 3D 导航的 `terrain_analysis` 直接使用 `/cloud_registered`。
- `fastlio_online_map_filter.py` 的输出主要用于保存 PCD。

对应代码：

- `pointcloud_nav_preprocessor.py:81-115`
- `traymover_nav.launch.py:117-140`
- `traymover_3d_nav.launch.py:161-170`

## 中风险问题

### 5. 无位姿或位姿过期时是“放行全部点”

`apply_body_box_exclusion()` 在没有位姿、或点云和最新位姿时间差超过阈值时，直接返回原始点云：

- `fastlio_online_map_filter.py:170-173`

这属于 fail-open 行为。它避免了错误删除静态结构，但会把尾随人员和车体后方设备写入地图。过期位姿没有单独的明确错误状态，也没有统计“多少帧实际启用了后方滤除”。

### 6. 后方矩形参数在不同链路中不一致

在线建图默认后方框为：

```text
x=[-1.25,-0.35], |y|<=0.90, z=[-0.50,2.00]
```

导航预处理 launch 又覆盖成：

```text
x=[-0.95,-0.20], |y|<=0.65, z=[-0.50,2.00]
```

对应代码：

- `fastlio_online_map_filter.py:330-335`
- `navigation_pointcloud.launch.py:45-53`

同一人员在建图 PCD 和实时定位点云中可能被不同程度地删除。矩形还可能误删靠近机器人后方的墙、柱或真实障碍物。

### 7. 位姿没有按点云时间同步，也没有校验坐标系

过滤器只保存最新一条 Odometry，没有按点云时间插值，也没有验证 Odometry 的 `header.frame_id` 是否与点云的 `header.frame_id` 一致：

- `fastlio_online_map_filter.py:368-406`
- `fastlio_online_map_filter.py:415-434`

高速运动、时间戳来源不同、TF 延迟或错误 remap 都可能把后方框偏移到错误位置。

### 8. Python 累积地图可能造成性能和内存压力

每帧会执行 voxel 坐标唯一化，并对每个 voxel 进行 Python 字典和 dataclass 操作：

- `fastlio_online_map_filter.py:201-247`

地图中的 stable voxel 只增不减，发布时还会周期性扫描整个字典：

- `fastlio_online_map_filter.py:262-280`
- `fastlio_online_map_filter.py:447-455`

在长时间、大范围建图中，CPU、内存和回调延迟可能持续增长，最终造成点云丢帧或过滤器滞后。

## 离线工具风险

### 9. 默认离线清洗不是动态物体清洗

菜单默认选择 `light`，而 `--light` 只启用温和 SOR：

- `scripts/traymover.sh:826-835`
- `pcd_clean.py:117-119`

SOR 适合去孤立噪声，不保证删除密集的人体鬼影。`floating` 和 DBSCAN 是基于高度、包围盒、点数和与静态簇距离的启发式判断，可能漏删贴墙人员，也可能误删吊牌、标识牌和悬挂设备：

- `pcd_clean.py:356-369`
- `pcd_clean.py:439-458`
- `pcd_clean.py:516-535`

### 10. 多会话交集强依赖初始对齐

`pcd_intersect.py` 要求多个 FAST-LIO PCD 在相同坐标系、相同起始位置和朝向下生成，否则 voxel 多数投票会把静态结构也当成少数点删除：

- `pcd_intersect.py:20-32`

输出被重建成 voxel 中心点，可能损失原始表面几何细节：

- `pcd_intersect.py:95-102`

### 11. `pcd_intersect.py` 可能输出缺少 intensity 的 PCD

`pcd_clean.py` 明确说明下游按 `pcl::PointXYZI` 读取，缺少 `intensity` 字段可能造成不兼容：

- `pcd_clean.py:173-177`

但 `pcd_intersect.py` 用 Open3D 直接写只包含 XYZ 的点云：

- `pcd_intersect.py:95-100`

交集输出应在进入定位前检查 PCD header 是否包含 `FIELDS x y z intensity`，否则可能出现读取异常或未初始化强度字段。

## 优先修复建议

1. 统一 FAST-LIO 位姿话题。建图选项 6 应直接使用 `/Odometry`，或在 FAST-LIO 启动节点上增加 `/Odometry -> /fastlio_odom` remap，再让过滤器订阅该唯一话题。不要让底盘轮速 `/odom` 与 FAST-LIO 位姿共用同一名称。
2. 在过滤器中检查 `cloud.header.frame_id`、`odom.header.frame_id`、`odom.child_frame_id`，不匹配时拒绝应用后方框，并输出明确错误计数。
3. 回放包录制 FAST-LIO 的 `/Odometry`，或者回放时显式 remap 后再启动过滤器。
4. 将“每帧重复命中”改成基于多视角、时间窗口和自由空间证据的动态判别；至少要让已经稳定的 voxel 具备局部时间窗口内的撤销机制。
5. 明确过滤目标：如果目标是改善 FAST-LIO/NDT 实时定位，过滤必须进入估计器输入或在估计器内部处理；只过滤最终 PCD 无法修复实时位姿。
6. 把后方框、voxel、观察次数和时间阈值放入统一 YAML，避免建图和导航使用两套硬编码参数。
7. 修改 `pcd_intersect.py` 输出 XYZI，并对 PCD header、点数、坐标范围和地图重叠率做保存前检查。

## 建议的现场验收

启动建图后至少检查：

```bash
ros2 topic info /odom -v
ros2 topic info /Odometry -v
ros2 topic echo --once /cloud_registered
ros2 topic echo --once /Odometry
```

同时记录过滤器日志中的：总扫描帧数、收到有效位姿的帧数、启用后方框的帧数、删除点数、候选 voxel 数和 stable voxel 数。应分别验证机器人静止、直行、原地旋转、人员尾随和人员横穿五种场景。

本报告是基于当前仓库源码和启动脚本的静态审查，不代表已经完成实机点云质量或定位精度验收。
