# FAST_LIO Debug Handoff

Updated: 2026-09-08

## Scope

Continue debugging FAST_LIO before changing NDT/Nav2. The failure reproduces with FAST_LIO alone, so NDT is not the current blocker.

Pipeline:

```text
/point_cloud_raw + /imu/data_raw -> FAST_LIO -> /Odometry, TF camera_init->body,
                                         /cloud_registered, /Laser_map, /path
```

## Workspace And Test

Remote workspace: `/home/traymover_ros2`

Bag:

```text
/home/data/rosbags/traymover_20260805_022110_pnm
```

Independent test command:

```bash
cd /home/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/test_fastlio_replay.sh /home/data/rosbags/traymover_20260805_022110_pnm 300
```

The independent test starts only rosbag, FAST_LIO, diagnostics, and RViz. RViz Fixed Frame is `camera_init`, so it does not require NDT's `map->odom`.

Logs are written to:

```text
~/.ros/traymover_fastlio_replay/<run-id>/
```

Important files:

```text
bag_timing_audit.txt
fastlio/fastlio_events.log
fastlio/fastlio_rosout.log
fastlio/fastlio_metrics.csv
processes/fastlio.log
ros/fastlio_mapping_*.log
rosbag.log
fastlio_test_summary.txt
```

Latest locally synced logs:

```text
/Users/fuyao/Desktop/ros/logs/fast_alone
```

## Latest Findings

ROS1 conversion playback being healthy does **not** by itself clear the ROS2
bag/playback path. The conversion rewrites the dataset through a different bag
container, serialization path, and playback scheduler. Treat this as three
separate questions:

```text
rosbag2 record timestamp  -> does ROS2 bag playback schedule at the right rate?
message header.stamp      -> does FAST_LIO see a continuous sensor time axis?
PointCloud2 time/t field  -> does FAST_LIO undistort each scan with the right unit?
```

`scripts/test_fastlio_replay.sh` now writes `bag_timing_audit.txt` before
playback. This file is the first artifact to inspect when ROS2 `ros2 topic hz`
shows 1-5 Hz but ROS1 converted playback appears normal.

For large `PointCloud2` topics, `ros2 topic hz` can itself become a misleading
measurement because it deserializes every message in Python. Compare it against
the raw serialized probe:

```bash
python3 scripts/ros2_raw_topic_rate.py /point_cloud_raw --qos sensor_data
python3 scripts/ros2_raw_topic_rate.py /point_cloud_raw --qos reliable
```

Remote testing showed `--qos reliable` receives `/point_cloud_raw` at about
20 Hz, while `--qos sensor_data` receives the same bag at about 1-5 Hz with
multi-second gaps. FAST_LIO's PointCloud2 subscription used `SensorDataQoS`, so
rosbag2 replay starved FAST_LIO even though a reliable subscriber could receive
the LiDAR stream normally. Offline replay now enables reliable LiDAR QoS in
FAST_LIO and plays the bag with an explicit reliable QoS override:

```bash
ros2 bag play /home/data/rosbags/traymover_20260805_022110_pnm \
  --clock \
  --topics /point_cloud_raw /imu/data_raw /tf_static \
  --qos-profile-overrides-path scripts/qos/fastlio_bag_play.yaml
```

1. IMU input is stable at about 100 Hz.
2. Bag metadata reports 17009 LiDAR messages over 853.44 s, nominally about 19.9 Hz.
3. Offline bag audit reports `/point_cloud_raw` header timestamps are stable at
   about 19.94 Hz with no gaps above 0.15 s.
4. PointCloud2 has fields `x/y/z/intensity/ring/time`; sampled point `time`
   ranges up to about 0.050 s, so `timestamp_unit=0` is correct.
5. Runtime `stamp_gap` values seen under the old best-effort replay path were
   delivery gaps between received messages, not proof that the bag's LiDAR
   header time axis was broken. Confirmed old runtime gaps included:

```text
stamp_gap=0.803954
stamp_gap=2.160058
stamp_gap=1.711749
stamp_gap=1.358505
stamp_gap=1.607974
```

6. During stable sections diagnostics observe 19-20 Hz LiDAR reception, but FAST_LIO costs around 0.20 s per processed frame. It therefore cannot keep up with 20 Hz input; `buffer_lidar` reaches about 3-4.
7. FAST_LIO loses registration around frame 13:

```text
frame 10: effective_features=1102
frame 13: effective_features=86
frame 15: effective_features=61
frame 17: effective_features=0
```

8. It publishes only about 9 valid odometry messages in the latest test. After that, diagnostics show `STALE_ODOM` while LiDAR and IMU continue.
9. The current guard prevents runaway map insertion, but state prediction still diverges. Example rejected-pose values eventually reach hundreds of meters.
10. Latest standalone process exits with a segmentation fault after frame 39, last visible stage:

```text
FAST_LIO frame=39 stage=AFTER_IMU
[ros2run]: Segmentation fault
```

The latest reliable-QoS replay changed the failure mode: Fast-LIO receives
about 20 Hz LiDAR and 100 Hz IMU, but its processing cost is roughly 0.22 s for
IMU undistortion plus 0.05-0.47 s for EKF/map work. The single-threaded node
therefore cannot consume a 20 Hz replay. Its subscription queue drops/overruns
old scans, visible as processed `stamp_gap` values of 0.3-0.75 s, followed by
registration divergence and `LOST`. This is a throughput/backlog problem after
QoS, not a broken LiDAR header time axis.

`test_fastlio_replay.sh` now defaults to `--rate 0.2` and
`point_filter_num=2` for the embedded target. Override them for experiments:

```bash
TRAYMOVER_FASTLIO_REPLAY_RATE=0.1 \
TRAYMOVER_FASTLIO_REPLAY_POINT_FILTER_NUM=4 \
./scripts/test_fastlio_replay.sh <bag_dir>
```

## Current Safeguards

Replay defaults currently use:

```text
diagnostics.min_effective_features=1000
diagnostics.max_update_translation=2.0
```

On rejection, `laserMapping.cpp` saves EKF state/covariance before the LiDAR update, restores them, skips odometry publication, and skips KD-tree insertion.

This stops map pollution. It does not stop IMU prediction from becoming invalid after repeated rejected frames. A real `LOST` state is still required.

## Existing Relevant Changes

Recent commits:

```text
fc7c338 fix: prevent fastlio stale scan processing
d925334 fix: reject divergent fastlio replay updates
1e44fe4 test: add standalone fastlio replay visualization
74e0c3f feat: trace fastlio lidar delivery gaps
```

Relevant files:

```text
src/traymover_robot_slam/FAST_LIO/src/laserMapping.cpp
src/traymover_robot_slam/FAST_LIO/src/IMU_Processing.hpp
src/traymover_robot_slam/FAST_LIO/config/traymover.yaml
scripts/fastlio_diagnostics.py
scripts/test_fastlio_replay.sh
scripts/rviz/fastlio_replay.rviz
```

Implemented diagnostics:

```text
FAST_LIO frame=... stage=BEGIN/AFTER_IMU/AFTER_DOWNSAMPLE/BEFORE_EKF/
AFTER_EKF/BEFORE_KDTREE/AFTER_KDTREE/REJECT_UPDATE
```

LiDAR input logs include `stamp_gap`, `wall_gap`, point count, `buffer_lidar`, and `buffer_imu`. `fastlio_diagnostics.py` writes `LIDAR_GAP` for gaps greater than 0.15 s and now accepts actual node name `laser_mapping` for rosout capture.

## Priority Plan

### 1. Locate The Segmentation Fault

Build with debug symbols and run the standalone executable under gdb. Enable core dumps if gdb is impractical:

```bash
ulimit -c unlimited
gdb --args ros2 run fast_lio fastlio_mapping
```

Inspect `map_incremental()`, ikd-tree rebuild/delete paths, point-vector sizes, and interaction between `REJECT_UPDATE` and the next frame. Verify `feats_down_size`, `effct_feat_num`, `Nearest_Points`, and `pointSearchInd_surf` on the crashing frame.

### 2. Add A Real LOST State

Add parameters such as `diagnostics.max_consecutive_rejects` and `diagnostics.reset_on_lost`.

Suggested behavior after 5 consecutive rejects:

1. Stop publishing odometry and registered clouds.
2. Clear queued stale LiDAR scans.
3. Stop adding points to the map.
4. Reset velocity and/or reinitialize IMU state safely.
5. Log `LOST` and wait for an explicit reset or a defined reinitialization condition.

Do not only restore the update state. `p_imu->Process()` already advances prediction before the LiDAR update.

### 3. Verify Bag Time Axis Offline

Read the SQLite bag and calculate consecutive `/point_cloud_raw` timestamp gaps. Compare bag record timestamps with `PointCloud2.header.stamp`; count gaps above 0.15 s and backward timestamps. `ros2 bag info` average rate is insufficient evidence of stable timing.

### 4. Reduce Per-Frame Load One Variable At A Time

Test each independently:

```text
point_filter_num=2
point_filter_num=4
publish.dense_publish_en=false
publish.map_en=false
publish.scan_publish_en=false
publish.path_en=false
```

Observe `AFTER_IMU`, `AFTER_EKF`, `buffer_lidar`, output odometry rate, and feature count.

### 5. Verify Per-Point Time Field

Current config uses:

```text
lidar_type=2
scan_line=32
scan_rate=10
timestamp_unit=0
```

`timestamp_unit=0` means the PointCloud2 `time` field is interpreted as seconds. Verify this against the actual bag message field. A seconds/milliseconds/microseconds mismatch corrupts scan-end time and motion undistortion, and is a strong candidate for registration loss.

## Success Criteria

- No segmentation fault.
- No long sequence of `effective_features=0`.
- No hundreds-of-meters state divergence.
- `buffer_lidar` does not steadily grow.
- Odometry stays active during valid LiDAR periods.
- `/cloud_registered`, `/Laser_map`, and `/path` are visible in standalone RViz.
- Offline bag timestamp statistics agree with runtime `stamp_gap` logs.

## Notes

`Frame [map] does not exist` in navigation RViz is not the FAST_LIO root cause. Independent FAST_LIO RViz correctly uses `camera_init`; `map` only becomes usable in the full stack after NDT establishes global localization.
