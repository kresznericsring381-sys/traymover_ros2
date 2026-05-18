#!/usr/bin/env bash
# Interactive launcher for the Traymover robot.
# Each "start" option opens the command in a new terminal window so the menu
# stays usable and multiple subsystems can run side by side.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_DIR}/install/setup.bash"
ROS_LOG_DIR_DEFAULT="/tmp/roslog"

DEFAULT_MAP_DIR="${WORKSPACE_DIR}/src/traymover_robot_nav2/map"
FASTLIO_PCD_DIR="${WORKSPACE_DIR}/src/traymover_robot_slam/FAST_LIO/PCD"
FASTLIO_ROSBAG_DIR="${WORKSPACE_DIR}/src/traymover_robot_slam/FAST_LIO/rosbag"
FASTLIO_POS_LOG="${WORKSPACE_DIR}/src/traymover_robot_slam/FAST_LIO/Log/pos_log.txt"
FASTLIO_FILTER_STAGING="${FASTLIO_PCD_DIR}/traymover_filtered.pcd"
NAV_PKG_DIR="${WORKSPACE_DIR}/src/traymover_robot_nav"
NAV_MAP_DIR="${NAV_PKG_DIR}/map"
NAV_SCRIPTS_DIR="${NAV_PKG_DIR}/scripts"
NAV_RUNTIME_DIR="/tmp/traymover_nav_runtime"
NAV_PGM_Z_MIN="0.10"
NAV_PGM_Z_MAX="2.20"
NAV_PGM_MIN_POINTS="1"
NAV_PGM_DILATE="2"
NAV_PGM_MIN_REGION_CELLS="2"

# ---- terminal spawning ------------------------------------------------------
# Pick whichever terminal emulator is installed. gnome-terminal on Jetson Orin
# Ubuntu 22.04, xterm as a generic fallback.
pick_terminal() {
    if command -v gnome-terminal >/dev/null 2>&1; then
        echo "gnome-terminal"
    elif command -v xterm >/dev/null 2>&1; then
        echo "xterm"
    else
        echo ""
    fi
}

# Spawn a ROS command in a new terminal window. Args: <window_title> <command...>
spawn_in_terminal() {
    local title="$1"; shift
    local cmd="$*"
    local term
    term="$(pick_terminal)"

    # Wrap the command so the shell sources ROS first and keeps the window open
    # on exit (so the user can read any error before the window closes).
    local wrapped
    wrapped="echo '[traymover] ${title}'; \
mkdir -p '${ROS_LOG_DIR_DEFAULT}'; \
export ROS_LOG_DIR='${ROS_LOG_DIR_DEFAULT}'; \
source '${ROS_DISTRO_SETUP}'; \
if [ -f '${WS_SETUP}' ]; then source '${WS_SETUP}'; else echo '[traymover] WARNING: ${WS_SETUP} not found — did you colcon build?'; fi; \
${cmd}; \
echo; echo '[traymover] ${title} exited. Press Enter to close.'; read"

    case "${term}" in
        gnome-terminal)
            gnome-terminal --title="${title}" -- bash -lc "${wrapped}" &
            ;;
        xterm)
            xterm -T "${title}" -e bash -lc "${wrapped}" &
            ;;
        *)
            echo "No supported terminal emulator found (gnome-terminal/xterm)."
            echo "Running in this window instead. Ctrl+C to return to the menu."
            bash -lc "${wrapped}"
            return
            ;;
    esac
    # Give the new window a moment to appear before redrawing the menu.
    sleep 0.3
}

# ---- clean slate ------------------------------------------------------------
# Kill anything this launcher may have started previously so each start action
# begins from a known-empty state. We match on launch/run command lines and
# known node executables — broad on purpose (this is a dedicated robot).
KILL_PATTERNS=(
    'ros2 launch turn_on_traymover_robot'
    'ros2 launch traymover_slam_toolbox'
    'ros2 launch traymover_robot_nav2'
    'ros2 launch traymover_robot_nav '
    'ros2 launch traymover_robot_nav traymover_3d_nav'
    'ros2 launch lslidar_driver'
    'ros2 launch fast_lio'
    'ros2 bag record'
    'ros2 bag play'
    'ros2 run traymover_robot_keyboard'
    'traymover_keyboard'
    'traymover_robot.py'
    'traymover_ekf_filter_node'
    'async_slam_toolbox_node'
    'sync_slam_toolbox_node'
    'lslidar_driver_node'
    'fastlio_mapping'
    'fastlio_online_map_filter'
    'pointcloud_to_laserscan'
    'pointcloud_nav_preprocessor'
    'imu_pose_broadcaster'
    'robot_state_publisher'
    'joint_state_publisher'
    'rviz2'
    'ekf_node'
    # traymover_robot_nav components
    'lidar_localization_node'
    'pose_to_odom'
    # CMU autonomous_exploration_development_environment executables
    'lib/terrain_analysis'
    'lib/local_planner'
    'lib/sensor_scan_generation'
    'goal_pose_to_waypoint'
    'twist_stamped_to_twist'
    # nav2 node executables live at /opt/ros/humble/lib/<pkg>/<exe>. pkill -f
    # matches substrings of the full command line, so the pattern must read
    # "lib/<pkg>" (not "<pkg>/lib"). Getting this wrong leaves orphaned
    # planner_server / controller_server / bt_navigator etc. running after
    # option 0, which re-publish /tf and /map and cause RViz to black out
    # on the next nav start until they self-terminate from lost heartbeats.
    'lib/nav2_map_server'
    'lib/nav2_planner'
    'lib/nav2_controller'
    'lib/nav2_behaviors'
    'lib/nav2_bt_navigator'
    'lib/nav2_waypoint_follower'
    'lib/nav2_collision_monitor'
    'lib/nav2_lifecycle_manager'
    'lib/nav2_velocity_smoother'
    '\[traymover\]'  # the wrapper banner, matches bash processes in spawned terminals
)

kill_previous() {
    local quiet="${1:-false}"
    [ "$quiet" = "true" ] || echo "[traymover] Cleaning up previous processes..."
    for p in "${KILL_PATTERNS[@]}"; do
        pkill -f "$p" 2>/dev/null || true
    done
    # Give processes a moment to shut down cleanly, then SIGKILL stragglers.
    sleep 0.8
    for p in "${KILL_PATTERNS[@]}"; do
        pkill -9 -f "$p" 2>/dev/null || true
    done
    [ "$quiet" = "true" ] || echo "[traymover] Cleanup done."
}

# ---- actions ----------------------------------------------------------------

action_start_chassis() {
    # Bringup without lidar/EKF/SLAM — for teleop, IMU, URDF, RViz only.
    spawn_in_terminal "traymover: chassis" \
        "ros2 launch turn_on_traymover_robot turn_on_traymover_robot.launch.py use_lidar:=false use_ekf:=false"
}

action_start_keyboard() {
    spawn_in_terminal "traymover: keyboard" \
        "ros2 run traymover_robot_keyboard traymover_keyboard"
}

action_start_teleop() {
    # 纯遥控模式:只启动底盘串口 + 键盘节点,不拉 LiDAR / IMU / EKF。
    # base_serial.launch.py 带 use_imu:=false 时只跑 wheeltec_robot(/cmd_vel 订阅、
    # 轮式里程计、/battery_state),够手动开着玩/搬运。
    spawn_in_terminal "traymover: chassis_serial" \
        "ros2 launch turn_on_traymover_robot base_serial.launch.py use_imu:=false"
    # 给串口一点时间初始化,避免键盘先起来发的第一条 cmd_vel 被丢。
    sleep 2
    spawn_in_terminal "traymover: keyboard" \
        "ros2 run traymover_robot_keyboard traymover_keyboard"
}

action_start_slam() {
    # online_async launch already includes chassis bringup with lidar + EKF.
    spawn_in_terminal "traymover: slam_toolbox" \
        "ros2 launch traymover_slam_toolbox online_async.launch.py"
}

action_save_fastlio_map() {
    # The online FAST-LIO map filter writes to a fixed staging PCD, then we
    # rename it into a user-chosen destination so multiple runs do not clobber
    # each other.
    local default_name="traymover_$(date +%Y%m%d_%H%M%S)"
    read -r -p "Map name (without .pcd) [default: ${default_name}]: " map_name
    map_name="${map_name:-${default_name}}"
    # Strip a trailing .pcd if the user typed one.
    map_name="${map_name%.pcd}"

    read -r -p "Save directory [default: ${FASTLIO_PCD_DIR}]: " map_dir
    map_dir="${map_dir:-${FASTLIO_PCD_DIR}}"
    mkdir -p "${map_dir}"

    set +u
    mkdir -p "${ROS_LOG_DIR_DEFAULT}"
    export ROS_LOG_DIR="${ROS_LOG_DIR_DEFAULT}"
    export ROS2CLI_NO_DAEMON=1
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi
    set -u

    local staging="${FASTLIO_FILTER_STAGING}"
    # Remove any stale staging file so we can tell whether this call actually wrote one.
    rm -f "${staging}"

    echo "[traymover] Calling /save_filtered_map on the FAST-LIO online filter (up to 10s)..."
    local srv_output
    srv_output="$(timeout 12 python3 "${NAV_SCRIPTS_DIR}/call_trigger_service.py" /save_filtered_map --timeout 10 2>&1)"
    local rc=$?
    if [ $rc -ne 0 ]; then
        echo "[traymover] /save_filtered_map call failed. Is FAST-LIO running (option 6 or 9)?"
        echo "${srv_output}"
        return 1
    fi

    if [ ! -f "${staging}" ]; then
        echo "[traymover] Service returned but ${staging} was not written."
        echo "[traymover] Likely the filtered cloud is still empty — move the robot and try again."
        echo "${srv_output}"
        return 1
    fi

    local dest="${map_dir%/}/${map_name}.pcd"
    if ! mv "${staging}" "${dest}"; then
        echo "[traymover] Failed to move ${staging} → ${dest}"
        return 1
    fi
    local size
    size="$(du -h "${dest}" | awk '{print $1}')"
    echo "[traymover] Saved ${dest} (${size})"
}

action_start_fastlio() {
    # LiDAR-IMU odometry via FAST-LIO. No STM32 wheel odometry is fed into
    # the estimator — pose comes purely from LSLiDAR CX32 + HiPNUC N300 Pro.
    # The base_serial node is still started so /cmd_vel can drive the chassis
    # (pair this option with option 2 "keyboard" to teleop during mapping).
    # IMU driver is launched standalone here; we pass use_imu:=false to
    # base_serial.launch.py so it does NOT spawn a second IMU publisher.
    # A sidecar online map filter subscribes to FAST-LIO's /cloud_registered in
    # camera_init frame, drops a near-rear exclusion box around the robot, and
    # only keeps voxels re-observed across multiple scans before saving to PCD.
    spawn_in_terminal "traymover: chassis_serial" \
        "ros2 launch turn_on_traymover_robot base_serial.launch.py use_imu:=false"
    spawn_in_terminal "traymover: imu" \
        "ros2 launch turn_on_traymover_robot traymover_imu.launch.py"
    spawn_in_terminal "traymover: lidar" \
        "ros2 launch turn_on_traymover_robot traymover_lidar.launch.py"
    # Give LiDAR/IMU a moment to come up so FAST-LIO's IMU init doesn't stall.
    sleep 5
    spawn_in_terminal "traymover: fast_lio" \
        "ros2 launch fast_lio mapping.launch.py config_file:=traymover.yaml rviz:=true"
    sleep 2
    spawn_in_terminal "traymover: fastlio_map_filter" \
        "python3 '${NAV_SCRIPTS_DIR}/fastlio_online_map_filter.py' --ros-args \
            -p input_topic:=/cloud_registered \
            -p odom_topic:=/odom \
            -p output_topic:=/filtered_map \
            -p map_file_path:='${FASTLIO_FILTER_STAGING}'"
}

action_kill_all() {
    kill_previous
}

action_replay_fastlio_bag() {
    # 用之前录的 rosbag 离线跑 FAST-LIO。关键点:fast_lio 和 bag 都走 sim_time
    # (bag play --clock 发布 /clock,fast_lio 用 use_sim_time:=true 订阅它),
    # 这样时间戳匹配、IMU 初始化不会因为 wall clock 差异失败。
    if [ ! -d "${FASTLIO_ROSBAG_DIR}" ] || [ -z "$(ls -A "${FASTLIO_ROSBAG_DIR}" 2>/dev/null)" ]; then
        echo "[traymover] No bags in ${FASTLIO_ROSBAG_DIR}. Record one with option 8 first."
        return 1
    fi

    # 列出候选,按修改时间排序,最新的在最上。
    local -a bags=()
    while IFS= read -r -d '' d; do
        bags+=("$d")
    done < <(find "${FASTLIO_ROSBAG_DIR}" -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 -I{} stat --format='%Y %n' {} 2>/dev/null | sort -rn | cut -d' ' -f2- | tr '\n' '\0')

    if [ ${#bags[@]} -eq 0 ]; then
        echo "[traymover] No bag directories found in ${FASTLIO_ROSBAG_DIR}."
        return 1
    fi

    echo "Available bags (newest first):"
    local i
    for i in "${!bags[@]}"; do
        printf "  [%d] %s\n" "$((i+1))" "$(basename "${bags[i]}")"
    done

    local pick bag_path
    read -r -p "Select bag number [default: 1 = newest]: " pick
    pick="${pick:-1}"
    if ! [[ "${pick}" =~ ^[0-9]+$ ]] || [ "${pick}" -lt 1 ] || [ "${pick}" -gt "${#bags[@]}" ]; then
        echo "[traymover] Invalid selection: ${pick}"
        return 1
    fi
    bag_path="${bags[$((pick-1))]}"

    echo "[traymover] Replaying bag: ${bag_path}"
    echo "[traymover]   FAST-LIO will run with use_sim_time:=true."
    echo "[traymover]   Select option 7 to save the filtered PCD afterwards."
    echo "[traymover]   Select option 0 to stop replay + fast_lio."

    # Start FAST-LIO first so it's ready to receive scans once the bag plays.
    spawn_in_terminal "traymover: fast_lio (sim_time)" \
        "ros2 launch fast_lio mapping.launch.py config_file:=traymover.yaml rviz:=true use_sim_time:=true"
    sleep 2
    spawn_in_terminal "traymover: fastlio_map_filter (sim_time)" \
        "python3 '${NAV_SCRIPTS_DIR}/fastlio_online_map_filter.py' --ros-args \
            -p input_topic:=/cloud_registered \
            -p odom_topic:=/odom \
            -p output_topic:=/filtered_map \
            -p map_file_path:='${FASTLIO_FILTER_STAGING}' \
            -p publish_period_sec:=1.0 \
            -p pose_timeout_sec:=0.50 \
            -p rear_filter_enabled:=true \
            -p use_sim_time:=true"
    # Let fast_lio subscribe + init before playback starts.
    sleep 3
    spawn_in_terminal "traymover: bag_play" \
        "ros2 bag play '${bag_path}' --clock"
}

action_record_fastlio_bag() {
    # 录制 FAST-LIO 离线建图所需的 topic:LiDAR 点云 + IMU + TF。
    # 传感器拓扑与 action_start_fastlio 对齐,保证 bag 可以直接 replay 给 fast_lio。
    # 启动 rosbag record,0 选项的 pkill -TERM 会让它干净地 flush metadata 再退出。
    mkdir -p "${FASTLIO_ROSBAG_DIR}"
    local ts bag_path
    ts="$(date +%Y%m%d_%H%M%S)"
    bag_path="${FASTLIO_ROSBAG_DIR}/traymover_${ts}"

    echo "[traymover] Starting FAST-LIO sensors + rosbag recording."
    echo "[traymover]   Bag path : ${bag_path}"
    echo "[traymover]   Topics   : /point_cloud_raw /imu/data_raw /tf_static /tf"
    echo "[traymover]   To drive : select option 2 (keyboard) in another pass."
    echo "[traymover]   To stop  : select option 0; bag metadata will be finalized."

    spawn_in_terminal "traymover: chassis_serial" \
        "ros2 launch turn_on_traymover_robot base_serial.launch.py use_imu:=false"
    spawn_in_terminal "traymover: imu" \
        "ros2 launch turn_on_traymover_robot traymover_imu.launch.py"
    # Same LiDAR launch as action_start_fastlio for consistency — the
    # angle_disable_min/max mask from traymover_param.yaml is applied.
    # The bagged /point_cloud_raw will already be front-180°-only, so the
    # recorded bag is appropriate for offline re-mapping without needing
    # post-hoc filtering.
    spawn_in_terminal "traymover: lidar" \
        "ros2 launch turn_on_traymover_robot traymover_lidar.launch.py"
    # Let sensors stabilize so the bag starts with valid data.
    sleep 5
    spawn_in_terminal "traymover: rosbag" \
        "ros2 bag record -o '${bag_path}' /point_cloud_raw /imu/data_raw /tf_static /tf"
}

action_show_battery() {
    # ROS setup.bash 里用了未初始化变量,需要临时关掉 nounset
    set +u
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi
    set -u

    echo "[traymover] Reading /battery_state (waiting up to 5s)..."
    local output
    output="$(timeout 5 ros2 topic echo /battery_state --once 2>&1)"
    local rc=$?
    if [ $rc -ne 0 ] || ! echo "${output}" | grep -q 'percentage:'; then
        echo "[traymover] Could not read /battery_state. Is the chassis running?"
        echo "[traymover] Start option 1 (chassis) or 3 (SLAM) first, then try again."
        return 1
    fi

    local pct temp charge_status
    pct="$(echo "${output}" | awk '/^percentage:/ {print $2}')"
    temp="$(echo "${output}" | awk '/^temperature:/ {print $2}')"
    charge_status="$(echo "${output}" | awk '/^power_supply_status:/ {print $2}')"

    local pct_display
    if [ -n "${pct}" ]; then
        pct_display="$(awk -v p="${pct}" 'BEGIN{printf "%.0f%%", p*100}')"
    else
        pct_display="?"
    fi

    local charge_text
    case "${charge_status}" in
        1) charge_text="CHARGING" ;;
        2) charge_text="DISCHARGING" ;;
        3) charge_text="NOT_CHARGING" ;;
        4) charge_text="FULL" ;;
        *) charge_text="UNKNOWN(${charge_status})" ;;
    esac

    echo "  Battery : ${pct_display}"
    echo "  Temp    : ${temp:-?} °C"
    echo "  Status  : ${charge_text}"
}

action_start_nav() {
    # Autonomous navigation bringup. Prompts the user to:
    #   1) pick a PCD map from FAST_LIO/PCD/ (the prior map for lidar_localization_ros2)
    #   2) regenerate a matching 2D occupancy PGM/YAML from that same PCD
    #      into /tmp so Nav2's static layer and NDT always share one source map
    #   3) choose whether to launch RViz
    # Then spawns chassis + lidar + nav stack in separate terminals so each
    # subsystem's log is readable.
    if [ ! -d "${FASTLIO_PCD_DIR}" ]; then
        echo "[traymover] FAST-LIO PCD directory missing: ${FASTLIO_PCD_DIR}"
        echo "[traymover] Record a map first (options 6 + 7)."
        return 1
    fi

    # Enumerate PCDs newest first.
    local -a pcds=()
    while IFS= read -r -d '' f; do
        pcds+=("$f")
    done < <(find "${FASTLIO_PCD_DIR}" -maxdepth 1 -name '*.pcd' -type f -print0 2>/dev/null \
             | xargs -0 -I{} stat --format='%Y %n' {} 2>/dev/null \
             | sort -rn | cut -d' ' -f2- | tr '\n' '\0')

    if [ ${#pcds[@]} -eq 0 ]; then
        echo "[traymover] No .pcd files in ${FASTLIO_PCD_DIR}. Save a map first (option 7)."
        return 1
    fi

    echo "Available PCD maps in ${FASTLIO_PCD_DIR} (newest first):"
    local i size
    for i in "${!pcds[@]}"; do
        size="$(du -h "${pcds[i]}" 2>/dev/null | awk '{print $1}')"
        printf "  [%d] %s  (%s)\n" "$((i+1))" "$(basename "${pcds[i]}")" "${size:-?}"
    done

    local pick pcd_path
    read -r -p "Select PCD number [default: 1 = newest]: " pick
    pick="${pick:-1}"
    if ! [[ "${pick}" =~ ^[0-9]+$ ]] || [ "${pick}" -lt 1 ] || [ "${pick}" -gt "${#pcds[@]}" ]; then
        echo "[traymover] Invalid selection: ${pick}"
        return 1
    fi
    pcd_path="${pcds[$((pick-1))]}"
    echo "[traymover] Selected: ${pcd_path}"

    # Always regenerate the 2D map from the selected PCD. Allowing a stale
    # traymover_2d.yaml to survive across runs makes Nav2's static map and the
    # PCD used by NDT diverge, which in turn shifts RViz 2D Pose Estimate away
    # from the 3D map and makes straight-ahead goals appear "behind" the robot.
    mkdir -p "${NAV_RUNTIME_DIR}"
    local runtime_name runtime_prefix runtime_map_yaml
    runtime_name="$(basename "${pcd_path}" .pcd)"
    runtime_prefix="${NAV_RUNTIME_DIR}/${runtime_name}_2d"
    runtime_map_yaml="${runtime_prefix}.yaml"

    set +u
    mkdir -p "${ROS_LOG_DIR_DEFAULT}"
    export ROS_LOG_DIR="${ROS_LOG_DIR_DEFAULT}"
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi
    set -u

    echo "[traymover] Generating runtime 2D nav map from selected PCD ..."
    if ! python3 "${NAV_SCRIPTS_DIR}/pcd2pgm.py" \
            --pcd "${pcd_path}" \
            --out "${runtime_prefix}" \
            --z-min "${NAV_PGM_Z_MIN}" \
            --z-max "${NAV_PGM_Z_MAX}" \
            --min-points "${NAV_PGM_MIN_POINTS}" \
            --dilate "${NAV_PGM_DILATE}" \
            --min-region-cells "${NAV_PGM_MIN_REGION_CELLS}"; then
        echo "[traymover] pcd2pgm failed. Aborting nav startup."
        return 1
    fi
    if [ ! -f "${runtime_map_yaml}" ]; then
        echo "[traymover] Expected runtime map YAML missing: ${runtime_map_yaml}"
        return 1
    fi
    echo "[traymover] Runtime map YAML: ${runtime_map_yaml}"
    echo "[traymover] Runtime map params: z=[${NAV_PGM_Z_MIN}, ${NAV_PGM_Z_MAX}] m above floor, min_points=${NAV_PGM_MIN_POINTS}, dilate=${NAV_PGM_DILATE}, min_region_cells=${NAV_PGM_MIN_REGION_CELLS}"

    local rviz_opt rviz_arg
    read -r -p "Launch RViz? [Y/n]: " rviz_opt
    rviz_opt="${rviz_opt:-Y}"
    if [[ "${rviz_opt}" =~ ^[Yy] ]]; then
        rviz_arg="launch_rviz:=true"
    else
        rviz_arg="launch_rviz:=false"
    fi

    echo "[traymover] Starting chassis + IMU (base_serial) ..."
    spawn_in_terminal "traymover: chassis_serial" \
        "ros2 launch turn_on_traymover_robot base_serial.launch.py odom_source_mode:=none"

    echo "[traymover] Starting LiDAR driver (Nav2 is not using scan-based avoidance) ..."
    spawn_in_terminal "traymover: lidar" \
        "ros2 launch turn_on_traymover_robot traymover_lidar.launch.py enable_scan_bridge:=false"

    # Let sensors stabilize before Nav2 bringup (lidar_localization needs /point_cloud_raw
    # and /imu/data_raw already flowing, otherwise it'll hang at Activating).
    sleep 6

    echo "[traymover] Starting nav stack with pcd_path=${pcd_path} map=${runtime_map_yaml} ..."
    spawn_in_terminal "traymover: nav_stack" \
        "ros2 launch traymover_robot_nav traymover_nav.launch.py ${rviz_arg} pcd_path:='${pcd_path}' map:='${runtime_map_yaml}'"

    cat <<EOF
[traymover] Nav stack starting. Next steps once lifecycle activates (~10-20 s):
  1) In RViz click '2D Pose Estimate' and drag on the map at the robot's current spot.
  2) Wait for RobotModel to snap to pose and /pcl_pose to converge.
  3) Click '2D Goal Pose' to send a navigation goal.
  Health checks:
    ros2 topic info /odom -v
    ./scripts/check_localization.sh
    ros2 action list | grep navigate_to_pose
    ros2 topic echo --once /cmd_vel
EOF
}

action_start_nav_speed_modes() {
    # Option 15 is an additive variant of option 10: same map selection,
    # runtime 2D map generation, hardware bringup, localization, Nav2 launch,
    # and RViz prompt; only the Nav2 params_file changes for speed modes 1/2.
    local speed_mode speed_label nav_params_file nav_params_arg
    cat <<EOF
[traymover] Navigation speed modes:
  0) Default slow navigation, same params as option 10
  1) Linear speed 2.0x, in-place turning speed unchanged
  2) Linear speed 2.5x, in-place turning speed 2.0x
EOF
    read -r -p "Select speed mode [default: 0]: " speed_mode
    speed_mode="${speed_mode:-0}"
    nav_params_file=""
    nav_params_arg=""
    case "${speed_mode}" in
        0)
            speed_label="mode 0 slow/default"
            ;;
        1)
            speed_label="mode 1 linear 2.0x"
            nav_params_file="${NAV_PKG_DIR}/config/nav2_params_linear_2x.yaml"
            ;;
        2)
            speed_label="mode 2 linear 2.5x + turn 2.0x"
            nav_params_file="${NAV_PKG_DIR}/config/nav2_params_linear_2_5x_turn_2x.yaml"
            ;;
        *)
            echo "[traymover] Invalid speed mode: ${speed_mode}"
            return 1
            ;;
    esac

    if [ -n "${nav_params_file}" ]; then
        if [ ! -f "${nav_params_file}" ]; then
            echo "[traymover] Missing Nav2 params file: ${nav_params_file}"
            return 1
        fi
        nav_params_arg="params_file:='${nav_params_file}'"
    fi
    echo "[traymover] Selected navigation speed: ${speed_label}"

    if [ ! -d "${FASTLIO_PCD_DIR}" ]; then
        echo "[traymover] FAST-LIO PCD directory missing: ${FASTLIO_PCD_DIR}"
        echo "[traymover] Record a map first (options 6 + 7)."
        return 1
    fi

    local -a pcds=()
    while IFS= read -r -d '' f; do
        pcds+=("$f")
    done < <(find "${FASTLIO_PCD_DIR}" -maxdepth 1 -name '*.pcd' -type f -print0 2>/dev/null \
             | xargs -0 -I{} stat --format='%Y %n' {} 2>/dev/null \
             | sort -rn | cut -d' ' -f2- | tr '\n' '\0')

    if [ ${#pcds[@]} -eq 0 ]; then
        echo "[traymover] No .pcd files in ${FASTLIO_PCD_DIR}. Save a map first (option 7)."
        return 1
    fi

    echo "Available PCD maps in ${FASTLIO_PCD_DIR} (newest first):"
    local i size
    for i in "${!pcds[@]}"; do
        size="$(du -h "${pcds[i]}" 2>/dev/null | awk '{print $1}')"
        printf "  [%d] %s  (%s)\n" "$((i+1))" "$(basename "${pcds[i]}")" "${size:-?}"
    done

    local pick pcd_path
    read -r -p "Select PCD number [default: 1 = newest]: " pick
    pick="${pick:-1}"
    if ! [[ "${pick}" =~ ^[0-9]+$ ]] || [ "${pick}" -lt 1 ] || [ "${pick}" -gt "${#pcds[@]}" ]; then
        echo "[traymover] Invalid selection: ${pick}"
        return 1
    fi
    pcd_path="${pcds[$((pick-1))]}"
    echo "[traymover] Selected: ${pcd_path}"

    mkdir -p "${NAV_RUNTIME_DIR}"
    local runtime_name runtime_prefix runtime_map_yaml
    runtime_name="$(basename "${pcd_path}" .pcd)"
    runtime_prefix="${NAV_RUNTIME_DIR}/${runtime_name}_2d"
    runtime_map_yaml="${runtime_prefix}.yaml"

    set +u
    mkdir -p "${ROS_LOG_DIR_DEFAULT}"
    export ROS_LOG_DIR="${ROS_LOG_DIR_DEFAULT}"
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi
    set -u

    echo "[traymover] Generating runtime 2D nav map from selected PCD ..."
    if ! python3 "${NAV_SCRIPTS_DIR}/pcd2pgm.py" \
            --pcd "${pcd_path}" \
            --out "${runtime_prefix}" \
            --z-min "${NAV_PGM_Z_MIN}" \
            --z-max "${NAV_PGM_Z_MAX}" \
            --min-points "${NAV_PGM_MIN_POINTS}" \
            --dilate "${NAV_PGM_DILATE}" \
            --min-region-cells "${NAV_PGM_MIN_REGION_CELLS}"; then
        echo "[traymover] pcd2pgm failed. Aborting nav startup."
        return 1
    fi
    if [ ! -f "${runtime_map_yaml}" ]; then
        echo "[traymover] Expected runtime map YAML missing: ${runtime_map_yaml}"
        return 1
    fi
    echo "[traymover] Runtime map YAML: ${runtime_map_yaml}"
    echo "[traymover] Runtime map params: z=[${NAV_PGM_Z_MIN}, ${NAV_PGM_Z_MAX}] m above floor, min_points=${NAV_PGM_MIN_POINTS}, dilate=${NAV_PGM_DILATE}, min_region_cells=${NAV_PGM_MIN_REGION_CELLS}"

    local rviz_opt rviz_arg
    read -r -p "Launch RViz? [Y/n]: " rviz_opt
    rviz_opt="${rviz_opt:-Y}"
    if [[ "${rviz_opt}" =~ ^[Yy] ]]; then
        rviz_arg="launch_rviz:=true"
    else
        rviz_arg="launch_rviz:=false"
    fi

    echo "[traymover] Starting chassis + IMU (base_serial) ..."
    spawn_in_terminal "traymover: chassis_serial" \
        "ros2 launch turn_on_traymover_robot base_serial.launch.py odom_source_mode:=none"

    echo "[traymover] Starting LiDAR driver (Nav2 is not using scan-based avoidance) ..."
    spawn_in_terminal "traymover: lidar" \
        "ros2 launch turn_on_traymover_robot traymover_lidar.launch.py enable_scan_bridge:=false"

    sleep 6

    local nav_cmd
    nav_cmd="ros2 launch traymover_robot_nav traymover_nav.launch.py ${rviz_arg} pcd_path:='${pcd_path}' map:='${runtime_map_yaml}'"
    if [ -n "${nav_params_arg}" ]; then
        nav_cmd="${nav_cmd} ${nav_params_arg}"
    fi

    echo "[traymover] Starting nav stack (${speed_label}) with pcd_path=${pcd_path} map=${runtime_map_yaml} ..."
    spawn_in_terminal "traymover: nav_stack ${speed_mode}" "${nav_cmd}"

    cat <<EOF
[traymover] Nav stack starting with ${speed_label}. Next steps once lifecycle activates (~10-20 s):
  1) In RViz click '2D Pose Estimate' and drag on the map at the robot's current spot.
  2) Wait for RobotModel to snap to pose and /pcl_pose to converge.
  3) Click '2D Goal Pose' to send a navigation goal.
  Health checks:
    ros2 topic info /odom -v
    ./scripts/check_localization.sh
    ros2 action list | grep navigate_to_pose
    ros2 topic echo --once /cmd_vel
EOF
}

action_start_3d_nav() {
    # Alternative to option 10 that skips the 2D PGM projection entirely
    # and drives from the 3D point cloud directly using CMU's
    # autonomous_exploration_development_environment stack
    # (terrain_analysis + local_planner). lidar_localization_ros2 (NDT)
    # still runs so goal coords remain map-consistent across sessions.
    if [ ! -d "${FASTLIO_PCD_DIR}" ]; then
        echo "[traymover] FAST-LIO PCD directory missing: ${FASTLIO_PCD_DIR}"
        echo "[traymover] Record a map first (options 6 + 7)."
        return 1
    fi

    local -a pcds=()
    while IFS= read -r -d '' f; do
        pcds+=("$f")
    done < <(find "${FASTLIO_PCD_DIR}" -maxdepth 1 -name '*.pcd' -type f -print0 2>/dev/null \
             | xargs -0 -I{} stat --format='%Y %n' {} 2>/dev/null \
             | sort -rn | cut -d' ' -f2- | tr '\n' '\0')

    if [ ${#pcds[@]} -eq 0 ]; then
        echo "[traymover] No .pcd files in ${FASTLIO_PCD_DIR}. Save a map first (option 7)."
        return 1
    fi

    echo "Available PCD maps in ${FASTLIO_PCD_DIR} (newest first):"
    local i size
    for i in "${!pcds[@]}"; do
        size="$(du -h "${pcds[i]}" 2>/dev/null | awk '{print $1}')"
        printf "  [%d] %s  (%s)\n" "$((i+1))" "$(basename "${pcds[i]}")" "${size:-?}"
    done

    local pick pcd_path
    read -r -p "Select PCD number [default: 1 = newest]: " pick
    pick="${pick:-1}"
    if ! [[ "${pick}" =~ ^[0-9]+$ ]] || [ "${pick}" -lt 1 ] || [ "${pick}" -gt "${#pcds[@]}" ]; then
        echo "[traymover] Invalid selection: ${pick}"
        return 1
    fi
    pcd_path="${pcds[$((pick-1))]}"
    echo "[traymover] Selected: ${pcd_path}"

    local rviz_opt rviz_arg
    read -r -p "Launch RViz? [Y/n]: " rviz_opt
    rviz_opt="${rviz_opt:-Y}"
    if [[ "${rviz_opt}" =~ ^[Yy] ]]; then
        rviz_arg="launch_rviz:=true"
    else
        rviz_arg="launch_rviz:=false"
    fi

    echo "[traymover] Starting chassis + IMU (base_serial) ..."
    spawn_in_terminal "traymover: chassis_serial" \
        "ros2 launch turn_on_traymover_robot base_serial.launch.py odom_source_mode:=none"

    echo "[traymover] Starting LiDAR driver (navigation owns /scan) ..."
    spawn_in_terminal "traymover: lidar" \
        "ros2 launch turn_on_traymover_robot traymover_lidar.launch.py enable_scan_bridge:=false"

    # Localization (FAST_LIO + NDT) needs /point_cloud_raw + /imu/data_raw
    # before its lifecycle activates; give sensors a moment.
    sleep 6

    echo "[traymover] Starting 3D nav stack with pcd_path=${pcd_path} ..."
    spawn_in_terminal "traymover: 3d_nav_stack" \
        "ros2 launch traymover_robot_nav traymover_3d_nav.launch.py ${rviz_arg} pcd_path:='${pcd_path}'"

    cat <<EOF
[traymover] 3D nav stack starting. Flow:
  1) Wait ~8-12 s for FAST_LIO IMU init + NDT lifecycle activation.
  2) In RViz: '2D Pose Estimate' to initialize NDT pose on the prior map.
  3) Once /pcl_pose stabilizes, click '2D Goal Pose'.
     goal_pose_to_waypoint bridge republishes it as /way_point (in camera_init
     frame) for CMU localPlanner.
  Synthetic waypoint probe (skip RViz):
    ros2 topic pub --once /way_point geometry_msgs/PointStamped \\
      "{header: {frame_id: camera_init}, point: {x: 2.0, y: 0.0, z: 0.0}}"
  — expect /cmd_vel_nav to emit non-zero linear.x within ~1 s.
EOF
}

action_clean_pcd() {
    # Offline dynamic-object cleanup for a FAST-LIO PCD. Wraps
    # scripts/pcd_clean.py (SOR, optional DBSCAN person-cluster filter).
    # Output goes back into FASTLIO_PCD_DIR with a "_clean" suffix so
    # option 10 will see it in its newest-first listing.
    if [ ! -d "${FASTLIO_PCD_DIR}" ]; then
        echo "[traymover] FAST-LIO PCD directory missing: ${FASTLIO_PCD_DIR}"
        return 1
    fi

    local -a pcds=()
    while IFS= read -r -d '' f; do
        pcds+=("$f")
    done < <(find "${FASTLIO_PCD_DIR}" -maxdepth 1 -name '*.pcd' -type f -print0 2>/dev/null \
             | xargs -0 -I{} stat --format='%Y %n' {} 2>/dev/null \
             | sort -rn | cut -d' ' -f2- | tr '\n' '\0')

    if [ ${#pcds[@]} -eq 0 ]; then
        echo "[traymover] No .pcd files in ${FASTLIO_PCD_DIR}."
        return 1
    fi

    echo "Available PCD maps in ${FASTLIO_PCD_DIR} (newest first):"
    local i size
    for i in "${!pcds[@]}"; do
        size="$(du -h "${pcds[i]}" 2>/dev/null | awk '{print $1}')"
        printf "  [%d] %s  (%s)\n" "$((i+1))" "$(basename "${pcds[i]}")" "${size:-?}"
    done

    local pick pcd_in
    read -r -p "Select PCD number to clean [default: 1 = newest]: " pick
    pick="${pick:-1}"
    if ! [[ "${pick}" =~ ^[0-9]+$ ]] || [ "${pick}" -lt 1 ] || [ "${pick}" -gt "${#pcds[@]}" ]; then
        echo "[traymover] Invalid selection: ${pick}"
        return 1
    fi
    pcd_in="${pcds[$((pick-1))]}"

    local base_name="$(basename "${pcd_in}" .pcd)_clean"
    local default_out="${FASTLIO_PCD_DIR}/${base_name}.pcd"
    local pcd_out
    read -r -p "Output path [default: ${default_out}]: " pcd_out
    pcd_out="${pcd_out:-${default_out}}"

    local mode
    read -r -p "Cleaning mode: [l]ight-preset / [s]OR-only / [f]loating-columns / [d]bscan / [a]ggressive-preset [default: l]: " mode
    mode="${mode:-l}"
    local extra_args=""
    case "${mode,,}" in
        l|light) extra_args="--light" ;;
        s|sor) extra_args="" ;;
        f|floating) extra_args="--floating" ;;
        d|dbscan) extra_args="--dbscan" ;;
        a|aggressive) extra_args="--aggressive" ;;
        *) echo "[traymover] Unknown mode '${mode}', defaulting to --light."
           extra_args="--light" ;;
    esac

    local corridor_opt
    read -r -p "Enable trajectory-corridor sparse cleanup near the driven path? [y/N]: " corridor_opt
    corridor_opt="${corridor_opt:-N}"
    if [[ "${corridor_opt}" =~ ^[Yy] ]]; then
        local traj_log
        read -r -p "Trajectory log [default: ${FASTLIO_POS_LOG}]: " traj_log
        traj_log="${traj_log:-${FASTLIO_POS_LOG}}"
        if [ ! -s "${traj_log}" ]; then
            echo "[traymover] Trajectory log missing or empty: ${traj_log}"
            echo "[traymover] FAST-LIO path logging is now enabled for future mapping runs;"
            echo "[traymover] re-map once, then retry corridor cleanup."
            return 1
        fi

        local corridor_width corridor_min_pts
        read -r -p "Corridor half-width [default: 0.80 m]: " corridor_width
        corridor_width="${corridor_width:-0.80}"
        read -r -p "Drop sparse corridor cells below point-count [default: 120]: " corridor_min_pts
        corridor_min_pts="${corridor_min_pts:-120}"

        extra_args="${extra_args} --trajectory-corridor --trajectory-log ${traj_log} --corridor-width ${corridor_width} --corridor-min-pts ${corridor_min_pts}"
    fi

    set +u
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi
    set -u

    echo "[traymover] Running pcd_clean.py ${extra_args} ..."
    if ! python3 "${NAV_SCRIPTS_DIR}/pcd_clean.py" \
            --pcd "${pcd_in}" --out "${pcd_out}" ${extra_args}; then
        echo "[traymover] pcd_clean.py failed."
        return 1
    fi
    echo "[traymover] Cleaned PCD written to ${pcd_out}"
    echo "[traymover] Option 10 will now list this as the newest PCD."
}

action_intersect_pcds() {
    # Intersect multiple FAST-LIO PCDs: keep only voxels that appear in a
    # majority of inputs. Effective at removing dynamic objects whose
    # positions differ between mapping sessions. Requires inputs to share
    # a coordinate frame (start each FAST-LIO run from the same physical
    # pose, or pre-align with ICP).
    if [ ! -d "${FASTLIO_PCD_DIR}" ]; then
        echo "[traymover] FAST-LIO PCD directory missing: ${FASTLIO_PCD_DIR}"
        return 1
    fi

    local -a pcds=()
    while IFS= read -r -d '' f; do
        pcds+=("$f")
    done < <(find "${FASTLIO_PCD_DIR}" -maxdepth 1 -name '*.pcd' -type f -print0 2>/dev/null \
             | xargs -0 -I{} stat --format='%Y %n' {} 2>/dev/null \
             | sort -rn | cut -d' ' -f2- | tr '\n' '\0')

    if [ ${#pcds[@]} -lt 2 ]; then
        echo "[traymover] Need at least 2 PCDs in ${FASTLIO_PCD_DIR}; found ${#pcds[@]}."
        return 1
    fi

    echo "Available PCD maps (newest first):"
    local i size
    for i in "${!pcds[@]}"; do
        size="$(du -h "${pcds[i]}" 2>/dev/null | awk '{print $1}')"
        printf "  [%d] %s  (%s)\n" "$((i+1))" "$(basename "${pcds[i]}")" "${size:-?}"
    done

    local picks
    read -r -p "Select 2+ PCDs to intersect (space-separated numbers, e.g. '1 2 3'): " picks
    local -a selected=()
    local tok
    for tok in ${picks}; do
        if ! [[ "${tok}" =~ ^[0-9]+$ ]] || [ "${tok}" -lt 1 ] || [ "${tok}" -gt "${#pcds[@]}" ]; then
            echo "[traymover] Invalid selection: ${tok}"
            return 1
        fi
        selected+=("${pcds[$((tok-1))]}")
    done
    if [ ${#selected[@]} -lt 2 ]; then
        echo "[traymover] Need at least 2 selections; got ${#selected[@]}."
        return 1
    fi

    local ts default_out pcd_out
    ts="$(date +%Y%m%d_%H%M%S)"
    default_out="${FASTLIO_PCD_DIR}/traymover_intersected_${ts}.pcd"
    read -r -p "Output path [default: ${default_out}]: " pcd_out
    pcd_out="${pcd_out:-${default_out}}"

    local voxel
    read -r -p "Voxel size in meters [default: 0.1]: " voxel
    voxel="${voxel:-0.1}"

    set +u
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi
    set -u

    echo "[traymover] Intersecting ${#selected[@]} PCDs (voxel=${voxel} m) ..."
    if ! python3 "${NAV_SCRIPTS_DIR}/pcd_intersect.py" \
            --pcds "${selected[@]}" --out "${pcd_out}" --voxel "${voxel}"; then
        echo "[traymover] pcd_intersect.py failed."
        return 1
    fi
    echo "[traymover] Intersected PCD written to ${pcd_out}"
    echo "[traymover] Option 10 will now list this as a selectable map."
}

action_save_map() {
    mkdir -p "${DEFAULT_MAP_DIR}"

    local default_name="traymover_$(date +%Y%m%d_%H%M%S)"
    read -r -p "Map name [default: ${default_name}]: " map_name
    map_name="${map_name:-${default_name}}"

    read -r -p "Save directory [default: ${DEFAULT_MAP_DIR}]: " map_dir
    map_dir="${map_dir:-${DEFAULT_MAP_DIR}}"
    mkdir -p "${map_dir}"

    local map_path="${map_dir%/}/${map_name}"
    echo "[traymover] Saving map to ${map_path}.{pgm,yaml} ..."

    # Run in this terminal so the user sees success/failure directly.
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi

    if ! ros2 run nav2_map_server map_saver_cli -f "${map_path}"; then
        echo "[traymover] map_saver_cli failed. Is slam_toolbox publishing /map?"
        return 1
    fi
    echo "[traymover] Saved ${map_path}.pgm and ${map_path}.yaml"
}

# ---- menu -------------------------------------------------------------------

print_menu() {
    cat <<EOF

===== Traymover Launcher =====
  Workspace: ${WORKSPACE_DIR}
  0) Kill all traymover processes (clean up before starting)
  1) Start chassis           (bringup: description + RViz + IMU + base serial)
  2) Start keyboard teleop
  3) Start SLAM (slam_toolbox async; includes chassis + lidar + EKF)
  4) Save SLAM map
  5) Show battery (requires chassis running)
  6) Start FAST-LIO (LiDAR-IMU odometry + RViz; no STM32 wheel odom)
  7) Save filtered FAST-LIO PCD map (requires option 6 or 9 running)
  8) Record FAST-LIO rosbag (sensors + bag; 0 stops and saves)
  9) Replay FAST-LIO rosbag offline (bag + fast_lio with sim_time)
 10) Start navigation (pick PCD -> regen 2D map -> chassis + lidar + Nav2 + RViz)
 11) Start teleop-only  (chassis serial + keyboard; no lidar / IMU / EKF)
 12) Clean FAST-LIO PCD  (SOR + optional DBSCAN; removes dynamic-object ghosts)
 13) Intersect multiple FAST-LIO PCDs  (majority vote across sessions)
 14) Start 3D point-cloud navigation  (FAST_LIO + NDT + CMU local_planner + RViz; no 2D PGM)
 15) Start navigation with speed mode  (option 10 flow + selectable Nav2 speed)
  q) Quit
==============================
EOF
}

main() {
    if [ ! -f "${ROS_DISTRO_SETUP}" ]; then
        echo "ERROR: ROS 2 Humble not found at ${ROS_DISTRO_SETUP}" >&2
        exit 1
    fi

    while true; do
        print_menu
        read -r -p "Select an option: " choice
        case "${choice}" in
            0) action_kill_all ;;
            1) action_start_chassis ;;
            2) action_start_keyboard ;;
            3) action_start_slam ;;
            4) action_save_map ;;
            5) action_show_battery ;;
            6) action_start_fastlio ;;
            7) action_save_fastlio_map ;;
            8) action_record_fastlio_bag ;;
            9) action_replay_fastlio_bag ;;
            10) action_start_nav ;;
            11) action_start_teleop ;;
            12) action_clean_pcd ;;
            13) action_intersect_pcds ;;
            14) action_start_3d_nav ;;
            15) action_start_nav_speed_modes ;;
            q|Q|quit|exit) echo "Bye."; exit 0 ;;
            "") ;;
            *) echo "Unknown option: ${choice}" ;;
        esac
    done
}

main "$@"
