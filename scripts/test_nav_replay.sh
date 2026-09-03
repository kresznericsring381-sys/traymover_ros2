#!/usr/bin/env bash
# Replay a recorded LiDAR/IMU rosbag through FAST-LIO + NDT localization + Nav2.
# Usage:
#   ./scripts/test_nav_replay.sh [--mode static|continuous] <map.pcd> <bag_dir> [start_offset_sec]
#
# static mode: short recorded window for manual initial pose -> NDT matching.
# continuous mode: FAST-LIO + NDT + Nav2 replay test (default).

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_DIR}/install/setup.bash"
PCD2PGM="${WORKSPACE_DIR}/src/traymover_robot_nav/scripts/pcd2pgm.py"
MONITOR_RVIZ="${WORKSPACE_DIR}/src/traymover_robot_nav/rviz/traymover_sensor_monitor.rviz"
RUNTIME_DIR="/tmp/traymover_nav_runtime"

usage() {
    echo "Usage: $0 [--mode static|continuous] <map.pcd> <bag_dir> [start_offset_sec]"
    echo "After startup, type: play | once | pause | resume | stop | restart | status | quit"
}

MODE="continuous"
if [[ "${1:-}" == "--mode" ]]; then
    [[ ${#} -ge 3 ]] || { usage >&2; exit 2; }
    MODE="$2"
    shift 2
fi

if [[ "${MODE}" != "static" && "${MODE}" != "continuous" ]]; then
    echo "Invalid mode: ${MODE} (choose static or continuous)" >&2
    exit 2
fi
if [[ ${#} -lt 2 || ${#} -gt 3 ]]; then
    usage >&2
    exit 2
fi

MAP_PATH="$1"
BAG_PATH="$2"
START_OFFSET="${3:-0}"

[[ "${MAP_PATH}" = /* ]] || MAP_PATH="${WORKSPACE_DIR}/${MAP_PATH}"
[[ -d "${BAG_PATH}" ]] || { echo "Bag directory not found: ${BAG_PATH}" >&2; exit 1; }
[[ -f "${MAP_PATH}" ]] || { echo "PCD map not found: ${MAP_PATH}" >&2; exit 1; }
[[ -f "${ROS_SETUP}" ]] || { echo "ROS Humble setup not found: ${ROS_SETUP}" >&2; exit 1; }
[[ -f "${WS_SETUP}" ]] || { echo "Workspace is not built: ${WS_SETUP}" >&2; exit 1; }
[[ -f "${PCD2PGM}" ]] || { echo "PCD conversion script not found: ${PCD2PGM}" >&2; exit 1; }
[[ -f "${MONITOR_RVIZ}" ]] || { echo "Sensor monitor RViz config not found: ${MONITOR_RVIZ}" >&2; exit 1; }

# ROS setup scripts reference optional variables that may be unset.  Source
# them with nounset temporarily disabled, then restore strict shell checking.
set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

for pkg in fast_lio lidar_localization_ros2 ndt_omp_ros2 traymover_robot_nav; do
    if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
        echo "ROS package not available: ${pkg}" >&2
        exit 1
    fi
done

if ! ros2 bag info "${BAG_PATH}" >/dev/null 2>&1; then
    echo "Unable to read rosbag: ${BAG_PATH}" >&2
    exit 1
fi

launch_pids=()
control_pid=""
control_fifo=""
cleanup() {
    local pid
    if [[ -n "${control_pid:-}" ]] && kill -0 "${control_pid}" 2>/dev/null; then
        kill "${control_pid}" 2>/dev/null || true
    fi
    [[ -n "${control_fifo:-}" ]] && rm -f "${control_fifo}"
    if [[ -n "${bag_pid:-}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
        [[ "${bag_paused:-0}" -eq 1 ]] && kill -CONT "${bag_pid}" 2>/dev/null || true
        kill -INT "${bag_pid}" 2>/dev/null || true
    fi
    if [[ -n "${bag_pid:-}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
        kill -INT "${bag_pid}" 2>/dev/null || true
    fi
    for pid in "${launch_pids[@]:-}"; do
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            echo "[traymover] Stopping launch (PID ${pid})..."
            kill -INT "${pid}" 2>/dev/null || true
        fi
    done
    for pid in "${launch_pids[@]:-}"; do
        [[ -n "${pid}" ]] && wait "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

if [[ "${MODE}" == "continuous" ]]; then
    mkdir -p "${RUNTIME_DIR}"
    map_name="$(basename "${MAP_PATH}" .pcd)"
    map_prefix="${RUNTIME_DIR}/${map_name}_replay_2d"
    map_yaml="${map_prefix}.yaml"

    echo "[traymover] Generating 2D Nav2 map from: ${MAP_PATH}"
    python3 "${PCD2PGM}" \
        --pcd "${MAP_PATH}" \
        --out "${map_prefix}" \
        --z-min 0.10 \
        --z-max 2.20 \
        --min-points 1 \
        --dilate 2 \
        --min-region-cells 2
    [[ -f "${map_yaml}" ]] || { echo "2D map generation failed: ${map_yaml}" >&2; exit 1; }

    echo "[traymover] Starting Nav2 + FAST-LIO + NDT (hardware bringup disabled)."
    ros2 launch traymover_robot_nav traymover_nav.launch.py \
        use_sim_time:=true \
        bringup_hardware:=false \
        launch_rviz:=true \
        "pcd_path:=${MAP_PATH}" \
        "map:=${map_yaml}" &
    launch_pids+=("$!")
    echo "[traymover] Starting sensor-monitor RViz (camera + raw LiDAR)."
    rviz2 -d "${MONITOR_RVIZ}" &
    launch_pids+=("$!")
else
    echo "[traymover] Starting point-cloud preprocessing (no Nav2)."
    ros2 launch traymover_robot_nav navigation_pointcloud.launch.py \
        use_sim_time:=true \
        input_cloud_topic:=/point_cloud_raw \
        localization_cloud_topic:=/point_cloud_localization \
        nav_cloud_topic:=/point_cloud_nav \
        publish_scan:=false &
    launch_pids+=("$!")

    echo "[traymover] Starting FAST-LIO + NDT localization only."
    ros2 launch traymover_robot_nav lidar_localization.launch.py \
        use_sim_time:=true \
        "pcd_path:=${MAP_PATH}" \
        cloud_topic:=/point_cloud_localization &
    launch_pids+=("$!")

    echo "[traymover] Starting RViz (set 2D Pose Estimate manually)."
    rviz2 -d "${WORKSPACE_DIR}/src/traymover_robot_nav/rviz/traymover_nav.rviz" &
    launch_pids+=("$!")
    echo "[traymover] Starting sensor-monitor RViz (camera + raw LiDAR)."
    rviz2 -d "${MONITOR_RVIZ}" &
    launch_pids+=("$!")
fi

echo "[traymover] Waiting for stack to initialize..."
sleep 8

# Camera topics are intentionally excluded: they are not consumed by this test.
bag_args=("${BAG_PATH}" --clock --topics \
    /point_cloud_raw /imu/data_raw /tf_static \
    /camera/camera/color/image_raw /camera/camera/color/camera_info)
if [[ "${START_OFFSET}" != "0" ]]; then
    bag_args+=(--start-offset "${START_OFFSET}")
fi
bag_pid=""
bag_paused=0
stop_bag() {
    if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
        kill -INT "${bag_pid}" 2>/dev/null || true
        wait "${bag_pid}" 2>/dev/null || true
    fi
    bag_pid=""
    bag_paused=0
}
play_bag() {
    stop_bag
    echo "[traymover] Starting loop playback."
    ros2 bag play "${bag_args[@]}" --loop &
    bag_pid="$!"
    bag_paused=0
}
once_bag() {
    stop_bag
    echo "[traymover] Playing bag once."
    ros2 bag play "${bag_args[@]}" &
    bag_pid="$!"
    bag_paused=0
}
pause_bag() {
    if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null && [[ "${bag_paused}" -eq 0 ]]; then
        kill -STOP "${bag_pid}" 2>/dev/null || true
        bag_paused=1
        echo "[traymover] rosbag paused; set initial pose, then use resume."
    else
        echo "[traymover] rosbag is not running or already paused."
    fi
}
resume_bag() {
    if [[ -n "${bag_pid}" ]] && [[ "${bag_paused}" -eq 1 ]]; then
        kill -CONT "${bag_pid}" 2>/dev/null || true
        bag_paused=0
        echo "[traymover] rosbag resumed."
    else
        echo "[traymover] rosbag is not paused."
    fi
}
status_bag() {
    if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
        echo "[traymover] rosbag is running (PID ${bag_pid})."
    else
        bag_pid=""
        echo "[traymover] rosbag is stopped."
    fi
}

start_control_terminal() {
    control_fifo="${RUNTIME_DIR}/control.fifo"
    mkdir -p "${RUNTIME_DIR}"
    rm -f "${control_fifo}"
    mkfifo "${control_fifo}"

    local fifo_q
    printf -v fifo_q '%q' "${control_fifo}"
    local control_cmd
    control_cmd="echo 'Traymover control: play | once | pause | resume | stop | restart | status | quit'; while true; do printf 'traymover> '; IFS= read -r c || exit 0; printf '%s\\n' \"\$c\" > ${fifo_q}; case \"\${c,,}\" in quit|exit|q) exit 0;; esac; done"

    local terminal_cmd=""
    if command -v x-terminal-emulator >/dev/null 2>&1; then
        terminal_cmd="x-terminal-emulator"
    elif command -v gnome-terminal >/dev/null 2>&1; then
        terminal_cmd="gnome-terminal"
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        terminal_cmd="xfce4-terminal"
    elif command -v xterm >/dev/null 2>&1; then
        terminal_cmd="xterm"
    fi

    if [[ -z "${terminal_cmd}" ]]; then
        echo "[traymover] No terminal emulator found; use this terminal for control."
        return 1
    fi

    case "${terminal_cmd}" in
        gnome-terminal) gnome-terminal -- bash -lc "${control_cmd}" & ;;
        xfce4-terminal) xfce4-terminal --command="bash -lc ${control_cmd@Q}" & ;;
        xterm) xterm -e bash -lc "${control_cmd}" & ;;
        *) x-terminal-emulator -e bash -lc "${control_cmd}" & ;;
    esac
    control_pid="$!"
    # Open read/write so opening the FIFO does not block before the new window
    # starts writing commands.
    exec 3<>"${control_fifo}"
    echo "[traymover] Control terminal started (PID ${control_pid})."
    return 0
}

control_external=1
if ! start_control_terminal; then
    control_external=0
    rm -f "${control_fifo}"
    control_fifo=""
    echo "[traymover] Control commands in this terminal: play, once, stop, restart, status, quit"
fi
while true; do
    if [[ "${control_external}" -eq 1 ]]; then
        IFS= read -r -u 3 command || command="quit"
    else
        read -r -p "traymover> " command || command="quit"
    fi
    case "${command,,}" in
        play|start) play_bag ;;
        once) once_bag ;;
        pause) pause_bag ;;
        resume|continue) resume_bag ;;
        stop) stop_bag ;;
        restart) play_bag ;;
        status) status_bag ;;
        quit|exit|q) stop_bag; break ;;
        "") ;;
        *) echo "Commands: play, once, pause, resume, stop, restart, status, quit" ;;
    esac
done
