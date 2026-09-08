#!/usr/bin/env bash
# Run FAST-LIO alone against a rosbag and open a 3D RViz view.
# Usage:
#   ./scripts/test_fastlio_replay.sh <bag_dir> [start_offset_sec]

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_DIR}/install/setup.bash"
FASTLIO_DIAGNOSTICS="${WORKSPACE_DIR}/scripts/fastlio_diagnostics.py"
RVIZ_CONFIG="${WORKSPACE_DIR}/scripts/rviz/fastlio_replay.rviz"
RUN_ROOT="${TRAYMOVER_FASTLIO_LOG_ROOT:-${HOME}/.ros/traymover_fastlio_replay}"

usage() {
    echo "Usage: $0 <bag_dir> [start_offset_sec]"
}

[[ $# -ge 1 && $# -le 2 ]] || { usage >&2; exit 2; }
BAG_PATH="$1"
START_OFFSET="${2:-0}"

[[ -d "${BAG_PATH}" ]] || { echo "Bag directory not found: ${BAG_PATH}" >&2; exit 1; }
[[ -f "${ROS_SETUP}" ]] || { echo "ROS Humble setup not found: ${ROS_SETUP}" >&2; exit 1; }
[[ -f "${WS_SETUP}" ]] || { echo "Workspace is not built: ${WS_SETUP}" >&2; exit 1; }
[[ -f "${FASTLIO_DIAGNOSTICS}" ]] || { echo "FAST-LIO diagnostics not found: ${FASTLIO_DIAGNOSTICS}" >&2; exit 1; }
[[ -f "${RVIZ_CONFIG}" ]] || { echo "RViz config not found: ${RVIZ_CONFIG}" >&2; exit 1; }
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${RUN_ROOT}/${RUN_ID}"
mkdir -p "${RUN_DIR}/processes" "${RUN_DIR}/ros" "${RUN_DIR}/fastlio"
printf 'bag=%s\nstart_offset=%s\nstarted_utc=%s\n' \
    "${BAG_PATH}" "${START_OFFSET}" "${RUN_ID}" \
    > "${RUN_DIR}/run_info.txt"
echo "[fastlio] Run logs: ${RUN_DIR}"

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="${RUN_DIR}/ros"

ros2 pkg prefix fast_lio >/dev/null 2>&1 || {
    echo "ROS package not available: fast_lio" >&2
    exit 1
}
ros2 bag info "${BAG_PATH}" > "${RUN_DIR}/bag_info.txt" 2>&1 || {
    echo "Unable to read rosbag: ${BAG_PATH}" >&2
    exit 1
}

pids=()
bag_pid=""
cleanup() {
    local pid
    if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
        kill -INT "${bag_pid}" 2>/dev/null || true
    fi
    for pid in "${pids[@]:-}"; do
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            kill -INT "${pid}" 2>/dev/null || true
        fi
    done
    wait "${bag_pid}" 2>/dev/null || true
    for pid in "${pids[@]:-}"; do
        [[ -n "${pid}" ]] && wait "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

start_process() {
    local name="$1"
    shift
    local log_file="${RUN_DIR}/processes/${name}.log"
    echo "[fastlio] Starting ${name}; log=${log_file}"
    "$@" > "${log_file}" 2>&1 &
    pids+=("$!")
}

fastlio_args=(
    ros2 run fast_lio fastlio_mapping
    --ros-args
    --params-file "${WORKSPACE_DIR}/src/traymover_robot_slam/FAST_LIO/config/traymover.yaml"
    -p use_sim_time:=true
    -p publish.path_en:=true
    -p publish.scan_publish_en:=true
    -p publish.dense_publish_en:=false
    -p publish.scan_bodyframe_pub_en:=true
    -p publish.map_en:=true
    -p diagnostics.frame_trace:=true
    -p diagnostics.min_effective_features:=${TRAYMOVER_FASTLIO_MIN_FEATURES:-1000}
    -p diagnostics.max_update_translation:=${TRAYMOVER_FASTLIO_MAX_UPDATE_TRANSLATION:-2.0}
)
start_process fastlio "${fastlio_args[@]}"

start_process diagnostics python3 "${FASTLIO_DIAGNOSTICS}" \
    --output-dir "${RUN_DIR}/fastlio" \
    --lidar-topic /point_cloud_raw \
    --imu-topic /imu/data_raw \
    --odom-topic /Odometry \
    --interval-sec "${TRAYMOVER_FASTLIO_LOG_INTERVAL_SEC:-2}"

start_process rviz rviz2 -d "${RVIZ_CONFIG}"

sleep "${TRAYMOVER_FASTLIO_STARTUP_SEC:-5}"
bag_args=("${BAG_PATH}" --clock --topics /point_cloud_raw /imu/data_raw /tf_static)
if [[ "${START_OFFSET}" != "0" ]]; then
    bag_args+=(--start-offset "${START_OFFSET}")
fi
echo "[fastlio] Playing rosbag; output=${RUN_DIR}/rosbag.log"
ros2 bag play "${bag_args[@]}" > "${RUN_DIR}/rosbag.log" 2>&1 &
bag_pid="$!"

echo "[fastlio] FAST-LIO visualization is running in RViz. Press Ctrl-C to stop."
wait "${bag_pid}" || true
echo "[fastlio] Bag playback finished. Logs remain in ${RUN_DIR}"
sleep "${TRAYMOVER_FASTLIO_SETTLE_SEC:-3}"

rosout_log="${RUN_DIR}/fastlio/fastlio_rosout.log"
event_log="${RUN_DIR}/fastlio/fastlio_events.log"
reject_count=0
if [[ -f "${rosout_log}" ]]; then
    reject_count="$(grep -c 'stage=REJECT_UPDATE' "${rosout_log}" || true)"
fi
{
    printf 'run_dir=%s\n' "${RUN_DIR}"
    printf 'reject_update_count=%s\n' "${reject_count}"
    printf 'last_frame_trace:\n'
    [[ -f "${rosout_log}" ]] && grep 'FAST_LIO frame=' "${rosout_log}" | tail -n 1 || true
    printf 'last_diagnostic_state:\n'
    [[ -f "${event_log}" ]] && grep 'event=REPORT' "${event_log}" | tail -n 1 || true
} > "${RUN_DIR}/fastlio_test_summary.txt"
echo "[fastlio] Summary: ${RUN_DIR}/fastlio_test_summary.txt"
