#!/usr/bin/env bash
set -euo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:${ROS_MASTER_PORT:-11311}}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
source /opt/ros/noetic/setup.bash

if [[ -f /opt/uav/base_stack/workspace/install/setup.bash ]]; then
  source /opt/uav/base_stack/workspace/install/setup.bash
elif [[ -f /opt/uav/base_stack/workspace/install_isolated/setup.bash ]]; then
  source /opt/uav/base_stack/workspace/install_isolated/setup.bash
else
  echo "workspace setup.bash not found" >&2
  exit 1
fi

export UAV_CONFIG_DIR="${UAV_CONFIG_DIR:-/opt/uav/configs}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/opt/uav/logs/ros}"
export TYI_TIMEBASE_MODE="${TYI_TIMEBASE_MODE:-monotonic_raw_epoch}"
export TYI_TIMEBASE_FILE="${TYI_TIMEBASE_FILE:-/tmp/tyi_timebase.env}"
DEFAULT_ROS_LOG_DIR="${ROS_LOG_DIR}"
FALLBACK_ROS_LOG_DIR="${HOME:-/home/uav}/.ros/log"

mkdir -p "${ROS_LOG_DIR}" 2>/dev/null || true
if [[ ! -d "${ROS_LOG_DIR}" || ! -w "${ROS_LOG_DIR}" ]]; then
  echo "ROS log dir '${DEFAULT_ROS_LOG_DIR}' is not writable for user '${USER:-uav}', falling back to '${FALLBACK_ROS_LOG_DIR}'." >&2
  export ROS_LOG_DIR="${FALLBACK_ROS_LOG_DIR}"
  mkdir -p "${ROS_LOG_DIR}"
fi

if [[ "${TYI_TIMEBASE_MODE}" == "monotonic_raw_epoch" ]]; then
  python3 - "${TYI_TIMEBASE_FILE}" <<'PY'
import os
import pathlib
import sys
import time

target = pathlib.Path(sys.argv[1])
target.parent.mkdir(parents=True, exist_ok=True)

clock_monotonic_raw = getattr(time, "CLOCK_MONOTONIC_RAW", None)
mono_ns = time.clock_gettime_ns(clock_monotonic_raw) if clock_monotonic_raw is not None else time.monotonic_ns()
unix_ns = time.time_ns()

tmp = target.with_suffix(target.suffix + ".tmp")
tmp.write_text(
    "TYI_TIMEBASE_MODE=monotonic_raw_epoch\n"
    f"UNIX_ANCHOR_NS={unix_ns}\n"
    f"MONOTONIC_RAW_ANCHOR_NS={mono_ns}\n",
    encoding="ascii",
)
os.replace(tmp, target)
PY
fi

roscore -p "${ROS_MASTER_PORT:-11311}" >"${ROS_LOG_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!

cleanup() {
  kill "${ROSCORE_PID}" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 30); do
  if rosnode list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

exec roslaunch uav_base_bringup base_stack.launch
