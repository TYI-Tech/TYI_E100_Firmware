#!/usr/bin/env bash
set -euo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:${ROS_MASTER_PORT:-11311}}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export UAV_CONFIG_DIR="${UAV_CONFIG_DIR:-/opt/uav/configs}"

for key in LIVOX_BD_LIST FCU_URL GCS_URL MAVROS_TGT_SYSTEM; do
  value="${!key:-}"
  if [[ -z "${value}" || "${value}" == "UNCONFIGURED" ]]; then
    echo "Required flight setting ${key} is not configured; refusing to start flight-core." >&2
    exit 78
  fi
done

if [[ "${GCS_URL}" == *127.0.0.1* || "${GCS_URL}" == *localhost* ]]; then
  echo "GCS_URL must target the actual ground station, not loopback." >&2
  exit 78
fi

if [[ ! "${MAVROS_TGT_SYSTEM}" =~ ^[1-9][0-9]{0,2}$ ]] || (( MAVROS_TGT_SYSTEM > 255 )); then
  echo "MAVROS_TGT_SYSTEM must be a configured integer from 1 to 255." >&2
  exit 78
fi

if [[ "${FCU_URL}" =~ ^(/dev/[^:]+):([1-9][0-9]*)$ ]]; then
  FCU_DEVICE_PATH="${BASH_REMATCH[1]}"
else
  echo "FCU_URL must use the form /dev/<serial-device>:<baud-rate>." >&2
  exit 78
fi
if [[ ! -c "${FCU_DEVICE_PATH}" ]]; then
  echo "Flight-controller serial device ${FCU_DEVICE_PATH} is not present." >&2
  exit 78
fi

python3 - "${UAV_CONFIG_DIR}/livox/MID360_config.json" <<'PY'
import json
import ipaddress
import sys
from pathlib import Path

config = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
mid360 = config.get("Mid360s") or config.get("MID360", {})
host_ip = mid360.get("host_net_info", {}).get("cmd_data_ip", "")
lidar_ip = (config.get("lidar_configs") or [{}])[0].get("ip", "")
if not host_ip or not lidar_ip:
    raise SystemExit("MID360 host and LiDAR IPs must be configured before starting flight-core.")
for value in (host_ip, lidar_ip):
    ipaddress.IPv4Address(value)
PY

source /opt/ros/noetic/setup.bash

if [[ -f /opt/uav/base_stack/workspace/install/setup.bash ]]; then
  source /opt/uav/base_stack/workspace/install/setup.bash
elif [[ -f /opt/uav/base_stack/workspace/install_isolated/setup.bash ]]; then
  source /opt/uav/base_stack/workspace/install_isolated/setup.bash
else
  echo "workspace setup.bash not found" >&2
  exit 1
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/opt/uav/logs/ros}"
export TYI_TIMEBASE_MODE="${TYI_TIMEBASE_MODE:-monotonic_raw_internal}"
export TYI_TIMEBASE_FILE="${TYI_TIMEBASE_FILE:-/tmp/tyi_timebase.env}"
DEFAULT_ROS_LOG_DIR="${ROS_LOG_DIR}"
FALLBACK_ROS_LOG_DIR="${HOME:-/home/uav}/.ros/log"

mkdir -p "${ROS_LOG_DIR}" 2>/dev/null || true
if [[ ! -d "${ROS_LOG_DIR}" || ! -w "${ROS_LOG_DIR}" ]]; then
  echo "ROS log dir '${DEFAULT_ROS_LOG_DIR}' is not writable for user '${USER:-uav}', falling back to '${FALLBACK_ROS_LOG_DIR}'." >&2
  export ROS_LOG_DIR="${FALLBACK_ROS_LOG_DIR}"
  mkdir -p "${ROS_LOG_DIR}"
fi

if [[ "${TYI_TIMEBASE_MODE}" == "monotonic_raw_internal" || "${TYI_TIMEBASE_MODE}" == "monotonic_raw_epoch" ]]; then
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
mode = os.environ.get("TYI_TIMEBASE_MODE", "monotonic_raw_internal")
timebase_ns = mono_ns if mode == "monotonic_raw_internal" else unix_ns

tmp = target.with_suffix(target.suffix + ".tmp")
tmp.write_text(
    f"TYI_TIMEBASE_MODE={mode}\n"
    f"INTERNAL_ANCHOR_NS={timebase_ns}\n"
    f"UNIX_ANCHOR_NS={timebase_ns}\n"
    f"MONOTONIC_RAW_ANCHOR_NS={mono_ns}\n",
    encoding="ascii",
)
os.replace(tmp, target)
PY
fi


# The Livox launch prefix waits for its source route independently of MAVROS.

roscore -p "${ROS_MASTER_PORT:-11311}" >"${ROS_LOG_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!
ROSCLOCK_PID=""

cleanup() {
  if [[ -n "${ROSCLOCK_PID}" ]]; then
    kill "${ROSCLOCK_PID}" 2>/dev/null || true
  fi
  kill "${ROSCORE_PID}" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 30); do
  if rosnode list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

TIMEBASE_SCRIPT_DIR="/opt/uav/base_stack/workspace/src/uav_base_bringup/scripts"
export PYTHONPATH="${TIMEBASE_SCRIPT_DIR}${PYTHONPATH:+:${PYTHONPATH}}"

start_shared_timebase_clock() {
  if [[ "${TYI_TIMEBASE_MODE}" != "monotonic_raw_internal" && "${TYI_TIMEBASE_MODE}" != "monotonic_raw_epoch" ]]; then
    return 0
  fi

  local clock_script="${TIMEBASE_SCRIPT_DIR}/shared_timebase_clock.py"
  if [[ ! -f "${clock_script}" ]]; then
    echo "Required clock publisher missing: ${clock_script}" >&2
    return 1
  fi

  rosparam set /use_sim_time true
  python3 "${clock_script}" >"${ROS_LOG_DIR}/shared_timebase_clock.log" 2>&1 &
  ROSCLOCK_PID=$!

  if ! timeout 10s python3 "${clock_script}" --check --timeout 5 || ! kill -0 "${ROSCLOCK_PID}" 2>/dev/null; then
    echo "Shared ROS clock did not advance in the configured timebase; refusing to start flight nodes." >&2
    return 1
  fi
  echo "Shared internal ROS clock started from ${TYI_TIMEBASE_FILE}." >&2
}

start_shared_timebase_clock

configure_mavros_message_intervals() {
  local rate_hz="${MAVROS_LOCAL_POSITION_RATE_HZ:-20}"
  local retry_sec="${MAVROS_MESSAGE_INTERVAL_RETRY_SEC:-5}"
  local startup_delay_sec="${MAVROS_MESSAGE_INTERVAL_STARTUP_DELAY_SEC:-35}"
  local refresh_sec="${MAVROS_MESSAGE_INTERVAL_REFRESH_SEC:-300}"

  local local_position_enabled=true
  if ! python3 - "${rate_hz}" >/dev/null 2>&1 <<'PYCHECK'
import math
import sys
rate = float(sys.argv[1])
sys.exit(0 if math.isfinite(rate) and rate > 0.0 else 1)
PYCHECK
  then
    echo "MAVROS local position message interval guard disabled (MAVROS_LOCAL_POSITION_RATE_HZ=${rate_hz})." >&2
    local_position_enabled=false
  fi

  (
    until rosservice info /mavros/set_message_interval >/dev/null 2>&1; do
      sleep "${retry_sec}"
    done
    if [[ "${startup_delay_sec}" != "0" ]]; then
      sleep "${startup_delay_sec}"
    fi

    while true; do
      if [[ "${local_position_enabled}" == true ]]; then
        # 32=LOCAL_POSITION_NED, 64=LOCAL_POSITION_NED_COV. PX4 may support either;
        # requesting both keeps /mavros/local_position/odom fresh without affecting flight mode.
        timeout 5 rosservice call /mavros/set_message_interval "message_id: 32
message_rate: ${rate_hz}" >/dev/null 2>&1 || true
        timeout 5 rosservice call /mavros/set_message_interval "message_id: 64
message_rate: ${rate_hz}" >/dev/null 2>&1 || true
        echo "Requested MAVROS local position messages at ${rate_hz}Hz." >&2
      fi
      # 291=ESC_STATUS, carrying the four motor RPMs on the companion link.
      timeout 5 rosservice call /mavros/set_message_interval "message_id: 291
message_rate: 100" >/dev/null 2>&1 || true
      sleep "${refresh_sec}"
    done
  ) &
}

configure_mavros_message_intervals

launch_file="${UAV_BASE_STACK_LAUNCH:-/opt/uav/base_stack/workspace/src/uav_base_bringup/launch/base_stack.launch}"
if [[ -f "${launch_file}" ]]; then
  exec roslaunch "${launch_file}"
fi
exec roslaunch uav_base_bringup base_stack.launch
