#!/usr/bin/env bash
set -euo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:${ROS_MASTER_PORT:-11311}}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export STRICT_TOPIC_HEALTHCHECK="${STRICT_TOPIC_HEALTHCHECK:-false}"
source /opt/ros/noetic/setup.bash

if [[ -f /opt/uav/base_stack/workspace/install/setup.bash ]]; then
  source /opt/uav/base_stack/workspace/install/setup.bash
elif [[ -f /opt/uav/base_stack/workspace/install_isolated/setup.bash ]]; then
  source /opt/uav/base_stack/workspace/install_isolated/setup.bash
else
  echo "workspace setup.bash not found" >&2
  exit 1
fi

required_nodes=(
  "/livox_lidar_publisher2"
  "/fastlio_to_mavros"
  "/mavros"
)

if [[ "${STRICT_TOPIC_HEALTHCHECK}" != "true" ]]; then
  nodes="$(rosnode list 2>/dev/null || true)"
  for node in "${required_nodes[@]}"; do
    if ! grep -qx "${node}" <<<"${nodes}"; then
      echo "missing node: ${node}" >&2
      exit 1
    fi
  done
  exit 0
fi

required_topics=(
  "/livox/lidar"
  "/livox/imu"
  "/robot/dlio/odom_node/odom"
  "/mavros/vision_pose/pose"
  "/mavros/state"
)

topics="$(rostopic list 2>/dev/null || true)"
for topic in "${required_topics[@]}"; do
  if ! grep -qx "${topic}" <<<"${topics}"; then
    echo "missing topic: ${topic}" >&2
    exit 1
  fi
done
