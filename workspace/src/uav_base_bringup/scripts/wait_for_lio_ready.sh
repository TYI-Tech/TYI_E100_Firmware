#!/usr/bin/env bash
set -euo pipefail

wait_timeout="${LIO_ODOM_WAIT_TIMEOUT_SEC:-60}"
rate_sample_secs="${LIO_ODOM_RATE_SAMPLE_SEC:-4}"
odom_min_hz="${LIO_ODOM_MIN_HZ:-20}"
settle_secs="${LIO_ODOM_STABILIZE_SEC:-2}"
odom_topic="${LIO_ODOM_TOPIC:-/tyi/e100/fastlio2/odom}"

echo "Waiting for LiDAR odometry before starting $*" >&2

rate_is_enough() {
  python3 - "$1" "$2" <<'PYRATE' >/dev/null 2>&1
import math
import sys
rate = float(sys.argv[1])
minimum = float(sys.argv[2])
sys.exit(0 if math.isfinite(rate) and rate >= minimum else 1)
PYRATE
}

# Use kernel uptime so a clock correction at boot cannot expire the wait.
read -r uptime _ < /proc/uptime
deadline=$(( ${uptime%.*} + wait_timeout ))
while read -r uptime _ < /proc/uptime && (( ${uptime%.*} < deadline )); do
  output="$(timeout "${rate_sample_secs}" rostopic hz "${odom_topic}" 2>&1 || true)"
  rate="$(printf '%s\n' "${output}" | awk '/average rate:/ {value=$3} END {print value}')"
  if rate_is_enough "${rate:-0}" "${odom_min_hz}"; then
    echo "LiDAR odometry ready at ${rate}Hz (min ${odom_min_hz}Hz)" >&2
    if [[ "${settle_secs}" != "0" ]]; then
      sleep "${settle_secs}"
    fi
    exec "$@"
  fi
  echo "Waiting for ${odom_topic} >= ${odom_min_hz}Hz; last rate=${rate:-none}" >&2
  sleep 1
done

echo "Timed out waiting for LiDAR odometry; starting $* anyway so node-level diagnostics remain visible" >&2
exec "$@"
