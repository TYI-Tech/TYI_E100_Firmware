#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${ROOT_DIR}/.env"
MACHINE_ENV_FILE="${ROOT_DIR}/machine.env"
LIVOX_CONFIG_FILE="${ROOT_DIR}/configs/livox/MID360_config.json"
BACKUP_DIR="${ROOT_DIR}/logs/config_backups"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

RESTART_STACK=true
APPLY_NETWORK=true
DRY_RUN=false

usage() {
  cat <<'EOF'
Usage:
  sudo ./scripts/configure_machine.sh [--dry-run] [--skip-network] [--skip-restart]

Options:
  --dry-run       Print planned changes without writing files or changing network state.
  --skip-network  Do not configure the host Ethernet interface IP.
  --skip-restart  Do not restart docker compose after updating config files.
  -h, --help      Show this help message.

This script reads machine-specific settings from:
  ./machine.env
EOF
}

log() {
  printf '[configure_machine] %s\n' "$*"
}

die() {
  printf '[configure_machine] ERROR: %s\n' "$*" >&2
  exit 1
}

load_machine_env() {
  if [[ ! -f "${MACHINE_ENV_FILE}" ]]; then
    die "Missing ${MACHINE_ENV_FILE}. Edit machine.env first."
  fi

  # shellcheck disable=SC1090
  set -a
  source "${MACHINE_ENV_FILE}"
  set +a
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=true
      ;;
    --skip-network)
      APPLY_NETWORK=false
      ;;
    --skip-restart)
      RESTART_STACK=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1"
      ;;
  esac
  shift
done

if [[ ! -f "${ENV_FILE}" ]]; then
  die "Missing ${ENV_FILE}. Run from firmware/base_stack_noetic and make sure .env exists."
fi

if [[ ! -f "${LIVOX_CONFIG_FILE}" ]]; then
  die "Missing ${LIVOX_CONFIG_FILE}."
fi

if [[ "${DRY_RUN}" != "true" && "${EUID}" -ne 0 ]]; then
  die "Run this script with sudo so it can update network configuration and restart docker."
fi

load_machine_env

FCU_DEVICE="${FCU_DEVICE:-/dev/ttyTHS0}"
FCU_BAUD="${FCU_BAUD:-115200}"

MID360_BD_LIST="${MID360_BD_LIST:-147MDLC40020242}"
MID360_SN_SUFFIX="${MID360_SN_SUFFIX:-42}"
MID360_LIDAR_IP="${MID360_LIDAR_IP:-}"

HOST_ETH_IFACE="${HOST_ETH_IFACE:-eth0}"
HOST_ETH_IP="${HOST_ETH_IP:-192.168.1.50}"
HOST_ETH_PREFIX="${HOST_ETH_PREFIX:-24}"
HOST_ETH_GATEWAY="${HOST_ETH_GATEWAY:-}"

MID360_CMD_DATA_PORT="${MID360_CMD_DATA_PORT:-56100}"
MID360_PUSH_MSG_PORT="${MID360_PUSH_MSG_PORT:-56200}"
MID360_POINT_DATA_PORT="${MID360_POINT_DATA_PORT:-56300}"
MID360_IMU_DATA_PORT="${MID360_IMU_DATA_PORT:-56400}"
MID360_LOG_DATA_PORT="${MID360_LOG_DATA_PORT:-56500}"

HOST_CMD_DATA_PORT="${HOST_CMD_DATA_PORT:-56101}"
HOST_PUSH_MSG_PORT="${HOST_PUSH_MSG_PORT:-56201}"
HOST_POINT_DATA_PORT="${HOST_POINT_DATA_PORT:-56301}"
HOST_IMU_DATA_PORT="${HOST_IMU_DATA_PORT:-56401}"
HOST_LOG_DATA_PORT="${HOST_LOG_DATA_PORT:-56501}"

MID360_SET_LOG_DATA_IP="${MID360_SET_LOG_DATA_IP:-false}"

derive_lidar_ip() {
  local suffix="$1"

  if [[ ! "${suffix}" =~ ^[0-9]{1,2}$ ]]; then
    die "MID360_SN_SUFFIX must be a 1-2 digit number, got '${suffix}'."
  fi

  local octet=$((100 + 10#${suffix}))
  if (( octet < 100 || octet > 199 )); then
    die "Derived LiDAR IP last octet ${octet} is out of supported range 100-199."
  fi

  printf '192.168.1.%d' "${octet}"
}

LIDAR_IP="${MID360_LIDAR_IP:-$(derive_lidar_ip "${MID360_SN_SUFFIX}")}"
FCU_URL="${FCU_DEVICE}:${FCU_BAUD}"

backup_file() {
  local src="$1"
  mkdir -p "${BACKUP_DIR}"
  cp "${src}" "${BACKUP_DIR}/$(basename "${src}").${TIMESTAMP}.bak"
}

ensure_runtime_dirs() {
  local runtime_owner_uid="${SUDO_UID:-$(id -u)}"
  local runtime_owner_gid="${SUDO_GID:-$(id -g)}"

  log "Preparing runtime directories under ${ROOT_DIR}/logs"
  mkdir -p "${ROOT_DIR}/logs/ros" "${BACKUP_DIR}"

  chown -R "${runtime_owner_uid}:${runtime_owner_gid}" "${ROOT_DIR}/logs"
  chmod 0775 "${ROOT_DIR}/logs" "${ROOT_DIR}/logs/ros" "${BACKUP_DIR}"
}

configure_env_file() {
  log "Updating ${ENV_FILE}"
  python3 - "${ENV_FILE}" "${FCU_URL}" "${MID360_BD_LIST}" <<'PY'
from pathlib import Path
import sys

env_path = Path(sys.argv[1])
fcu_url = sys.argv[2]
bd_list = sys.argv[3]

required = {
    "FCU_URL": fcu_url,
    "LIVOX_BD_LIST": bd_list,
}

lines = env_path.read_text().splitlines()
seen = set()
output = []
for line in lines:
    if "=" not in line or line.lstrip().startswith("#"):
        output.append(line)
        continue
    key, _ = line.split("=", 1)
    if key in required:
        output.append(f"{key}={required[key]}")
        seen.add(key)
    else:
        output.append(line)

for key, value in required.items():
    if key not in seen:
        output.append(f"{key}={value}")

env_path.write_text("\n".join(output) + "\n")
PY
}

configure_livox_json() {
  log "Updating ${LIVOX_CONFIG_FILE}"
  python3 - \
    "${LIVOX_CONFIG_FILE}" \
    "${HOST_ETH_IP}" \
    "${LIDAR_IP}" \
    "${MID360_CMD_DATA_PORT}" \
    "${MID360_PUSH_MSG_PORT}" \
    "${MID360_POINT_DATA_PORT}" \
    "${MID360_IMU_DATA_PORT}" \
    "${MID360_LOG_DATA_PORT}" \
    "${HOST_CMD_DATA_PORT}" \
    "${HOST_PUSH_MSG_PORT}" \
    "${HOST_POINT_DATA_PORT}" \
    "${HOST_IMU_DATA_PORT}" \
    "${HOST_LOG_DATA_PORT}" \
    "${MID360_SET_LOG_DATA_IP}" <<'PY'
from pathlib import Path
import json
import sys

(
    config_path,
    host_ip,
    lidar_ip,
    lidar_cmd_port,
    lidar_push_port,
    lidar_point_port,
    lidar_imu_port,
    lidar_log_port,
    host_cmd_port,
    host_push_port,
    host_point_port,
    host_imu_port,
    host_log_port,
    set_log_ip,
) = sys.argv[1:]

data = json.loads(Path(config_path).read_text())

mid360 = data.setdefault("MID360", {})
lidar_net_info = mid360.setdefault("lidar_net_info", {})
host_net_info = mid360.setdefault("host_net_info", {})
lidar_configs = data.setdefault("lidar_configs", [])
if not lidar_configs:
    raise SystemExit("MID360_config.json has no lidar_configs entry")

lidar_net_info["cmd_data_port"] = int(lidar_cmd_port)
lidar_net_info["push_msg_port"] = int(lidar_push_port)
lidar_net_info["point_data_port"] = int(lidar_point_port)
lidar_net_info["imu_data_port"] = int(lidar_imu_port)
lidar_net_info["log_data_port"] = int(lidar_log_port)

host_net_info["cmd_data_ip"] = host_ip
host_net_info["cmd_data_port"] = int(host_cmd_port)
host_net_info["push_msg_ip"] = host_ip
host_net_info["push_msg_port"] = int(host_push_port)
host_net_info["point_data_ip"] = host_ip
host_net_info["point_data_port"] = int(host_point_port)
host_net_info["imu_data_ip"] = host_ip
host_net_info["imu_data_port"] = int(host_imu_port)
host_net_info["log_data_ip"] = host_ip if set_log_ip.lower() == "true" else ""
host_net_info["log_data_port"] = int(host_log_port)

lidar_configs[0]["ip"] = lidar_ip

Path(config_path).write_text(json.dumps(data, indent=2) + "\n")
PY
}

configure_network() {
  log "Configuring Ethernet interface ${HOST_ETH_IFACE} -> ${HOST_ETH_IP}/${HOST_ETH_PREFIX}"

  if command -v nmcli >/dev/null 2>&1; then
    local conn_name
    conn_name="$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: -v dev="${HOST_ETH_IFACE}" '$2 == dev {print $1; exit}')"

    if [[ -z "${conn_name}" ]]; then
      conn_name="$(nmcli -t -f NAME,DEVICE connection show | awk -F: -v dev="${HOST_ETH_IFACE}" '$2 == dev {print $1; exit}')"
    fi

    if [[ -z "${conn_name}" ]]; then
      conn_name="tyi-mid360-${HOST_ETH_IFACE}"
      nmcli connection add type ethernet ifname "${HOST_ETH_IFACE}" con-name "${conn_name}" >/dev/null
    fi

    nmcli connection modify "${conn_name}" \
      connection.autoconnect yes \
      ipv4.method manual \
      ipv4.addresses "${HOST_ETH_IP}/${HOST_ETH_PREFIX}" \
      ipv4.never-default yes

    if [[ -n "${HOST_ETH_GATEWAY}" ]]; then
      nmcli connection modify "${conn_name}" ipv4.gateway "${HOST_ETH_GATEWAY}"
    else
      nmcli connection modify "${conn_name}" ipv4.gateway ""
    fi

    nmcli connection up "${conn_name}" >/dev/null
    return
  fi

  if command -v ip >/dev/null 2>&1; then
    ip addr flush dev "${HOST_ETH_IFACE}"
    ip addr add "${HOST_ETH_IP}/${HOST_ETH_PREFIX}" dev "${HOST_ETH_IFACE}"
    ip link set "${HOST_ETH_IFACE}" up
    log "Applied temporary IP with iproute2 because nmcli is unavailable."
    return
  fi

  die "Neither nmcli nor ip is available; cannot configure ${HOST_ETH_IFACE}."
}

restart_stack() {
  log "Restarting docker compose stack"
  (
    cd "${ROOT_DIR}"
    docker compose up -d --force-recreate
  )
}

print_summary() {
  cat <<EOF
[configure_machine] Summary
  FCU_URL=${FCU_URL}
  MID360_BD_LIST=${MID360_BD_LIST}
  HOST_ETH_IFACE=${HOST_ETH_IFACE}
  HOST_ETH_IP=${HOST_ETH_IP}/${HOST_ETH_PREFIX}
  MID360_LIDAR_IP=${LIDAR_IP}
  MID360 host ports=${HOST_CMD_DATA_PORT},${HOST_PUSH_MSG_PORT},${HOST_POINT_DATA_PORT},${HOST_IMU_DATA_PORT},${HOST_LOG_DATA_PORT}
  MID360 lidar ports=${MID360_CMD_DATA_PORT},${MID360_PUSH_MSG_PORT},${MID360_POINT_DATA_PORT},${MID360_IMU_DATA_PORT},${MID360_LOG_DATA_PORT}
EOF
}

print_summary

if [[ "${DRY_RUN}" == "true" ]]; then
  log "Dry run only. No files or network settings were changed."
  exit 0
fi

backup_file "${ENV_FILE}"
backup_file "${LIVOX_CONFIG_FILE}"

configure_env_file
configure_livox_json
ensure_runtime_dirs

if [[ "${APPLY_NETWORK}" == "true" ]]; then
  configure_network
fi

if [[ "${RESTART_STACK}" == "true" ]]; then
  restart_stack
fi

log "Completed successfully."
