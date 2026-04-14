#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

log() {
  printf '[tyi_uav_firmware] %s\n' "$*"
}

compose_cmd() {
  if docker info >/dev/null 2>&1; then
    printf 'docker compose'
    return
  fi

  if command -v sudo >/dev/null 2>&1; then
    printf 'sudo docker compose'
    return
  fi

  printf '[tyi_uav_firmware] ERROR: docker daemon is not accessible and sudo is unavailable.\n' >&2
  exit 1
}

run_compose() {
  cd "${ROOT_DIR}"
  local cmd
  cmd="$(compose_cmd)"
  # shellcheck disable=SC2086
  ${cmd} -f docker-compose.yml "$@"
}

run_compose_build_mode() {
  cd "${ROOT_DIR}"
  local cmd
  cmd="$(compose_cmd)"
  # shellcheck disable=SC2086
  ${cmd} -f docker-compose.yml -f docker-compose.build.yml "$@"
}

require_file() {
  local path="$1"
  if [[ ! -f "${ROOT_DIR}/${path}" ]]; then
    printf '[tyi_uav_firmware] ERROR: missing required file %s\n' "${path}" >&2
    exit 1
  fi
}
