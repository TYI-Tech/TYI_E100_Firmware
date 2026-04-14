#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

log() {
  printf '[base_stack_customer] %s\n' "$*"
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

  printf '[base_stack_customer] ERROR: docker daemon is not accessible and sudo is unavailable.\n' >&2
  exit 1
}

run_compose() {
  cd "${ROOT_DIR}"
  local cmd
  cmd="$(compose_cmd)"
  # shellcheck disable=SC2086
  ${cmd} "$@"
}

require_file() {
  local path="$1"
  if [[ ! -f "${ROOT_DIR}/${path}" ]]; then
    printf '[base_stack_customer] ERROR: missing required file %s\n' "${path}" >&2
    exit 1
  fi
}
