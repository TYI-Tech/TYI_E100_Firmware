#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

cd "${ROOT_DIR}"

if ! command -v docker >/dev/null 2>&1; then
  printf '[tyi_uav_firmware] ERROR: docker is not installed or not in PATH.\n' >&2
  exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
  printf '[tyi_uav_firmware] ERROR: docker compose is unavailable.\n' >&2
  exit 1
fi

require_file ".env.example"
require_file "machine.env.example"
require_file "configs/livox/MID360_config.example.json"

if [[ ! -e machine.env ]]; then
  cp -n machine.env.example machine.env
fi
require_file "machine.env"

log "host check passed"
