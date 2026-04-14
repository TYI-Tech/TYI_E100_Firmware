#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

cd "${ROOT_DIR}"

if ! command -v docker >/dev/null 2>&1; then
  printf '[base_stack_customer] ERROR: docker is not installed or not in PATH.\n' >&2
  exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
  printf '[base_stack_customer] ERROR: docker compose is unavailable.\n' >&2
  exit 1
fi

require_file ".env"
require_file "machine.env"
require_file "configs/livox/MID360_config.json"

log "host check passed"
