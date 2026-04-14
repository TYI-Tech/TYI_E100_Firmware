#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/lib/common.sh"

cd "${ROOT_DIR}"
require_file ".env"
require_file "machine.env"
require_file "configs/livox/MID360_config.json"
sudocmd="sudo"
if [[ "${EUID}" -eq 0 ]]; then
  sudocmd=""
fi
${sudocmd} bash "${SCRIPT_DIR}/configure_machine.sh"
run_compose build
run_compose up -d
