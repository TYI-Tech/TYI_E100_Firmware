#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"
if [[ "${1:-}" == "--build" ]]; then
  run_compose_build_mode build flight-core
fi
run_compose up -d --pull never flight-core
