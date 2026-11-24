#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/lib/common.sh"
run_compose exec -T flight-core /opt/uav/base_stack/docker/runtime/healthcheck.sh </dev/null
printf 'flight-core healthy\n'
