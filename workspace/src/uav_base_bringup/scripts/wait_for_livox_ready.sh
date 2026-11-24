#!/usr/bin/env bash
set -euo pipefail

printf 'Waiting for sustained Livox topics before starting %s\n' "$*" >&2
python3 "$(dirname "$0")/wait_for_livox_ready.py"
exec "$@"
