#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

cd "${ROOT_DIR}"
case "${1:---pull}" in
  --pull|--local|--build) mode="${1:---pull}" ;;
  -h|--help) printf 'Usage: bash ./deploy.sh [--pull|--local|--build]\n'; exit 0 ;;
  *) printf 'Usage: bash ./deploy.sh [--pull|--local|--build]\n' >&2; exit 2 ;;
esac

require_file machine.env
case "${mode}" in
  --pull)
    tag="v$(<VERSION)"
    archive="tyi-e100-firmware-${tag}-linux-arm64.tar.zst"
    checksum="${archive}.sha256"
    release_url="https://github.com/TYI-Tech/TYI_E100_Firmware/releases/download/${tag}"
    download_dir=$(mktemp -d)
    trap 'rm -rf "${download_dir}"' EXIT
    curl --fail --location --retry 3 --output "${download_dir}/${archive}" "${release_url}/${archive}"
    curl --fail --location --retry 3 --output "${download_dir}/${checksum}" "${release_url}/${checksum}"
    (
      cd "${download_dir}"
      sha256sum -c "${checksum}"
      if docker info >/dev/null 2>&1; then
        zstd -dc "${archive}" | docker load
      else
        zstd -dc "${archive}" | sudo docker load
      fi
    )
    ;;
  --build) run_compose_build_mode build flight-core ;;
esac

if [[ "${EUID}" -eq 0 ]]; then
  bash ./scripts/configure_machine.sh --skip-network --skip-restart
else
  sudo bash ./scripts/configure_machine.sh --skip-network --skip-restart
fi

run_compose up -d --pull never --wait --wait-timeout 120 flight-core
