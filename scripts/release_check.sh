#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/lib/common.sh"

cd "${ROOT_DIR}"

VERSION="$(tr -d '[:space:]' < VERSION)"
CHANGELOG_FILE="CHANGELOG.md"
ZH_RELEASE_NOTES="docs/zh_CN/版本说明.md"
EN_RELEASE_NOTES="docs/en_US/release_notes.md"

require_file "VERSION"
require_file "${CHANGELOG_FILE}"
require_file "${ZH_RELEASE_NOTES}"
require_file "${EN_RELEASE_NOTES}"

if [[ -z "${VERSION}" ]]; then
  printf '[tyi_uav_firmware] ERROR: VERSION is empty.\n' >&2
  exit 1
fi

check_contains() {
  local file="$1"
  local pattern="$2"
  if ! grep -Fq "${pattern}" "${file}"; then
    printf '[tyi_uav_firmware] ERROR: %s does not contain "%s"\n' "${file}" "${pattern}" >&2
    exit 1
  fi
}

log "checking release metadata for version ${VERSION}"
check_contains "${CHANGELOG_FILE}" "## ${VERSION}"
check_contains "${ZH_RELEASE_NOTES}" "## ${VERSION}"
check_contains "${EN_RELEASE_NOTES}" "## ${VERSION}"

if ! git diff --quiet -- README.md README_EN.md docs README_CN.md CHANGELOG.md VERSION scripts 2>/dev/null; then
  printf '[tyi_uav_firmware] ERROR: release-related files have uncommitted changes.\n' >&2
  exit 1
fi

log "release metadata is consistent"
