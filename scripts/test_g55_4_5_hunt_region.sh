#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="${TMPDIR:-/tmp}/host_g55_4_5_hunt_region"

${CXX:-c++} \
  -std=c++17 \
  -Wall \
  -Wextra \
  -Werror \
  -I"${repo_root}/src" \
  "${repo_root}/test/host_g55_4_5_hunt_region.cpp" \
  -o "${binary}"

"${binary}"
