#!/usr/bin/env sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT INT TERM

${CXX:-c++} \
    ${CXXFLAGS:-} \
    -std=c++17 \
    -Wall -Wextra -Werror \
    -I"$repo_dir/src" \
    "$repo_dir/src/tcc_direct_slip_controller.cpp" \
    "$repo_dir/test/host_tcc_direct_slip.cpp" \
    ${LDFLAGS:-} \
    -o "$build_dir/tcc-direct-slip-test"

"$build_dir/tcc-direct-slip-test"
