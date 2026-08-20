#!/bin/sh
set -eu

cxx="${CXX:-c++}"
cxxflags="${CXXFLAGS:-}"
tmp="${TMPDIR:-/tmp}"
build_dir="${BUILD_DIR:-build}"
mkdir -p "$build_dir"
out="$tmp/ultimate-nag52-host-tcc-transient"
"$cxx" $cxxflags -std=c++17 -Wall -Wextra -Werror -Isrc \
    test/host_tcc_transient.cpp src/tcc_transient_controller.cpp -o "$out"
"$out"

sim="$build_dir/host_tcc_closed_loop"
"$cxx" $cxxflags -std=c++17 -Wall -Wextra -Werror -Isrc \
    test/host_tcc_closed_loop.cpp src/tcc_transient_controller.cpp -o "$sim"
if [ -n "${CLOSED_LOOP_JSON:-}" ]; then
    "$sim" --json-summary "$CLOSED_LOOP_JSON"
else
    "$sim"
fi

replay="$build_dir/host_tcc_replay"
"$cxx" $cxxflags -std=c++17 -Wall -Wextra -Werror -Isrc \
    test/host_tcc_replay.cpp src/tcc_transient_controller.cpp -o "$replay"
"$replay" --limits >/dev/null
printf '%s\n' \
    '0 1 0 1 331 89 1344 2 0' \
    '20 1 0 1 316 89 1344 2 0' \
    '40 1 0 1 301 89 1344 2 0' \
    '60 1 0 1 286 89 1344 2 0' |
    "$replay" >/dev/null
