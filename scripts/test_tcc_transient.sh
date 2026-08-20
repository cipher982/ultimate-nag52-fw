#!/bin/sh
set -eu

cxx="${CXX:-c++}"
tmp="${TMPDIR:-/tmp}"
build_dir="${BUILD_DIR:-build}"
mkdir -p "$build_dir"
out="$tmp/ultimate-nag52-host-tcc-transient"
"$cxx" -std=c++17 -Wall -Wextra -Werror -Isrc \
    test/host_tcc_transient.cpp src/tcc_transient_controller.cpp -o "$out"
"$out"

sim="$build_dir/host_tcc_closed_loop"
"$cxx" -std=c++17 -Wall -Wextra -Werror -Isrc \
    test/host_tcc_closed_loop.cpp src/tcc_transient_controller.cpp -o "$sim"
"$sim"

replay="$build_dir/host_tcc_replay"
"$cxx" -std=c++17 -Isrc src/tcc_transient_controller.cpp \
    test/host_tcc_replay.cpp -o "$replay"
printf '%s\n' \
    '0 1 0 1 331 89 1344 2 0' \
    '20 1 0 1 316 89 1344 2 0' \
    '40 1 0 1 301 89 1344 2 0' \
    '60 1 0 1 286 89 1344 2 0' |
    "$replay" >/dev/null
