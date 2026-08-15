#!/bin/sh
set -eu

cxx="${CXX:-c++}"
tmp="${TMPDIR:-/tmp}"
out="$tmp/ultimate-nag52-host-tcc-transient"
"$cxx" -std=c++17 -Wall -Wextra -Werror -Isrc \
    test/host_tcc_transient.cpp src/tcc_transient_controller.cpp -o "$out"
"$out"

sim="$tmp/ultimate-nag52-host-tcc-closed-loop"
"$cxx" -std=c++17 -Wall -Wextra -Werror -Isrc \
    test/host_tcc_closed_loop.cpp src/tcc_transient_controller.cpp -o "$sim"
"$sim"
