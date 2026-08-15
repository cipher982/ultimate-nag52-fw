#!/bin/sh
set -eu

cxx="${CXX:-c++}"
out="${TMPDIR:-/tmp}/ultimate-nag52-host-tcc-transient"
"$cxx" -std=c++17 -Wall -Wextra -Werror -Isrc \
    test/host_tcc_transient.cpp src/tcc_transient_controller.cpp -o "$out"
"$out"
