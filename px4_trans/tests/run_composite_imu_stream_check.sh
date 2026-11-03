#!/usr/bin/env bash
set -euo pipefail
test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
px4_root="${1:-$test_dir/../px4}"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
# Only platform includes are substituted. The stream implementation is compiled unchanged.
sed '/^#include /d' "$px4_root/src/modules/mavlink/streams/COMPOSITE_IMU.hpp" > "$build_dir/COMPOSITE_IMU_under_test.hpp"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -I"$build_dir" \
 "$test_dir/composite_imu_stream_check.cpp" -o "$build_dir/check"
"$build_dir/check"
