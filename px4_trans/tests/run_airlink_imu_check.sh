#!/usr/bin/env bash
set -euo pipefail
test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
px4_root="${1:-$test_dir/../px4}"
module_dir="$px4_root/src/modules/airlink"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
# Substitute only uORB/platform includes; compile the source reader unchanged.
sed '/^#include /d' "$module_dir/AirlinkImu.hpp" > "$build_dir/AirlinkImu_under_test.hpp"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -I"$build_dir" -I"$module_dir" \
 "$test_dir/airlink_imu_check.cpp" -o "$build_dir/check"
"$build_dir/check"
