#!/usr/bin/env bash
set -euo pipefail

px4_root="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../px4" && pwd)}"
module_dir="$px4_root/src/modules/comp_attitude"
test_file="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/composite_attitude_core_check.cpp"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

test -f "$module_dir/CompositeMath.hpp"
test -f "$module_dir/CompAttitude.cpp"

if ! rg -q '"wq:composite_ekf", 6144' "$module_dir/CompAttitude.cpp"; then
	echo "FAIL: composite EKF must use its dedicated 6144-byte work queue" >&2
	exit 1
fi

if rg -q 'ScheduledWorkItem\(MODULE_NAME, px4::wq_configurations::nav_and_controllers\)' \
	"$module_dir/CompAttitude.cpp"; then
	echo "FAIL: composite EKF must not run on the nav_and_controllers work queue" >&2
	exit 1
fi

"${CXX:-c++}" \
	-std=c++17 \
	-Wall \
	-Wextra \
	-Werror \
	-I"$module_dir" \
	"$test_file" \
	-o "$build_dir/composite_attitude_core_check"

"$build_dir/composite_attitude_core_check"
