#!/usr/bin/env bash
# Host checks for the RC-supervised ground-station control gate.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

g++ -std=c++17 -O2 -Wall -Wextra -Werror \
	-I"$ROOT/px4/src/modules/nn_control" \
	"$ROOT/tests/ground_station_gate_check.cpp" \
	-o "$WORK/ground_station_gate_check"

"$WORK/ground_station_gate_check"
