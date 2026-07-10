#!/bin/sh
#
# Sets up a mavros workspace with the battery charge/capacity NaN fix applied,
# for machines (e.g. the companion computer) that only have access to this
# SVEA-PX4-Autopilot_psu repo and not a separate mavros fork.
#
# Usage: run from anywhere; creates ./mavros_ws next to wherever this is invoked.
#   sh tools/mavros_battery_fix/setup_mavros.sh
#
# Requires: colcon, a sourced ROS 2 (jazzy) environment, git.

set -e

PATCH_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="${1:-mavros_ws}"

mkdir -p "$WS/src"
cd "$WS/src"

if [ ! -d mavros ]; then
	git clone --branch 2.14.0 --depth 1 https://github.com/mavlink/mavros.git
fi

cd mavros
if ! git apply --check --reverse "$PATCH_DIR/sys_status_charge_capacity.patch" 2>/dev/null; then
	git apply "$PATCH_DIR/sys_status_charge_capacity.patch"
fi
cd ../..

colcon build --packages-select mavros --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF

echo "Done. Source $WS/install/setup.sh and restart mavros_node."
