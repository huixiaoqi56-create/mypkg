#!/bin/bash
# SPDX-FileCopyrightText: 2025 hakozaki teruki
# SPDX-License-Identifier: BSD-3-Clause

set -e

echo "[TEST] start system health test"

dir="$HOME"
[ "$1" != "" ] && dir="$1"

WS="$dir/ros2_ws"

cd "$WS"
colcon build --packages-select mypkg
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[TEST] check executables"
ros2 pkg executables mypkg | grep system_health_monitor
ros2 pkg executables mypkg | grep system_health_listener

LOG=/tmp/system_health_test.log
rm -f "$LOG"

echo "[TEST] run launch (timeout)"
timeout 10 ros2 launch mypkg system_health.launch.py > "$LOG" 2>&1 || true

echo "[TEST] check log content"

grep "SystemHealthMonitor started" "$LOG"
grep "SystemHealthListener started" "$LOG"

CSV=/tmp/system_health_log.csv
echo "[TEST] check csv log"

test -f "$CSV"
test -s "$CSV"

echo "[TEST] finished"
exit 0
