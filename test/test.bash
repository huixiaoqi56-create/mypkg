#!/bin/bash
# SPDX-FileCopyrightText: 2025 hakozaki teruki
# SPDX-License-Identifier: BSD-3-Clause

set -eu

LOG=/tmp/system_health_log.csv

echo "[TEST] start system health test"

echo "[TEST] check executables"
ros2 pkg executables mypkg | grep system_health_monitor
ros2 pkg executables mypkg | grep system_health_listener

rm -f "$LOG"

echo "[TEST] start monitor (background)"
timeout 3 ros2 run mypkg system_health_monitor &
MON_PID=$!

sleep 1

echo "[TEST] start listener (background)"
timeout 5 ros2 run mypkg system_health_listener \
  --ros-args -p log_path:="$LOG" &
LIS_PID=$!


sleep 3

echo "[TEST] check log file"
if [ ! -f "$LOG" ]; then
  echo "[FAIL] log file not found"
  exit 1
fi

echo "[TEST] log file exists"
cat "$LOG"

kill $MON_PID 2>/dev/null || true
kill $LIS_PID 2>/dev/null || true

echo "[TEST] finished successfully"
exit 0
