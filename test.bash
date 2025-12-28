#!/bin/bash
# SPDX-FileCopyrightText: 2025 hakozaki teruki
# SPDX-License-Identifier: BSD-3-Clause

set -e

echo "[TEST] start system health test"

# ROS 2 環境を読み込む
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "[TEST] check executables"
ros2 pkg executables mypkg | grep system_health_monitor > /dev/null
ros2 pkg executables mypkg | grep system_health_listener > /dev/null

echo "[TEST] start monitor (timeout)"
timeout 3 ros2 run mypkg system_health_monitor || true

echo "[TEST] start listener (timeout)"
timeout 3 ros2 run mypkg system_health_listener || true

echo "[TEST] check log file"
LOG_FILE="/tmp/system_health_log.csv"

if [ ! -f "$LOG_FILE" ]; then
  echo "[FAIL] log file not found"
  exit 1
fi

echo "[TEST] log file exists"

echo "[TEST] finished successfully"
exit 0
