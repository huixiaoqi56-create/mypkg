#!/bin/bash
# SPDX-FileCopyrightText: 2025 hakozaki teruki
# SPDX-License-Identifier: BSD-3-Clause
set -e

echo "[TEST] start system health test"

pkill -f system_health_monitor || true
pkill -f system_health_listener || true
sleep 1

echo "[TEST] check executables"
ros2 pkg executables mypkg | grep system_health_monitor
ros2 pkg executables mypkg | grep system_health_listener

echo "[TEST] run launch (timeout)"
timeout 5 ros2 launch mypkg system_health.launch.py > /tmp/system_health_test.log 2>&1 || true

echo "[TEST] check log content"
grep "SystemHealthMonitor started" /tmp/system_health_test.log
grep "SystemHealthListener started" /tmp/system_health_test.log

echo "[TEST] check csv log"
test -f /tmp/system_health_log.csv

echo "[TEST] finished"
