#!/usr/bin/env bash
# run_with_log.sh — launch the paint bot and auto-save all console output.
#
# Usage:
#   ./run_with_log.sh painting_robot_bringup bringup.launch.py [extra launch args...]
#
# Everything printed to the terminal is ALSO written to:
#   ~/paint_logs/debug/<date>_<time>.log
# Upload that file here when you want help debugging a session.

set -u

PKG="${1:?usage: run_with_log.sh <pkg> <launch_file> [args...]}"
LAUNCH="${2:?usage: run_with_log.sh <pkg> <launch_file> [args...]}"
shift 2

LOG_DIR="${PAINT_LOG_DIR:-$HOME/paint_logs/debug}"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/$(date +%Y-%m-%d_%H%M%S).log"

# Clean, shareable log formatting (timestamp, level, node, message).
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
# Force line-buffered so the last lines survive a crash / Ctrl-C.
export PYTHONUNBUFFERED=1

echo "logging this session to: $LOG_FILE"
echo "-------------------------------------------------------------"

# stdbuf keeps stdout/stderr line-buffered through the pipe so nothing
# is lost on a crash. 2>&1 folds ROS's stderr logs in with stdout.
stdbuf -oL -eL ros2 launch "$PKG" "$LAUNCH" "$@" 2>&1 | tee "$LOG_FILE"

echo "-------------------------------------------------------------"
echo "session log saved: $LOG_FILE"
