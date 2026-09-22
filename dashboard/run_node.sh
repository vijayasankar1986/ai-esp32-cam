#!/usr/bin/env bash
# Start the arm_poc node, replacing any that is already running.
#
# Used by the dashboard's restart button and safe to run by hand. Only one
# process may own the serial port, so existing nodes are killed first and the
# kill is verified before starting a new one: two nodes on one port split each
# other's replies and produce failures that look like controller faults.
# No 'set -u': ROS's setup.bash reads unset variables and would abort under it.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="${ARM_NODE_LOG:-$HOME/arm_node.log}"

pkill -9 -f "lib/arm_poc/poc" 2>/dev/null
for _ in $(seq 20); do
  pgrep -f "lib/arm_poc/poc" >/dev/null || break
  sleep 0.25
done
if pgrep -f "lib/arm_poc/poc" >/dev/null; then
  echo "failed to stop the running node" >&2
  exit 1
fi
[ "${1:-start}" = "stop" ] && { echo "node stopped"; exit 0; }

# The ESP32 resets when the port opens; give it a moment before reconnecting.
sleep 2
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
cd "$ROOT"
setsid nohup ros2 run arm_poc poc --ros-args \
  --params-file "$ROOT/ros2_ws/src/arm_poc/config/poc.yaml" \
  > "$LOG" 2>&1 < /dev/null &
echo "node starting, log at $LOG"
