#!/usr/bin/env bash
# Patch ydlidar_ros2_driver's single-argument declare_parameter("name") calls
# for ROS 2 Jazzy, which removed that overload (it now requires a type or
# default value). Upstream still targets an older rclcpp. Idempotent: once
# patched, the exact-match substitutions below no longer find anything to
# replace, so a rerun after a fresh re-clone is harmless.
#
#   tools/patch_ydlidar_jazzy.sh [path to ydlidar_ros2_driver checkout]
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/ros2_ws"
DRIVER_DIR="${1:-$WS/src/ydlidar_ros2_driver}"
FILE="$DRIVER_DIR/src/ydlidar_ros2_driver_node.cpp"

if [ ! -f "$FILE" ]; then
  echo "Not found: $FILE (run tools/fetch_ydlidar_driver.sh first)" >&2
  exit 1
fi

# Each substitution supplies the default already sitting in the local
# variable the line right above initialises, so behaviour is unchanged.
sed -i \
  -e 's/declare_parameter("port");/declare_parameter("port", str_optvalue);/' \
  -e 's/declare_parameter("ignore_array");/declare_parameter("ignore_array", str_optvalue);/' \
  -e 's/declare_parameter("frame_id");/declare_parameter("frame_id", frame_id);/' \
  -e 's/declare_parameter("baudrate");/declare_parameter("baudrate", optval);/' \
  -e 's/declare_parameter("lidar_type");/declare_parameter("lidar_type", optval);/' \
  -e 's/declare_parameter("device_type");/declare_parameter("device_type", optval);/' \
  -e 's/declare_parameter("sample_rate");/declare_parameter("sample_rate", optval);/' \
  -e 's/declare_parameter("abnormal_check_count");/declare_parameter("abnormal_check_count", optval);/' \
  -e 's/declare_parameter("intensity_bit");/declare_parameter("intensity_bit", optval);/' \
  -e 's/declare_parameter("fixed_resolution");/declare_parameter("fixed_resolution", b_optvalue);/' \
  -e 's/declare_parameter("reversion");/declare_parameter("reversion", b_optvalue);/' \
  -e 's/declare_parameter("inverted");/declare_parameter("inverted", b_optvalue);/' \
  -e 's/declare_parameter("auto_reconnect");/declare_parameter("auto_reconnect", b_optvalue);/' \
  -e 's/declare_parameter("isSingleChannel");/declare_parameter("isSingleChannel", b_optvalue);/' \
  -e 's/declare_parameter("intensity");/declare_parameter("intensity", b_optvalue);/' \
  -e 's/declare_parameter("support_motor_dtr");/declare_parameter("support_motor_dtr", b_optvalue);/' \
  -e 's/declare_parameter("debug");/declare_parameter("debug", b_optvalue);/' \
  -e 's/declare_parameter("angle_max");/declare_parameter("angle_max", f_optvalue);/' \
  -e 's/declare_parameter("angle_min");/declare_parameter("angle_min", f_optvalue);/' \
  -e 's/declare_parameter("range_max");/declare_parameter("range_max", f_optvalue);/' \
  -e 's/declare_parameter("range_min");/declare_parameter("range_min", f_optvalue);/' \
  -e 's/declare_parameter("frequency");/declare_parameter("frequency", f_optvalue);/' \
  -e 's/declare_parameter("invalid_range_is_inf");/declare_parameter("invalid_range_is_inf", invalid_range_is_inf);/' \
  -e 's/declare_parameter("m1_mode");/declare_parameter("m1_mode", i_v);/' \
  -e 's/declare_parameter("m2_mode");/declare_parameter("m2_mode", i_v);/' \
  -e 's/declare_parameter("m3_mode");/declare_parameter("m3_mode", i_v);/' \
  "$FILE"

remaining=$(grep -c 'declare_parameter("[a-zA-Z0-9_]*");' "$FILE" || true)
if [ "$remaining" -ne 0 ]; then
  echo "Warning: $remaining single-argument declare_parameter call(s) left unpatched in $FILE" >&2
fi
echo "Patched $FILE for Jazzy's declare_parameter API."
