#!/usr/bin/env bash
# Fetch and build the YDLIDAR SDK and its ROS 2 driver for the X2/X2L.
#
# Not vendored in this repo: it's third-party C++ that git would otherwise
# have to track wholesale, same reasoning as tools/fetch_platform_model.py
# for model weights. Run once on whichever machine runs the LiDAR node
# (the Pi), then rerun `colcon build` as printed at the end.
#
#   tools/fetch_ydlidar_driver.sh
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/ros2_ws"
SDK_DIR="$HOME/opt/YDLidar-SDK"

if [ ! -d "$SDK_DIR" ]; then
  git clone https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
fi
cmake -S "$SDK_DIR" -B "$SDK_DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$SDK_DIR/build" -j"$(nproc)"
sudo cmake --install "$SDK_DIR/build"
sudo ldconfig

# Cloned straight into ros2_ws/src; gitignored there (see .gitignore) so this
# repo's history never carries the driver's own commits.
if [ ! -d "$WS/src/ydlidar_ros2_driver" ]; then
  git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git "$WS/src/ydlidar_ros2_driver"
fi

# Upstream targets an older rclcpp; without this, colcon build fails on
# ROS 2 Jazzy with "no matching function for call to declare_parameter".
"$(dirname "${BASH_SOURCE[0]}")/patch_ydlidar_jazzy.sh" "$WS/src/ydlidar_ros2_driver"

echo
echo "SDK installed. Build the ROS 2 driver with:"
echo "    cd $WS && colcon build --symlink-install --packages-select ydlidar_ros2_driver arm_lidar"
echo
echo "Then set up tools/99-ydlidar.rules (see docs/LIDAR.md) before launching."
