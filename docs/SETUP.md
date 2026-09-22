# Raspberry Pi setup

## 1. Operating system

This guide targets **64-bit Ubuntu 24.04 + ROS 2 Jazzy** on a supported Pi. Check your Pi model and OS before installing. If you have Ubuntu 22.04, use the matching Humble installation instructions instead; this starter has not been runtime-tested there. Do not reinstall your OS just to follow this guide.

Install ROS 2 using the [official Jazzy Debian package instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), including repository configuration. Then:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-cv-bridge python3-opencv python3-serial python3-requests python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
```

Copy this project to `~/ai-esp32cam` on the Pi. No Windows path is assumed on the Pi.

## 2. Camera

Keep the camera's existing streaming firmware if it works. Connect Pi and camera to a reachable network. Verify the feed in a browser, then find the actual JPEG snapshot or MJPEG stream request in browser developer tools / Network. Put that URL in `camera_url` in the YAML file. Do not assume `/stream`, `/capture`, or a particular port. This project does not flash the camera because its exact module and pinout are unconfirmed.

The reader supports repeated JPEG snapshots or continuous MJPEG. Other protocols, HTML viewer pages, and proprietary Hiwonder streams need another adapter. No credentials should be committed in the URL.

## 3. Build and dry run

```bash
cd ~/ai-esp32cam/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run arm_poc poc --ros-args --params-file src/arm_poc/config/poc.yaml
```

The default `dry_run: true` does not open the serial port. A camera URL is still required. In another sourced terminal:

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /vision/color_detected
```

Optional desktop viewer: install `ros-jazzy-rqt-image-view`, run `ros2 run rqt_image_view rqt_image_view`, select `/camera/image_raw`, and use best-effort QoS if offered. Headless Pi users can view from another ROS computer configured on the same network.

## 4. Servo controller

Open `firmware/arm_controller/arm_controller.ino` in Arduino IDE. Install **esp32 by Espressif Systems, version 3.x**, choose your actual controller board and USB port, and upload. Complete HARDWARE.md calibration first. No external servo library is required.

On the Pi, identify the controller with `ls -l /dev/serial/by-id/`; use its stable device path for `serial_port`. Where required, add your user to `dialout` with `sudo usermod -aG dialout "$USER"`, then log out and back in. Close the Arduino serial monitor before ROS opens the port.

The protocol at 115200 baud is newline-delimited ASCII:

```text
PING                 -> READY
MOVE 90 90 90 90     -> OK (target accepted, not proof movement finished)
STOP                 -> STOPPED
```

Invalid or uncalibrated moves return `ERR ...`. `STOP` disables PWM and holding torque. The ROS controller repeats targets every 0.5 seconds; firmware disables PWM after 2 seconds without an accepted MOVE. USB reconnection is deliberately manual: restart the ROS node after investigating a disconnect.

## 5. Enable movement

Set calibrated `home_pose`, `trigger_pose`, `min_angles`, and `max_angles` in YAML. With clear workspace, supported arm, and physical power switch accessible, change `dry_run` to `false` and restart. Hardware mode commands home at startup. Three consecutive colour-positive frames trigger the target; after `hold_seconds` it returns home. Three negative frames are required before another trigger. Keep the object removed while the arm returns home.

Set `target_color` to red, green, blue, yellow, orange or purple, and tune `color_fraction` for lighting and object size. Detection measures the fraction of matching pixels in the whole image; this is not object localization or a pick coordinate. Allow enough hold time for the slowest joint to finish moving. A lost camera feed causes STOP after `camera_stale_seconds` and requires restarting the node.
