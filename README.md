# Camera-triggered robotic arm with ROS 2

Starter project for your Hiwonder ESP32-CAM, four-servo mechanical arm, separate ESP32 servo controller, and Raspberry Pi running Ubuntu.

**POC:** detect a coloured object in a fixed camera view and move the arm to a calibrated preset. This is a preset-motion demo, not yet arbitrary object picking or inverse kinematics.

## Architecture

```text
Hiwonder camera -- HTTP JPEG/MJPEG over Wi-Fi --> Raspberry Pi / ROS 2
                                                     |
                                      TCP, port 3333, over Wi-Fi
                                                     |
                                               Separate ESP32
                                                     |
                                                Four servos
```

The ROS node publishes `/camera/image_raw`, `/vision/color_detected` and `/arm/joint_states`. A rising detection triggers one preset; removing the object rearms detection. The tracked colour is set by `target_color` in the YAML and defaults to red. Hardware movement is disabled by default. No micro-ROS or PCA9685 is required.

## Files

- [Wiring and calibration](docs/HARDWARE.md)
- [Pi installation and operation](docs/SETUP.md)
- [Acceptance checks and troubleshooting](docs/TESTING.md)
- [Verified Raspberry Pi connection and deployment](docs/PI_DEPLOYMENT.md)
- [YDLIDAR X2 setup](docs/LIDAR.md)
- [ESP32 controller firmware](firmware/arm_controller/arm_controller.ino)
- [Camera station-mode firmware](docs/CAMERA_FIRMWARE.md)
- [Camera browser object-detection firmware](docs/CAMERA_OBJECT_DETECT.md)
- [ROS configuration](ros2_ws/src/arm_poc/config/poc.yaml)
- [ROS node](ros2_ws/src/arm_poc/arm_poc/node.py)
- [Manual jog tool](tools/jog.py)

## Confirm before hardware operation

1. Raspberry Pi model and Ubuntu version: `cat /proc/device-tree/model` and `cat /etc/os-release`.
2. Exact separate ESP32 board. Firmware GPIO defaults are examples for a classic ESP32 DevKit, not universal pin assignments.
3. Servo labels: assumed SG90 from our conversation; verify all four.
4. Actual camera HTTP JPEG or MJPEG endpoint. A browser viewer page is not necessarily the stream URL.
5. Safe joint angles, pulse widths, and servo power requirements for the assembled mechanism.

Start with the dry-run workflow in SETUP.md. Firmware intentionally requires editing `CALIBRATED` before it accepts movement. Never infer joint travel from the servo's advertised range.

## Sources

- [Hiwonder image transmission documentation](https://docs.hiwonder.com/projects/ESP32-S3/en/latest/docs/3.Image_Recognition_Course.html): applies if your fitted module is the documented ESP32-S3 version.
- [ROS 2 Jazzy on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
- [Arduino ESP32 LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html): firmware targets Arduino-ESP32 3.x.
- [Your mechanical kit](https://www.kitkraft.in/products/diy-mini-robotic-arm-3d-printed-mechanical-kit-compatible-with-for-sg90-mg90-servo-motors).

## Validation status

Four logic tests passed on Windows and the Raspberry Pi. ROS 2 Jazzy is installed on the Pi, the package built successfully with colcon, and a synthetic-camera integration test passed with 33 published images, red detection, preset sequencing, removal rearming, and stale-feed fault handling. No serial port was opened during that test. Firmware compilation, real-camera compatibility, electrical wiring, and physical movement remain unverified. See docs/PI_DEPLOYMENT.md for the installed environment and remaining camera-network work.
