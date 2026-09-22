# Raspberry Pi deployment

Verified over SSH through Windows WSL on 2026-09-22:

- Host: `192.168.1.9`
- User: `bbt`
- Hardware: Raspberry Pi 4 Model B Rev 1.5, 8 GB RAM
- OS: Ubuntu 24.04.5 LTS, ARM64
- Project location: `/home/bbt/ai-esp32cam`
- ROS target: Jazzy

## Installed and verified

ROS 2 Jazzy ros-base and project dependencies are installed. The `arm_poc` package is built in `/home/bbt/ai-esp32cam/ros2_ws`. Four unit tests and the synthetic-camera ROS integration test passed (33 images). No automatic startup service was installed. Both USB boards retain their existing firmware; no serial commands or servo movement were initiated.

No password is stored in project files.

Connect from Windows:

```powershell
wsl.exe -d Ubuntu-22.04 -- ssh bbt@192.168.1.9
```

The WSL distribution is the SSH client. ROS runs on the Pi's Ubuntu 24.04, not in the Ubuntu 22.04 WSL client.

Two adapters were visible; neither has been identified as the servo controller:

- `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`

USB enumeration showed serial adapters, not a USB webcam. Existing `/dev/video*` devices were the Pi's internal codec/ISP interfaces; they are not evidence of an ESP32 video feed. USB powers/programs these boards; the configured camera bridge still requires a network JPEG/MJPEG feed.

## Camera network finding

The Pi detected the nearby hotspot `HW_ESP32S3CAM_88`, likely the Hiwonder camera. Its identity has not been physically confirmed. Hiwonder documents AP viewing at `192.168.5.1` after joining the camera hotspot; this address is not reachable merely because USB is connected.

The Pi currently uses `wlan0` for `192.168.1.9`; Ethernet is disconnected and the user has no Ethernet available. Switching that Wi-Fi interface would interrupt SSH. Its network configuration has therefore been preserved. Live integration needs either camera station-mode configuration for the existing LAN or a second Wi-Fi adapter on the Pi. The factory station-mode example uses a particular hotspot name/password; it does not establish that arbitrary router credentials can be set through the web page. Inspect the exact firmware before changing it.

Reference: [Hiwonder network modes](https://docs.hiwonder.com/projects/ESP32-S3/en/latest/docs/3.Image_Recognition_Course.html).

Do not select a port or flash firmware until the physical board is identified. The camera IP/stream URL is also still required.

After installation/build, validate without connected camera or moving hardware:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ai-esp32cam/ros2_ws/install/setup.bash
cd ~/ai-esp32cam
python3 -m unittest discover -s tests -v
python3 tests/pi_smoke_test.py
```

The smoke test serves synthetic JPEGs on loopback and checks actual ROS images, detection, preset state changes, and stale-camera handling. It always uses dry-run mode and never opens serial.
