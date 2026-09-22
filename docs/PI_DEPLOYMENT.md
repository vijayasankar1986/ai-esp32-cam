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

## Board identification

Both USB boards were identified on 2026-09-22 by reading boot output over
serial with servo power switched off. No firmware was flashed.

| Port | USB bridge | Board | Evidence |
|---|---|---|---|
| `usb-1a86_USB_Serial-if00-port0` (`ttyUSB0`) | CH340 `1a86:7523` | **Hiwonder camera** | Prints `WiFi AP Started` / `AP IP Address: 192.168.5.1` at boot; ignores `PING` and `STOP` |
| `usb-Silicon_Labs_CP2102_..._0001-if00-port0` (`ttyUSB1`) | CP2102 `10c4:ea60` | **Servo controller** (by elimination) | Not yet confirmed by reply; see the disconnect note below |

Use the CP2102 path for `serial_port` in `poc.yaml`, and confirm it replies
`READY` to `PING` before enabling hardware mode.

The camera exposes no serial configuration interface. It is silent to a bare
newline and to `help`, `AT`, `?` and `status`, so its Wi-Fi mode can only be
changed through its own web page at `192.168.5.1`. USB carries power and
programming only; video is Wi-Fi.

`bbt` was added to the `dialout` group so serial access no longer needs sudo.
This takes effect at the next login.

### Controller disconnect

The CP2102 board dropped off the USB bus 78 minutes into the 2026-09-22
session and did not re-enumerate:

```text
usb 1-1.2: USB disconnect, device number 4
cp210x ttyUSB1: cp210x converter now disconnected from ttyUSB1
```

Check that cable and its power before relying on the controller. A controller
that vanishes mid-run causes the ROS node to latch a fault, which is the
intended behaviour but is not a substitute for a reliable connection.

USB enumeration showed serial adapters, not a USB webcam. Existing `/dev/video*` devices were the Pi's internal codec/ISP interfaces; they are not evidence of an ESP32 video feed. USB powers/programs these boards; the configured camera bridge still requires a network JPEG/MJPEG feed.

## Camera network finding

The Pi detects `HW_ESP32S3CAM_88` at full signal, and the camera's own serial boot output confirms it is the source: the board reports `WiFi AP Started` at `192.168.5.1`. The camera therefore ships with working firmware and needs no flashing; only its network mode is in question. Hiwonder documents AP viewing at `192.168.5.1` after joining the camera hotspot; this address is not reachable merely because USB is connected.

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
