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
| `usb-Silicon_Labs_CP2102_..._0001-if00-port0` (`ttyUSB1`) | CP2102 `10c4:ea60` | **Servo controller**, confirmed | ESP32-D0WDQ6 rev v1.1, 4 MB flash, MAC `fc:e8:c0:e1:dd:00`. Read over USB on 2026-09-22 |

Use the CP2102 path for `serial_port` in `poc.yaml`, and confirm it replies
`READY` to `PING` before enabling hardware mode.

The camera exposes no serial configuration interface. It is silent to a bare
newline and to `help`, `AT`, `?` and `status`, so its Wi-Fi mode can only be
changed through its own web page at `192.168.5.1`. USB carries power and
programming only; video is Wi-Fi.

`bbt` was added to the `dialout` group so serial access no longer needs sudo.
This takes effect at the next login.

### Controller firmware now installed

`firmware/arm_controller` was flashed on 2026-09-22 (compile clean, 22% of
program storage, hash verified) and the protocol confirmed over USB at 115200:

```text
PING                   -> READY
MOVE 90 90 90 90       -> ERR calibration required
MOVE 90 90 90          -> ERR format
MOVE 200 90 90 90      -> ERR calibration required
STOP                   -> STOPPED
```

The handshake, the malformed-input rejection and the PWM teardown all work, and
the board refuses movement exactly as intended while `CALIBRATED` is false.

`CALIBRATED` was set true on 2026-09-22 at the operator's request, which armed
the controller. Joint-limit enforcement is now verified, and could be tested
without moving anything because a rejected `MOVE` never reaches `enableServos`:

```text
MOVE 200 90 90 90      -> ERR limits    above MAX_ANGLE
MOVE 79 90 90 90       -> ERR limits    below MIN_ANGLE
MOVE 90 90 90          -> ERR format
```

Still untested: any accepted `MOVE`. No servo has been driven, no PWM has been
attached, and the 80-100 limits remain a conservative placeholder rather than
measured travel. Upload used 115200; this CP2102 fails above that, as it did on
the flash read.

### Controller firmware as found

The board did not ship with this project's firmware. It was running a Classic
Bluetooth SPP sketch advertising as `Robotic_Arm`, built 18 Dec 2025 against
Arduino ESP32 core 3.3.5 on a different machine, which takes 1- and 2-byte
values and refers to joints as base, shoulder, elbow and wrist. That is why it
never answered `PING`: it has never spoken this project's protocol.

A full 4 MB backup was taken before anything was changed, kept outside this
repository at
`~/Documents/Arduino/hiwonder-cam-backup/arm_controller_esp32_fce8c0e1dd00_4MB.bin`,
so the Bluetooth firmware can be restored. It reads only at 115200; the CP2102
fails at 230400 and above.

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

## Camera endpoints (verified)

Taken from the camera's own firmware, not assumed. An 8 MB flash backup was
read over USB and its embedded web page decompressed, which gives:

```js
var baseHost  = document.location.origin
var streamUrl = baseHost + ':81'
view.src = `${streamUrl}/stream`          // MJPEG, port 81
view.src = `${baseHost}/capture?_cb=...`  // single JPEG, port 80
fetch(`${baseHost}/status`)
```

| Purpose | URL | Port |
|---|---|---|
| MJPEG stream (use this for `camera_url`) | `http://<ip>:81/stream` | 81 |
| Single JPEG still | `http://<ip>/capture` | 80 |
| Sensor status JSON | `http://<ip>/status` | 80 |
| Sensor control | `http://<ip>/control?var=<name>&val=<n>` | 80 |

The stream boundary is `multipart/x-mixed-replace;boundary=123456789000000000000987654321`,
the stock esp32-camera signature, which the node's reader already handles.
Other endpoints present: `/bmp`, `/jpeg`, `/resolution`, `/xclk`, `/reg`, `/greg`,
`/pll`, `/probe`, `/console`, `/uart`, `/secondary`.

## Camera hardware and firmware

| Property | Value |
|---|---|
| Chip | ESP32-S3 (QFN56) rev v0.2 |
| PSRAM | 8 MB embedded (AP_3v3) |
| Flash | 8 MB (mfr 0x20, dev 0x4017) |
| MAC | `e0:72:a1:ce:d1:88` (SSID suffix `_88` derives from it) |
| Sensor | GC2145 |
| Built with | `arduino-lib-builder`, ESP-IDF v4.4.5, 12 Jun 2023 |
| USB bridge | CH340 (`1a86:7523`) |

The firmware is **access-point only**. Its single Wi-Fi app string is
`WiFi AP Started`, and it generates its SSID from the MAC via
`HW_ESP32S3CAM_%02X`. The `sta.*` names in the binary are ESP-IDF's internal
NVS key table, present in every build, and are not evidence of station
support. Station mode therefore requires reflashing, which needs this board's
GC2145 pin mapping. That mapping is not yet known.

A full flash backup is held outside this repository at
`~/Documents/Arduino/hiwonder-cam-backup/hiwonder_esp32s3cam_e072a1ced188_8MB.bin`
(sha256 `78cfbafd0ff858cb2f89ca6f19ed17ce79c29f8e9a963dcba2602a32fb718703`),
so any reflash is reversible.

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
