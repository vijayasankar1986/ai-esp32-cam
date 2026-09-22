# Camera firmware: station mode

The factory Hiwonder firmware is **access-point only**. It starts
`HW_ESP32S3CAM_<MAC>` and serves the stream at `192.168.5.1`, so anything that
wants the video has to leave your LAN and join the camera instead. On a Pi with
one Wi-Fi interface that means losing SSH.

`firmware/camera_station/` replaces it with firmware that joins your existing
network, so camera, Pi and laptop all sit on one Wi-Fi. The endpoints are kept
identical to the factory ones, so the ROS node needs no change.

## Before you start: back up

The factory firmware is not downloadable. Read it out first, or you cannot go
back.

```powershell
$ESP = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe"
& $ESP --port COM5 --baud 460800 read-flash 0 0x800000 hiwonder_backup_8MB.bin
```

Use 460800, not 921600. The CH340 bridge corrupts transfers at the higher rate
(`Corrupt data, expected 0x1000 bytes but received 0xffc bytes`). A good read
reports exactly 8388608 bytes.

Restore with:

```powershell
& $ESP --port COM5 --baud 460800 write-flash 0 hiwonder_backup_8MB.bin
```

## Credentials

```bash
cp firmware/camera_station/secrets.h.example firmware/camera_station/secrets.h
```

Fill in your SSID and password. `secrets.h` is gitignored; this repository is
public, so never commit it. Leave `HOST_IP` empty for DHCP, or set it for a
fixed address so `camera_url` never has to change.

## The pin mapping problem

Hiwonder does not publish this board's camera pinout, and it is not recoverable
from the factory binary: the `camera_config_t` is built on the stack, so the
GPIO numbers live in instruction immediates rather than in a struct you can
find by scanning for `xclk_freq_hz`.

So the firmware probes instead. `CANDIDATES` holds the known ESP32-S3 camera
layouts, and at boot each is tried until one both initialises **and** returns a
real frame. Initialisation alone is not proof, which is why a frame is
demanded. The winner is printed over serial:

```text
Probing pin map 0: S3-CAM/Freenove/S3-EYE ... OK
Sensor PID 0x2145 (GC2145 is 0x2145)
Working map index 0 -- set PIN_FORCE to skip probing
```

Set `PIN_FORCE` to that index to skip probing on later boots. If every
candidate fails, the board uses a layout not in the list: add it to
`CANDIDATES`, or restore the backup and use a USB Wi-Fi adapter on the Pi
instead.

## Build and flash

Board **ESP32S3 Dev Module**, with **PSRAM enabled** — the camera needs it for
frame buffers. This board has 8 MB.

```powershell
$CLI = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
& $CLI compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=8M firmware/camera_station
& $CLI upload  --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=8M -p COM5 firmware/camera_station
```

Then watch the serial output at 115200 for the probe result and the address.

## Endpoints

Unchanged from the factory firmware, so `camera_url` keeps the same shape:

| Purpose | URL |
|---|---|
| MJPEG stream (use for `camera_url`) | `http://<ip>:81/stream` |
| Single JPEG | `http://<ip>/capture` |
| Health JSON | `http://<ip>/status` |

`/status` reports which pin map won, the IP, RSSI, frames served, whether PSRAM
was found, and free heap. The camera also advertises `armcam.local` over mDNS,
which avoids chasing a DHCP address, though the Pi resolves `.local` names only
with `avahi-daemon` installed.

## Behaviour notes

Wi-Fi power save is disabled, because it adds latency and stalls MJPEG. If the
link drops the board restarts rather than sitting idle, so the camera comes
back without you touching it. Frame size starts at QVGA to match the factory
default; raise it once the link is proven stable.

The stream boundary is `frame` rather than the factory's
`123456789000000000000987654321`. The node's reader scans for JPEG markers and
does not depend on the boundary string, so this does not matter to it.
