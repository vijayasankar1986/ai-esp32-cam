# Camera firmware: browser object detection

`firmware/object_detect/` turns the AI-Thinker ESP32-CAM (OV2640) into a web
page that detects any of the 80 COCO objects. The ESP32 only serves JPEG
stills; detection runs in the viewer's browser with TensorFlow.js COCO-SSD.

This replaces `camera_station`, not adds to it. The `/stream`, `/capture`
and `/status` endpoints the ROS node reads are gone while this is flashed.
Reflash `camera_station` to go back.

## How it works

```text
ESP32-CAM --- /?getstill (JPEG) ---> browser: COCO-SSD in TensorFlow.js
    ^                                       |
    +---- /?detectCount=<object>;<n>;stop --+   printed over serial
```

The browser loads jQuery, TensorFlow.js and the model from public CDNs, so
the device viewing the page needs internet access. The ESP32 itself does not.

## Credentials

Shares the gitignored `secrets.h` format with `camera_station`:

```bash
cp firmware/object_detect/secrets.h.example firmware/object_detect/secrets.h
```

Set `WIFI_SSID` and `WIFI_PASSWORD`. A fallback access point always runs
alongside, named `<ip>_ESP32-CAM` with password `esp32cam`; override either
by defining `AP_SSID` / `AP_PASSWORD` in `secrets.h`.

## Build and flash

Built against Arduino-ESP32 3.3.x. Flash at 115200: the CH340 bridge corrupts
transfers at higher rates.

```bash
arduino-cli compile --fqbn esp32:esp32:esp32cam firmware/object_detect
arduino-cli upload  --fqbn esp32:esp32:esp32cam -p /dev/ttyUSB0 \
  --board-options UploadSpeed=115200 firmware/object_detect
```

On the Pi the camera is the CH340 port,
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`. Confirm with
`esptool chip-id` that it reports a classic ESP32 before flashing; the
Hiwonder S3 camera sits on the same bridge and needs a different build.

At boot the flash LED blinks five times when Wi-Fi joined, twice when only
the access point is up. The IP is printed at 115200.

## Use

Open `http://<ip>/`, wait for "Please wait for loading model" to clear, then
press **Start Detect**.

Deployed 2026-09-23 on the AI-Thinker board on the Pi's CH340 port
(ESP32-D0WD-V3, OV2640 PID 0x26, 4 MB PSRAM). It joined Wi-Fi at
`192.168.1.2`; stills measured 0.2 s each at QVGA, and a switch to VGA
returned 640x480. Pick the object to count and a minimum score.
Detections of that object are reported over serial:

```text
person = 2
```

## Command API

Every setting is a GET on `/?<cmd>=<p1>;<p2>;...`. Append `;stop` to close
the connection without waiting for the page.

| Command | Parameters | Effect |
|---|---|---|
| `getstill` | any | One JPEG frame |
| `framesize` | `QQVGA` .. `UXGA` | Resolution; QVGA by default |
| `quality` | 10-63 | JPEG quality, lower is better |
| `brightness`, `contrast` | -2..2 | Sensor tuning |
| `flash` | 0-255 | Flash LED (GPIO 4) PWM level |
| `digitalwrite` | pin; 0/1 | Drive a GPIO |
| `analogwrite` | pin; 0-255 | PWM a GPIO |
| `ip`, `mac` | none | Network details |
| `resetwifi` | ssid; password | Rejoin another network |
| `tcp` | host; port; path; wait | Relay an HTTP(S) GET |
| `restart` | none | Reboot |

## Changes from the original sketch

- LEDC calls ported to the 3.x API. Flash and PWM use pinned channels 4 and
  5, because the camera's XCLK takes channel 0 through the IDF and the
  Arduino allocator cannot see it.
- `if (P1="4")` assigned instead of comparing, so `analogwrite` always drove
  GPIO 4.
- `camera_config_t` is zero-initialised. The esp32-camera bundled with core
  3.x added `jpeg_buffer_size`; left as stack garbage it sized the buffer
  wrong, every frame logged `cam_hal: FB-OVF`, and every grab returned NULL.
- LINE Notify commands removed; the service shut down in March 2025.
- `CAMERA_GRAB_LATEST` with two PSRAM buffers, so each detection sees the
  newest frame rather than one buffered from the previous request.
- The flash LED is no longer forced off after every frame, so the slider holds.
