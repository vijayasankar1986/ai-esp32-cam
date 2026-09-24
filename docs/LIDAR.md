# YDLIDAR X2/X2L

Verified on hardware 2026-09-24: connected on `/dev/ttyUSB0` at 115200 baud,
health check passed, `/scan` published at ~12 Hz.

The X2's bundled adapter and the arm controller's board both report vendor
`10c4`, product `ea60` (CP2102) **and the same unprogrammed serial `0001`**,
so they cannot be told apart by USB descriptor alone when both are plugged
in together. The udev rule below keys on the physical port instead.

## Wiring

The X2/X2L ships with a small adapter board (CP2102 USB-UART, same chip as
the arm controller) that plugs straight into a Pi USB port. No GPIO wiring
required; the adapter also switches the spin motor over its DTR line.

## One-time setup on the Pi

1. `tools/fetch_ydlidar_driver.sh` — builds and installs the YDLidar-SDK,
   then clones `ydlidar_ros2_driver` into `ros2_ws/src` (gitignored there;
   it's third-party code, not vendored into this repo).
2. Plug in the LiDAR on its own, then `ls /dev/ttyUSB*` and
   `udevadm info -q path -n /dev/ttyUSBx` to read its physical port (the
   segment like `1-1.4` before the `:1.0`).
3. Copy `tools/99-ydlidar.rules.example` to `/etc/udev/rules.d/99-ydlidar.rules`,
   fill in that port, then `sudo udevadm control --reload-rules && sudo udevadm trigger`.
   Confirm `/dev/ydlidar` appears. If the LiDAR is later moved to a different
   USB port, update the rule and reload again.
4. `cd ros2_ws && colcon build --symlink-install --packages-select ydlidar_ros2_driver arm_lidar`

## Running

```
source ros2_ws/install/setup.bash
ros2 launch arm_lidar x2.launch.py
```

Verify with `ros2 topic hz /scan` and `ros2 topic echo /scan --once`.

## Params

`ros2_ws/src/arm_lidar/config/x2.yaml` holds the X2-specific settings
(115200 baud, single-channel, DTR-switched motor). Check them against
`ydlidar_ros2_driver`'s own `params/X2.yaml` after fetching, in case upstream
has changed defaults since this was written.

## Verified 2026-09-24

`/etc/udev/rules.d/99-ydlidar.rules` installed with `KERNELS=="1-1.4"`
(the Pi's port the X2 was plugged into). `/dev/ydlidar -> ttyUSB0` resolves
correctly even with the arm controller unplugged, and
`ros2 launch arm_lidar x2.launch.py` holds `/scan` at a steady ~12 Hz.

Benign, ignorable log line at startup: `Real points 251 > fixed points 250`.
The SDK's fixed buffer for this sample rate is one point short of what the
unit actually returns per revolution; nothing breaks, the scan still
publishes every cycle. Lowering `sample_rate` slightly would quiet it if it
ever becomes annoying.

## Not yet done

- Nothing in `arm_poc` or the dashboard consumes `/scan` yet; this only gets
  the sensor publishing.
- No mount point or frame transform from `laser_frame` to the arm's base is
  defined in `arm_description`'s URDF.
- No systemd unit; run the launch file manually for now.
- The udev rule ties `/dev/ydlidar` to port `1-1.4` specifically; moving the
  LiDAR to a different USB port needs the rule redone (see setup steps above).
