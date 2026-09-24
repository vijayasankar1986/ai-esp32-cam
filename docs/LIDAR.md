# YDLIDAR X2/X2L

Not verified on hardware yet: the unit had not been plugged into the Pi as
of 2026-09-24. This documents the intended setup; confirm each step as you go.

## Wiring

The X2/X2L ships with a small adapter board (CP2102 USB-UART, same chip as
the arm controller) that plugs straight into a Pi USB port. No GPIO wiring
required; the adapter also switches the spin motor over its DTR line.

## One-time setup on the Pi

1. `tools/fetch_ydlidar_driver.sh` — builds and installs the YDLidar-SDK,
   then clones `ydlidar_ros2_driver` into `ros2_ws/src` (gitignored there;
   it's third-party code, not vendored into this repo).
2. Plug in the LiDAR, then `ls /dev/ttyUSB*` and
   `udevadm info -a -n /dev/ttyUSBx | grep '{serial}'` to read its serial,
   since it uses the same CP2102 vendor/product IDs as the arm controller and
   so can't be told apart by those alone.
3. Copy `tools/99-ydlidar.rules.example` to `/etc/udev/rules.d/99-ydlidar.rules`,
   fill in that serial, then `sudo udevadm control --reload-rules && sudo udevadm trigger`.
   Confirm `/dev/ydlidar` appears.
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

## Not yet done

- Nothing in `arm_poc` or the dashboard consumes `/scan` yet; this only gets
  the sensor publishing.
- No mount point or frame transform from `laser_frame` to the arm's base is
  defined in `arm_description`'s URDF.
- No systemd unit; run the launch file manually for now.
