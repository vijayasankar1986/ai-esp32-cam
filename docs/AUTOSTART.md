# Starting everything after a reboot

Until now only the dashboard came back after a power cut. `arm_poc` had no
unit, so every reboot left a working web page reporting a stopped arm and no
camera, until somebody started the node by hand. These units fix that.

| Unit | What it runs | Enabled by install.sh |
|---|---|---|
| `arm-dashboard.service` | the web dashboard on port 8080 | already installed |
| `arm-node.service` | `arm_poc`: camera, detection, servo control | yes |
| `arm-vision.service` | neural object detection | no, optional |

## Install

On the Pi:

```bash
cd ~/ai-esp32cam
sudo systemd/install.sh
sudo systemctl start arm-node.service      # or just reboot
```

Optionally add the detector, which needs the model files from
`tools/fetch_detection_model.sh`:

```bash
sudo systemctl enable --now arm-vision.service
```

## The one setting that matters

`/etc/default/arm-lab`, created by the installer:

```sh
ARM_DRY_RUN=true       # observe only: serial is never opened
ARM_TARGET_CLASS=      # COCO class for the detector, e.g. cup
```

**`ARM_DRY_RUN` defaults to `true`, and that is deliberate.** In hardware mode
the node opens the serial port and **commands home on startup**, so the arm
moves every time the Pi powers on — including after an unattended power cut,
with nobody watching and nobody near the servo switch. Given the brownout in
[HARDWARE.md](HARDWARE.md), an arm that lunges home at every boot is the worst
possible way to find out the supply is still undersized.

When the servo supply is sorted:

```bash
sudo sed -i 's/^ARM_DRY_RUN=.*/ARM_DRY_RUN=false/' /etc/default/arm-lab
sudo systemctl restart arm-node.service    # this moves the arm
```

## Why the node does not auto-restart

`arm-node.service` sets `Restart=no`, which looks wrong next to the dashboard's
`Restart=on-failure`. It is not.

The dashboard's **Restart service** button runs `run_node.sh`, which `pkill`s
the node before starting its own copy. systemd would read that kill as a
failure, wait `RestartSec`, and start a *second* node. Two nodes on one serial
port split each other's replies, and the result looks exactly like a controller
fault. So the unit brings the node up at boot and then stays out of the way;
the dashboard button owns restarts from then on.

A latched fault would not be restarted by systemd anyway — the node catches
controller errors, stops commanding and keeps running, by design.

## What still is not a service

The MoveIt stack. Planning is a session you start when you want it, it is
heavy on a Pi, and execution should be a deliberate act rather than something
that comes up with the machine:

```bash
ros2 launch arm_moveit_config arm_moveit.launch.py rviz:=true
```

See [MOVEIT.md](MOVEIT.md).

## Checking it worked

```bash
systemctl status arm-node arm-dashboard
journalctl -u arm-node -b --no-pager | tail
curl -s localhost:8080/api/status | python3 -m json.tool | head -20
```

After a reboot the dashboard should show **Observer online**, **Receiving
frames** and **Service running** without anyone touching it.
