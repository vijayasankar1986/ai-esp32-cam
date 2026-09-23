#!/usr/bin/env bash
# Install the Arm Lab systemd units so everything returns after a reboot.
#
# Run with sudo on the Pi:  sudo systemd/install.sh
#
# Installs and enables:
#   arm-node.service       the ROS node: camera, detection, servo control
#   arm-vision.service     neural object detection (optional)
# and leaves the existing arm-dashboard.service alone.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Run this with sudo." >&2
  exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The defaults file is the one place to change how the node starts. Created
# only if absent, so a reinstall never quietly re-arms the hardware.
if [ ! -f /etc/default/arm-lab ]; then
  cat > /etc/default/arm-lab <<'DEFAULTS'
# Arm Lab startup settings.

# true  = observe only. Serial is never opened and the arm cannot move.
# false = hardware mode. The node commands home at every boot, so the arm
#         MOVES when the Pi powers on, with nobody necessarily watching.
#         Do not set this until the servo supply is sorted: docs/HARDWARE.md.
ARM_DRY_RUN=true

# COCO class for the detector to hunt, e.g. cup. Empty means anything the arm
# could plausibly pick up.
ARM_TARGET_CLASS=
DEFAULTS
  echo "Created /etc/default/arm-lab (dry run, nothing will move)"
else
  echo "Kept existing /etc/default/arm-lab"
fi

install -m 644 "$HERE/arm-node.service" /etc/systemd/system/
install -m 644 "$HERE/arm-vision.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable arm-node.service

echo
echo "arm-node enabled. The detector is optional:"
echo "    sudo systemctl enable --now arm-vision.service"
echo
echo "Start the node now without rebooting:"
echo "    sudo systemctl start arm-node.service"
echo
echo "Current mode:"
grep -E '^ARM_(DRY_RUN|TARGET_CLASS)=' /etc/default/arm-lab
