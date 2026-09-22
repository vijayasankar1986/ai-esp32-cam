# Wiring and calibration

## Power

Use a regulated servo supply at the voltage specified by your actual servos (commonly 5 V for SG90). Size its current capacity for all four motors including stall/startup current; use supplier specifications. Do not power the four servos from the Pi or ESP32 3.3 V pin.

Connect servo supply negative, all servo grounds, and controller ESP32 GND together. Feed servo positive wires from the servo supply. Power the controller over USB from the Pi. Do not connect the external servo positive rail to the ESP32 USB/5 V rail without checking the board's power design. Use a physical switch to cut servo power; software cannot guarantee an emergency stop or prevent an unpowered arm falling.

## Signal connections

Read from the actual wiring on 2026-09-22. The controller is an
**ESP32-D0WDQ6** (classic ESP32, 4 MB flash, MAC `fc:e8:c0:e1:dd:00`), not an
S3. All four pins are LEDC-capable outputs and none is a boot strapping pin.

| Logical joint | ESP32 GPIO | Mechanism | Confirmed |
|---|---|---|---|
| 0 | 27 (D27) | base rotation | yes |
| 1 | 26 (D26) | shoulder | yes |
| 2 | 25 (D25) | elbow | yes |
| 3 | 33 (D33) | gripper / wrist | yes |

Confirmed by the builder on 2026-09-22. Still worth re-checking on the first
powered move, one joint at a time: a wrong mapping sends a shoulder pose to the
gripper, and the gripper has the least travel to spare.

Servos are blue SG90-class 9 g units from the KitKraft 3D-printed kit, so the
1000-2000 us pulse range in the firmware is the right starting point. Their
usable travel is limited by the mechanism, not the servo, which is what
`MIN_ANGLE` and `MAX_ANGLE` are for.

Each servo has ground, supply, and signal; verify its wire colours against the servo documentation. Camera remains a separate device on Wi-Fi. Use a data-capable USB cable between Pi and controller.

## Servo power on this build

The servos on this build are fed from **USB**, not a separate supply. That is
not the recommended arrangement above, so be clear about what it costs.

Four SG90s stall at roughly 650-750 mA each, around 2.5-3 A together. A USB
port supplies 500 mA, or up to about 1.5 A from a charging port. Commanding all
four joints at once therefore risks sagging the 5 V rail until the ESP32 hits
its brownout threshold and resets. The arm drops at the same moment the
controller reboots, which is the worst combination available.

The firmware mitigates this with `SEQUENTIAL_MOTION`, which steps **one joint
at a time**, so moving current is a single servo plus three holding rather than
four moving together. Holding current is also bounded: PWM is detached after
two seconds without an accepted `MOVE`, which releases torque entirely.

This reduces the risk; it does not remove it. A single SG90 can still stall
above what a weak port delivers, particularly lifting against gravity. If the
controller reboots mid-motion, or joints twitch when another starts moving,
that is the rail sagging and the answer is a separate supply, not more tuning.

Because motion is now sequential, a four-joint move takes about four times
longer. Raise `hold_seconds` in `poc.yaml` so a preset completes before the
node commands the return.

## Calibration workflow

1. Support the arm and disconnect servo horns/linkages. Start with one unloaded servo.
2. Confirm GPIO assignments and supply wiring. Firmware defaults to no PWM output and refuses motion until `CALIBRATED` is enabled.
3. With the servo unloaded, review the example 1000–2000 us pulse mapping, narrow 80–100 degree limits, and 90 degree startup position. Set `CALIBRATED = true` only for this controlled commissioning step, then upload.
4. Jog with `python tools/jog.py COM6` (or the `/dev/serial/by-id/...` path on
   the Pi). Keys 1-4 pick a joint, `+`/`-` step one degree, `[`/`]` step five,
   space sends `STOP`, `q` quits. All configured outputs become active on the
   first accepted `MOVE`, so connect only the servo being tested.

   Use the tool rather than a serial terminal. The firmware detaches PWM two
   seconds after the last accepted `MOVE`, so a hand-typed command makes the
   arm hold briefly and then go limp; the tool resends continuously. It also
   shows the controller's reply live, so `ERR limits` or a brownout is obvious
   as it happens.
5. Test small steps inside the configured range. Send `STOP` to detach PWM. This releases holding torque; support the mechanism.
6. Fit horns at known positions, then test each connected joint with narrowly bounded movement. Stop immediately for binding, buzzing, or excessive current. Record the safe limits below and edit firmware limits accordingly.
7. Set matching conservative limits and calibrated home/trigger poses in `poc.yaml`. Increase range only after physical verification. Default poses are identical, so the starter performs no deliberate pose change.

| Joint | GPIO | Minimum angle | Maximum angle | Home | Trigger |
|---|---|---|---|---|---|
| 0 | | | | | |
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |

Angles are commanded values, not measured joint feedback. Firmware ramps commands by roughly 1 degree per 20 ms, but the first PWM enable can move abruptly from an unknown physical position. Joint limits do not prevent collisions between links or the table.
