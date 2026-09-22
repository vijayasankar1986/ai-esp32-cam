# Wiring and calibration

## Power

Use a regulated servo supply at the voltage specified by your actual servos (commonly 5 V for SG90). Size its current capacity for all four motors including stall/startup current; use supplier specifications. Do not power the four servos from the Pi or ESP32 3.3 V pin.

Connect servo supply negative, all servo grounds, and controller ESP32 GND together. Feed servo positive wires from the servo supply. Power the controller over USB from the Pi. Do not connect the external servo positive rail to the ESP32 USB/5 V rail without checking the board's power design. Use a physical switch to cut servo power; software cannot guarantee an emergency stop or prevent an unpowered arm falling.

## Signal connections

Read from the actual wiring on 2026-09-22. The controller is an
**ESP32-D0WDQ6** (classic ESP32, 4 MB flash, MAC `fc:e8:c0:e1:dd:00`), not an
S3. All four pins are LEDC-capable outputs and none is a boot strapping pin.

| Logical joint | ESP32 GPIO | Assumed mechanism | Confirmed? |
|---|---|---|---|
| 0 | 27 (D27) | base rotation | no |
| 1 | 26 (D26) | shoulder | no |
| 2 | 25 (D25) | elbow | no |
| 3 | 33 (D33) | gripper / wrist | no |

The **pins are known; the order is not**. Which GPIO drives which joint was not
recorded, so the mapping above is an assumption taken from the order the pins
were reported. Confirm it during calibration by commanding one joint at a time
and writing down which one actually moves, then correct the table. Getting this
wrong means a pose intended for the shoulder is sent to the gripper.

Servos are blue SG90-class 9 g units from the KitKraft 3D-printed kit, so the
1000-2000 us pulse range in the firmware is the right starting point. Their
usable travel is limited by the mechanism, not the servo, which is what
`MIN_ANGLE` and `MAX_ANGLE` are for.

Each servo has ground, supply, and signal; verify its wire colours against the servo documentation. Camera remains a separate device on Wi-Fi. Use a data-capable USB cable between Pi and controller.

## Calibration workflow

1. Support the arm and disconnect servo horns/linkages. Start with one unloaded servo.
2. Confirm GPIO assignments and supply wiring. Firmware defaults to no PWM output and refuses motion until `CALIBRATED` is enabled.
3. With the servo unloaded, review the example 1000–2000 us pulse mapping, narrow 80–100 degree limits, and 90 degree startup position. Set `CALIBRATED = true` only for this controlled commissioning step, then upload.
4. Use a serial terminal at 115200, newline ending. Send `MOVE 90 90 90 90`. All configured outputs become active, so only connect the servo currently being tested.
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
