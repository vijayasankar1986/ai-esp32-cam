# Wiring and calibration

## Power

Use a regulated servo supply at the voltage specified by your actual servos (commonly 5 V for SG90). Size its current capacity for all four motors including stall/startup current; use supplier specifications. Do not power the four servos from the Pi or ESP32 3.3 V pin.

Connect servo supply negative, all servo grounds, and controller ESP32 GND together. Feed servo positive wires from the servo supply. Power the controller over USB from the Pi. Do not connect the external servo positive rail to the ESP32 USB/5 V rail without checking the board's power design. Use a physical switch to cut servo power; software cannot guarantee an emergency stop or prevent an unpowered arm falling.

## Example signal connections

These are **classic ESP32 DevKit examples only**. Confirm your board pinout first. Do not use these as Hiwonder camera pin assignments.

| Logical joint | ESP32 GPIO example | Actual mechanism |
|---|---|---|
| 0 | 18 | Record your joint here |
| 1 | 19 | Record your joint here |
| 2 | 21 | Record your joint here |
| 3 | 22 | Record your joint here |

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
