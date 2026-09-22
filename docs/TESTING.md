# Acceptance checks

## Local validation — 2026-09-22

- Four unit tests passed: stable detection, noise rejection, command encoding, and invalid angle rejection.
- Python source compilation passed (Python 3.14 on the Windows development machine).
- ROS package XML parsed successfully.
- Firmware compilation remains pending; Arduino CLI was not available locally.

## Raspberry Pi validation — 2026-09-22

- Installed ROS 2 Jazzy ros-base, cv_bridge, OpenCV, pyserial, requests, and colcon on Ubuntu 24.04 ARM64; apt completed with exit code 0.
- All four logic tests passed on the Pi.
- `colcon build --symlink-install` succeeded: one package built.
- `python3 tests/pi_smoke_test.py` passed: 33 synthetic JPEG images published, red detection, target/return phases, removal rearm, and stale-camera fault confirmed.
- The intentional HTTP 503 in the smoke test produced the expected stale-feed ERROR log; this is a tested fault condition, not a failed test.
- Serial remained unopened. Real-camera video, servo firmware, calibration, and physical actuation remain pending.

Run local logic tests from the project root:

```bash
python -m unittest discover -s tests -v
```

## Pi / hardware checks (not yet performed)

- Build with colcon and verify camera messages arrive with `bgr8` encoding and timestamps.
- Dry run: hold a red object steady for at least three processed frames; expect one trigger log. Keep it in view: no retrigger. Remove for three frames, wait for the sequence to finish, and reintroduce it: one new trigger.
- Show non-red objects and test changing light. Adjust the red fraction threshold to reduce false positives.
- Firmware: confirm `PING`, valid MOVE, out-of-range MOVE, malformed MOVE, and STOP replies. Invalid commands must not move servos or renew the watchdog.
- With unloaded servos, stop accepted MOVE messages: PWM should detach after about two seconds.
- Disconnect camera in hardware mode: the node must report stale feed and send STOP. Reconnection must not resume automatic motion until restart.
- Disconnect USB: the firmware watchdog must detach PWM. Support the arm because torque is lost.
- After calibration, run 20 staged red/non-red trials and record trigger success, false triggers, and mechanical problems. Do not claim a successful pick-and-place demo from preset motion alone.

## Troubleshooting

| Symptom | Check |
|---|---|
| No camera frames | Actual JPEG/MJPEG endpoint, Pi-to-camera network reachability, unsupported stream format |
| Camera freezes | Wi-Fi strength, camera power, lower camera resolution; stale feed stops hardware |
| Serial permission denied | Device path, dialout membership and new login session |
| ERR calibration required | Finish unloaded calibration and edit firmware flag |
| ERR limits | Firmware limits and YAML poses must agree |
| Servo jitter / ESP32 reset | Servo supply capacity, common ground, wiring and mechanical binding |
| Image topic exists but viewer blank | Select best-effort sensor-data QoS |

Current limits: no position feedback, no collision model, no inverse kinematics, no camera calibration, no MoveIt integration, and no payload rating established.
