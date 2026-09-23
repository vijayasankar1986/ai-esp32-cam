# Neural object detection

Names the object instead of guessing it from its colour. SSD MobileNet v3 runs
through OpenCV's DNN module on the Pi, so a detection carries a class — the arm
can be told to fetch *the cup* rather than *the nearest red thing*.

This does not replace the colour detector in `arm_poc`. Both can run at once;
they publish to different topics and `vision_pick` is pointed at whichever you
want.

## The chain

```
ESP32-CAM ──http──▶ arm_poc ──/camera/image_raw──▶ arm_vision detector
                                                        │
                                          /vision/object_point (u, v, size)
                                                        │
                                                   vision_pick
                                        pixel → ray → table plane → (x, y, z)
                                                        │
                                                   MoveIt plan
                                                        │
                                    arm_moveit_bridge ──▶ /arm/manual_pose ──▶ servos
```

No TensorFlow anywhere. OpenCV reads the frozen graph directly, and `cv2` was
already a dependency of `arm_poc`.

## Install

The weights are 48 MB, so they are not in git:

```bash
tools/fetch_detection_model.sh          # downloads to ~/models
```

OpenCV cannot read TensorFlow's frozen graph on its own — it needs a text graph
describing the same network, which is not in the tarball and is not hosted
anywhere dependable. The script generates it with OpenCV's own
`tf_text_graph_ssd.py`, which parses the protobuf itself and needs no
TensorFlow install.

Then build:

```bash
cd ~/ai-esp32cam/ros2_ws
colcon build --packages-select arm_vision
source install/setup.bash
```

## Run

`arm_poc` must be running to supply `/camera/image_raw`. For vision work alone
it is safest in observe-only mode, which never opens the serial port:

```bash
ros2 run arm_poc poc --ros-args \
  --params-file ros2_ws/src/arm_poc/config/poc.yaml -p dry_run:=true
```

Then the detector, and optionally the planner behind it:

```bash
ros2 launch arm_vision detector.launch.py
ros2 launch arm_vision detector.launch.py target_class:=cup

ros2 launch arm_moveit_config arm_moveit.launch.py \
  vision:=true vision_topic:=/vision/object_point
```

## Topics

| Topic | Type | Meaning |
|---|---|---|
| `/vision/object_point` | `PointStamped` | `x` = u, `y` = v (0–1), `z` = fraction of frame filled |
| `/vision/object_detected` | `Bool` | whether anything was chosen this frame |
| `/vision/object_label` | `String` | the class name, e.g. `cup` |
| `/vision/annotated` | `Image` | frame with boxes drawn, for RViz or rqt |

`/vision/object_point` deliberately matches the shape `arm_poc` publishes on
`/vision/target_point`, so `vision_pick` consumes either without knowing which
detector is running. They stay on separate topics because two publishers on one
topic would interleave and the consumer could not tell whose pixel it had.

## Parameters worth knowing

| Parameter | Default | Note |
|---|---|---|
| `target_class` | *(empty)* | Empty means "anything graspable". Set to a COCO class to hunt one thing. |
| `confidence` | `0.35` | See below — this is not the textbook 0.5 for a reason. |
| `graspable_only` | `true` | Filters out chairs, cars, people; see `GRASPABLE` in `coco_labels.py`. |
| `min_period` | `0.4` | Seconds between inferences. Frames are dropped, not queued. |

## Measured on the Pi 4

- **0.33 s per frame** inference, 320×240, CPU only
- **~1.2 Hz** end-to-end detection rate against a 2.5 Hz camera
- Live chain verified: `cup` at pixel (0.38, 0.54) → approach
  (-0.092, 0.149, 0.050) → MoveIt planned 61 trajectory points

## Tuning, and why the defaults are odd

**Confidence is 0.35, not 0.5.** On this 320×240 Wi-Fi feed a genuine cup
scores about 0.30–0.35. At the textbook 0.5 the detector finds nothing at all.
Lowering it buys detections at the cost of false positives — the honest fix is
a higher camera resolution, not a lower threshold.

**`vision_pick`'s `min_fraction` is 0.005, not 0.05.** A bounding box around a
cup at arm's length fills one to three percent of the frame. The colour
detector's 0.05 was a blob threshold and silently rejected every real object.

## What this does not solve

- **It still only gives a pixel.** A box centre is the same kind of answer the
  colour centroid was. Turning it into a point in space still relies on the
  camera pose and table-plane assumption in `vision_pick`, and those numbers
  are still unmeasured placeholders. Better labels, same geometry problem.
- **COCO classes are everyday objects** — cup, bottle, banana, cell phone. A
  3D-printed cube or a coloured block has no class and will never be detected.
  For those, the existing colour threshold remains the better tool: faster,
  deterministic and debuggable.
- **Nothing moves yet.** `vision_execute` is still off by default, and the
  controller brownout in `HARDWARE.md` has to be resolved before any of this
  reaches a servo.

See [MOVEIT.md](MOVEIT.md) for the planning and execution half.
