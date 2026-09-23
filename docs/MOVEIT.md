# MoveIt 2 integration

Planning and execution for the four-servo arm on ROS 2 Jazzy. Three packages:

| Package | What it holds |
|---|---|
| `arm_description` | URDF/xacro model. **Link lengths are unverified placeholders.** |
| `arm_moveit_config` | SRDF, kinematics, joint limits, controller list, launch, RViz |
| `arm_moveit_bridge` | `FollowJointTrajectory` server that drives `/arm/manual_pose` |

## Install

MoveIt is not part of `ros-base` and was not installed with the rest of the
project. On the Pi:

```bash
sudo apt update
sudo apt install -y ros-jazzy-moveit ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
```

Then build and source as usual:

```bash
cd ~/ai-esp32cam/ros2_ws
colcon build --packages-select arm_description arm_moveit_config arm_moveit_bridge
source install/setup.bash
```

## Run

`arm_poc` keeps running as it always has. It owns the serial port, the camera
watchdog and the joint limits, and this stack attaches to it rather than
replacing it. Do not stop it to run MoveIt.

```bash
# Plan and display only. Nothing reaches a servo.
ros2 launch arm_moveit_config arm_moveit.launch.py rviz:=true

# Plan and execute. Execute in RViz moves the real arm.
ros2 launch arm_moveit_config arm_moveit.launch.py rviz:=true execute:=true
```

RViz needs a display. Run it on the Pi's own desktop session, or over SSH with
X forwarding (`ssh -X`); the Ubuntu 22.04 WSL client on the Windows machine
cannot run Jazzy binaries.

## How execution reaches the servos

```
move_group  --FollowJointTrajectory-->  arm_moveit_bridge
            --/arm/manual_pose @10Hz-->  arm_poc  --serial-->  ESP32  -->  servos
```

The bridge never opens the serial port. It republishes trajectory points as
manual poses, which means every existing guard still applies: `min_angles` and
`max_angles` from `poc.yaml`, the camera-fault halt, and the `manual_timeout`
that returns the arm home if poses stop arriving. `dry_run: true` still blocks
everything, whatever MoveIt thinks it is doing.

Two conversions live in the bridge and nowhere else:

- **Units.** `arm_poc` speaks the servo scale where 90 degrees is neutral. The
  URDF is zero-centred, so the bridge adds and subtracts 90 degrees.
- **Time.** The firmware steps one joint at a time (`SEQUENTIAL_MOTION`) and
  ramps about a degree per 20 ms, so the arm is still moving well after the
  trajectory's nominal end. The bridge holds the final pose and waits out an
  estimate of the real travel before reporting success.

## What this does not give you

- **No position feedback.** `/joint_states` is the commanded pose echoed back,
  not measurement. A successful execution means the poses were accepted, not
  that the arm reached them. Collision checking is therefore advisory: the
  planner avoids collisions in the model, and the model is not the arm.
- **Three degrees of freedom.** `joint3` is the gripper, not a wrist, so the
  arm group is `joint0`–`joint2`. IK runs `position_only_ik`; ask for a point
  in space, not an orientation. Six-DOF pose goals will fail.
- **Placeholder geometry.** Replace the four properties at the top of
  `arm_description/urdf/arm.urdf.xacro` with measured values before trusting a
  plan. Nothing else needs to change when you do.
- **The path is not tracked in time.** Sequential single-joint motion means the
  arm arrives at each waypoint by a different route than the planner drew.
  Between waypoints, the collision-free guarantee does not hold.

## Before the first execution

The calibration table in `HARDWARE.md` is still empty and `poc.yaml` still
carries the full 0–180 range. Narrow `min_angles`/`max_angles` to the travel the
mechanism actually has before letting a planner choose poses inside it, and
keep the servo power switch within reach: four SG90s on USB power can brown out
the controller mid-trajectory.
