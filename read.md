# GO2-D1 Mobile Manipulation — Full Stack README (with Arm)

> Quadruped approach + bottle perception + WBC stance + compliant arm “soft-touch” reach.
> ROS 2 Humble, Unitree D1 Arm, RGB-D camera.

---

## 0) TL;DR — Quick Start

```bash
# 0.1) One-time: install deps (Ubuntu 22.04 + ROS 2 Humble assumed)
sudo apt update && sudo apt install -y \
  ros-humble-tf2-ros ros-humble-kdl-parser-py ros-humble-urdfdom-py \
  ros-humble-pykdl ros-humble-vision-opencv python3-colcon-common-extensions
pip3 install --user ultralytics numpy==1.26.*  # (Yolo v8; avoid np.float issues)

# 0.2) Build & source
cd ~/go2_ws
colcon build --symlink-install --packages-select go2_d1_description go2_d1_tools go2_d1_control go2_d1_perception
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

# 0.3) Put the D1 Arm into SDK/Programming JOINT MODE (Unitree App)
# - Open the Unitree app, connect to robot, switch Arm control → "Programming / SDK".
# - Enable the arm (power on), zero/calibrate if requested by app. Keep app open.

# 0.4) Launch the whole stack (single file)
ros2 launch go2_d1_control handover_real.launch.py
```

If everything is alive you should see:

* `/perception/bottle_pose` at camera rate when a bottle/person is visible.
* `/perception/bottle_pose_base` relayed into `base_link`.
* `/approach/ready` toggling true once close/centered.
* `/arm/gripper_pose` continuous end-effector pose in `base_link`.
* `/wbc/arm_cmd` RELIABLE subscription by `wbc_to_unitree_arm`.
* `/arm_Command` BEST\_EFFORT publisher by `wbc_to_unitree_arm` (and by you if you test).

**Safety**: keep people out of the workspace until you confirm gentle motion. Start with small joint excursions and low impedance gains.

---

## 1) What’s inside (Architecture)

**Perception (RGB-D):**

* `bottle_human_detector` (YOLOv8) → `/perception/bottle_pose` (in `camera_color_optical_frame`)
* `pose_tf_relay` → transforms to `/perception/bottle_pose_base` (in `base_link`)
* `freshness gate` (inside approach + arm nodes) ignores stale measurements

**Locomotion (Base):**

* `approach_controller` → bounded centering control (vx, vy, yaw), stop band near goal
* `cmdvel_to_unitree_req` → converts `/cmd_vel` to Unitree walking requests
* `wbc_balance` → maintains posture references compatible with arm reach

**Manipulation (Arm):**

* `gripper_pose_publisher` → FK/KDL pose `base_link`→`gripper_link` at `/arm/gripper_pose`
* `arm_impedance_node` → optional compliant policy → `/wbc/arm_cmd` (Float64MultiArray, 7-values)
* `wbc_to_unitree_arm` → bridges `/wbc/arm_cmd` to Unitree `/arm_Command` (ArmString) using **JOINTCTRL** format
* `arm_bottle_reacher` → computes IK to an offset target near the bottle and publishes JSON command to `/arm_Command` in **funcode=2** (mode=1) format

**TFs (static):**

* `base_link → camera_link`, `camera_link → camera_color_optical_frame` published by two `static_transform_publisher`s

---

## 2) Topics & Interfaces (cheat sheet)

* **Input camera**:
  `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/aligned_depth_to_color/image_raw`

* **Perception**:
  `/perception/bottle_pose` (`geometry_msgs/PoseStamped` in camera optical frame)
  `/perception/bottle_pose_base` (`geometry_msgs/PoseStamped` in `base_link`)

* **Approach**:
  `/approach/ready` (`std_msgs/Bool`) — true when centered & within stop band
  `/cmd_vel` (`geometry_msgs/Twist`) — bounded vx, vy, wz

* **Arm**:
  `/arm_Command` (`unitree_arm/ArmString`) — **BEST\_EFFORT**, depth=1
   • **JSON mode** (recommended): `{"seq":4,"address":1,"funcode":2,"data":{"mode":1,"angle0":...,"angle6":...}}`
   • **String mode** (bridge): `"JOINTCTRL,deg0,deg1,deg2,deg3,deg4,deg5,deg6"`
  `/wbc/arm_cmd` (`std_msgs/Float64MultiArray`) — 7 values \[J1..J6(rad), gripper(deg)]
  `/arm_Feedback` (`unitree_arm/ArmString`) — status/angles/enable/power/error

* **EE Pose**:
  `/arm/gripper_pose` (`geometry_msgs/PoseStamped` in `base_link`)

> ❗ **QoS**: `/arm_Command` must be **BEST\_EFFORT, KEEP\_LAST(1), VOLATILE** to talk to Unitree bridge.
> Do **not** publish different message types on `/arm_Command` at the same time.

---

## 3) Launch files

### 3.1 Single all-in-one

```
ros2 launch go2_d1_control handover_real.launch.py
```

Spawns:

* static tf publishers (`cam_tf`, `cam_optical_tf`)
* `bottle_human_detector`, `pose_tf_relay`
* `approach_controller`, `cmdvel_to_unitree_req`
* `wbc_balance`, `wbc_to_unitree_arm`
* `gripper_pose_publisher`
* `arm_impedance_node` **and**/or `arm_bottle_reacher` (depending on your branch)

**Parameters you may want to tune** (examples):

```bash
# approach
ros2 param set /approach_controller x_goal 0.55
ros2 param set /approach_controller kx 0.8
ros2 param set /approach_controller kyaw 1.2
ros2 param set /approach_controller stop_band 0.08

# arm reacher
ros2 param set /arm_bottle_reacher reach_offset_x 0.10
ros2 param set /arm_bottle_reacher reach_offset_z 0.03
ros2 param set /arm_bottle_reacher y_limit 0.25
ros2 param set /arm_bottle_reacher rate_hz 30.0
```

---

## 4) Running pieces independently

### 4.1 Verify perception + TF

```bash
# (already started via launch) otherwise:
ros2 run tf2_ros static_transform_publisher 0.30 0.015 0.22 0 0 0 1 base_link camera_link
ros2 run tf2_ros static_transform_publisher 0 0 0 -0.5 0.5 -0.5 0.5 camera_link camera_color_optical_frame

ros2 run go2_d1_perception bottle_human_detector --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p mode:=yolo -p yolo_model:=yolov8n.pt -p yolo_imgsz:=512

ros2 run go2_d1_tools pose_tf_relay --ros-args \
  -p in_topic:=/perception/bottle_pose \
  -p out_topic:=/perception/bottle_pose_base \
  -p target_frame:=base_link
```

**Fake a bottle** (for dry runs):

```bash
ros2 topic pub -1 /perception/bottle_pose_base geometry_msgs/PoseStamped \
'{header: {frame_id: base_link}, pose: {position: {x: 0.50, y: -0.05, z: 0.70},
 orientation: {x: 0, y: 0, z: 0, w: 1}}}'
```

### 4.2 Base approach only

```bash
ros2 run go2_d1_control approach_controller
ros2 run go2_d1_tools cmdvel_to_unitree_req
```

Check:

```bash
ros2 topic echo --once /approach/ready
```

### 4.3 Arm only — **direct JSON**

> Make sure **SDK/Programming Joint Mode** is selected in the app, and arm is **enabled**.

```bash
# Small, safe pose (degrees). funcode=2, mode=1
ros2 topic pub -r 20 /arm_Command unitree_arm/msg/ArmString \
'{data: "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\
\"angle0\":0.0,\"angle1\":-10.0,\"angle2\":10.0,\"angle3\":0.0,\
\"angle4\":5.0,\"angle5\":0.0,\"angle6\":0.0}}"}' \
--qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 1
```

**Read feedback**:

```bash
ros2 topic echo --once /arm_Feedback
```

You should see periodic `funcode:1` (angles) and `funcode:3` (enable/power/error).
If `enable_status != 1` or `power_status != 1`, toggle enable in the app.

### 4.4 Arm via WBC bridge (rad → degrees)

```bash
# Start the bridge
ros2 run go2_d1_control wbc_to_unitree_arm

# Publish radians (J1..J6) + gripper in degrees
ros2 topic pub -r 20 /wbc/arm_cmd std_msgs/msg/Float64MultiArray \
'{"data":[0.10,-0.70,1.10,0.00,0.70,0.00,0.0]}' \
--qos-reliability reliable --qos-durability volatile --qos-history keep_last --qos-depth 10
```

Check that `/wbc/arm_cmd` shows **1 publisher & 1 subscriber** (the bridge).

---

## 5) How the *center–stop–reach* strategy works

1. **Center**: YOLO detections → `/perception/bottle_pose` → relay to `base_link`.
   The `approach_controller` drives `(vx, vy, yaw)` to keep the bottle near image center and null the lateral/yaw error.

2. **Stop**: A **stop band** around the target distance prevents oscillation. When inside, `/approach/ready := True`. Base holds a balanced stance (WBC posture references).

3. **Reach**: `arm_bottle_reacher` converts the current bottle pose to a reachable, **offset** end-effector target (slightly short in x, slightly high in z). IK via KDL solves 6-DoF joints; a JSON command (funcode=2, mode=1) is sent to `/arm_Command`.
   During contact, a **low-gain Cartesian impedance** (if `arm_impedance_node` is enabled) softens interaction.

---

## 6) Parameters (selected)

### 6.1 Perception

* `yolo_model` (default `yolov8n.pt`)
* `yolo_imgsz` (e.g., 512)
* `conf_bottle`, `conf_person` (0.35–0.5 reasonable)

### 6.2 Approach

* `x_goal` (e.g., 0.55 m)
* `stop_band` (e.g., 0.08 m)
* `kx`, `kvy`, `kyaw` gains
* `max_vx`, `max_vy`, `max_wz`
* `fresh_ms` (ignore stale poses)

### 6.3 Arm reacher

* `reach_offset_x` (0.10 m short of the bottle)
* `reach_offset_z` (0.03 m above)
* `min_x/max_x`, `min_z/max_z`, `y_limit`
* `rate_hz` (30–50 Hz)
* `gripper_deg` (0..45)

### 6.4 Impedance (optional)

* Gains are inside `arm_impedance.py` / `arm_impedance_node.py`. Start low (e.g., Kp=200–400 N/m, Kd=5–10 N·s/m) and increase cautiously.

---

## 7) Sanity checks

```bash
# One publisher (you), one subscriber (bridge)
ros2 topic info -v /wbc/arm_cmd

# Arm command publishers (bridge, maybe you) & one subscriber (Unitree daemon)
ros2 topic info -v /arm_Command

# Feedback is streaming
ros2 topic hz /arm_Feedback

# End-effector pose
ros2 topic echo --once /arm/gripper_pose
```

---

## 8) Troubleshooting (battle-tested)

* **No arm motion**
  ▸ Not in **SDK/Programming Joint Mode**. Toggle in Unitree app.
  ▸ Arm not **enabled/powered** in app.
  ▸ `/arm_Command` QoS mismatch → use **BEST\_EFFORT + KEEP\_LAST(1) + VOLATILE** when publishing.
  ▸ Mixed message types on `/arm_Command` → **never** publish `std_msgs/Float64MultiArray` there; only `unitree_arm/ArmString`.
  ▸ Bridge not running → start `wbc_to_unitree_arm` for `/wbc/arm_cmd` path.
  ▸ Angles out of range → command small, safe targets first.

* **`Cannot echo topic '/arm_Command', as it contains more than one type`**
  ▸ Some node is publishing `Float64MultiArray` on `/arm_Command`. Stop it; keep only `ArmString`.

* **`ParameterNotDeclaredException: robot_description`**
  ▸ Pass URDF text to nodes that parse URDF (`gripper_pose_publisher`, `arm_bottle_reacher`, `arm_impedance_node`). The launch already loads it: we read the URDF file and inject the **string** as a parameter.

* **`ValueError: Unicode strings with encoding declaration are not supported`**
  ▸ Caused by XML strings starting with `<?xml ...?>`. Our code reads the file text and feeds it to `URDF.from_xml_string()`; keep the standard header or strip it—our current nodes handle it.

* **`transforms3d` / `np.float` deprecation**
  ▸ We avoid `transforms3d` by using KDL. If you still need `tf_transformations`, pin `numpy==1.26.*`.

* **Approach says “waiting\_ready” forever**
  ▸ Bottle not detected → check `/perception/bottle_pose`.
  ▸ Too strict stop band / gains → relax `stop_band` or increase `kx/kyaw`.

* **IK fails / arm doesn’t move in reach phase**
  ▸ Target outside workspace → adjust `reach_offset_x/z`, clamp limits.
  ▸ Try a nearer fake pose to confirm kinematics.

---

## 9) Repro sequences

### 9.1 Dry-run with fake bottle → reach

```bash
# Launch full stack (no arm motion if SDK off)
ros2 launch go2_d1_control handover_real.launch.py

# Fake bottle near the robot:
ros2 topic pub -1 /perception/bottle_pose_base geometry_msgs/PoseStamped \
'{header: {frame_id: base_link},
  pose: {position: {x: 0.50, y: 0.00, z: 0.70}, orientation: {x:0,y:0,z:0,w:1}}}'

# Manually set "ready" to true (if you want to force reach phase)
ros2 topic pub -1 /approach/ready std_msgs/Bool '{data: true}'
```

### 9.2 Live camera, real bottle

* Verify `/perception/bottle_pose_base` exists while viewing the scene.
* Walk the robot; it should center & stop.
* Arm should “soft-touch” after stop (if `arm_bottle_reacher` running and SDK mode is active).

---

## 10) Safety & limits

* Keep people clear during first tests; use small joint targets.
* Start with **low impedance** gains.
* Respect the arm workspace and joint limits; clamp IK targets.
* Add e-stop / quick kill for the publisher (Ctrl-C).

---

## 11) Project layout (key files)

```
go2_d1_description/
  urdf/go2_d1.urdf

go2_d1_perception/
  bottle_human_detector.py

go2_d1_tools/
  pose_tf_relay.py
  cmdvel_to_unitree_req.py
  approach_to_bottle.py (optional CLI)

go2_d1_control/
  handover_real.launch.py
  approach_controller.py
  wbc_balance.py
  gripper_pose_publisher.py
  arm_impedance.py
  arm_impedance_node.py
  wbc_to_unitree_arm.py
  arm_bottle_reacher.py
  arm_direct_test.py
```

---

## 12) Evaluation hooks (for later experiments)

Record what you need for metrics & HRI studies:

```bash
ros2 bag record /perception/bottle_pose_base /approach/ready /cmd_vel \
/arm/gripper_pose /arm_Feedback /arm_Command /wbc/arm_cmd /tf /tf_static
```

* **Accuracy**: distance from commanded EE target to measured EE pose.
* **Smoothness/Jerk**: derivative of EE velocity/acceleration.
* **Latency/Dropouts**: timestamps of perception vs arm command.
* **Subjective**: safety, comfort, legibility (Likert).
* **Success**: rate of “soft-touch without push or jitter”.

---

## 13) Known good commands (clipboard)

**Direct JSON, degrees:**

```bash
ros2 topic pub -r 20 /arm_Command unitree_arm/msg/ArmString \
'{data: "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\
\"angle0\":0.0,\"angle1\":-10.0,\"angle2\":10.0,\"angle3\":0.0,\
\"angle4\":5.0,\"angle5\":0.0,\"angle6\":0.0}}"}' \
--qos-reliability best_effort --qos-durability volatile --qos-history keep_last --qos-depth 1
```

**WBC bridge (rad + gripper deg):**

```bash
ros2 run go2_d1_control wbc_to_unitree_arm
ros2 topic pub -r 20 /wbc/arm_cmd std_msgs/msg/Float64MultiArray \
'{"data":[0.10,-0.70,1.10,0.00,0.70,0.00,0.0]}'
```

**Echo correctly (ROS 2):**

```bash
ros2 topic echo --once /arm_Feedback
```

---

## 14) FAQs

* **Why BEST\_EFFORT on `/arm_Command`?**
  Matches the Unitree DDS endpoint; RELIABLE often doesn’t connect.

* **Why two formats on `/arm_Command`?**
  Unitree accepts both the CSV `"JOINTCTRL,...` and the JSON `{"funcode":2,...}`. We recommend JSON for clarity; the `wbc_to_unitree_arm` uses the CSV string for backward compatibility.

* **Why is gripper in degrees while joints are radians elsewhere?**
  `/wbc/arm_cmd` uses radians for the 6 joints and **degrees for the gripper** for historical reasons. The bridge converts as needed.

---

## 15) Credits & License

* YOLOv8 by Ultralytics; KDL & ROS 2 Humble stack.
* Unitree D1 Arm low-level protocol via `/arm_Command` and `/arm_Feedback`.
* Internal code © your lab/team. Choose a license and add it here.

---

### Final checklist before demos

* [ ] Arm in **SDK/Programming JOINT MODE** and **enabled** in the app
* [ ] `/arm_Feedback` streaming (angles + status)
* [ ] `/arm_Command` BEST\_EFFORT publisher connects (no type collision)
* [ ] Perception running; `/perception/bottle_pose_base` non-stale
* [ ] `/approach/ready` flips true within stop band
* [ ] Soft-touch reach produces small, compliant contact

You’re set. Go make that soft, non-jittery bottle touch happen. 🫡

