# go2 human space Project (ROS 2 Humble)

This workspace contains a **unique end-to-end pipeline** for *human–robot bottle handover* on the `go2_d1` quadruped with an arm:
- **Perception**: ArUco-based 6D pose of the bottle + coarse person/hand detection.
- **Approach**: Base controller to approach within reach.
- **Whole-body (lightweight)**: Support polygon & COM proxy visualization.
- **Arm control**: Cartesian **impedance** servo to smoothly reach and grasp.
- **Gripper**: Simple position control of prismatic fingers.

> The workspace was generated on: 2025-09-09T11:58:27.681014

## 1) System setup (Ubuntu + ROS 2 Humble)

```bash
# ROS 2 Humble (if not already)
sudo apt update
sudo apt install -y ros-humble-desktop   ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control   ros-humble-ros2-control ros-humble-ros2-controllers   ros-humble-control-msgs ros-humble-vision-msgs   ros-humble-kdl-parser-py ros-humble-urdfdom-py   ros-humble-tf-transformations python3-opencv python3-colcon-common-extensions
```

### (Optional) Real-time hints
- Use a PREEMPT\_RT kernel and set `nice` and CPU pinning for critical nodes.
- In ROS 2, prefer **MultiThreadedExecutor**, set `reliability=best_effort` for high-rate topics.
- Sync clocks (`chrony`) and set `RMW_FASTRTPS_TRANSPORT=udp` on wired Ethernet.

## 2) Build
```bash
cd ~/go2_ws
# Copy/extract this workspace to ~/go2_ws (see download link below)
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3) Simulation (Gazebo + RViz)
```bash
ros2 launch go2_d1_bringup gazebo_handover.launch.py
# In a new terminal:
ros2 run go2_d1_perception bottle_human_detector --ros-args --params-file $(ros2 pkg prefix go2_d1_perception)/share/go2_d1_perception/config/perception.yaml
# Arm impedance + balance + gripper + coordinator:
ros2 run go2_d1_control arm_impedance --ros-args --params-file $(ros2 pkg prefix go2_d1_control)/share/go2_d1_control/config/impedance.yaml
ros2 run go2_d1_control wbc_balance
ros2 run go2_d1_control gripper_controller
ros2 run go2_d1_control approach_controller
ros2 run go2_d1_control handover_coordinator
# RViz (optional):
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix go2_d1_description)/share/go2_d1_description/rviz/handover_demo.rviz
```

The Gazebo world spawns a **human placeholder** and a **bottle** ~1 m in front of the robot. The perception node estimates the bottle pose using an ArUco marker (id=0) and approximates the human hand location.

## 4) Hardware hookup (Unitree Go2-like, generic)
- **Networking**: connect robot's Ethernet to the PC. Assign static IPs or use DHCP. Verify `ping` both ways.
- **ROS Domain**: Set `ROS_DOMAIN_ID` same on both. Export `ROS_IP` on each device.
- **Time sync**: run `chrony` or `ntpdate` to prevent TF drift.
- **Vendor SDK**: If your quadruped exposes `/cmd_vel`, keep `approach_controller`. If you use a vendor gait API, bridge it to `/cmd_vel` or modify `approach_controller` accordingly.
- **Camera**: ensure your front camera publishes `/front_camera/image_raw` and `/front_camera/camera_info`. Calibrate intrinsics if needed (`camera_calibration` pkg).
- **Transforms**: The URDF contains `front_camera` fixed to `base` and a helper static transform defines `front_camera_optical` (OpenCV convention).

## 5) Handover logic (states)
1. **SEEK**: wait for bottle detection; open gripper; move to pregrasp (15 cm behind bottle along camera Z).
2. **APPROACH**: reduce standoff to 2 cm, aligning orientation.
3. **GRASP**: close gripper (symmetric prismatic joints `Joint7_1`, `Joint7_2`).
4. **RETRACT**: back off 25 cm.
5. **DONE**.

The **arm impedance** node implements a Cartesian admittance-like servo:
\( \dot q = J^T (K e - D J \dot q) \) with damping and velocity limits, resulting in smooth, compliant motions.

## 6) Safety and "whole body" stabilization
- A support polygon and COM proxy are published on `/wbc/support_polygon` and `/wbc/com_marker` for RViz (blue polygon + green sphere). Stay within the polygon while grasping.
- Add **safety bubble**: configure `stop_distance_m` in `approach_controller` to halt base within 0.5 m and let the arm finish with impedance.
- Set reasonable joint limits (your URDF already defines them). The controllers file references your **arm joints**: `Joint1..Joint6` and **gripper**: `Joint7_1`, `Joint7_2`.

## 7) Upgrading detection (unique options)
- **Markerless**: swap ArUco with a YOLOv8/ONNX model for `bottle` and a hand keypoint detector (e.g., MediaPipe). The perception node is structured so you can plug this in.
- **Held-by-human check**: if (bottle center) is within 15 cm of the inferred hand, classify as "held"; otherwise ignore.
- **Intent**: require the hand to be extended (pixel y lower than torso center) before initiating handover.

## 8) Real robot notes
- Replace Gazebo with robot hardware by running `robot_state_publisher` with your `go2_d1.urdf` and your hardware drivers. Keep the same topics.
- For torque control, port `arm_impedance` to a low-level controller (e.g., `effort_controllers/JointGroupEffortController`) and compute \( \tau = J^T F \). For now we use a position trajectory interface which is general and safe.

## 9) Known simplifications
- The balance controller is **visualization-only**; whole-body QP is beyond scope here but the impedance at the arm plus conservative base halt distance achieves smooth & safe behavior.
- The person detector uses OpenCV HOG (fast but coarse). For strong results, plug in a modern detector. The code path is isolated in `go2_d1_perception/bottle_human_detector.py`.

---

### Package overview
- `go2_d1_description`: URDF + RViz; `urdf/go2_d1_gazebo.urdf` injects Gazebo plugins.
- `go2_d1_bringup`: Gazebo world + controller config + launch.
- `go2_d1_perception`: ArUco + HOG detection; publishes `/perception/bottle_pose`, `/perception/hand_pose` and RViz markers.
- `go2_d1_control`: Arm impedance, support polygon visualization, base approach, gripper, and handover state machine.

Happy building!












### Start with this



# For Robot system

ssh unitree@192.168.123.18

A) Robot (Foxy): turn ON aligned depth
Option 1 — flip the param live (no relaunch)

Run these on the robot terminal where the RealSense node is running:

# Does the camera node exist?
ros2 node list | grep ^/camera/camera

# See the align params (you already listed params and saw align_depth.enable)
ros2 param get /camera/camera align_depth.enable

# Turn it ON (this is the correct key for your wrapper)
ros2 param set /camera/camera align_depth.enable true

# Verify the topic appears and streams
ros2 topic list | grep aligned_depth_to_color
ros2 topic hz /camera/aligned_depth_to_color/image_raw


You should now see /camera/aligned_depth_to_color/image_raw at ~30 Hz.

Option 2 — relaunch with the right parameter name

If setting the param live doesn’t work, relaunch the node with the correct key:

# robot (Foxy)
source /opt/ros/foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true initial_reset:=true \
  enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
  depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 \
  depth_module.emitter_enabled:=1 rgb_camera.power_line_frequency:=2


Then verify:

ros2 topic list | grep aligned_depth_to_color
ros2 topic hz /camera/aligned_depth_to_color/image_raw


The “Depth stream start failure … Hardware Error” you saw earlier often happens if the camera didn’t reset cleanly. If aligned depth still doesn’t appear, unplug/re-plug the USB-C cable, try the other USB-3 port, and relaunch with initial_reset:=true. Your lsof/fuser outputs show only the realsense node is using /dev/video*, so nothing else is blocking the device.



2) Rebuild cleanly and source (PC)
cd ~/go2_ws
rm -rf build/go2_d1_perception install/go2_d1_perception
colcon build --symlink-install --packages-select go2_d1_perception

source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

# Sanity: the console script must exist now
ros2 pkg executables go2_d1_perception
ls -l install/go2_d1_perception/lib/go2_d1_perception/


You should see bottle_human_detector listed in both commands.

3) Keep TF running (PC)
# in its own terminal, keep it open
ros2 run tf2_ros static_transform_publisher \
  0.30 0.00 0.22  0 0 0 1  base_link  camera_link
  
 
#sml-workstation@smlworkstation:~/unitree_api_ws$ 
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
[INFO] [1757761628.581979541] [tf2_echo]: Waiting for transform base_link ->  camera_color_optical_frame: Invalid frame ID "base_link" passed to canTransform argument target_frame - frame does not exist
At time 0.0
- Translation: [0.300, 0.015, 0.220]
- Rotation: in Quaternion (xyzw) [-0.500, 0.501, -0.498, 0.501]
- Rotation: in RPY (radian) [-1.573, 0.003, -1.569]
- Rotation: in RPY (degree) [-90.137, 0.167, -89.877]
- Matrix:
  0.002 -0.002  1.000  0.300
 -1.000  0.003  0.002  0.015
 -0.003 -1.000 -0.002  0.220
  0.000  0.000  0.000  1.000


4) Verify the PC sees the camera topics from the robot
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw


If you don’t see them on the PC, set the SAME middleware/domain on both robot & PC, then re-source:

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # or rmw_cyclonedds_cpp, but MATCH on both machines


# Terminal B – your detector on PC
5) Run the detector (this is the step you skipped)
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
ros2 run go2_d1_perception bottle_human_detector --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p mode:=yolo -p yolo_model:=yolov8n.pt -p yolo_imgsz:=512 \
  -p conf_bottle:=0.35 -p conf_person:=0.35 -p require_holding:=false




# Terminal C – typed bridge to the robot (already running)
source /opt/ros/humble/setup.bash
source ~/unitree_api_ws/install/setup.bash
python3 ~/cmdvel_to_unitree_req.py

# Terminal D – new approach node (THIS replaces the old approach_controller)
source /opt/ros/humble/setup.bash
python3 ~/approach_to_bottle.py

#You should see: “rotating …” → “tracking vx=…” → “at_goal -> stop” when within ~0.55 m.

ros2 topic echo /perception/bottle_pose --once
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

#RVIZ2 Open

rviz2 -d ~/go2_ws/src/go2_d1_description/rviz/handover_demo.rviz

You should see:

[INFO] ... Loaded YOLO model: yolov8n.pt
[INFO] ... Perception node started. image:= ... | camera_info:= ... | mode=yolo


Leave this terminal open (the node must keep running).

If ros2 run errors, copy the error text here — that’s the smoking gun.

6) Confirm topics and visualize

In a second PC terminal:

ros2 node list | grep bottle_human_detector
ros2 node info /bottle_human_detector

ros2 topic list | grep ^/perception
ros2 topic echo /perception/status --once


You should see publishers for:

/perception/debug_image

/perception/bottle_pose

/perception/status

/perception/handover_ready

Open RViz2:

Fixed Frame: base_link

Add → TF

Add → Image → /perception/debug_image (Reliability Best Effort, tick Unreliable)

Add → Pose → /perception/bottle_pose


#13) Useful commands
# See executables
ros2 pkg executables go2_d1_tools
ros2 pkg executables go2_d1_perception

# Verify topics
ros2 topic list | grep perception
ros2 topic echo /perception/bottle_pose --once
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# Debug Unitree API channels
ros2 topic info -v /api/sport/request
ros2 topic echo /api/sport/response unitree_api/Response



##Kill everything and start.
pkill -9 -f "ros2 (launch|run)|go2_d1_|tf2_ros|unitree_|ultralytics|approach_to_bottle|bottle_human_detector|cmdvel_to_unitree_req"
ros2 daemon stop; ros2 daemon start





Great—thanks for the full context + the launch file. Below is a tight, do-this-then-that recipe to get the whole “see bottle → approach → (optional WBC) → bridge to GO2” loop working across robot (Foxy) ↔ PC (Humble) over your eth1 link, and to kill the errors you’re seeing.

# 0) One-time DDS sanity (robot & PC)

Use the **same** middleware + domain on both machines. Let’s pick CycloneDDS and domain 0.

```bash
# robot (Foxy) *and* PC (Humble)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY        # make sure it's OFF across hosts
```

CycloneDDS will automatically choose your active NIC, but since you said you’re on **eth1**, you can force it:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General>
<NetworkInterfaceAddress>eth1</NetworkInterfaceAddress>
</General></Domain></CycloneDDS>'
```

(If you stay on FastDDS, the “SHM port” spam goes away by `export FASTDDS_SHM_OFF=1`, but matching Cyclone on both sides is cleaner.)

# 1) Robot side – RealSense aligned depth ON (Foxy)

SSH into the robot and turn on aligned depth without relaunching first:

```bash
ssh unitree@192.168.123.18
source /opt/ros/foxy/setup.bash

ros2 node list | grep ^/camera/camera
ros2 param get /camera/camera align_depth.enable
ros2 param set /camera/camera align_depth.enable true

ros2 topic list | grep aligned_depth_to_color
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

If that param doesn’t “stick”, relaunch with the right key:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true initial_reset:=true \
  enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
  depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 \
  depth_module.emitter_enabled:=1 rgb_camera.power_line_frequency:=2
```

If you still see “Depth stream start failure / Hardware Error”, unplug/re-plug the USB-C and try the other USB-3 port, then relaunch with `initial_reset:=true`.

# 2) PC – clean build + source

```bash
cd ~/go2_ws
rm -rf build/go2_d1_perception install/go2_d1_perception
colcon build --symlink-install --packages-select go2_d1_perception
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 pkg executables go2_d1_perception
ls -l install/go2_d1_perception/lib/go2_d1_perception/
```

You should see the `bottle_human_detector` console script.

# 3) Keep TFs up (PC)

Run these in **their own** terminal and leave them running:

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher  \
  0.30 0.015 0.22  0 0 0 1  base_link camera_link

ros2 run tf2_ros static_transform_publisher  \
  0 0 0  -0.5 0.5 -0.5 0.5  camera_link camera_color_optical_frame
```

Now `tf2_echo` will resolve:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

# 4) Verify the PC can see the robot’s camera topics

In a fresh PC terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

If they don’t appear, your DDS settings don’t match (redo step 0 on both sides).

# 5) Start the perception node (PC)

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 run go2_d1_perception bottle_human_detector --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p mode:=yolo -p yolo_model:=yolov8n.pt -p yolo_imgsz:=512 \
  -p conf_bottle:=0.35 -p conf_person:=0.35 -p require_holding:=false
```

This node publishes:

* ` /perception/bottle_pose` (camera optical frame) and ` /perception/bottle_pose_base` (already in base_link)
* a **freshness header** at ` /perception/bottle_pose/header`
* ` /perception/debug_image` for RViz
* ` /perception/handover_ready` boolean

(All of that behavior lives in your file: YOLO detect, depth back-projection, TF to base, header publisher. )

Quick checks:

```bash
ros2 topic echo /perception/status --once
ros2 topic echo /perception/bottle_pose_base --once
```

# 6) Start the approach controller (PC)

Run your approach node (it consumes `…_base`, centers with yaw+vy, forward-only vx, has a **stop band** and publishes `/cmd_vel`):

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 run go2_d1_tools approach_to_bottle --ros-args \
  -p target_frame:=base_link \
  -p bottle_topic:=/perception/bottle_pose_base \
  -p publish_rate_hz:=20.0 \
  -p stop_distance_m:=0.60 -p stop_band_m:=0.08 -p yaw_dead_rad:=0.12 \
  -p kx:=0.9 -p kyaw:=1.4 -p ky_lat:=1.0 \
  -p max_vx:=0.25 -p max_vy:=0.20 -p max_wz:=0.60 \
  -p ema_alpha:=0.5 -p min_detections:=2 -p fresh_window_s:=0.5 -p lost_timeout_s:=0.7 \
  -p search_mode:=yaw_sweep -p search_wz:=0.25 -p search_period_s:=4.0 \
  -p use_imu:=false -p emit_header_for_gate:=false
```

You’ll see states like `search …`, then `follow vx=…` and finally `ready …` when inside the hold band. (All those parameters/logic are exactly as implemented in your node. )

# 7) (Optional) Start the WBC stabilizer (PC)

This filters `/cmd_vel → /cmd_vel_stab` so the predicted CoM stays inside a shrunken support polygon (derived from foot TFs). It also dampens yaw with IMU and publishes RViz markers.

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 run go2_d1_control wbc_balance --ros-args \
  -p base_frame:=base_link \
  -p publish_rate_hz:=50.0 \
  -p poly_shrink_m:=0.02 -p predict_dt:=0.10 -p beta_x:=1.0 -p beta_y:=1.0 \
  -p max_vx:=0.25 -p max_vy:=0.20 -p max_wz:=0.60 \
  -p use_imu:=false -p publish_markers:=true
```

(That functionality is straight from your `WBCBalance` node: feet → polygon half-spaces, QP/POCS projection, `/wbc/*` markers. )

# 8) Bridge to Unitree (PC)

This node gates motion on the detector’s **freshness header**, and when fresh, converts Twist to **Unitree SPORT** (api_id **1008**) request; on zero or stale it sends **STAND 1002** (and can pass yaw-only during search). Signs let you flip directions without touching controllers.

**With WBC (reading `/cmd_vel_stab`):**

```bash
ros2 run go2_d1_tools cmdvel_to_unitree_req --ros-args \
  -p sign_vx:=-1.0 -p sign_vy:=1.0 -p sign_wz:=-1.0 \
  -p use_detector_gate:=true -p detector_topic:=/perception/bottle_pose \
  -p detector_fresh_s:=0.6 -p allow_search_when_stale:=true -p search_wz_min:=0.05
```

(Behavior & API IDs are exactly as defined in your bridge. )

**If you skipped WBC**, remap the approach topic directly to the bridge’s `/cmd_vel`.

# 9) RViz (PC)

```bash
rviz2 -d ~/go2_ws/src/go2_d1_description/rviz/handover_demo.rviz
```

* Fixed Frame: `base_link`
* Add → TF
* Add → Image → `/perception/debug_image` (set Reliability “Best Effort” and tick “Unreliable”)
* Add → Pose → `/perception/bottle_pose_base`
* (If WBC) Add → `/wbc/support_polygon` and `/wbc/com_marker`

# 10) Quick “everything-alive?” checklist

* `ros2 topic hz /camera/color/image_raw` → ~30 Hz
* `ros2 topic hz /camera/aligned_depth_to_color/image_raw` → ~30 Hz
* `ros2 topic echo /perception/bottle_pose_base --once` → one Pose
* `ros2 topic echo /perception/bottle_pose/header --once` → one Header (freshness gate)
* `ros2 topic echo /approach/state --once` → a state string (e.g., `follow …` or `ready …`)
* `ros2 topic echo /api/sport/request --once` → SPORT requests flowing to GO2

# 11) Known gotchas (what caused your errors)

* **“Invalid frame ID base_link …” in tf2_echo** → the static TF publishers weren’t running yet. Start step #3 first, then echo.
* **Realsense “Hardware Error / Depth stream start failure”** → camera didn’t reset cleanly; use `initial_reset:=true`, reseat the cable/port, then relaunch.
* **No aligned depth on PC** → the robot’s node didn’t have `align_depth.enable:=true` set; once set, the cross-host DDS will carry `/camera/aligned_depth_to_color/image_raw`.
* **DDS / SHM port spam** → mismatched RMW (FastDDS vs Cyclone) or SHM on FastDDS; pick Cyclone on both (recommended), or `FASTDDS_SHM_OFF=1` if you must stay on FastDDS.
* **Gating stops motion** → the bridge **requires** the detector header be fresh within `detector_fresh_s` (0.6 s by default). You’ll see STAND 1002 when stale; during search the bridge can pass yaw-only if `allow_search_when_stale:=true` and `|wz| ≥ search_wz_min`. 

---

## If you prefer to run it all via your new launch file

Your launch file already wires: static TFs → detector → approach → (optional) `wbc_balance` → bridge with freshness gate and the sign flips. Just do:

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
ros2 launch go2_d1_ws go2_eval_hwc_hwc_eval.launch.py use_wbc:=true use_imu:=false sign_vx:=-1.0 sign_wz:=-1.0
```

(And make sure the **robot** is publishing the camera topics and **both** machines share DDS settings from step 0.)

---

If anything in those steps fails, paste the exact command and the next 15–20 lines of output; that will pinpoint whether it’s DDS, TF, or node-level.



# Locomotion-in-Human-Space
