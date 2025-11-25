

# go2 Human Space Project (ROS 2 Humble)

This workspace implements an end-to-end **real-robot** pipeline for *human–robot bottle handover* using a Go2-style quadruped with a 6-DoF D1 arm:

- **Perception**: YOLO/ArUco-based bottle detection, depth-based 3D pose, and coarse person/hand estimation.
- **Base motion**: Velocity controller to approach the human within a safe standoff region.
- **Lightweight whole-body support**: Support polygon and COM proxy for RViz; conservative velocity shaping.
- **Arm control**: Cartesian impedance-like servo that reaches and grasps smoothly.
- **Gripper**: Simple symmetric prismatic fingers (`Joint7_1`, `Joint7_2`).

> The workspace snapshot in this repo was generated on: `2025-09-09T11:58:27.681014`.

---


## Demo video

Real-robot following bottle on the Go2 platform:

<video src="IMG_6808.MOV"
       controls
       loop
       muted
       playsinline
       style="max-width: 100%; height: auto;">
  Your browser does not support the video tag.
</video>



---

## 1. System setup

### 1.1 Control PC (Ubuntu + ROS 2 Humble)

Install ROS 2 Humble and required dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-control-msgs ros-humble-vision-msgs \
  ros-humble-kdl-parser-py ros-humble-urdfdom-py \
  ros-humble-tf-transformations \
  python3-opencv python3-colcon-common-extensions
````

Clone this repo into `~/go2_ws` and build:

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 1.2 Robot side (Unitree Go2-like, ROS 2 Foxy)

On the robot (e.g., Unitree controller with **ROS 2 Foxy**):

* Vendor SDK / SPORT API for gait control.
* Intel RealSense (e.g., D435) mounted at the front.
* ROS 2 Foxy installed and working.

SSH access:

```bash
ssh unitree@<robot-ip>
```

---

## 2. Networking and DDS configuration

PC (Humble) and robot (Foxy) must be on the same network and share DDS settings.

On **both** machines:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
```

Optional: pin CycloneDDS to a specific interface (`eth1`):

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General>
<NetworkInterfaceAddress>eth1</NetworkInterfaceAddress>
</General></Domain></CycloneDDS>'
```

Check connectivity:

```bash
ping <robot-ip>
ping <pc-ip>
```

---

## 3. RealSense aligned depth on the robot (Foxy)

On the **robot**:

```bash
ssh unitree@<robot-ip>
source /opt/ros/foxy/setup.bash

# Check the camera node
ros2 node list | grep ^/camera/camera

# Read the aligned depth setting
ros2 param get /camera/camera align_depth.enable

# Turn on aligned depth
ros2 param set /camera/camera align_depth.enable true
```

Verify topics:

```bash
ros2 topic list | grep aligned_depth_to_color
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

You should see `/camera/aligned_depth_to_color/image_raw` at ~30 Hz.

If that doesn’t work, relaunch the RealSense driver:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true initial_reset:=true \
  enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30 \
  depth_module.emitter_enabled:=1 \
  rgb_camera.power_line_frequency:=2
```

If you see “Depth stream start failure / Hardware Error”:

* Replug the camera,
* Try a different USB-3 port,
* Relaunch with `initial_reset:=true`.

---

## 4. Build and source (PC)

On the **PC**:

```bash
cd ~/go2_ws
rm -rf build/go2_d1_perception install/go2_d1_perception  # optional clean
colcon build --symlink-install --packages-select go2_d1_perception

source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 pkg executables go2_d1_perception
ls -l install/go2_d1_perception/lib/go2_d1_perception/
```

You should see `bottle_human_detector` in both outputs.

---

## 5. Runtime pipeline (real robot)

The core loop is:

> RealSense (robot) → perception (PC) → approach + WBC (PC) → Unitree API bridge → robot motion.

### 5.1 Static camera transforms (PC)

On the **PC**:

```bash
source /opt/ros/humble/setup.bash

# base_link → camera_link
ros2 run tf2_ros static_transform_publisher \
  0.30 0.015 0.22  0 0 0 1  base_link camera_link

# camera_link → camera_color_optical_frame
ros2 run tf2_ros static_transform_publisher \
  0 0 0  -0.5 0.5 -0.5 0.5  camera_link camera_color_optical_frame
```

Check:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

### 5.2 Verify camera topics seen from PC

```bash
source /opt/ros/humble/setup.bash

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

If you don’t see messages, DDS settings likely don’t match between robot and PC.

### 5.3 Perception node (PC)

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 run go2_d1_perception bottle_human_detector --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p mode:=yolo \
  -p yolo_model:=yolov8n.pt \
  -p yolo_imgsz:=512 \
  -p conf_bottle:=0.35 \
  -p conf_person:=0.35 \
  -p require_holding:=false
```

Publishes:

* `/perception/bottle_pose` and `/perception/bottle_pose_base`
* `/perception/debug_image`
* `/perception/status`
* `/perception/handover_ready`

Quick checks:

```bash
ros2 topic echo /perception/status --once
ros2 topic echo /perception/bottle_pose_base --once
```

### 5.4 Approach controller (PC)

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 run go2_d1_tools approach_to_bottle --ros-args \
  -p target_frame:=base_link \
  -p bottle_topic:=/perception/bottle_pose_base \
  -p publish_rate_hz:=20.0 \
  -p stop_distance_m:=0.60 \
  -p stop_band_m:=0.08 \
  -p yaw_dead_rad:=0.12 \
  -p kx:=0.9 \
  -p kyaw:=1.4 \
  -p ky_lat:=1.0 \
  -p max_vx:=0.25 \
  -p max_vy:=0.20 \
  -p max_wz:=0.60 \
  -p ema_alpha:=0.5 \
  -p min_detections:=2 \
  -p fresh_window_s:=0.5 \
  -p lost_timeout_s:=0.7 \
  -p search_mode:=yaw_sweep \
  -p search_wz:=0.25 \
  -p search_period_s:=4.0 \
  -p use_imu:=false \
  -p emit_header_for_gate:=false
```

States: `search → follow → ready`.

### 5.5 Optional: WBC stabilizer (PC)

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 run go2_d1_control wbc_balance --ros-args \
  -p base_frame:=base_link \
  -p publish_rate_hz:=50.0 \
  -p poly_shrink_m:=0.02 \
  -p predict_dt:=0.10 \
  -p beta_x:=1.0 \
  -p beta_y:=1.0 \
  -p max_vx:=0.25 \
  -p max_vy:=0.20 \
  -p max_wz:=0.60 \
  -p use_imu:=false \
  -p publish_markers:=true
```

### 5.6 Bridge to Unitree SPORT API (PC)

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

ros2 run go2_d1_tools cmdvel_to_unitree_req --ros-args \
  -p sign_vx:=-1.0 \
  -p sign_vy:=1.0 \
  -p sign_wz:=-1.0 \
  -p use_detector_gate:=true \
  -p detector_topic:=/perception/bottle_pose \
  -p detector_fresh_s:=0.6 \
  -p allow_search_when_stale:=true \
  -p search_wz_min:=0.05
```

On the Unitree API side, a separate node in your `unitree_api_ws` should be translating `/api/sport/request` into actual robot commands.

---

## 6. RViz visualization

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

rviz2 -d ~/go2_ws/src/go2_d1_description/rviz/handover_demo.rviz
```

Add:

* **TF**
* **Image**: `/perception/debug_image` (Best Effort + Unreliable)
* **Pose**: `/perception/bottle_pose_base`
* **Markers (if WBC)**: `/wbc/support_polygon`, `/wbc/com_marker`

---

## 7. Handover logic (high level)

1. **SEEK**: Wait for bottle detection; gripper open; move to pre-grasp ~15 cm behind bottle along camera Z.
2. **APPROACH**: Close in to ~2 cm, align orientation.
3. **GRASP**: Close gripper (`Joint7_1`, `Joint7_2`).
4. **RETRACT**: Back off ~25 cm.
5. **DONE**: Hold posture.

The arm impedance controller implements:

[
\dot{q} = J^\top (K e - D J \dot{q})
]

with damping and velocity limits for smooth, compliant motion.

---

## 8. Known simplifications & future work

* WBC is “visualization-first”: COM/projection is used for visualization and conservative velocity shaping, not full QP-based loco-manipulation.
* Person detector currently uses a simple pipeline; the perception node is structured to allow:

  * YOLOv8 ONNX for bottle/person,
  * keypoint-based hand localization,
  * intent inference (e.g., extended hand).

Future work:

* True whole-body QP controller,
* Torque-level arm control ((\tau = J^\top F)),
* Markerless, robust human pose and intent estimation.

---

## 9. Package overview

* **`go2_d1_description`** – URDF and RViz config.
* **`go2_d1_perception`** – YOLO/ArUco perception and debug visualization.
* **`go2_d1_tools`** – Base approach controller and Unitree SPORT bridge.
* **`go2_d1_control`** – WBC-style balance helper and other control utilities.
* **`go2_d1_eval` / `go2_d1_wbc_eval`** – Evaluation, logging, and plots (centering error, jerk proxy, entry timing, etc.).

---

Happy building in human spaces 🤖🚶‍♂️
If you use this for a paper, consider adding a citation section once the paper is published.

```

---

If you want, next step we can:

- Add a `LICENSE` and `CITATION.cff`,
- Add a minimal `requirements.txt` / `environment.yml`,
- Or add a short “Reproducing Fig. X / Table Y” section tied to your `experiments/` and `wbc_data/`.
```

