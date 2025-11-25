#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def load_urdf_text():
    d = get_package_share_directory('go2_d1_description')
    return (Path(d)/'urdf'/'go2_d1.urdf').read_text()

def generate_launch_description():
    # ---------- Launch-time knobs ----------
    sign_vx_arg = DeclareLaunchArgument(
        'sign_vx', default_value='-1.0',
        description="Flip forward command if robot walks backward. Try -1.0 first; set +1.0 if still backward."
    )
    sign_vy_arg = DeclareLaunchArgument('sign_vy', default_value='1.0')
    sign_wz_arg = DeclareLaunchArgument('sign_wz', default_value='-1.0')
    use_wbc_arg = DeclareLaunchArgument('use_wbc', default_value='true')
    use_imu_arg = DeclareLaunchArgument('use_imu', default_value='false')

    sign_vx = LaunchConfiguration('sign_vx')
    sign_vy = LaunchConfiguration('sign_vy')
    sign_wz = LaunchConfiguration('sign_wz')
    use_wbc  = LaunchConfiguration('use_wbc')
    use_imu  = LaunchConfiguration('use_imu')

    # ---------- Static TF: base_link -> camera -> optical ----------
    cam_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='cam_tf',
        arguments=['0.30','0.015','0.22','0','0','0','1','base_link','camera_link'],
        output='screen'
    )
    cam_optical_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='cam_optical_tf',
        arguments=['0','0','0','-0.5','0.5','-0.5','0.5','camera_link','camera_color_optical_frame'],
        output='screen'
    )

    # ---------- Perception (publishes camera & base poses) ----------
    detector_params = {
        'image_topic':'/camera/color/image_raw',
        'camera_info_topic':'/camera/color/camera_info',
        'depth_topic':'/camera/aligned_depth_to_color/image_raw',
        'mode':'yolo','yolo_model':'yolov8n.pt','yolo_imgsz':512,
        'conf_bottle':0.35,'conf_person':0.35,'require_holding':False,
        'optical_frame':'camera_color_optical_frame',
        'base_frame':'base_link'
    }
    detector = Node(
        package='go2_d1_perception', executable='bottle_human_detector', name='bottle_human_detector',
        parameters=[detector_params], output='screen'
    )

    # ---------- Approach controller (/cmd_vel in base_link) ----------
    approach_params = {
        'target_frame':'base_link',
        'bottle_topic':'/perception/bottle_pose_base',   # already in base_link
        'handover_ready_topic':'/perception/handover_ready',
        'require_ready': False,
        'publish_rate_hz': 20.0,
        'stop_distance_m': 0.60,
        'stop_band_m': 0.08,
        'yaw_dead_rad': 0.12,
        'kx': 0.9, 'kyaw': 1.4, 'ky_lat': 1.0,
        'max_vx': 0.25, 'max_vy': 0.20, 'max_wz': 0.60,
        'ema_alpha': 0.5, 'min_detections': 2, 'fresh_window_s': 0.5,
        'lost_timeout_s': 0.7,
        'search_mode':'yaw_sweep','search_wz':0.25,'search_period_s':4.0,
        'use_imu': use_imu,
        # header published so the bridge can gate on detector freshness
        'emit_header_for_gate': True, 'header_topic':'/perception/bottle_pose/header'
    }
    approach = Node(
        package='go2_d1_tools', executable='approach_to_bottle', name='approach_to_bottle',
        parameters=[approach_params], output='screen'
    )

    # ---------- Whole-Body QP stabilizer (/cmd_vel → /cmd_vel_stab) ----------
    wbc_params = {
        'base_frame':'base_link',
        'publish_rate_hz': 50.0,
        'max_vx': 0.25, 'max_vy': 0.20, 'max_wz': 0.60,
        'poly_shrink_m': 0.02,
        'predict_dt': 0.10, 'beta_x': 1.0, 'beta_y': 1.0,
        'use_imu': use_imu, 'imu_topic':'/imu/data',
        'kd_wz': 0.2, 'tilt_soft_deg': 5.0, 'tilt_limit_deg': 15.0,
        'publish_markers': True
    }
    wbc = Node(
        package='go2_d1_control', executable='wbc_balance', name='wbc_balance',
        parameters=[wbc_params], output='screen',
        condition=IfCondition(use_wbc)
    )

    # ---------- Unitree bridge (choose stabilized or raw) ----------
    bridge_params = {
        'sign_vx': sign_vx, 'sign_vy': sign_vy, 'sign_wz': sign_wz,
        'max_vx': 0.25, 'max_vy': 0.20, 'max_wz': 0.60,
        'rate_hz': 20.0,
        'zero_behavior':'stand', 'zero_eps': 1e-3,
        'use_detector_gate': True, 'detector_topic':'/perception/bottle_pose', 'detector_fresh_s': 0.6
    }
    bridge_stab = Node(
        package='go2_d1_tools', executable='cmdvel_to_unitree_req', name='cmdvel_to_unitree_req',
        parameters=[bridge_params], remappings=[('/cmd_vel','/cmd_vel_stab')], output='screen',
        condition=IfCondition(use_wbc)
    )
    bridge_raw = Node(
        package='go2_d1_tools', executable='cmdvel_to_unitree_req', name='cmdvel_to_unitree_req_raw',
        parameters=[bridge_params], remappings=[('/cmd_vel','/cmd_vel')], output='screen',
        condition=UnlessCondition(use_wbc)
    )

    return LaunchDescription([
        sign_vx_arg, sign_vy_arg, sign_wz_arg, use_wbc_arg, use_imu_arg,
        cam_tf, cam_optical_tf,
        detector,
        approach,
        wbc,           # only if use_wbc:=true
        bridge_stab,   # only if use_wbc:=true
        bridge_raw     # only if use_wbc:=false
    ])

