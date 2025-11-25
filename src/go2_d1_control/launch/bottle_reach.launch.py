#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def load_urdf_text():
    pkg = get_package_share_directory('go2_d1_description')
    return (Path(pkg) / 'urdf' / 'go2_d1.urdf').read_text()

def generate_launch_description():
    base_frame_arg    = DeclareLaunchArgument('base_frame', default_value='base')
    ee_frame_arg      = DeclareLaunchArgument('ee_frame', default_value='Link6')
    optical_frame_arg = DeclareLaunchArgument('optical_frame', default_value='camera_color_optical_frame')

    base_frame    = LaunchConfiguration('base_frame')
    ee_frame      = LaunchConfiguration('ee_frame')
    optical_frame = LaunchConfiguration('optical_frame')

    urdf_txt = load_urdf_text()

    # Detector (from go2_d1_perception)
    detector = Node(
        package='go2_d1_perception', executable='bottle_human_detector', name='bottle_human_detector',
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'mode': 'yolo', 'yolo_model': 'yolov8n.pt', 'yolo_imgsz': 512,
            'conf_bottle': 0.35, 'conf_person': 0.35, 'require_holding': False,
            'optical_frame': optical_frame,
            'base_frame': base_frame,
        }],
        output='screen'
    )

    # IK Reacher
    reacher = Node(
        package='go2_d1_control', executable='arm_bottle_reacher', name='arm_bottle_reacher',
        parameters=[{
            'robot_description': urdf_txt,
            'base_frame': base_frame,
            'ee_frame': ee_frame,
            'reach_offset_x': 0.10,
            'reach_offset_z': 0.03,
            'y_limit': 0.25,
            'rate_hz': 30.0
        }],
        output='screen'
    )

    # Optional: visualize EE link pose in RViz (FK only)
    gripper_pose = Node(
        package='go2_d1_control', executable='gripper_pose_publisher', name='gripper_pose_publisher',
        parameters=[{
            'robot_description': urdf_txt,
            'base_link': base_frame,
            'gripper_link': ee_frame
        }],
        output='screen'
    )

    return LaunchDescription([base_frame_arg, ee_frame_arg, optical_frame_arg, detector, reacher, gripper_pose])

