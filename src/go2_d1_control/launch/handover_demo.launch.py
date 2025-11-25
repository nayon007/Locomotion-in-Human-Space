# launch/bottle_reach.launch.py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def load_urdf_text():
    pkg = get_package_share_directory('go2_d1_description')
    return (Path(pkg) / 'urdf' / 'go2_d1.urdf').read_text()

def generate_launch_description():
    urdf_txt = load_urdf_text()

    pose_tf = Node(
        package='go2_d1_control', executable='pose_tf_relay', name='pose_tf_relay',
        parameters=[{
            'in_topic': '/perception/bottle_pose',       # from detector
            'out_topic': '/perception/bottle_pose_base', # for IK
            'target_frame': 'base_link'
        }],
        output='screen'
    )

    reacher = Node(
        package='go2_d1_control', executable='arm_bottle_reacher', name='arm_bottle_reacher',
        parameters=[{
            'robot_description': urdf_txt,
            'base_frame': 'base_link',
            # try 'gripper_link' first; if your URDF uses Link6, swap it below:
            'ee_frame': 'gripper_link',
            'reach_offset_x': 0.10,  # stop ~10 cm short
            'reach_offset_z': 0.03,  # aim a bit high
            'y_limit': 0.25,
            'gripper_deg': 0.0,      # set >0 to pre-open, if supported
            'rate_hz': 30.0
        }],
        output='screen'
    )

    # Optional: publish EE pose for RViz/debug (FK/TF-only)
    gripper_pose = Node(
        package='go2_d1_control', executable='gripper_pose_publisher', name='gripper_pose_publisher',
        parameters=[{
            'robot_description': urdf_txt,
            'base_link': 'base_link',
            'gripper_link': 'gripper_link'
        }],
        output='screen'
    )

    return LaunchDescription([pose_tf, reacher, gripper_pose])

