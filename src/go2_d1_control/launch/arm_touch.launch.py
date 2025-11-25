#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def load_urdf_text():
    # Adjust if your URDF lives elsewhere
    pkg = get_package_share_directory('go2_d1_description')
    return (Path(pkg) / 'urdf' / 'go2_d1.urdf').read_text()

def n(pkg, exe, name=None, params=None, remaps=None, output='screen', **kw):
    return Node(package=pkg, executable=exe, name=name or exe,
                parameters=params or [], remappings=remaps or [],
                output=output, **kw)

def generate_launch_description():
    # ---------- args ----------
    arm_rate   = DeclareLaunchArgument('arm_rate_hz',      default_value='50.0')
    cmd_topic  = DeclareLaunchArgument('arm_cmd_topic',    default_value='/wbc/arm_cmd')
    base_link  = DeclareLaunchArgument('base_link',        default_value='base_link')
    ee_link    = DeclareLaunchArgument('ee_link',          default_value='gripper_link')
    # Optional: start a tiny pose publisher for visualization (FK/TF-only)
    viz        = DeclareLaunchArgument('publish_gripper_pose', default_value='true')

    urdf_txt = load_urdf_text()

    # ---------- ARM IMPEDANCE → /wbc/arm_cmd ----------
    # This node holds/steps a safe joint target and publishes an arm state string.
    # (You can later extend it to track a bottle-relative target.)
    arm_impedance = n(
        'go2_d1_control', 'arm_impedance_node', name='arm_impedance',
        params=[{
            'robot_description': urdf_txt,
            'cmd_topic': LaunchConfiguration('arm_cmd_topic'),
            'rate_hz':  LaunchConfiguration('arm_rate_hz'),
        }]
    )  # publishes: /wbc/arm_cmd (Float64MultiArray), /arm/state="hold"
    # ref: arm_impedance_node.py :contentReference[oaicite:0]{index=0}

    # ---------- BRIDGE /wbc/arm_cmd → /arm_Command (Unitree JSON funcode=2) ----------
    wbc_to_unitree = n(
        'go2_d1_control', 'wbc_to_unitree_arm', name='wbc_to_unitree_arm'
    )  # subscribes /wbc/arm_cmd, publishes BEST_EFFORT /arm_Command
    # ref: wbc_to_unitree_arm.py :contentReference[oaicite:1]{index=1}

    # ---------- (Optional) publish EE pose for RViz/debug ----------
    gripper_pose = n(
        'go2_d1_control', 'gripper_pose_publisher', name='gripper_pose_publisher',
        params=[{
            'robot_description': urdf_txt,
            'base_link': LaunchConfiguration('base_link'),
            'gripper_link': LaunchConfiguration('ee_link')
        }],
        condition=None  # always on; toggle via the arg if you prefer
    )
    # ref: gripper_pose_publisher.py :contentReference[oaicite:2]{index=2}

    # NOTE:
    # We intentionally do NOT start:
    #  • Base approach (/cmd_vel) controller (kept separate)  :contentReference[oaicite:3]{index=3}
    #  • Any Unitree /cmd_vel bridges                         :contentReference[oaicite:4]{index=4} :contentReference[oaicite:5]{index=5}
    #  • IK reacher (that one drives /arm_Command directly)   :contentReference[oaicite:6]{index=6}
    #
    # This launch is strictly the ARM path: impedance → wbc bridge → D1 arm.

    return LaunchDescription([
        arm_rate, cmd_topic, base_link, ee_link, viz,
        arm_impedance,
        wbc_to_unitree,
        gripper_pose,
    ])

