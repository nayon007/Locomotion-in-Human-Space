from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    trial_id   = LaunchConfiguration('trial_id')
    out_dir    = LaunchConfiguration('out_dir')
    stop_d     = LaunchConfiguration('stop_distance_m')
    band       = LaunchConfiguration('stop_band_m')
    yaw_dead   = LaunchConfiguration('yaw_dead_rad')
    tau_enter  = LaunchConfiguration('tau_enter_s')
    tau_hold   = LaunchConfiguration('tau_hold_s')
    Tmax       = LaunchConfiguration('max_duration_s')

    return LaunchDescription([
        DeclareLaunchArgument('trial_id', default_value='trial_000'),
        DeclareLaunchArgument('out_dir', default_value='~/go2_ws/experiments'),
        DeclareLaunchArgument('stop_distance_m', default_value='0.60'),
        DeclareLaunchArgument('stop_band_m',     default_value='0.08'),
        DeclareLaunchArgument('yaw_dead_rad',    default_value='0.12'),
        DeclareLaunchArgument('tau_enter_s',     default_value='0.25'),
        DeclareLaunchArgument('tau_hold_s',      default_value='1.00'),
        DeclareLaunchArgument('max_duration_s',  default_value='60.0'),

        Node(
            package='go2_d1_eval', executable='csr_logger', name='csr_logger', output='screen',
            parameters=[{
                'trial_id': trial_id,
                'out_dir': out_dir,
                'stop_distance_m': stop_d,
                'stop_band_m': band,
                'yaw_dead_rad': yaw_dead,
                'tau_enter_s': tau_enter,
                'tau_hold_s': tau_hold,
                'max_duration_s': Tmax,
                # topics (default to your existing stack)
                'pose_cam_topic': '/perception/bottle_pose',
                'pose_base_topic':'/perception/bottle_pose_base',
                'fresh_header_topic':'/perception/bottle_pose/header',
                'camera_info_topic':'/camera/color/camera_info',
                'cmd_vel_topic':'/cmd_vel',
                'approach_state_topic':'/approach/state',
                'approach_ready_topic':'/approach/ready'
            }]
        )
    ])
