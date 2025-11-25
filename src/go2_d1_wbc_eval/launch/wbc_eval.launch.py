from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def n(pkg, exe, name=None, params=None, remaps=None, output='screen'):
    return Node(package=pkg, executable=exe, name=name or exe,
                parameters=params or [], remappings=remaps or [], output=output)

def generate_launch_description():
    mode = LaunchConfiguration('mode')
    data_dir = LaunchConfiguration('data_dir')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='osqp'),
        DeclareLaunchArgument('data_dir', default_value='wbc_data'),

        # Router: chooses raw vs stabilized stream to /cmd_vel
        n('go2_d1_wbc_eval','wbc_router.py','wbc_router', params=[{'mode': mode}]),

        # Logger: writes per-trial folder with timeseries.csv + metrics.csv
        n('go2_d1_wbc_eval','wbc_logger.py','wbc_logger',
          params=[{'mode': mode, 'data_dir': data_dir, 'poly_shrink_m': 0.02, 'tilt_limit_deg': 15.0}]),
    ])
