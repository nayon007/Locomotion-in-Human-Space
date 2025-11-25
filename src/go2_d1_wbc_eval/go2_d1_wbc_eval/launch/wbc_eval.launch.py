from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def n(pkg, exe, name=None, params=None, remaps=None, output='screen'):
    return Node(package=pkg, executable=exe, name=name or exe,
                parameters=params or [], remappings=remaps or [], output=output)

def generate_launch_description():
    mode = LaunchConfiguration('mode')
    # IMPORTANT:
    # Remap your controller (approach_to_bottle) to publish desired cmds to /cmd_vel_des
    # so the router can produce stabilized /cmd_vel for the Unitree bridge.
    approach_remap = [('/cmd_vel','/cmd_vel_des')]  # only if you start the controller here

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='osqp'),

        # Example: if you also start your approach node here, remap its /cmd_vel → /cmd_vel_des
        # n('go2_d1_tools','approach_to_bottle', 'approach_to_bottle', remaps=approach_remap),

        # Router (stabilizer)
        n('go2_d1_wbc_eval','wbc_router','wbc_router', params=[{'mode': mode}]),

        # Logger
        n('go2_d1_wbc_eval','wbc_logger','wbc_logger', params=[{'mode': mode, 'data_dir':'wbc_data'}]),
    ])
