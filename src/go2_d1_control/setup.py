from setuptools import setup
package_name = 'go2_d1_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # You can keep this config installed (harmless even if unused by arm_touch)
        ('share/' + package_name + '/config', [
            'config/impedance.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/handover_real.launch.py',
            'launch/arm_touch.launch.py',
            'launch/handover_demo.launch.py',
            'launch/bottle_reach.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='go2_d1', maintainer_email='n/a',
    description='Go2 base approach + arm impedance + bridges',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            #'approach_controller    = go2_d1_control.approach_controller:main',
            'approach_to_bottle     = go2_d1_control.approach_to_bottle:main',
            'unitree_cmdvel_bridge  = go2_d1_control.unitree_cmdvel_bridge:main',
            'arm_impedance_node     = go2_d1_control.arm_impedance_node:main',   # publishes /wbc/arm_cmd
            'handover_coordinator   = go2_d1_control.handover_coordinator:main',
            'gripper_controller     = go2_d1_control.gripper_controller:main',
            'wbc_to_unitree_arm     = go2_d1_control.wbc_to_unitree_arm:main',
            'gripper_pose_publisher = go2_d1_control.gripper_pose_publisher:main',
            'arm_bottle_reacher     = go2_d1_control.arm_bottle_reacher:main',
            'pose_tf_relay          = go2_d1_control.pose_tf_relay:main',
            'wbc_balance            = go2_d1_control.wbc_balance:main',
        ],
    },
)

