from setuptools import setup

package_name = 'go2_d1_wbc_eval'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wbc_eval.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Evaluation utilities for velocity-level whole-body control (WBC).',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'wbc_eval_logger = go2_d1_wbc_eval.wbc_eval_logger:main',
        ],
    },
)
