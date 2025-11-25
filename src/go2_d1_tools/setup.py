from setuptools import setup, find_packages

package_name = 'go2_d1_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='GO2/D1 Python tools: approach, bridges, arm sender.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'approach_to_bottle = go2_d1_tools.approach_to_bottle:main',
            'cmdvel_to_unitree_req = go2_d1_tools.cmdvel_to_unitree_req:main',
        ],
    },
)
