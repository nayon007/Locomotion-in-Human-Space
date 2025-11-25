from setuptools import setup, find_packages
package_name = 'go2_d1_perception'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sml',
    maintainer_email='none@example.com',
    description='Bottle + human detector (YOLO) with depth → PoseStamped.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bottle_human_detector = go2_d1_perception.bottle_human_detector:main',
            'perception_viz = go2_d1_perception.perception_viz:main',
        ],
    },
)
