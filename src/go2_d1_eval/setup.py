from setuptools import setup

package_name = 'go2_d1_eval'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/csr_eval.launch.py']),
        ('share/' + package_name + '/scripts', [
            'scripts/run_trial.sh',
            'scripts/plot_trajectories.py',
            'scripts/summarize_metrics.py'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Center–Stop experiment logging, metrics, plots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csr_logger = go2_d1_eval.csr_logger:main',
        ],
    },
)
