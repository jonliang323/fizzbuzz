import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mfact',
    maintainer_email='mfacton1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu = robot.imu:main',
            'move = robot.move:main',

            # 'state_machine = robot.state_machine_node:main',
            # 'mock_state_machine = robot.mock_state_machine_node:main',
            # 'cube_detect_subscriber = robot.cube_detect_node:main',
            # 'raven_subscriber = robot.raven_node:main',
            # 'test_motor_subscriber = robot.test_motor_node:main',
            # 'pic_subscriber = robot.pic_node:main',
            # 'live_label_subscriber = robot.live_label_node:main',
            # 'test_elevator_subscriber = robot.test_elevator_node:main',
        ],
    },
)
