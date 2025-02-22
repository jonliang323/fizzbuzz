from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_processing'

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
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = image_processing.state_machine_node:main',
            'mock_state_machine = image_processing.mock_state_machine_node:main',
            'cube_detect_subscriber = image_processing.cube_detect_node:main',
            'raven_subscriber = image_processing.raven_node:main',
            'test_motor_subscriber = image_processing.test_motor_node:main',
            'pic_subscriber = image_processing.pic_node:main',
            'live_label_subscriber = image_processing.live_label_node:main',
            'test_elevator_subscriber = image_processing.test_elevator_node:main',
        ],
    },
)