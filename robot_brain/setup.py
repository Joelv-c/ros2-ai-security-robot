from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_brain'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Register the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer Name',
    maintainer_email='user@todo.todo',
    description='Autonomous security robot controller using YOLOv8 and GoPiGo3 hardware.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = robot_brain.vision_node:main',
            'driver_node = robot_brain.driver_node:main',
            'keyboard_node = robot_brain.keyboard_node:main',
        ],
    },
)