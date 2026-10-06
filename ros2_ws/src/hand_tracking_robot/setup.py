import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hand_tracking_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carrie',
    maintainer_email='carrie@todo.todo',
    description='Bridge HTS hand landmarks to JAKA dual-arm Cartesian teleoperation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hand_tracking_bridge = hand_tracking_robot.hand_tracking_bridge_node:main',
        ],
    },
)
