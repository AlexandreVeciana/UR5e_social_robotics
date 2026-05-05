from setuptools import setup
import os
from glob import glob

package_name = 'social_robot_gesture'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manel',
    maintainer_email='manel@example.com',
    description='Gesture classification package for social human-robot interaction.',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            'gesture_classifier_node = social_robot_gesture.gesture_classifier_node:main',
        ],
    },
)
