import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur5e_motion_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),

        # Install config YAML files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manel',
    maintainer_email='manel.puig@ub.edu',
    description='UR5e motion server nodes for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5e_sequence_server = ur5e_motion_server.ur5e_sequence_server:main',
            'ur5e_pose_server = ur5e_motion_server.ur5e_pose_server:main',
            'ur5e_joint_server = ur5e_motion_server.ur5e_joint_server:main',
        ],
    },
)