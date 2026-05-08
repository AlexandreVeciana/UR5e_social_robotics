from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5e_kinematics_pymoveit2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='UR5e forward and inverse kinematics demo using MoveIt services.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ur5e_forward_kinematics_exe = ur5e_kinematics_pymoveit2.ur5e_forward_kinematics_node:main',
            'ur5e_inverse_kinematics_exe = ur5e_kinematics_pymoveit2.ur5e_inverse_kinematics_node:main',
            'ur5e_move_to_pose_exe = ur5e_kinematics_pymoveit2.ur5e_move_to_pose:main',
            'ur5e_move_to_pose_table_exe = ur5e_kinematics_pymoveit2.ur5e_move_to_pose_table:main',
            'ur5e_pick_place_exe = ur5e_kinematics_pymoveit2.ur5e_pick_place:main',
            "ur5e_gripper_command_exe = ur5e_kinematics_pymoveit2.ur5e_gripper_command:main",
        ],
    },
)
