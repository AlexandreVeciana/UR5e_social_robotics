from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5e_kinematics_control'

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
    description='UR5e forward kinematics.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ur5e_joint_target_exec = ur5e_kinematics_control.ur5e_joint_target:main',
            'ur5e_joint_targets_exec = ur5e_kinematics_control.ur5e_joint_targets:main',
        ],
    },
)
