from setuptools import setup
import os
from glob import glob

package_name = 'ur5e_video_classification'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AlexandreVeciana',
    maintainer_email='aveciaga7@alumnes.ub.edu',
    description='Video classification and social motion for UR5e robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_classification_node = ur5e_video_classification.video_classification_node:main',
            'robot_command_node        = ur5e_video_classification.robot_command_node:main',
            # Standalone gesture tester — run a single YAML directly without the prediction pipeline:
            #   ros2 run ur5e_video_classification ur5e_trajectory_runner <path_to_yaml>
            'ur5e_trajectory_runner    = ur5e_video_classification.ur5e_trajectory_runner:main',
        ],
    },
)