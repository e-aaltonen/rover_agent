from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EA',
    maintainer_email='ea@todo.todo',
    description='Rover autopilot auxiliary functions for Mavros/Ardupilot',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rcout_to_cmd_vel = src.rcout_to_cmd_vel:main",
            "rcin_messenger = src.rcin_messenger:main",
            "mission_client = src.mission_client:main",
            "mission_server = src.mission_server:main",
            "rc_arm_disarm = src.rc_arm_disarm:main",
            "servo_manager = src.servo_manager:main",
            "bunker_rc_state_messenger = src.bunker_rc_state_messenger:main",
            "autogui = src.autogui:main",
            "backwards_mission = src.backwards_mission:main",
            "offset_mission = src.offset_mission:main",
            "offset_one_wp = src.offset_one_wp:main",
        ],
    },
)
