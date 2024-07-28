from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_agent_gui'

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
    description='GUI functions for rover_agent',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"autogui = src.autogui:main",
        ],
    },
)
