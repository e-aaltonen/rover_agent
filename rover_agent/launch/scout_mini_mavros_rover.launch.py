import os
from ament_index_python import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    scout_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('scout_base'),
                'launch',
                'scout_mini_base.launch.py'
            ])
        ])
    )
    ld.add_action(scout_base)

    mavros = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavros'),
                'launch/apm.launch',
            )
        )
    )
    ld.add_action(mavros)

    rover_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_agent'),
                'launch',
                'rcstate_rover.launch'
            ])
        ])
    )
    ld.add_action(rover_agent)


    return ld