import os
from ament_index_python import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    sllidar_a3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_a3_launch.py'
            ])
        ])
    )
    ld.add_action(sllidar_a3)

    mavros = GroupAction(
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('mavros'),
                        'launch/apm.launch'
                    )
                ),
                launch_arguments={
                    'fcu_url': '/dev/ttyUSB0:57600',
                }.items()
            )
        ]
    ) 
    ld.add_action(mavros)

    topic_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/scan', '/mavros/obstacle/send']
    )
    ld.add_action(topic_relay)

    rover_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_agent'),
                'launch',
                'bunker_rcstate_rover.launch.py'
            ])
        ])
    )
    ld.add_action(rover_agent)


    return ld