import os
from ament_index_python import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    opt_mode_value = LaunchConfiguration('opt_mode')
    opt_mode_launch_arg = DeclareLaunchArgument('opt_mode', default_value="RTL")
    ld.add_action(opt_mode_launch_arg)
    
    bunker_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bunker_base'),
                'launch',
                'bunker_base.launch.py'
            ])
        ])
    )
    ld.add_action(bunker_base)

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

    rover_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_agent'),
                'bunker_rcstate_rover.launch.py'
            ])
        ]),
        launch_arguments={
            'opt_mode': opt_mode_value
        }.items()
    )
    ld.add_action(rover_agent)


    return ld