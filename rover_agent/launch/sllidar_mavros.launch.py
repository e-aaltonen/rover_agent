from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
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

    mavros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'apm.launch'
            ])
        ]),
        launch_arguments={
            'fcu_url': '/dev/ttyACM0:57600',
        }.items()
    )
    ld.add_action(mavros)

    topic_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/scan', '/mavros/obstacle/send']
    )
    ld.add_action(topic_relay)

    return ld