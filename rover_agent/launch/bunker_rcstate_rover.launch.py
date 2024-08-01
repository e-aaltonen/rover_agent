from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    opt_mode_value = LaunchConfiguration('opt_mode')
    opt_mode_launch_arg = DeclareLaunchArgument('opt_mode', default_value="GUIDED")
    ld.add_action(opt_mode_launch_arg)
    
    rc_state_messenger = Node(
            package='rover_agent',
            executable='bunker_rc_state_messenger',
    )
    ld.add_action(rc_state_messenger)

    rcout_to_cmd_vel = Node(
            package='rover_agent',
            executable='rcout_to_cmd_vel',
    )
    ld.add_action(rcout_to_cmd_vel)
    
    rc_arm_disarm = Node(
            package='rover_agent',
            executable='rc_arm_disarm',
            parameters=[
                {"opt_mode": opt_mode_value}
            ]
    )
    ld.add_action(rc_arm_disarm)

    mission_server = Node(
            package='rover_agent',
            executable='mission_server',
            parameters=[
                {"k_lat": 111194},
                {"k_long": 50519}
            ]
    )
    ld.add_action(mission_server)

    mission_client = Node(
            package='rover_agent',
            executable='mission_client',
    )
    ld.add_action(mission_client)

    return ld