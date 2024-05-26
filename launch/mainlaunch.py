from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_param',
            default_value='default_start',
            description='The value for the "start" parameter'
        ),
        DeclareLaunchArgument(
            'team_param',
            default_value='default_team',
            description='The value for the "team" parameter'
        ),
        Node(
            package="abu_framework",
            executable="main_node.py",
            name="main_node",
            output="log",
            parameters=[
                {"start": LaunchConfiguration('start_param')},
                {"team": LaunchConfiguration('team_param')}
            ],
        )
    ])
