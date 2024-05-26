# Test package launch file : 
import os

import launch
import launch_ros.actions
import launch.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import xacro

from ament_index_python.packages import get_package_share_directory

home = os.path.expanduser('~')

def generate_launch_description():

     # Specify the name of the package and path to xacro file within the package
    file_subpath = 'description/robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory('tlhx_bot'),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    mecanum_controller_instant = launch_ros.actions.Node(
        package='mecanum_controller',
        executable='mecanum_controller',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("mecanum_controller"), 'params', 'mecanum.yaml')]
    )


    # ABU Nodes and executables

    # Button Team Mode selector launch
    abu_teammode_instant = launch_ros.actions.Node(
        package='abu_framework',
	name='abu_teammode_node',
        executable='abu_teammode_node.py',
	output='screen'
    )

    # abu_nav node param
    nav_param_dir = launch.substitutions.LaunchConfiguration(
        'nav_param_dir',
        default=os.path.join(
            get_package_share_directory('abu_nav'),
            'params',
            'abu_params.yaml'))

    # abu_nav node launch
    abu_nav_instant = launch_ros.actions.Node(
        package='abu_nav',
        executable='abu_nav_node',
        parameters=[nav_param_dir]
    )

    # ball_check_node launch
    abu_ballcheck_instant = launch_ros.actions.Node(
        package='abu_framework',
        name='abu_ball_check_node',
        executable='ball_check_node.py',
        output='screen'
    )

    return launch.LaunchDescription([
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
        DeclareLaunchArgument(
            'nav_param_dir',
            default_value=nav_param_dir,
            description='Param for abu_nav node'),

        #node_robot_state_publisher,
        mecanum_controller_instant,
        abu_teammode_instant,
        abu_nav_instant,
        abu_ballcheck_instant,
    ])

