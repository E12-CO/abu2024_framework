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
    pkg_name = 'tlhx_bot'
    file_subpath = 'description/robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
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

    imu_instant = launch_ros.actions.Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("mpu6050driver"), 'params', 'mpu6050.yaml')]
    )

    mag_instant = launch_ros.actions.Node(
        package='hmc5883ldriver',
        executable='hmc5883ldriver',
        output='screen'
    )

    madgwick_fusion = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("abu_framework"), 'config', 'imu_filter.yaml')]

    )

    ekf_fusion = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("abu_framework"), 'config', 'ekf_abu.yaml')]
    )

    delayed_ekf = launch.actions.TimerAction(period=3.0, actions=[madgwick_fusion, ekf_fusion])

    rel_pos_instant = launch_ros.actions.Node(
        package='rel_pos_commander',
        executable='rel_pos_commander',
        output='screen'
    )

    mega_instant = launch_ros.actions.Node(
        package='abu_framework',
        executable='mega_interface.py',
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
        #node_robot_state_publisher,
        mecanum_controller_instant,
	#imu_instant,
	#mag_instant,
	#delayed_ekf,
        mega_instant,
        rel_pos_instant,
    ])

