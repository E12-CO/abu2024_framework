import os
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch
def generate_launch_description():
	param_track = os.path.join(get_package_share_directory('abu_framework'), 'config/camera_track_params.yaml')
	dir = get_package_share_directory('mecanum_controller')
	included_launch = launch.actions.IncludeLaunchDescription(
	launch.launch_description_sources.PythonLaunchDescriptionSource(
	dir + '/launch/mecanum_launch.py'))
	return LaunchDescription([
		Node(
			package='abu_framework',
			executable='mega_interface.py'),
		Node(
			package='rel_pos_commander',
			executable='rel_pos_commander'
			),
		included_launch,
	])

