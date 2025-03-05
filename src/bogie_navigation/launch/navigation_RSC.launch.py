from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	share_dir = get_package_share_directory('bogie_navigation')
	nav2_share_dir = get_package_share_directory('nav2_bringup')
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')
	map_dir = LaunchConfiguration(
		'map',
		default = os.path.join(share_dir,'maps', 'maptest.yaml')
	)	
	param_dir = LaunchConfiguration(
		'params_file',
		default = os.path.join(share_dir,'config', 'nav2_RSC_params.yaml')
	)
	nav2_launch_file_dir = os.path.join(nav2_share_dir,'launch')
	rviz_config_dir = os.path.join(nav2_share_dir,'rviz','nav2_default_view.rviz')
	
	return LaunchDescription([
		DeclareLaunchArgument(
			'map',
			default_value = map_dir,
			description = 'Path of map' 
		),
		DeclareLaunchArgument(
			'params_file',
			default_value = param_dir,
		),
		DeclareLaunchArgument(
			'use_sim_time',
			default_value = 'false',
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
			launch_arguments = {
			'map':map_dir,
			'use_sim_time': use_sim_time,
			'params_file': param_dir
			}.items(),
		),
 		Node(
			package= 'rviz2',
			executable= 'rviz2',
			name= 'rviz2',
			arguments= ['-d', rviz_config_dir],
			parameters = [{'use_sim_time': use_sim_time}],
			output = 'screen',
			), 
		
	
	])
	
	
	
