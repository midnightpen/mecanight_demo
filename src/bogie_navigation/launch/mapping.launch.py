# sudo apt install ros-iron-cartographer
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
import os

def generate_launch_description():
    share_dir = get_package_share_directory('bogie_navigation')
    config_dir = os.path.join(share_dir,'config')
    rviz_config_file = os.path.join(share_dir, 'config', 'cartographer.rviz')

    return LaunchDescription([
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='mapping_node',
            output='screen',
            arguments=[
            	'-configuration_directory', config_dir,
            	'-configuration_basename', 'cartographer.lua'
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
