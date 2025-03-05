from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bogie_navigation',
            executable='webcam',
            name='webcam_node',
            #output='screen',
        ),
        # Node(
        #     package='bogie_navigation',
        #     executable='qr_code',
        #     name='qr_code_node',
        #     #output='screen',
        # ),
        # Node(
        #     package='bogie_navigation',
        #     executable='led',
        #     name='led_display_node',
        #     #output='screen',
        # ),
    ])