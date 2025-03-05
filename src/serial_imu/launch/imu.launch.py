#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions 


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='serial_imu',
            executable='talker',
            output='screen'
            ),
        # launch_ros.actions.Node(
        #     package='serial_imu',
        #     executable='listener',
        #     output='screen'
        #     ),

        ])

