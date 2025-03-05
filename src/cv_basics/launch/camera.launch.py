#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
            Node(
                package='cv_basics',
                executable='img_publisher',
                name='camera_node',
                output='screen',
            ),

    ])