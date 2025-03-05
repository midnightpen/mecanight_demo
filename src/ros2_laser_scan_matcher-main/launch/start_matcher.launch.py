import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time=LaunchConfiguration('use_sim_time')

    odometry_node = Node(
        package='ros2_laser_scan_matcher',
        parameters=[{
                'base_frame': 'base_footprint',
                'odom_frame': 'odom_matcher',
                'laser_frame': 'base_laser_Link',
                'publish_odom': '/odom_lidar',
                'publish_tf': False
            }],
        executable='laser_scan_matcher',
        name='odometry_publisher',
    )

    # Create and return the launch description
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                        description='Flag to enable use_sim_time'),
        odometry_node
    ])