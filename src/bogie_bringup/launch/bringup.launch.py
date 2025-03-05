#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # lidar_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('sllidar_ros2'),'launch','sllidar_c1_launch.py']
    # )

    camera_launch_path = PathJoinSubstitution(
        [FindPackageShare('cv_basics'),'launch','camera.launch.py']
    )

    ld_lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('ldlidar'),'launch','ldlidar.launch.py']
    )

    lidar_odom_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros2_laser_scan_matcher'),'launch','start_matcher.launch.py']
    )

    bogie_description_launch = PathJoinSubstitution(
        [FindPackageShare('bogie_description'),'launch','display.launch.py']
    )

    # Imu_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('serial_imu'),'launch','imu.launch.py']
    # )

    # Ekf_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('ekf_ros2'),'launch','ekf.launch.py']
    # )

    Ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('bogie_navigation'),'config','ekf.yaml']
    )

    return LaunchDescription([
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(lidar_launch_path)
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch_path)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ld_lidar_launch_path)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bogie_description_launch)
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(Imu_launch_path)
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_odom_launch_path)
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(Ekf_launch_path)
            # ),
            Node(
                package='bogie_bringup',
                executable='base_controller',
                name='base_controller_node',
                output='screen',
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                remappings=[("/odometry/filtered","/odom")],
                parameters=[
                    Ekf_config_path
                ],
            ),

    ])