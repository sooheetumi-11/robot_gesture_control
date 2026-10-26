# SPDX-License-Identifier: Apache-2.0
"""Launch file for TurtleBot 4 Gesture Recognition and Robot Behavior nodes."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_gesture_control')
    config_file = os.path.join(pkg_share, 'config', 'gesture_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='Webcam device ID (default: 0 for /dev/video0)',
        ),
        DeclareLaunchArgument(
            'show_window',
            default_value='true',
            description='Whether to display OpenCV visual feedback window',
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.15',
            description='Linear speed of TurtleBot 4 (m/s)',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.40',
            description='Angular speed of TurtleBot 4 (rad/s)',
        ),
        DeclareLaunchArgument(
            'watchdog_timeout',
            default_value='1.2',
            description='Watchdog timeout to stop robot when gesture signal is lost',
        ),

        # Gesture Recognition Node
        Node(
            package='robot_gesture_control',
            executable='gesture_recognition_node',
            name='gesture_recognition_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'camera_id': LaunchConfiguration('camera_id'),
                    'show_window': LaunchConfiguration('show_window'),
                },
            ],
        ),

        # Robot Behavior Node
        Node(
            package='robot_gesture_control',
            executable='robot_behavior_node',
            name='robot_behavior_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'linear_speed': LaunchConfiguration('linear_speed'),
                    'angular_speed': LaunchConfiguration('angular_speed'),
                    'watchdog_timeout': LaunchConfiguration('watchdog_timeout'),
                },
            ],
        ),
    ])
