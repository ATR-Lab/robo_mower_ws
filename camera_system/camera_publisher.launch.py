#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'camera_device',
            default_value='0',
            description='Camera device index (0, 1, 2, etc.)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30.0',
            description='Publishing rate in Hz'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Image width in pixels'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Image height in pixels'
        ),
        DeclareLaunchArgument(
            'compressed',
            default_value='true',
            description='Publish compressed images (recommended for network)'
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_link',
            description='Camera frame ID'
        ),
        
        # Camera publisher node
        Node(
            package='',  # Will be run directly with python3
            executable='camera_publisher.py',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'camera_device': LaunchConfiguration('camera_device'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'compressed': LaunchConfiguration('compressed'),
                'camera_frame': LaunchConfiguration('camera_frame'),
            }]
        )
    ])
