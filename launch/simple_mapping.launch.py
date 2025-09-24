#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # LiDAR node - starts immediately
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
        ),
        
        # Motor driver node
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros2_roboclaw_driver',
                    executable='roboclaw_driver_node',
                    name='roboclaw_driver',
                    parameters=['/home/alwinsdon/ros2_ws/src/ros2_roboclaw_driver/config/motor_driver.yaml'],
                    output='screen'
                )
            ]
        ),
        
        # Static transforms for robot geometry
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_broadcaster', 
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),
        
        # Map to odom transform (for simple mapping without odometry)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # Odom to base_footprint (robot pose in odom frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen'
        ),
        
        # No additional mapping nodes - just visualize laser scans
        
        # RViz for visualization
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', '/home/alwinsdon/ros2_ws/simple_mapping.rviz'],
                    output='screen'
                )
            ]
        ),
    ])
