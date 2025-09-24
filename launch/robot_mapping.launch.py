#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

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
        
        # Motor driver node - starts after short delay
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
        
        # Static transform from base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),
        
        # Base footprint transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        
        # Odom to base_footprint transform (for mapping without encoders)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
        ),
        
        # Simple mapping using hector_slam (lightweight SLAM without wheel odometry)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='hector_mapping',
                    executable='hector_mapping',
                    name='hector_mapping',
                    parameters=[{
                        'map_frame': 'map',
                        'base_frame': 'base_footprint',
                        'odom_frame': 'odom',
                        'pub_map_odom_transform': True,
                        'pub_map_scanmatch_transform': True,
                        'pub_odometry': True,
                        'map_resolution': 0.05,
                        'map_size': 2048,
                        'map_start_x': 0.5,
                        'map_start_y': 0.5,
                        'map_multi_res_levels': 2,
                        'use_tf_scan_transformation': True,
                        'use_tf_pose_start_estimate': False,
                        'map_update_distance_thresh': 0.4,
                        'map_update_angle_thresh': 0.06,
                        'laser_z_min_value': -1.0,
                        'laser_z_max_value': 1.0,
                    }],
                    output='screen'
                )
            ]
        ),
        
        # RViz for visualization - starts last
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', '/home/alwinsdon/ros2_ws/robot_mapping.rviz'],
                    output='screen'
                )
            ]
        ),
    ])
