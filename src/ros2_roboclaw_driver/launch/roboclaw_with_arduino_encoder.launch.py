import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ros2_roboclaw_driver')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'motor_driver.yaml'),
        description='Path to the motor driver configuration file'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM1',
        description='Arduino serial port'
    )
    
    # RoboClaw driver node
    roboclaw_driver_node = Node(
        package='ros2_roboclaw_driver',
        executable='ros2_roboclaw_driver_node',
        name='roboclaw_driver',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    # Arduino encoder reader node
    arduino_encoder_node = Node(
        package='ros2_roboclaw_driver',
        executable='arduino_encoder_reader.py',
        name='arduino_encoder_reader',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 38400,
            'publish_rate': 20.0
        }],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        serial_port_arg,
        roboclaw_driver_node,
        arduino_encoder_node
    ])
