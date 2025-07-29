#!/usr/bin/env python3

"""
Arduino Encoder Reader for ROS2
Reads encoder data from Arduino bridge serial output and publishes to ROS2 topics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import serial
import threading
import re
import time

class ArduinoEncoderReader(Node):
    def __init__(self):
        super().__init__('arduino_encoder_reader')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 38400)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.encoder_pub = self.create_publisher(Int64, 'arduino_encoder', 10)
        self.encoder_delta_pub = self.create_publisher(Int64, 'arduino_encoder_delta', 10)
        
        # Encoder state
        self.last_encoder_count = 0
        self.current_encoder_count = 0
        self.encoder_delta = 0
        
        # Serial connection
        self.serial_conn = None
        self.serial_thread = None
        self.running = True
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_encoder_data)
        
        # Start serial reading thread
        self.start_serial_reading()
        
        self.get_logger().info(f'Arduino Encoder Reader started on {self.serial_port}')

    def start_serial_reading(self):
        """Start the serial reading thread"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                exclusive=False  # Allow sharing with RoboClaw driver
            )
            
            self.serial_thread = threading.Thread(target=self.read_serial_data)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
            self.get_logger().info('Serial connection established')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.get_logger().warn('Will retry in 5 seconds...')
            self.create_timer(5.0, self.retry_serial_connection)

    def retry_serial_connection(self):
        """Retry serial connection"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.start_serial_reading()

    def read_serial_data(self):
        """Read data from serial port in separate thread"""
        buffer = ""
        
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self.process_serial_line(line.strip())
                else:
                    time.sleep(0.01)  # Small delay to prevent busy waiting
                    
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(1.0)
                break

    def process_serial_line(self, line):
        """Process a line of serial data"""
        # Look for encoder data: "ENCODER: 1234 (Δ5)"
        encoder_match = re.match(r'ENCODER:\s*(-?\d+)(?:\s*\(Δ(-?\d+)\))?', line)
        
        if encoder_match:
            try:
                encoder_count = int(encoder_match.group(1))
                delta = int(encoder_match.group(2)) if encoder_match.group(2) else 0
                
                self.current_encoder_count = encoder_count
                self.encoder_delta = delta
                
                self.get_logger().debug(f'Encoder: {encoder_count}, Delta: {delta}')
                
            except ValueError as e:
                self.get_logger().warn(f'Failed to parse encoder data: {line} - {e}')

    def publish_encoder_data(self):
        """Publish encoder data to ROS topics"""
        # Publish current encoder count
        encoder_msg = Int64()
        encoder_msg.data = self.current_encoder_count
        self.encoder_pub.publish(encoder_msg)
        
        # Publish encoder delta
        delta_msg = Int64()
        delta_msg.data = self.encoder_delta
        self.encoder_delta_pub.publish(delta_msg)
        
        # Reset delta after publishing
        self.encoder_delta = 0

    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = ArduinoEncoderReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
