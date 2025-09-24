#!/usr/bin/env python3
"""
ROS2 Arduino Encoder Bridge Node - FIXED VERSION
Reads encoder data from Arduino and publishes as ROS topics
✅ Both encoders working with interrupt+polling solution
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
import serial
import serial.serialutil
import threading
import time


class ArduinoEncoderBridge(Node):
    def __init__(self):
        super().__init__('arduino_encoder_bridge')
        
        # Thread safety
        self.lock = threading.Lock()
        self.encoder1_count = 0
        self.encoder2_count = 0
        
        # Single topic for both encoders
        self.encoder_pub = self.create_publisher(JointState, '/arduino_encoder', 10)
        
        # Timer to publish at 20Hz
        self.timer = self.create_timer(0.05, self.publish_encoders)
        
        # Arduino serial connection
        self.arduino_port = '/dev/ttyACM0'
        self.baud_rate = 38400
        self.arduino = None
        
        # Connect to Arduino
        self.connect_arduino()
        
        # Start reading thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_arduino_data)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        self.get_logger().info('🎉 Arduino Encoder Bridge (FIXED) started!')
        self.get_logger().info('📡 Publishing both encoders to: /arduino_encoder')
    def connect_arduino(self):
        """Connect to Arduino via serial"""
        try:
            # Close existing connection if any
            if self.arduino and self.arduino.is_open:
                self.arduino.close()
                time.sleep(1)
            
            # Configure serial connection with better parameters
            self.arduino = serial.Serial(
                port=self.arduino_port, 
                baudrate=self.baud_rate, 
                timeout=0.5,  # Shorter timeout
                write_timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(2)  # Give Arduino time to reset
            
            # Clear any stale data in buffer
            if self.arduino.is_open:
                self.arduino.flushInput()
                self.arduino.flushOutput()
                self.get_logger().info(f'✅ Connected to Arduino on {self.arduino_port}')
            else:
                raise Exception("Port failed to open")
                
        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect to Arduino: {e}')
            self.arduino = None

    def read_arduino_data(self):
        """Read encoder data from Arduino in separate thread"""
        consecutive_errors = 0
        while self.running and rclpy.ok():
            try:
                if self.arduino and self.arduino.is_open:
                    if self.arduino.in_waiting > 0:
                        try:
                            line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                            if line:  # Only process non-empty lines
                                self.get_logger().debug(f'Raw Arduino data: {line}')
                                if 'ENC1:' in line and 'ENC2:' in line:
                                    self.parse_encoder_data(line)
                                    consecutive_errors = 0  # Reset error counter on success
                            else:
                                # Empty line received - device might be disconnected
                                consecutive_errors += 1
                                if consecutive_errors > 10:
                                    self.get_logger().warn('Received multiple empty lines, checking connection...')
                                    time.sleep(0.1)
                        except serial.SerialTimeoutException:
                            # Timeout is normal, just continue
                            pass
                        except UnicodeDecodeError as e:
                            self.get_logger().debug(f'Unicode decode error: {e}')
                            consecutive_errors += 1
                    time.sleep(0.02)  # Slightly longer delay to reduce CPU load
                else:
                    # Arduino not connected, try to reconnect
                    self.get_logger().warn('Arduino disconnected, attempting reconnection...')
                    self.connect_arduino()
                    time.sleep(2.0)  # Wait before retry
            except (serial.SerialException, OSError, UnicodeDecodeError) as e:
                consecutive_errors += 1
                self.get_logger().warn(f'Serial error ({consecutive_errors}): {e}')
                
                # If too many consecutive errors, try to reconnect
                if consecutive_errors >= 5:
                    self.get_logger().error('Too many serial errors, attempting reconnection...')
                    try:
                        if self.arduino:
                            self.arduino.close()
                    except:
                        pass
                    self.arduino = None
                    self.connect_arduino()
                    consecutive_errors = 0
                    time.sleep(2.0)
                else:
                    time.sleep(0.5)
            except Exception as e:
                self.get_logger().warn(f'Unexpected error: {e}')
                time.sleep(0.1)

    def parse_encoder_data(self, data_line):
        """Parse encoder data from Arduino"""
        try:
            # Parse data: "ENC1:1234,ENC2:5678"
            parts = data_line.split(',')
            
            encoder1_count = None
            encoder2_count = None
            
            for part in parts:
                if part.startswith('ENC1:'):
                    encoder1_count = int(part.split(':')[1])
                elif part.startswith('ENC2:'):
                    encoder2_count = int(part.split(':')[1])
            
            # Update values with thread safety
            if encoder1_count is not None and encoder2_count is not None:
                with self.lock:
                    self.encoder1_count = encoder1_count
                    self.encoder2_count = encoder2_count
                    
        except Exception as e:
            self.get_logger().warn(f'Parse error: {e} | Data: {data_line}')

    def publish_encoders(self):
        """Publish both encoder values to single arduino_encoder topic"""
        with self.lock:
            enc1 = self.encoder1_count
            enc2 = self.encoder2_count
        
        # Create JointState message with both encoders
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['encoder1', 'encoder2']
        msg.position = [float(enc1), float(enc2)]  # Raw counts as position
        msg.velocity = []
        msg.effort = []
        
        # Publish to single topic
        self.encoder_pub.publish(msg)
        
        # Log occasionally
        if enc1 % 100 == 0 or enc2 % 10 == 0:
            self.get_logger().info(f'📊 Encoder1: {enc1}, Encoder2: {enc2}')

    def reset_encoders(self):
        """Send reset command to Arduino"""
        if self.arduino:
            try:
                self.arduino.write(b'RESET\n')
                self.get_logger().info('🔄 Reset command sent to Arduino')
            except Exception as e:
                self.get_logger().error(f'Failed to send reset: {e}')

    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if hasattr(self, 'read_thread') and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        encoder_bridge = ArduinoEncoderBridge()
        rclpy.spin(encoder_bridge)
    except KeyboardInterrupt:
        print('\n🛑 Shutting down Arduino Encoder Bridge...')
    finally:
        if 'encoder_bridge' in locals():
            encoder_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
