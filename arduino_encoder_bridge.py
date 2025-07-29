#!/usr/bin/env python3

"""
Arduino Encoder Bridge for ROS2
Reads encoder values from Arduino and publishes to ROS topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import threading
import time

class ArduinoEncoderBridge(Node):
    def __init__(self):
        super().__init__('arduino_encoder_bridge')
        
        # Initialize thread lock first
        self.lock = threading.Lock()
        self.encoder1_value = 0
        self.encoder2_value = 0
        
        # Publisher for encoder data
        self.encoder_pub = self.create_publisher(JointState, 'arduino_encoder', 10)
        
        # Timer to publish encoder data at 20Hz
        self.timer = self.create_timer(0.05, self.publish_encoder)
        
        # Serial connection to Arduino
        try:
            self.arduino = serial.Serial('/dev/ttyACM1', 38400, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            self.get_logger().info("Connected to Arduino on /dev/ttyACM1")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            return
        
        # Start reading thread
        self.reading_thread = threading.Thread(target=self.read_arduino)
        self.reading_thread.daemon = True
        self.reading_thread.start()
        
        self.get_logger().info("Arduino Dual Encoder Bridge started")
    
    def read_arduino(self):
        """Read encoder values from Arduino in separate thread"""
        while True:
            try:
                if self.arduino and self.arduino.in_waiting:
                    line = self.arduino.readline().decode().strip()
                    # More robust parsing to handle corrupted data
                    if "ENC1:" in line and "ENC2:" in line:
                        try:
                            # Find ENC1 and ENC2 values using regex-like approach
                            enc1_start = line.find("ENC1:") + 5
                            enc1_end = line.find(",", enc1_start)
                            if enc1_end == -1:
                                enc1_end = line.find("ENC2:", enc1_start)
                            
                            enc2_start = line.find("ENC2:") + 5
                            enc2_end = len(line)
                            for i in range(enc2_start, len(line)):
                                if not line[i].isdigit():
                                    enc2_end = i
                                    break
                            
                            enc1_str = line[enc1_start:enc1_end]
                            enc2_str = line[enc2_start:enc2_end]
                            
                            # Extract only digits
                            enc1_digits = ''.join(filter(str.isdigit, enc1_str))
                            enc2_digits = ''.join(filter(str.isdigit, enc2_str))
                            
                            if enc1_digits and enc2_digits:
                                enc1_value = int(enc1_digits)
                                enc2_value = int(enc2_digits)
                                
                                with self.lock:
                                    self.encoder1_value = enc1_value
                                    self.encoder2_value = enc2_value
                                # Reduce logging frequency to avoid spam
                                if enc1_value % 1000 == 0:  # Log every 1000 counts
                                    self.get_logger().info(f"Encoder values: ENC1={enc1_value}, ENC2={enc2_value}")
                        except (ValueError, IndexError) as e:
                            self.get_logger().debug(f"Parse error on line '{line}': {e}")
                    elif line.startswith("ENC:"):
                        # Fallback for single encoder format
                        try:
                            value = int(line[4:])
                            with self.lock:
                                self.encoder1_value = value
                        except ValueError:
                            pass
            except Exception as e:
                self.get_logger().error(f"Error reading Arduino: {e}")
                time.sleep(0.1)
    
    def publish_encoder(self):
        """Publish encoder data to ROS topic"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['front_left_wheel', 'front_right_wheel']  # Match existing topic names
        
        with self.lock:
            # Convert encoder counts to position (assuming 1000 counts per revolution)
            position1 = (self.encoder1_value / 1000.0) * 2.0 * 3.14159  # radians
            position2 = (self.encoder2_value / 1000.0) * 2.0 * 3.14159  # radians
            msg.position = [position1, position2]
            msg.velocity = []  # Empty for now
            msg.effort = []
        
        self.encoder_pub.publish(msg)
        # Reduce logging frequency
        if self.encoder1_value % 2000 == 0:  # Log occasionally
            self.get_logger().info(f"Publishing: right={position1:.2f} rad ({self.encoder1_value} counts), left={position2:.2f} rad ({self.encoder2_value} counts)")

def main(args=None):
    rclpy.init(args=args)
    
    node = ArduinoEncoderBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
