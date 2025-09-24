#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
import cv2
import numpy as np
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('compressed', True)  # Publish compressed images for network efficiency
        self.declare_parameter('camera_frame', 'camera_link')
        
        # Get parameters
        camera_device = self.get_parameter('camera_device').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.compressed = self.get_parameter('compressed').get_parameter_value().bool_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera device {camera_device}')
            return
            
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, publish_rate)
        
        # Create publishers - only compressed for simplicity
        self.image_pub = self.create_publisher(
            CompressedImage, 
            'camera/image_raw/compressed', 
            10
        )
        self.get_logger().info('Publishing compressed images to /camera/image_raw/compressed')
        
        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Camera publisher started at {publish_rate} Hz')
        self.get_logger().info(f'Camera resolution: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'Camera frame: {self.camera_frame}')
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera')
            return
            
        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.camera_frame
        
        try:
            # Create compressed image message
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            
            # Encode frame as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]  # 80% quality
            result, encimg = cv2.imencode('.jpg', frame, encode_param)
            
            if result:
                compressed_msg.data = np.array(encimg).tobytes()
                self.image_pub.publish(compressed_msg)
                # Log every 30 frames to avoid spam
                if hasattr(self, 'frame_count'):
                    self.frame_count += 1
                else:
                    self.frame_count = 1
                    
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Published frame {self.frame_count}, size: {len(compressed_msg.data)} bytes')
            else:
                self.get_logger().warn('Failed to compress image')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')
    
    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'camera_publisher' in locals():
            camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
