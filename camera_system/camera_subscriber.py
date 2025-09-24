#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Declare parameters
        self.declare_parameter('compressed', True)
        self.declare_parameter('display_window', True)
        self.declare_parameter('save_images', False)
        self.declare_parameter('save_path', '/tmp/camera_images/')
        
        # Get parameters
        self.compressed = self.get_parameter('compressed').get_parameter_value().bool_value
        self.display_window = self.get_parameter('display_window').get_parameter_value().bool_value
        self.save_images = self.get_parameter('save_images').get_parameter_value().bool_value
        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        self.image_counter = 0
        
        # Create subscriber
        if self.compressed:
            self.subscription = self.create_subscription(
                CompressedImage,
                'camera/image_raw/compressed',
                self.compressed_callback,
                10
            )
            self.get_logger().info('Subscribed to /camera/image_raw/compressed')
        else:
            self.subscription = self.create_subscription(
                Image,
                'camera/image_raw',
                self.image_callback,
                10
            )
            self.get_logger().info('Subscribed to /camera/image_raw')
        
        if self.save_images:
            import os
            os.makedirs(self.save_path, exist_ok=True)
            self.get_logger().info(f'Saving images to {self.save_path}')
        
        if self.display_window:
            self.get_logger().info('Press "q" to quit the display window')
    
    def compressed_callback(self, msg):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                self.process_image(cv_image, msg.header.stamp)
            else:
                self.get_logger().warn('Failed to decode compressed image')
                
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {str(e)}')
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(cv_image, msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def process_image(self, cv_image, timestamp):
        # Display image
        if self.display_window:
            cv2.imshow('Camera Feed', cv_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed')
                rclpy.shutdown()
        
        # Save image
        if self.save_images:
            filename = f'{self.save_path}/image_{self.image_counter:06d}.jpg'
            cv2.imwrite(filename, cv_image)
            self.image_counter += 1
            
            if self.image_counter % 100 == 0:
                self.get_logger().info(f'Saved {self.image_counter} images')
        
        # Log reception (every 30 frames to avoid spam)
        if self.image_counter % 30 == 0:
            self.get_logger().info(f'Received image {self.image_counter}, size: {cv_image.shape}')
        
        self.image_counter += 1
    
    def destroy_node(self):
        if self.display_window:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_subscriber = CameraSubscriber()
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'camera_subscriber' in locals():
            camera_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
